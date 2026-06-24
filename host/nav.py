#!/usr/bin/env python3
"""
core navigation loop.

follow_path()     drives the robot through a waypoint list using live pose feedback.
_turn_command()   converts a heading error into a compensated TURN:N command.
"""
import math
import time

from config import (
    ARRIVE_PX, TURN_TOL_DEG, TURN_COMMIT_DEG, TURN_SLOPE, TURN_COAST_DEG,
    MAX_STEP_MM, MIN_STEP_MM, FORWARD_CMD_SCALE, ACTUAL_PX_PER_MM,
    MAX_POSE_MISS, REPLAN_PX, REPLAN_EVERY_N, HEADING_LOOKAHEAD_PX,
)


def _turn_command(err_deg):
    """compensated turn command: inverts actual=TURN_SLOPE*cmd+TURN_COAST_DEG to land on target."""
    mag = (abs(err_deg) - TURN_COAST_DEG) / TURN_SLOPE
    mag = max(2, int(round(mag)))
    return "TURN:{}".format(int(math.copysign(mag, err_deg)))


def follow_path(get_pose, link, waypoints, px_per_mm,
                face_deg=None, on_step=None, on_arrive=None, replan=None):
    """drive the robot through waypoints using live pose feedback. returns True on completion."""
    idx = 0
    misses = 0
    just_turned = False
    steps = 0
    fwd_steps = 0
    max_steps = 40 * len(waypoints) + 60

    while idx < len(waypoints):
        steps += 1
        if steps > max_steps:
            print("Exceeded {} steps without finishing - aborting (not converging).".format(max_steps))
            link.send_and_wait("STOP")
            return False
        pose = get_pose()
        if pose is None:
            misses += 1
            if misses == 1:
                link.send_and_wait("STOP")
            if misses % 5 == 0 or misses == MAX_POSE_MISS:
                print("[MISS {}/{}] robot marker not visible".format(misses, MAX_POSE_MISS))
            if misses >= MAX_POSE_MISS:
                print("Lost the robot marker for too long - aborting.")
                return False
            time.sleep(0.05)
            continue
        misses = 0

        x, y, heading = pose
        tx, ty = waypoints[idx]
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)

        if dist < ARRIVE_PX:
            # enforce heading toward the next waypoint - ONE correction turn,
            # then advance immediately WITHOUT re-checking position.
            # re-checking position after a turn causes an infinite loop because
            # the robot's pivot offset moves the center away from the waypoint.
            if idx + 1 < len(waypoints):
                nx, ny = waypoints[idx + 1]
                req = math.degrees(math.atan2(ny - y, nx - x))
                head_err = (req - heading + 180.0) % 360.0 - 180.0
                if abs(head_err) > TURN_TOL_DEG:
                    turn_cmd = _turn_command(head_err)
                    if on_step:
                        on_step(pose, waypoints[idx], idx, turn_cmd, len(waypoints))
                    if not link.send_and_wait(turn_cmd):
                        print("No ack for {} - aborting.".format(turn_cmd))
                        return False
                    just_turned = True
            if on_arrive:
                on_arrive(waypoints[idx], idx, waypoints)
            idx += 1
            just_turned = False
            continue

        if replan is not None and dist > REPLAN_PX:
            new_wp = replan()
            if new_wp:
                waypoints = new_wp
                idx = 0
                continue

        bearing = math.degrees(math.atan2(dy, dx))
        step_mm = max(MIN_STEP_MM, min(MAX_STEP_MM, dist / ACTUAL_PX_PER_MM))
        in_close_approach = dist < 2 * ARRIVE_PX

        # interpolate heading target toward the next waypoint's bearing so heading
        # is reached *during* the approach, not corrected post-arrival.
        # heading_target is separate from the movement bearing so the TURN pre-aligns
        # without redirecting the FORWARD step - the robot still drives toward the
        # current waypoint, just already facing where it needs to go on arrival.
        # linear blend: 0% next-bearing at HEADING_LOOKAHEAD_PX, 100% at close-approach.
        # skip if next bearing is >90° away - don't shortcut corners.
        heading_target = bearing
        if (not in_close_approach
                and dist < HEADING_LOOKAHEAD_PX
                and idx + 1 < len(waypoints)):
            nx, ny = waypoints[idx + 1]
            nxt_bearing = math.degrees(math.atan2(ny - y, nx - x))
            delta = (nxt_bearing - bearing + 180.0) % 360.0 - 180.0
            if abs(delta) < 90:
                t = 1.0 - (dist - 2 * ARRIVE_PX) / (HEADING_LOOKAHEAD_PX - 2 * ARRIVE_PX)
                t = max(0.0, min(1.0, t))
                heading_target = bearing + t * delta

        # err drives TURN decisions; move_err drives REVERSE (waypoint behind robot).
        # separating them means pre-turning toward the next bearing never triggers
        # a spurious REVERSE when the current waypoint is still ahead.
        err = (heading_target - heading + 180.0) % 360.0 - 180.0
        move_err = (bearing - heading + 180.0) % 360.0 - 180.0

        # only turn when meaningfully off-heading, AND not immediately after another
        # turn unless still badly off (> TURN_COMMIT_DEG). forcing a forward step
        # between turns breaks the overshoot limit-cycle.
        # when the waypoint is directly behind (|move_err| > 150°) just reverse -
        # eliminates U-turn oscillation on small overshoots. within 2*ARRIVE_PX
        # suppress all heading correction: turning in place drifts the ArUco marker.
        if abs(move_err) > 150 and not in_close_approach:
            cmd = "REVERSE:{}".format(int(round(step_mm)))
            just_turned = False
            fwd_steps += 1
        elif (abs(err) > TURN_TOL_DEG
              and not in_close_approach
              and not (just_turned and abs(err) <= TURN_COMMIT_DEG)):
            cmd = _turn_command(err)
            just_turned = True
        else:
            cmd = "FORWARD:{}".format(int(round(step_mm * FORWARD_CMD_SCALE)))
            just_turned = False
            fwd_steps += 1

        print("[NAV] brg={:.0f}deg head={:.0f}deg err={:.0f}deg move_err={:.0f}deg dist={:.0f}px -> {}".format(
            bearing, heading, err, move_err, dist, cmd))
        if on_step:
            on_step(pose, waypoints[idx], idx, cmd, len(waypoints))
        t_send = time.time()
        if not link.send_and_wait(cmd):
            print("No ack for {} - aborting.".format(cmd))
            return False
        elapsed = time.time() - t_send
        after = get_pose()
        if after is not None:
            d_head = (after[2] - heading + 180.0) % 360.0 - 180.0
            d_dist = math.hypot(after[0] - x, after[1] - y)
            print("[ACK {:.1f}s] pose=({:.0f},{:.0f},{:.0f}deg) dhead={:.0f}deg dpos={:.0f}px".format(
                elapsed, after[0], after[1], after[2], d_head, d_dist))
        else:
            print("[ACK {:.1f}s] pose lost after cmd".format(elapsed))

        is_forward_move = cmd.startswith("FORWARD") or cmd.startswith("REVERSE")
        if (is_forward_move and replan is not None
                and fwd_steps > 0 and fwd_steps % REPLAN_EVERY_N == 0):
            new_wp = replan()
            if new_wp:
                waypoints = new_wp
                idx = 0
                max_steps = 40 * len(waypoints) + 60

    # arrived: optionally face the hole, then stop.
    if face_deg is not None:
        pose = get_pose()
        if pose is not None:
            err = (face_deg - pose[2] + 180.0) % 360.0 - 180.0
            if abs(err) > TURN_TOL_DEG:
                link.send_and_wait(_turn_command(err))
    link.send_and_wait("STOP")
    return True
