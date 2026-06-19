#!/usr/bin/env python3
"""
Core navigation loop.

follow_path()     drives the robot through a waypoint list using live pose feedback.
_turn_command()   converts a heading error into a compensated TURN:N command.
"""
import math
import time

from config import (
    ARRIVE_PX, TURN_TOL_DEG, TURN_COMMIT_DEG, TURN_SLOPE, TURN_COAST_DEG,
    MAX_STEP_MM, MIN_STEP_MM, FORWARD_CMD_SCALE, ACTUAL_PX_PER_MM,
    MAX_POSE_MISS, REPLAN_PX, REPLAN_EVERY_N,
)


def _turn_command(err_deg):
    """Compensated TURN command so the robot actually rotates ~err_deg.

    Inverts the measured response: actual = TURN_SLOPE*cmd + TURN_COAST_DEG.
    """
    mag = (abs(err_deg) - TURN_COAST_DEG) / TURN_SLOPE
    mag = max(2, int(round(mag)))
    return "TURN:{}".format(int(math.copysign(mag, err_deg)))


def follow_path(get_pose, link, waypoints, px_per_mm,
                face_deg=None, on_step=None, on_arrive=None, replan=None):
    """Drive the robot through `waypoints` using live pose feedback.

    get_pose(): -> (x, y, heading_deg) in image space, or None when the robot
                is not currently visible.
    link:       object with send_and_wait(command)->bool.
    replan():   optional callback -> new waypoint list, called when the robot
                drifts more than REPLAN_PX from its target.

    Returns True if the path was completed, False if aborted (pose lost / abort).
    """
    idx = 0
    misses = 0
    just_turned = False
    steps = 0
    fwd_steps = 0
    max_steps = 40 * len(waypoints) + 60

    while idx < len(waypoints):
        steps += 1
        if steps > max_steps:
            print("Exceeded {} steps without finishing — aborting (not converging).".format(max_steps))
            link.send_and_wait("STOP")
            return False
        pose = get_pose()
        if pose is None:
            misses += 1
            if misses == 1:
                link.send_and_wait("STOP")
            if misses >= MAX_POSE_MISS:
                print("Lost the robot marker for too long — aborting.")
                return False
            time.sleep(0.05)
            continue
        misses = 0

        x, y, heading = pose
        tx, ty = waypoints[idx]
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)

        if dist < ARRIVE_PX:
            # Enforce heading toward the next waypoint — ONE correction turn,
            # then advance immediately WITHOUT re-checking position.
            # Re-checking position after a turn causes an infinite loop because
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
                        print("No ack for {} — aborting.".format(turn_cmd))
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
        err = (bearing - heading + 180.0) % 360.0 - 180.0

        # Turn only when meaningfully off-heading, AND not immediately after
        # another turn unless we're still badly off (> TURN_COMMIT_DEG). Forcing
        # a forward step between turns breaks the overshoot limit-cycle.
        # When the waypoint is directly behind (|err| > 150°) don't turn 180° —
        # just reverse. This eliminates U-turn oscillation on small overshoots.
        # Use actual camera px/mm (measured) for step sizing, not the planner's
        # value which is ~4x too low and causes every close approach to hit MAX_STEP_MM.
        step_mm = max(MIN_STEP_MM, min(MAX_STEP_MM, dist / ACTUAL_PX_PER_MM))
        if abs(err) > 150:
            cmd = "REVERSE:{}".format(int(round(step_mm)))
            just_turned = False
            fwd_steps += 1
        elif (abs(err) > TURN_TOL_DEG
              and not (just_turned and abs(err) <= TURN_COMMIT_DEG)):
            cmd = _turn_command(err)
            just_turned = True
        else:
            cmd = "FORWARD:{}".format(int(round(step_mm * FORWARD_CMD_SCALE)))
            just_turned = False
            fwd_steps += 1

        if on_step:
            on_step(pose, waypoints[idx], idx, cmd, len(waypoints))
        if not link.send_and_wait(cmd):
            print("No ack for {} — aborting.".format(cmd))
            return False

        is_forward_move = cmd.startswith("FORWARD") or cmd.startswith("REVERSE")
        if (is_forward_move and replan is not None
                and fwd_steps > 0 and fwd_steps % REPLAN_EVERY_N == 0):
            new_wp = replan()
            if new_wp:
                waypoints = new_wp
                idx = 0
                max_steps = 40 * len(waypoints) + 60

    # Arrived: optionally face the hole, then stop.
    if face_deg is not None:
        pose = get_pose()
        if pose is not None:
            err = (face_deg - pose[2] + 180.0) % 360.0 - 180.0
            if abs(err) > TURN_TOL_DEG:
                link.send_and_wait(_turn_command(err))
    link.send_and_wait("STOP")
    return True
