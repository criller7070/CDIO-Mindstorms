#!/usr/bin/env python3
"""
Closed-loop navigation controller (HOST PC).

Instead of computing a whole mission and dead-reckoning it (open-loop, which
drifts as small per-command errors accumulate), this drives the robot ONE small
move at a time while continuously watching it through the camera:

    observe robot pose (ArUco)  ->  compare to next waypoint  ->  send ONE move
            ^                                                          |
            +----------------  wait for the robot's DONE  <-----------+

Because the host re-measures the robot's real position and heading after every
single move, translation/heading errors are corrected before they grow, so the
robot stays on the planned path.

Modes:
    python closed_loop_controller.py                 # live: camera + Bluetooth
    python closed_loop_controller.py --probe         # just print live ArUco pose
    python closed_loop_controller.py --plan-only     # plan from one frame, show path
    python closed_loop_controller.py --sim           # validate the loop, no hardware
    python closed_loop_controller.py --camera 0      # override camera index

Requires an ArUco marker on the robot — see generate_aruco_marker.py.
"""
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import sys
import math
import time
import threading

import cv2
import numpy as np

from tools_path_planner import FieldPlanner, FIELD_WIDTH_MM, FIELD_HEIGHT_MM
from vision_config import (
    CAMERA_INDEX, WALL_MARGIN, CENTER_RADIUS, INITIAL_HEADING_DEG,
    HOLE_FRAC_X, HOLE_FRAC_Y,
    ROBOT_WIDTH_MM, ROBOT_LENGTH_MM, ROBOT_PIVOT_OFFSET_MM,
    ROBOFLOW_API_KEY, ROBOFLOW_API_URL, ROBOFLOW_MODEL_ID,
    load_color_ranges,
)
from vision_detector import BallDetector


# ── Control-loop tunables ─────────────────────────────────────────────────────
ARRIVE_PX      = 12.0   # waypoint counts as reached within this many pixels.
                        # Must exceed one forward step, or the robot steps PAST a
                        # waypoint without registering arrival and spins to go back.
                        # MAX_STEP_MM * px_per_mm ≈ 7px — so 12px is the safe floor.
                        # Ball waypoints are trimmed back ~66px (half robot length)
                        # so the nose stops at ball ± 12px = ± ~34mm.
TURN_TOL_DEG   = 8.0    # rotate only for heading errors larger than this. Must
                        # exceed the EV3 gyro turn's coast/overshoot, or the loop
                        # limit-cycles (turn past target, correct back, repeat).
                        # TURN_COAST_DEG=3° so an 8° threshold leaves 5° of cmd
                        # headroom — small overshoots won't re-trigger a correction.
TURN_COMMIT_DEG = 45.0  # after a turn, drive a forward step before turning again
                        # unless the heading error still exceeds this. Prevents
                        # turn-turn-turn oscillation from small overshoots.
# Measured EV3 follow-mode turn response: actual = TURN_SLOPE*commanded + coast.
# Compensate so a requested heading change actually lands on target:
#   command = (desired - TURN_COAST_DEG) / TURN_SLOPE
# Recalibrated from run log (3 observed turns at 45 deg/s):
#   cmd=104 → actual=108, cmd=61 → actual=63, cmd=156 → actual=158
#   Best fit: actual ≈ 1.0*cmd + 3  (coast much smaller than original 8 deg)
TURN_SLOPE     = 1.0
TURN_COAST_DEG = 3.0
MAX_STEP_MM    = 20     # never drive more than this (physical mm) between observations
                        # CRITICAL: must satisfy MAX_STEP_MM * px_per_mm < ARRIVE_PX
                        # 20mm * 0.358px/mm ≈ 7px << ARRIVE_PX=20px — safe.
MIN_STEP_MM    = 10     # smallest forward nudge worth sending
# robot/main.py executes FORWARD:v as straight(-v / 3.2288), i.e. the command
# value is ~3.2x the physical mm travelled. Scale the command so a requested
# physical step actually moves that far (otherwise the robot crawls ~1/3 speed).
FORWARD_CMD_SCALE = 3.2288
MAX_POSE_MISS  = 60     # give up after this many consecutive frames with no marker
REPLAN_PX      = 150.0  # re-plan when robot is >150px off its target. Lower than
                        # original 400 so drift is caught early, not just on major
                        # divergence. Per-step heading correction handles smaller errors.
REPLAN_EVERY_N = 8      # also force a replan after every N forward steps so ball
                        # positions are refreshed even when drift is under REPLAN_PX.
DENSIFY_GAP_PX = 20.0   # maximum pixel gap between consecutive waypoints after
                        # densification.  Intermediate points are inserted along each
                        # segment so the heading correction fires every ~56mm of
                        # travel, preventing lateral drift from accumulating over a
                        # long single step.  Tighter than original 30px to give more
                        # heading corrections on the final approach to each ball.
GATE_OPEN_DEG  = 90    # motor angle sent with GATE_OPEN (open to collect a ball)
GATE_CLOSE_DEG = 90    # motor angle sent with GATE_CLOSE (close to retain a ball)
BALL_GATE_THRESHOLD_PX = 80  # waypoint is treated as a ball pickup point when
                              # it is within this many pixels of a detected ball.
                              # Must exceed the half-length trim (~66px at 0.358px/mm)
                              # or _near_ball() returns False for trimmed ball waypoints
                              # and the gate never fires.
# Extra pixels added to the camera-detected X radius before passing it to A*.
# The X arms are thin so minEnclosingCircle underestimates the physical obstacle.
CENTER_OBSTACLE_EXTRA_PX = 40


# ──────────────────────────────────────────────────────────────────────────────
# Robot links: where commands actually go.  Both expose send_and_wait(cmd)->bool.
# ──────────────────────────────────────────────────────────────────────────────
class BluetoothLink:
    """Sends single commands to the EV3 bridge and blocks for its DONE ack."""

    def __init__(self, ev3_address=None):
        from tools_mission_sender import HostMissionSender
        self.sender = HostMissionSender(ev3_address)

    def connect(self):
        if not self.sender.connect_to_ev3(self.sender.ev3_address):
            return False
        # Confirm the bridge is alive before we start driving.
        return self.sender.send_and_wait("PING", timeout=5.0, ack_token="PONG")

    def send_and_wait(self, command):
        return self.sender.send_and_wait(command)

    def close(self):
        self.sender.disconnect()


class TCPLink:
    """Sends single commands to the EV3 bridge over TCP/WiFi and blocks for DONE.

    Lower latency and simpler than Bluetooth, and the robot is already on WiFi.
    Matches robot/ev3_server.py --tcp.
    """

    def __init__(self, host, port=9999, timeout=20.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None

    def connect(self):
        import socket
        try:
            self.sock = socket.create_connection((self.host, self.port), timeout=10.0)
        except OSError as e:
            print("TCP connect to {}:{} failed: {}".format(self.host, self.port, e))
            return False
        # Confirm the bridge is alive before we start driving.
        return self._send_wait("PING\n", "PONG")

    def send_and_wait(self, command):
        if not command.endswith("\n"):
            command += "\n"
        return self._send_wait(command, "DONE")

    def _send_wait(self, payload, token):
        import socket
        try:
            self.sock.sendall(payload.encode("utf-8"))
        except OSError as e:
            print("TCP send failed: {}".format(e))
            return False
        self.sock.settimeout(self.timeout)
        buf = ""
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            try:
                data = self.sock.recv(1024)
            except socket.timeout:
                break
            if not data:
                print("TCP connection closed by robot.")
                return False
            buf += data.decode("utf-8", errors="ignore")
            if token in buf:
                return True
            if "TIMEOUT" in buf:
                print("Robot reported TIMEOUT on: {}".format(payload.strip()))
                return False
        print("TCP timeout waiting for {} (cmd: {})".format(token, payload.strip()))
        return False

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass


class SimLink:
    """A virtual robot for testing the loop with no hardware.

    Mirrors robot/main.py command semantics so the sim is faithful: FORWARD:v
    moves v / FORWARD_CMD_SCALE physical mm, and TURN coasts a constant
    `turn_overshoot_deg` past the commanded angle (the real cause of the limit
    cycle this loop must avoid).  Optional gains/noise add error so we can
    confirm the closed loop corrects drift without oscillating.
    """

    def __init__(self, pose, px_per_mm, turn_gain=1.0, fwd_gain=1.0,
                 lateral_noise_mm=0.0, turn_overshoot_deg=0.0, seed=0):
        self.x, self.y, self.heading = pose
        self.px_per_mm = px_per_mm
        self.turn_gain = turn_gain
        self.fwd_gain = fwd_gain
        self.lateral_noise_mm = lateral_noise_mm
        self.turn_overshoot_deg = turn_overshoot_deg
        self.rng = np.random.default_rng(seed)
        self.trail = [(self.x, self.y)]

    def send_and_wait(self, command):
        cmd, _, val = command.partition(":")
        cmd = cmd.strip().upper()
        if cmd == "TURN":
            v = float(val)
            applied = v * self.turn_gain
            if v != 0:                       # constant coast past the target
                applied += math.copysign(self.turn_overshoot_deg, v)
            self.heading += applied
            self.heading = (self.heading + 180.0) % 360.0 - 180.0
        elif cmd == "FORWARD":
            d_mm = (float(val) / FORWARD_CMD_SCALE) * self.fwd_gain
            d_px = d_mm * self.px_per_mm
            hr = math.radians(self.heading)
            self.x += d_px * math.cos(hr)
            self.y += d_px * math.sin(hr)
            if self.lateral_noise_mm:
                lat = self.rng.normal(0.0, self.lateral_noise_mm) * self.px_per_mm
                self.x += lat * math.cos(hr + math.pi / 2)
                self.y += lat * math.sin(hr + math.pi / 2)
            self.trail.append((self.x, self.y))
        # STOP / SPEED / PING: no pose change.
        return True

    def pose(self):
        return (self.x, self.y, self.heading)


# ──────────────────────────────────────────────────────────────────────────────
# Planning: turn a frame into a list of pixel waypoints to track.
# ──────────────────────────────────────────────────────────────────────────────
def plan_waypoints(detector, frame, robot_pos):
    """Plan the full route from robot_pos and return it as pixel waypoints.

    Reuses FieldPlanner exactly as the open-loop planner does, then flattens the
    planner's driven polylines into an ordered waypoint list that the control
    loop tracks with live pose feedback.

    Returns (waypoints, planner, analysis, dropoff, face_deg).
    """
    analysis = detector.analyze_course(frame)
    x_min, y_min, x_max, y_max = analysis['field_bounds']
    center_pos    = analysis['center_pos']
    wall_margin   = analysis['wall_margin'] or WALL_MARGIN
    center_radius = (analysis.get('center_radius') or CENTER_RADIUS) + CENTER_OBSTACLE_EXTRA_PX

    if analysis['field_detected']:
        raw_dropoff = (x_min, (y_min + y_max) // 2)
        face_deg = 180.0
    else:
        raw_dropoff = (int(x_min + (x_max - x_min) * HOLE_FRAC_X),
                       int(y_min + (y_max - y_min) * HOLE_FRAC_Y))
        face_deg = 180.0

    planner = FieldPlanner(
        field_bounds=analysis['field_bounds'],
        center_pos=center_pos,
        wall_margin=wall_margin,
        center_radius=center_radius,
        field_width_mm=FIELD_WIDTH_MM,
        field_height_mm=FIELD_HEIGHT_MM,
        field_hull=analysis['field_hull'],
        robot_width_mm=ROBOT_WIDTH_MM,
        robot_length_mm=ROBOT_LENGTH_MM,
        pivot_offset_mm=ROBOT_PIVOT_OFFSET_MM,
    )
    dropoff = planner.snap_to_navigable(*raw_dropoff)
    ball_positions = [(b['x'], b['y']) for b in analysis['balls']]

    planner.plan_trips(
        robot_pos=robot_pos,
        ball_positions=ball_positions,
        dropoff_pos=dropoff,
        capacity=6,
        initial_heading_deg=INITIAL_HEADING_DEG,
        face_deg=face_deg,
    )

    segs = getattr(planner, '_debug_path_segs', [])
    waypoints = _flatten_segs(segs)
    return waypoints, planner, analysis, dropoff, face_deg


def _flatten_segs(segs, min_gap_px=6.0):
    """Concatenate planner polylines into one ordered, de-duplicated waypoint list."""
    pts = []
    for seg in segs:
        for p in seg:
            fp = (float(p[0]), float(p[1]))
            if not pts or math.hypot(fp[0] - pts[-1][0], fp[1] - pts[-1][1]) >= min_gap_px:
                pts.append(fp)
    return pts


def _densify_waypoints(waypoints, max_gap_px=DENSIFY_GAP_PX):
    """Insert intermediate checkpoints so no consecutive pair is more than
    max_gap_px apart.

    Each checkpoint forces a pose measurement and heading correction, so
    lateral drift can't accumulate over a long single forward step.  At the
    typical camera scale (~3.5 px/mm) a 40 px gap equals ~11 mm of travel —
    roughly a quarter of a golf ball diameter.
    """
    if len(waypoints) < 2:
        return list(waypoints)
    dense = [waypoints[0]]
    for p1 in waypoints[1:]:
        p0 = dense[-1]
        d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        if d > max_gap_px:
            n = int(math.ceil(d / max_gap_px))
            for j in range(1, n):
                t = j / n
                dense.append((p0[0] + t * (p1[0] - p0[0]),
                               p0[1] + t * (p1[1] - p0[1])))
        dense.append(p1)
    return dense


# ──────────────────────────────────────────────────────────────────────────────
# The control loop.
# ──────────────────────────────────────────────────────────────────────────────
def _turn_command(err_deg):
    """Compensated TURN command so the robot actually rotates ~err_deg.

    Inverts the measured response actual = TURN_SLOPE*cmd + TURN_COAST_DEG.
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
    just_turned = False   # don't turn twice in a row (anti-oscillation)
    steps = 0
    fwd_steps = 0         # count forward steps for periodic replan
    max_steps = 40 * len(waypoints) + 60   # guard against a non-converging loop

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
                link.send_and_wait("STOP")   # freeze while we can't see it
            if misses >= MAX_POSE_MISS:
                print("Lost the robot marker for too long — aborting.")
                return False
            time.sleep(0.05)   # avoid tight-looping while waiting for marker
            continue
        misses = 0

        x, y, heading = pose
        tx, ty = waypoints[idx]
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)

        if dist < ARRIVE_PX:
            if on_arrive:
                on_arrive(waypoints[idx], idx, waypoints)
            idx += 1
            continue

        # Strayed far from the path? Re-plan from where we actually are.
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
        turn_now = (abs(err) > TURN_TOL_DEG
                    and not (just_turned and abs(err) <= TURN_COMMIT_DEG))
        if turn_now:
            cmd = _turn_command(err)
            just_turned = True
        else:
            step_mm = max(MIN_STEP_MM, min(MAX_STEP_MM, dist / px_per_mm))
            cmd = "FORWARD:{}".format(int(round(step_mm * FORWARD_CMD_SCALE)))
            just_turned = False
            fwd_steps += 1

        if on_step:
            on_step(pose, waypoints[idx], idx, cmd, len(waypoints))
        if not link.send_and_wait(cmd):
            print("No ack for {} — aborting.".format(cmd))
            return False

        # Periodic replan: refresh ball positions and recalculate route from
        # current pose every REPLAN_EVERY_N forward steps, even when drift is
        # small. This corrects for accumulated position error and re-detects
        # balls that have moved or were missed in the initial plan.
        if (not turn_now and replan is not None
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


# ──────────────────────────────────────────────────────────────────────────────
# Live camera pose source.
# ──────────────────────────────────────────────────────────────────────────────
class CameraPoseSource:
    """Continuously grabs frames on a background thread so the control loop
    always gets a fresh pose — even after a long blocking send_and_wait().

    Previously, reads happened on the main thread with a small flush buffer,
    so frames could pile up during a blocking move and detection would
    momentarily fail on the stale frames that followed.  The background thread
    eliminates that: it drains the camera as fast as it can, so __call__()
    always sees the most recently captured frame.

    The debug window (imshow) runs inside the grab loop thread so the view
    refreshes at camera frame rate even while the main thread is blocked
    inside send_and_wait(). On Linux/X11 this is safe; imshow/waitKey can
    be called from non-main threads.
    """

    def __init__(self, cap, detector, flip=True, show=True):
        self.cap = cap
        self.detector = detector
        self.flip = flip
        self.show = show
        self.aborted = False
        self.target = None
        self.waypoints = None
        self._lock = threading.Lock()
        self._latest_frame = None
        self._latest_pose = None
        self._running = True
        self._thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._thread.start()

    def _grab_loop(self):
        while self._running:
            ret, frame = self.cap.read()
            if not ret:
                continue
            if self.flip:
                frame = cv2.flip(frame, 1)
            pose = self.detector.detect_robot(frame)
            with self._lock:
                self._latest_frame = frame
                self._latest_pose = pose
                waypoints = self.waypoints   # snapshot for render — benign race
                target = self.target
            if self.show:
                self._render(frame, pose, waypoints, target)

    def grab(self):
        with self._lock:
            return self._latest_frame

    def stop(self):
        self._running = False
        self._thread.join(timeout=2.0)

    def __call__(self):
        with self._lock:
            return self._latest_pose

    def _render(self, frame, pose, waypoints, target):
        vis = frame.copy()
        if waypoints:
            for i in range(1, len(waypoints)):
                p0 = tuple(map(int, waypoints[i - 1]))
                p1 = tuple(map(int, waypoints[i]))
                cv2.line(vis, p0, p1, (80, 80, 80), 1)
        if target is not None:
            t = tuple(map(int, target))
            cv2.circle(vis, t, int(ARRIVE_PX), (255, 0, 255), 1)
            cv2.drawMarker(vis, t, (255, 0, 255), cv2.MARKER_TILTED_CROSS, 12, 2)
        if pose is not None:
            x, y, h = pose
            cv2.circle(vis, (x, y), 7, (0, 255, 0), -1)
            hr = math.radians(h)
            cv2.line(vis, (x, y),
                     (int(x + 35 * math.cos(hr)), int(y + 35 * math.sin(hr))),
                     (0, 255, 0), 2)
            cv2.putText(vis, "pose {} {} {:.0f}deg".format(x, y, h),
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        else:
            cv2.putText(vis, "NO ROBOT MARKER", (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow("Closed-loop control", vis)
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            self.aborted = True


# ──────────────────────────────────────────────────────────────────────────────
# Entry points / modes.
# ──────────────────────────────────────────────────────────────────────────────
def _open_camera(index):
    cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    # Fixed exposure prevents brightness cycling that drops balls below the
    # HSV threshold on dark frames, causing intermittent missed detections.
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25 = manual in MSMF
    cap.set(cv2.CAP_PROP_EXPOSURE, -5)
    # Keep only the newest frame so reads after a blocking move aren't stale
    # (stale/lagging frames were causing intermittent "lost marker" aborts).
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    return cap


def run_probe(camera_index):
    """Print (and show) the live ArUco pose. No Bluetooth — just verify tracking."""
    detector = BallDetector(load_color_ranges(),
                            roboflow_api_key=ROBOFLOW_API_KEY,
                            roboflow_model_id=ROBOFLOW_MODEL_ID,
                            roboflow_api_url=ROBOFLOW_API_URL)
    cap = _open_camera(camera_index)
    if not cap.isOpened():
        print("ERROR: camera index {} did not open.".format(camera_index))
        return
    print("Probing robot pose. Press 'q' to quit.")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            frame = cv2.flip(frame, 1)
            pose = detector.detect_robot(frame)
            vis = frame.copy()
            if pose is not None:
                x, y, h = pose
                cv2.circle(vis, (x, y), 7, (0, 255, 0), -1)
                hr = math.radians(h)
                cv2.line(vis, (x, y),
                         (int(x + 40 * math.cos(hr)), int(y + 40 * math.sin(hr))),
                         (0, 255, 0), 2)
                cv2.putText(vis, "x={} y={} heading={:.1f}".format(x, y, h),
                            (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(vis, "no marker", (8, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow("ArUco pose probe", vis)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


def run_plan_only(camera_index):
    """Grab one frame, plan from the detected robot pose, and show the path."""
    detector = BallDetector(load_color_ranges(),
                            roboflow_api_key=ROBOFLOW_API_KEY,
                            roboflow_model_id=ROBOFLOW_MODEL_ID,
                            roboflow_api_url=ROBOFLOW_API_URL)
    cap = _open_camera(camera_index)
    if not cap.isOpened():
        print("ERROR: camera index {} did not open.".format(camera_index))
        return
    # Warm up YOLO: the background thread won't make its first API call until
    # YOLO_CALL_INTERVAL seconds have elapsed, so grab frames for that long.
    # ball_confirm_frames=1 so a single YOLO response is enough to confirm a ball.
    detector.ball_confirm_frames = 1
    yolo_mode = getattr(detector, '_roboflow_client', None) is not None
    warmup_s = (detector.YOLO_CALL_INTERVAL + 0.5) if yolo_mode else 0.3
    print("Warming up detector ({:.0f}s)...".format(warmup_s))
    deadline = time.monotonic() + warmup_s
    frame = None
    while time.monotonic() < deadline or (yolo_mode and not detector._yolo_cached_result):
        ret, f = cap.read()
        if ret:
            frame = cv2.flip(f, 1)
            detector.analyze_course(frame)
        if yolo_mode and detector._yolo_cached_result and time.monotonic() > deadline:
            break
        time.sleep(0.05)
    cap.release()
    if frame is None:
        print("ERROR: no frame captured.")
        return

    pose = detector.detect_robot(frame)
    if pose is None:
        print("No robot marker found — cannot plan. Is the marker visible?")
        return
    robot_pos = (pose[0], pose[1])
    waypoints, planner, analysis, dropoff, face_deg = plan_waypoints(
        detector, frame, robot_pos)
    print("Robot at {}, heading {:.1f} deg".format(robot_pos, pose[2]))
    print("Planned {} waypoints, dropoff {}, face {} deg".format(
        len(waypoints), dropoff, face_deg))

    vis = frame.copy()
    for i in range(1, len(waypoints)):
        cv2.line(vis, tuple(map(int, waypoints[i - 1])),
                 tuple(map(int, waypoints[i])), (0, 200, 255), 2)
    cv2.circle(vis, robot_pos, 7, (0, 255, 0), -1)
    cv2.circle(vis, (int(dropoff[0]), int(dropoff[1])), 9, (255, 0, 0), 2)
    cv2.imshow("Planned path (close to exit)", vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def run_live(camera_index, link):
    """The real thing: camera + (Bluetooth or TCP) closed-loop run.

    `link` is a connected-capable link object (BluetoothLink or TCPLink) whose
    .connect()/.send_and_wait()/.close() drive the robot.
    """
    detector = BallDetector(load_color_ranges(),
                            roboflow_api_key=ROBOFLOW_API_KEY,
                            roboflow_model_id=ROBOFLOW_MODEL_ID,
                            roboflow_api_url=ROBOFLOW_API_URL)
    cap = _open_camera(camera_index)
    if not cap.isOpened():
        print("ERROR: camera index {} did not open.".format(camera_index))
        return

    source = CameraPoseSource(cap, detector, flip=True, show=True)

    # Wait for the robot to be visible, then plan from its real pose.
    print("Waiting for the robot marker to plan the route...")
    robot_pose = None
    for _ in range(200):
        robot_pose = source()
        if source.aborted:
            source.stop(); cap.release(); cv2.destroyAllWindows(); return
        if robot_pose is not None:
            break
        time.sleep(0.05)
    if robot_pose is None:
        print("Never saw the robot marker — aborting.")
        source.stop(); cap.release(); cv2.destroyAllWindows(); return

    # Warm up ball detection before planning.
    # With Roboflow backend the worker thread enforces YOLO_CALL_INTERVAL (1 s)
    # before its first API call, so we must wait for the cache to populate.
    # With HoughCircles we just need enough frames to pass ball_confirm_frames.
    detector.ball_confirm_frames = 1  # single-frame is fine for planning
    yolo_mode = getattr(detector, '_roboflow_client', None) is not None
    wait_s = (detector.YOLO_CALL_INTERVAL + 0.5) if yolo_mode else 0.0
    print("Warming up ball detector ({})...".format(
        "YOLO — waiting {:.0f}s for first API result".format(wait_s)
        if yolo_mode else "HoughCircles"))
    deadline = time.monotonic() + max(wait_s, 0.5)
    while time.monotonic() < deadline or (yolo_mode and not detector._yolo_cached_result):
        f = source.grab()
        if f is not None:
            detector.analyze_course(f)
        if source.aborted:
            source.stop(); cap.release(); cv2.destroyAllWindows(); return
        time.sleep(0.05)
        if yolo_mode and detector._yolo_cached_result and time.monotonic() > deadline:
            break

    frame = source.grab()
    robot_pos = (robot_pose[0], robot_pose[1])
    waypoints, planner, analysis, dropoff, face_deg = plan_waypoints(
        detector, frame, robot_pos)
    waypoints = _densify_waypoints(waypoints)
    source.waypoints = waypoints
    print("Robot at {} heading {:.1f}deg".format(robot_pos, robot_pose[2]))
    print("Field detected: {}  bounds: {}".format(
        analysis['field_detected'], analysis['field_bounds']))
    print("Balls found: {}  dropoff: {}".format(
        len(analysis['balls']), dropoff))
    print("Waypoints ({}): {}".format(len(waypoints),
        [(int(x), int(y)) for x, y in waypoints[:10]]))
    if not waypoints:
        print("Planner produced no path — nothing to do.")
        source.stop(); cap.release(); cv2.destroyAllWindows(); return
    print("Planned {} waypoints.".format(len(waypoints)))
    print("px_per_mm: {:.3f}  MAX_STEP_MM: {}  step_px: {:.1f}  ARRIVE_PX: {}".format(
        planner.px_per_mm, MAX_STEP_MM,
        MAX_STEP_MM * planner.px_per_mm, ARRIVE_PX))

    if not link.connect():
        print("ERROR: could not connect/handshake with the EV3 bridge.")
        source.stop(); cap.release(); cv2.destroyAllWindows(); return

    link.send_and_wait("SPEED:300")

    def get_pose():
        if source.aborted:
            return None
        return source()

    def on_step(pose, target, idx, cmd, total_wps):
        source.target = target
        dist = math.hypot(target[0] - pose[0], target[1] - pose[1])
        print("wp {}/{} pose=({},{},{:.0f}) dist={:.0f}px -> {}".format(
            idx + 1, total_wps, pose[0], pose[1], pose[2], dist, cmd))

    # Gate state: track which balls we've opened/closed for and the dropoff.
    gate_state = {
        'ball_pxs': [(b['x'], b['y']) for b in analysis['balls']],
        'dropoff':  dropoff,
        'open':     False,
    }

    def _near_ball(wp):
        bps = gate_state['ball_pxs']
        if not bps:
            return False
        return min(math.hypot(wp[0]-b[0], wp[1]-b[1]) for b in bps) < BALL_GATE_THRESHOLD_PX

    # (no _near_dropoff — we use the exact last waypoint index instead)

    def on_arrive(waypoint, idx, wps):
        if _near_ball(waypoint) and gate_state['open']:
            link.send_and_wait("GATE_CLOSE:{}".format(GATE_CLOSE_DEG))
            gate_state['open'] = False
            print("[GATE] CLOSE — ball retained")
        # Release at the very last waypoint (always the dropoff)
        if idx == len(wps) - 1 and not gate_state['open']:
            link.send_and_wait("GATE_OPEN:{}".format(GATE_OPEN_DEG))
            gate_state['open'] = True
            print("[GATE] OPEN — releasing at dropoff")
        # Pre-open gate when a ball is within the next 4 waypoints (~90 mm at
        # typical camera scale) so the gate is fully deployed before the robot
        # gets close enough to push the ball during the opening sweep.
        if not gate_state['open']:
            for look in range(1, min(5, len(wps) - idx)):
                if _near_ball(wps[idx + look]):
                    link.send_and_wait("GATE_OPEN:{}".format(GATE_OPEN_DEG))
                    gate_state['open'] = True
                    print("[GATE] OPEN — ball {} wp(s) ahead".format(look))
                    break

    def replan():
        f = source.grab()
        p = detector.detect_robot(f)
        if p is None:
            return None
        wp, _, new_analysis, new_dropoff, _ = plan_waypoints(detector, f, (p[0], p[1]))
        wp = _densify_waypoints(wp)
        source.waypoints = wp
        gate_state['ball_pxs'] = [(b['x'], b['y']) for b in new_analysis['balls']]
        gate_state['dropoff'] = new_dropoff
        print("Re-planned: {} waypoints, {} balls.".format(len(wp), len(gate_state['ball_pxs'])))
        return wp

    try:
        ok = follow_path(get_pose, link, waypoints, planner.px_per_mm,
                         face_deg=face_deg, on_step=on_step,
                         on_arrive=on_arrive, replan=replan)
        print("Run complete." if ok else "Run aborted.")
    finally:
        link.send_and_wait("STOP")
        link.close()
        source.stop()
        cap.release()
        cv2.destroyAllWindows()


def run_sim():
    """Validate the control loop with no hardware: a simulated robot + drift.

    Builds a zig-zag path, starts the virtual robot facing the wrong way and
    with turn/forward gain error plus lateral noise, then checks the loop drives
    it to the final waypoint anyway (proving feedback corrects accumulated drift).
    """
    px_per_mm = 0.35
    waypoints = [(100.0, 400.0), (250.0, 250.0), (450.0, 300.0),
                 (520.0, 120.0), (300.0, 100.0)]
    start_pose = (100.0, 400.0, 0.0)   # facing +x (right), but first leg goes up-right

    # Model the measured follow-mode turn response (actual = gain*cmd + coast),
    # deliberately MISMATCHED vs the controller's compensation (1.05 / 8 deg) so
    # the test proves convergence is robust to real-world scatter, not just exact
    # cancellation.
    sim = SimLink(start_pose, px_per_mm,
                  turn_gain=1.12, fwd_gain=0.95, lateral_noise_mm=4.0,
                  turn_overshoot_deg=10.0, seed=1)

    steps = {"n": 0, "turns": 0, "fwd": 0}

    def get_pose():
        return sim.pose()

    def on_step(pose, target, idx, cmd, total_wps):
        steps["n"] += 1
        if cmd.startswith("TURN"):
            steps["turns"] += 1
        elif cmd.startswith("FORWARD"):
            steps["fwd"] += 1

    ok = follow_path(get_pose, sim, list(waypoints), px_per_mm,
                     face_deg=180.0, on_step=on_step)

    fx, fy, fh = sim.pose()
    goal = waypoints[-1]
    err_px = math.hypot(fx - goal[0], fy - goal[1])
    print("sim: completed={} steps={} (turns={} fwd={}) final=({:.1f},{:.1f},{:.0f}deg)".format(
        ok, steps["n"], steps["turns"], steps["fwd"], fx, fy, fh))
    print("sim: distance from final waypoint = {:.1f}px "
          "(arrive tol {:.0f}px) -> {}".format(
              err_px, ARRIVE_PX, "PASS" if err_px < ARRIVE_PX + 5 else "FAIL"))
    print("sim: facing error vs hole(180) = {:.1f} deg".format(
        (180.0 - fh + 180.0) % 360.0 - 180.0))


def main():
    args = sys.argv[1:]
    camera_index = CAMERA_INDEX
    if "--camera" in args:
        camera_index = int(args[args.index("--camera") + 1])

    if "--sim" in args:
        run_sim()
    elif "--probe" in args:
        run_probe(camera_index)
    elif "--plan-only" in args:
        run_plan_only(camera_index)
    else:
        # Transport: --tcp <host>[:port] (WiFi) or default Bluetooth.
        if "--tcp" in args:
            spec = args[args.index("--tcp") + 1]
            host, _, port = spec.partition(":")
            link = TCPLink(host, int(port) if port else 9999)
        else:
            link = BluetoothLink()
        run_live(camera_index, link)


if __name__ == "__main__":
    main()
