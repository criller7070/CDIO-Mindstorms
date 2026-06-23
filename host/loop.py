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
    python loop.py                 # live: camera + TCP
    python loop.py --probe         # just print live ArUco pose
    python loop.py --plan-only     # plan from one frame, show path
    python loop.py --sim           # validate the loop, no hardware
    python loop.py --camera 0      # override camera index

Requires an ArUco marker on the robot — see tools/tools_generate_aruco_marker.py.
"""
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import sys
import math
import time
import threading

import cv2
import numpy as np

from pathfinding import FieldPlanner, FIELD_WIDTH_MM, FIELD_HEIGHT_MM
from config import (
    CAMERA_INDEX, WALL_MARGIN, CENTER_RADIUS, INITIAL_HEADING_DEG,
    HOLE_FRAC_X, HOLE_FRAC_Y,
    ROBOT_WIDTH_MM, ROBOT_LENGTH_MM, ROBOT_PIVOT_OFFSET_MM, GATE_ARM_MM,
    ARUCO_FROM_BACK_FRAC,
    load_color_ranges,
    ARRIVE_PX, MAX_STEP_MM, FORWARD_CMD_SCALE, DENSIFY_GAP_PX,
    BALL_GATE_THRESHOLD_PX, GATE_OPEN_DEG, GATE_CLOSE_DEG, GATE_DROPOFF_DEG,
    CENTER_OBSTACLE_EXTRA_PX,
    ROBOFLOW_API_KEY, ROBOFLOW_API_URL, ROBOFLOW_MODEL_ID,
)
from detection import BallDetector
from link import BluetoothLink, TCPLink, SimLink
from nav import follow_path


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
    # minEnclosingCircle on X arms measures tip-to-tip, so cap to avoid over-blocking.
    detected_r = analysis.get('center_radius') or CENTER_RADIUS
    center_radius = min(detected_r, CENTER_RADIUS) + CENTER_OBSTACLE_EXTRA_PX
    print("X obstacle: raw_r={}px  capped_r={}px  base_r={}px".format(
        detected_r, min(detected_r, CENTER_RADIUS), center_radius))

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
        gate_open=True,
        gate_arm_mm=GATE_ARM_MM,
        aruco_from_back_frac=ARUCO_FROM_BACK_FRAC,
    )
    print("Footprint: half_w={:.0f}px  half_l={:.0f}px  eff_w={:.0f}px  center_clr={:.0f}px".format(
        planner.robot_half_width_px, planner.robot_half_length_px,
        planner.effective_half_width_px, planner.center_clearance_px))
    dropoff = planner.snap_to_navigable(*raw_dropoff)
    ball_positions = [(b['x'], b['y']) for b in analysis['balls']]

    planner.plan_trips(
        robot_pos=robot_pos,
        ball_positions=ball_positions,
        dropoff_pos=dropoff,
        capacity=8,
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


def _add_dropoff_approach(waypoints, dropoff, offset_px=100):
    """Insert a perpendicular-approach waypoint before the dropoff.

    The dropoff is on the left wall so the robot must face west (180°).
    Inserting a waypoint offset_px to the right of the dropoff forces the
    robot to align on the west-bound line BEFORE the final approach,
    so heading is already correct at the wall rather than corrected there.
    """
    if len(waypoints) < 2:
        return waypoints
    dx, dy = dropoff
    approach = (dx + offset_px, dy)
    result = list(waypoints)
    result.insert(len(result) - 1, approach)
    return result


def _densify_waypoints(waypoints, max_gap_px=DENSIFY_GAP_PX):
    """Insert intermediate checkpoints so no consecutive pair is more than
    max_gap_px apart.

    Each checkpoint forces a pose measurement and heading correction, so
    lateral drift can't accumulate over a long single forward step.
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
# Live camera pose source.
# ──────────────────────────────────────────────────────────────────────────────
class CameraPoseSource:
    """Continuously grabs frames on a background thread so the control loop
    always gets a fresh pose — even after a long blocking send_and_wait().

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
        self.ball_pxs = []   # [(x,y), ...] updated from gate_state during run
        self.footprint = {}  # set after planning: half_w, half_l, eff_w, center_clr (px)
        self._lock = threading.Lock()
        self._latest_frame = None
        self._latest_pose = None
        self._marker_perimeter = 0.0
        self._fps = 0.0
        self._frame_count = 0
        self._fps_t0 = time.time()
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
            perim = getattr(self.detector, '_last_marker_perimeter', 0.0)
            self._frame_count += 1
            now = time.time()
            dt = now - self._fps_t0
            if dt >= 1.0:
                self._fps = self._frame_count / dt
                self._frame_count = 0
                self._fps_t0 = now
            with self._lock:
                self._latest_frame = frame
                self._latest_pose = pose
                self._marker_perimeter = perim
                waypoints = self.waypoints
                target = self.target
                ball_pxs = list(self.ball_pxs)
            if self.show:
                self._render(frame, pose, waypoints, target, ball_pxs)

    def grab(self):
        with self._lock:
            return self._latest_frame

    def stop(self):
        self._running = False
        self._thread.join(timeout=2.0)

    def __call__(self):
        with self._lock:
            return self._latest_pose

    @property
    def marker_perimeter(self):
        with self._lock:
            return self._marker_perimeter

    @property
    def fps(self):
        return self._fps

    def _render(self, frame, pose, waypoints, target, ball_pxs=None):
        vis = frame.copy()
        if waypoints:
            for i in range(1, len(waypoints)):
                p0 = tuple(map(int, waypoints[i - 1]))
                p1 = tuple(map(int, waypoints[i]))
                cv2.line(vis, p0, p1, (80, 80, 80), 1)
        # Draw detected balls with pickup-order numbers.
        if ball_pxs:
            for n, (bx, by) in enumerate(ball_pxs, 1):
                bpt = (int(bx), int(by))
                cv2.circle(vis, bpt, 8, (0, 200, 255), 2)
                cv2.putText(vis, str(n), (bpt[0] + 10, bpt[1] - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)
        if target is not None:
            t = tuple(map(int, target))
            cv2.circle(vis, t, int(ARRIVE_PX), (255, 0, 255), 1)
            cv2.drawMarker(vis, t, (255, 0, 255), cv2.MARKER_TILTED_CROSS, 12, 2)
        perim = getattr(self.detector, '_last_marker_perimeter', 0.0)
        fps   = self._fps
        fp    = self.footprint
        if pose is not None:
            x, y, h = pose
            hr = math.radians(h)
            fwd  = (math.cos(hr), math.sin(hr))
            perp = (-math.sin(hr), math.cos(hr))
            if fp:
                hl = fp.get('half_l', 0)
                hw = fp.get('half_w', 0)
                ew = fp.get('eff_w', 0)
                # ArUco is at ARUCO_FROM_BACK_FRAC along the body (0=back, 1=front).
                # Geometric centre is at 50%, so it sits (50%-frac)*length ahead of ArUco.
                aruco_to_center = (0.5 - ARUCO_FROM_BACK_FRAC) * hl * 2
                cx = x + fwd[0] * aruco_to_center
                cy = y + fwd[1] * aruco_to_center
                # Robot body box (cyan)
                body_corners = np.array([
                    [cx + fwd[0]*hl + perp[0]*hw,  cy + fwd[1]*hl + perp[1]*hw],
                    [cx + fwd[0]*hl - perp[0]*hw,  cy + fwd[1]*hl - perp[1]*hw],
                    [cx - fwd[0]*hl - perp[0]*hw,  cy - fwd[1]*hl - perp[1]*hw],
                    [cx - fwd[0]*hl + perp[0]*hw,  cy - fwd[1]*hl + perp[1]*hw],
                ], dtype=np.int32)
                cv2.polylines(vis, [body_corners], True, (0, 220, 220), 1)
                # Effective footprint box incl. open gate arms (blue-white)
                if ew > hw:
                    eff_corners = np.array([
                        [cx + fwd[0]*hl + perp[0]*ew,  cy + fwd[1]*hl + perp[1]*ew],
                        [cx + fwd[0]*hl - perp[0]*ew,  cy + fwd[1]*hl - perp[1]*ew],
                        [cx - fwd[0]*hl - perp[0]*ew,  cy - fwd[1]*hl - perp[1]*ew],
                        [cx - fwd[0]*hl + perp[0]*ew,  cy - fwd[1]*hl + perp[1]*ew],
                    ], dtype=np.int32)
                    cv2.polylines(vis, [eff_corners], True, (255, 200, 0), 1)
            # Centre dot + heading arrow
            cv2.circle(vis, (x, y), 4, (0, 255, 0), -1)
            cv2.line(vis, (x, y),
                     (int(x + 35 * fwd[0]), int(y + 35 * fwd[1])),
                     (0, 255, 0), 2)
            cv2.putText(vis, "pose ({},{}) {:.0f}deg  marker={:.0f}px  {:.0f}fps".format(
                        x, y, h, perim, fps),
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        else:
            cv2.putText(vis, "NO ROBOT MARKER  {:.0f}fps".format(fps), (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        if fp:
            cv2.putText(vis,
                        "body w={:.0f}px l={:.0f}px  eff_w={:.0f}px  cross_clr={:.0f}px".format(
                            fp.get('half_w', 0) * 2, fp.get('half_l', 0) * 2,
                            fp.get('eff_w', 0) * 2, fp.get('center_clr', 0)),
                        (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 0), 1)
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
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    return cap


def run_probe(camera_index):
    """Print (and show) the live ArUco pose. No robot connection — just verify tracking."""
    detector = BallDetector(load_color_ranges())
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
    """The real thing: camera + (Bluetooth or TCP) closed-loop run."""
    detector = BallDetector(load_color_ranges(),
                            roboflow_api_key=ROBOFLOW_API_KEY,
                            roboflow_model_id=ROBOFLOW_MODEL_ID,
                            roboflow_api_url=ROBOFLOW_API_URL)
    cap = _open_camera(camera_index)
    if not cap.isOpened():
        print("ERROR: camera index {} did not open.".format(camera_index))
        return

    source = CameraPoseSource(cap, detector, flip=True, show=True)

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
    detector.ball_confirm_frames = 1
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
    source.footprint = {
        'half_w': planner.robot_half_width_px,
        'half_l': planner.robot_half_length_px,
        'eff_w':  planner.effective_half_width_px,
        'center_clr': planner.center_clearance_px,
    }
    waypoints = _densify_waypoints(waypoints)
    waypoints = _add_dropoff_approach(waypoints, dropoff)
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

    _step_count = [0]
    def on_step(pose, target, idx, cmd, total_wps):
        source.target = target
        dist = math.hypot(target[0] - pose[0], target[1] - pose[1])
        marker_px = source.marker_perimeter
        gate_str = "OPEN" if gate_state['open'] else "closed"
        print("wp {}/{} pose=({},{},{:.0f}) dist={:.0f}px marker={:.0f}px gate={} fps={:.0f} -> {}".format(
            idx + 1, total_wps, pose[0], pose[1], pose[2], dist,
            marker_px, gate_str, source.fps, cmd))
        _step_count[0] += 1
        if _step_count[0] % 10 == 1:
            balls = gate_state['ball_pxs']
            ball_strs = ["({},{})".format(int(b[0]), int(b[1])) for b in balls]
            print("[FIELD] {} ball(s): {}  dropoff: {}".format(
                len(balls), " ".join(ball_strs), dropoff))

    gate_state = {
        'ball_pxs':   [(b['x'], b['y']) for b in analysis['balls']],
        'dropoff':    dropoff,
        'open':       False,
        'collected':  [],   # positions of balls already collected — filter from replans
    }
    source.ball_pxs = gate_state['ball_pxs']  # shared reference — updates live in renderer

    def _near_ball(wp):
        bps = gate_state['ball_pxs']
        if not bps:
            return False
        return min(math.hypot(wp[0]-b[0], wp[1]-b[1]) for b in bps) < BALL_GATE_THRESHOLD_PX

    def on_arrive(waypoint, idx, wps):
        # At a ball waypoint: open if still closed (sweep in ball), then close to retain.
        if _near_ball(waypoint):
            if not gate_state['open']:
                link.send_and_wait("GATE_OPEN:{}".format(GATE_OPEN_DEG))
                gate_state['open'] = True
            link.send_and_wait("GATE_CLOSE:{}".format(GATE_CLOSE_DEG))
            gate_state['open'] = False
            # Remove this ball from the active list so the same ball doesn't
            # re-trigger gate operations at the next nearby waypoint.
            nearest_idx = min(range(len(gate_state['ball_pxs'])),
                              key=lambda i: math.hypot(
                                  waypoint[0] - gate_state['ball_pxs'][i][0],
                                  waypoint[1] - gate_state['ball_pxs'][i][1]))
            collected_pos = gate_state['ball_pxs'].pop(nearest_idx)
            gate_state['collected'].append(collected_pos)
            print("[GATE] COLLECT — ball retained, {} collected so far".format(
                len(gate_state['collected'])))

        # Dropoff: partial-open (45°) to release — wide enough to let balls roll
        # out but narrow enough not to jam against the wall.
        if idx == len(wps) - 1 and not gate_state['open']:
            link.send_and_wait("GATE_OPEN:{}".format(GATE_DROPOFF_DEG))
            gate_state['open'] = True
            print("[GATE] OPEN {}° — releasing at dropoff".format(GATE_DROPOFF_DEG))

        # Pre-open gate when a ball is 3-5 waypoints away (~60-100 mm).
        # Minimum of 3 so the gate is fully deployed before reaching the ball
        # and doesn't stay open during long transit segments between balls.
        if not gate_state['open']:
            for look in range(3, min(6, len(wps) - idx)):
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
        wp, new_planner, new_analysis, new_dropoff, _ = plan_waypoints(detector, f, (p[0], p[1]))
        source.footprint = {
            'half_w': new_planner.robot_half_width_px,
            'half_l': new_planner.robot_half_length_px,
            'eff_w':  new_planner.effective_half_width_px,
            'center_clr': new_planner.center_clearance_px,
        }
        wp = _densify_waypoints(wp)
        wp = _add_dropoff_approach(wp, new_dropoff)
        source.waypoints = wp
        COLLECTED_FILTER_PX = 40
        all_balls = [(b['x'], b['y']) for b in new_analysis['balls']]
        filtered = [b for b in all_balls if not any(
            math.hypot(b[0]-c[0], b[1]-c[1]) < COLLECTED_FILTER_PX
            for c in gate_state['collected'])]
        if len(filtered) < len(all_balls):
            print("[REPLAN] filtered {} ghost ball(s) at collected positions".format(
                len(all_balls) - len(filtered)))
        gate_state['ball_pxs'] = filtered
        source.ball_pxs = gate_state['ball_pxs']
        gate_state['dropoff'] = new_dropoff
        print("Re-planned: {} waypoints, {} balls ({} collected).".format(
            len(wp), len(gate_state['ball_pxs']), len(gate_state['collected'])))
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
    """Validate the control loop with no hardware: a simulated robot + drift."""
    px_per_mm = 0.35
    waypoints = [(100.0, 400.0), (250.0, 250.0), (450.0, 300.0),
                 (520.0, 120.0), (300.0, 100.0)]

    sim = SimLink((100.0, 400.0, 0.0), px_per_mm,
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


def _restart_robot(host, ssh_user="robot"):
    """Kill stale pybricks/bridge, reset IPC, restart both, wait for ready.

    Called automatically when using --profile or --tcp so the user only
    needs to run loop.py once — no manual SSH cleanup required.
    """
    import subprocess

    target = "{}@{}".format(ssh_user, host)
    ssh_base = ["ssh", "-T", "-o", "StrictHostKeyChecking=no",
                "-o", "ConnectTimeout=10", "-o", "BatchMode=yes"]

    def _run_capture(cmd, timeout=10):
        """Run SSH, capture stdout, return string. Non-blocking kill on timeout."""
        try:
            proc = subprocess.Popen(
                ssh_base + [target, cmd],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            return ""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                out, _ = proc.communicate()
                return out.decode(errors="replace")
            time.sleep(0.3)
        proc.kill()
        return ""

    print("--- Robot restart ({}) ---".format(host))

    # Fire restart.sh and don't block waiting for SSH to return.
    # Windows SSH.exe can hold open after backgrounding processes on the robot;
    # avoiding wait entirely sidesteps the issue.  The script takes ~4s to
    # finish; we then poll main.log directly.
    try:
        subprocess.Popen(
            ssh_base + [target, "bash /home/robot/restart.sh"],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception as e:
        print("SSH failed: {}".format(e))
        return False
    print("Restart triggered, waiting for EV3 to boot...")
    time.sleep(8)  # restart.sh takes ~4s; give brickrun time to start writing

    # Poll main.log until "Follow loop ready", printing new lines as they arrive.
    print("Waiting for EV3...")
    deadline = time.monotonic() + 55
    last_len = 0
    while time.monotonic() < deadline:
        time.sleep(2)
        log = _run_capture("cat /tmp/main.log")
        if len(log) > last_len:
            for line in log[last_len:].splitlines():
                print("  [EV3] " + line)
                if "[DEBUG]" in line and "not found" in line:
                    print("  *** SENSOR/MOTOR MISSING — replug cable and restart ***")
            last_len = len(log)
        if "Follow loop ready" in log:
            print("--- EV3 ready ---")
            return True
        if "[FATAL ERROR]" in log:
            print("EV3 FATAL ERROR — cannot proceed.")
            return False
    print("EV3 did not reach ready within 55 s.")
    return False


def _load_profile(name: str) -> str:
    """Return the IP for a named profile from .env."""
    profiles_path = os.path.join(os.path.dirname(__file__), "..", ".env")
    try:
        with open(profiles_path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                key, _, val = line.partition("=")
                if key.strip() == name:
                    return val.strip()
    except FileNotFoundError:
        raise SystemExit(f".env not found at {profiles_path}")
    known = []
    with open(profiles_path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                known.append(line.split("=")[0].strip())
    raise SystemExit(f"Profile '{name}' not found. Available: {', '.join(known)}")


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
        no_restart = "--no-restart" in args
        if "--profile" in args:
            ip = _load_profile(args[args.index("--profile") + 1])
            if not no_restart and not _restart_robot(ip):
                return
            link = TCPLink(ip, 9999, timeout=60.0)
        elif "--tcp" in args:
            spec = args[args.index("--tcp") + 1]
            host, _, port = spec.partition(":")
            if not no_restart and not _restart_robot(host):
                return
            link = TCPLink(host, int(port) if port else 9999, timeout=60.0)
        else:
            link = BluetoothLink()
        run_live(camera_index, link)


if __name__ == "__main__":
    main()
