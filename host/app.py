#!/usr/bin/env python3
"""
Main application loop — the interactive planning menu.
Ties together detection, calibration, path planning, and screenshots.

Run directly:  python vision_app.py
"""
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import math
import cv2
import numpy as np
import glob
import re
import subprocess
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pathfinding import FieldPlanner, FIELD_WIDTH_MM, FIELD_HEIGHT_MM
from config import (
    CAMERA_INDEX, MISSION_FILE, SCREENSHOT_DIR, MASK_DIR,
    WALL_MARGIN, CENTER_RADIUS, INITIAL_HEADING_DEG,
    HOLE_FRAC_X, HOLE_FRAC_Y,
    ROBOT_WIDTH_MM, ROBOT_LENGTH_MM, ROBOT_PIVOT_OFFSET_MM,
    ROBOFLOW_API_KEY, ROBOFLOW_API_URL, ROBOFLOW_MODEL_ID,
    load_color_ranges,
)
from detection import BallDetector
from calibration import run_calibration
from calibration_auto import run_auto_calibration


class VisionApp:
    def __init__(self, camera_index=CAMERA_INDEX, mission_file=MISSION_FILE):
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        # Fixed exposure prevents the camera from cycling brightness, which
        # would cause balls to flicker below the HSV threshold on dark frames.
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25 = manual in MSMF
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -5)

        self.mission_file      = mission_file
        self.mission_commands  = []

        os.makedirs(SCREENSHOT_DIR, exist_ok=True)
        os.makedirs(MASK_DIR,       exist_ok=True)

        # color_ranges is a shared dict — calibration mutates it in-place so
        # the detector picks up changes without any extra wiring.
        self.color_ranges = load_color_ranges()
        self.detector     = BallDetector(self.color_ranges,
                                         roboflow_api_key=ROBOFLOW_API_KEY,
                                         roboflow_model_id=ROBOFLOW_MODEL_ID,
                                         roboflow_api_url=ROBOFLOW_API_URL)

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        print("Ball Hunting Navigator")
        print("=" * 50)
        print("  LEFT-CLICK  : click a white ball to lock robot to it, or click anywhere to set manually")
        print("  RIGHT-CLICK : print HSV at cursor to console")
        print("  HOVER       : live HSV shown at bottom-left")
        print("  C           : open HSV calibration (trackbars)")
        print("  A           : auto-calibration (click to sample colours)")
        print("  SPACE       : analyse frame and generate mission")
        print("  D           : show planned path overlay")
        print("  V           : open simulator window")
        print("  H           : enter hole-placement mode (left-click to place, H again to reset)")
        print("  S           : take screenshot")
        print("  M           : toggle mask windows")
        print("  Q           : quit")
        print()

        robot_pos       = None
        locked_ball_pos = None   # if set, robot_pos tracks the nearest white ball
        hole_override   = None   # manually placed hole pos; None = use HOLE_FRAC_X/Y
        placing_hole    = False  # when True, next left-click sets the hole
        last_frame      = None
        debug_vis       = None
        masks_visible   = True
        mission_ready   = False
        screenshot_num  = self._next_screenshot_number() - 1
        mouse_pos       = [0, 0]
        last_balls      = []     # updated each frame, readable by mouse callback

        def on_mouse(event, x, y, flags, param):
            nonlocal robot_pos, locked_ball_pos, hole_override, placing_hole
            mouse_pos[0], mouse_pos[1] = x, y
            if event == cv2.EVENT_LBUTTONDOWN:
                if placing_hole:
                    hole_override = (x, y)
                    placing_hole  = False
                    print("Hole placed at ({}, {})".format(x, y))
                    return
                white_balls = [b for b in last_balls if b['color'] == 'WHITE']
                nearest, nearest_dist = None, float('inf')
                for ball in white_balls:
                    d = np.hypot(x - ball['x'], y - ball['y'])
                    if d < nearest_dist:
                        nearest, nearest_dist = ball, d
                if nearest is not None and nearest_dist <= nearest['radius'] + 15:
                    locked_ball_pos = (nearest['x'], nearest['y'])
                    robot_pos = locked_ball_pos
                    print("Robot locked to white ball at ({}, {})".format(*locked_ball_pos))
                else:
                    locked_ball_pos = None
                    robot_pos = (x, y)
                    print("Robot position set to ({}, {})".format(x, y))
            elif event == cv2.EVENT_RBUTTONDOWN and last_frame is not None:
                hsv_f = cv2.cvtColor(last_frame, cv2.COLOR_BGR2HSV)
                fh, fw = hsv_f.shape[:2]
                cx, cy = max(0, min(x, fw - 1)), max(0, min(y, fh - 1))
                h_v, s_v, v_v = hsv_f[cy, cx]
                print("HSV at ({},{}): H={} S={} V={} | BGR={}".format(
                    cx, cy, h_v, s_v, v_v, last_frame[cy, cx]))

        cv2.namedWindow('Ball Detection')
        cv2.setMouseCallback('Ball Detection', on_mouse)

        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to read frame")
                    break

                frame      = cv2.flip(frame, 1)
                last_frame = frame.copy()

                analysis = self.detector.analyze_course(frame)
                balls    = analysis['balls']
                bounds   = analysis['field_bounds']

                last_balls[:] = balls

                # Follow locked white ball each frame
                if locked_ball_pos is not None:
                    wb = [b for b in balls if b['color'] == 'WHITE']
                    nearest, nearest_dist = None, float('inf')
                    for ball in wb:
                        d = np.hypot(locked_ball_pos[0] - ball['x'],
                                     locked_ball_pos[1] - ball['y'])
                        if d < nearest_dist:
                            nearest, nearest_dist = ball, d
                    if nearest is not None and nearest_dist <= 40:
                        locked_ball_pos = (nearest['x'], nearest['y'])
                        robot_pos = locked_ball_pos

                display = frame.copy()
                fh, fw  = display.shape[:2]

                # ── Field boundary ──────────────────────────────────────────
                x0, y0, x1, y1 = bounds
                field_detected = analysis['field_detected']
                if field_detected:
                    red_cnts, _ = cv2.findContours(analysis['red_walls']['mask'],
                                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    hull = cv2.convexHull(np.vstack(red_cnts))
                    cv2.drawContours(display, [hull], -1, (60, 60, 200), 2)
                else:
                    cv2.rectangle(display, (x0, y0), (x1, y1), (80, 80, 160), 1)

                # ── Centre position (from detector) ─────────────────────────
                cx, cy = analysis['center_pos']
                # Radius measured from the X marker; fall back to the constant.
                center_radius = analysis.get('center_radius') or CENTER_RADIUS

                # ── No-go zone (semi-transparent fill) ──────────────────────
                _nogo = display.copy()
                cv2.circle(_nogo, (cx, cy), center_radius, (15, 8, 40), -1)
                cv2.addWeighted(_nogo, 0.45, display, 0.55, 0, display)
                cv2.circle(display, (cx, cy), center_radius, (90, 55, 160), 1)

                # ── Hole ────────────────────────────────────────────────────
                if hole_override:
                    hx, hy = hole_override
                    hole_dot = (180, 80, 200)
                    hole_ring = (220, 130, 255)
                elif hasattr(self, '_last_dropoff') and not hole_override:
                    # After planning: show the snapped navigable position
                    hx, hy = self._last_dropoff
                    hole_dot = (170, 60, 60)
                    hole_ring = (220, 100, 100)
                else:
                    hx = x0 if field_detected else int(x0 + (x1 - x0) * HOLE_FRAC_X)
                    hy = (y0 + y1) // 2
                    hole_dot = (170, 60, 60)
                    hole_ring = (220, 100, 100)
                cv2.circle(display, (hx, hy), 10, hole_dot, -1)
                cv2.circle(display, (hx, hy), 11, hole_ring, 1)
                hole_label = "HOLE*" if hole_override else "HOLE"
                cv2.putText(display, hole_label, (hx + 14, hy + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, hole_ring, 1)

                # ── Balls ───────────────────────────────────────────────────
                white_balls  = [b for b in balls if b['color'] == 'WHITE']
                orange_balls = [b for b in balls if b['color'] == 'ORANGE']
                white_count  = len(white_balls)
                orange_count = len(orange_balls)

                for rank, ball in enumerate(white_balls, 1):
                    cv2.circle(display, (ball['x'], ball['y']), ball['radius'], (220, 220, 220), 2)
                    cv2.circle(display, (ball['x'], ball['y']), 3, (255, 255, 255), -1)
                    cv2.putText(display, str(rank),
                                (ball['x'] + ball['radius'] + 3, ball['y'] + 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)

                for rank, ball in enumerate(orange_balls, 1):
                    cv2.circle(display, (ball['x'], ball['y']), ball['radius'], (0, 150, 255), 2)
                    cv2.circle(display, (ball['x'], ball['y']), 3, (0, 165, 255), -1)
                    cv2.putText(display, str(rank),
                                (ball['x'] + ball['radius'] + 3, ball['y'] + 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 165, 255), 1)

                # ── Robot marker (oriented box) ─────────────────────────────
                if robot_pos:
                    _px_per_mm = ((x1 - x0) / FIELD_WIDTH_MM + (y1 - y0) / FIELD_HEIGHT_MM) / 2.0
                    _half_l = ROBOT_LENGTH_MM * _px_per_mm / 2.0
                    _half_w = ROBOT_WIDTH_MM  * _px_per_mm / 2.0
                    _rad = math.radians(INITIAL_HEADING_DEG)
                    _fwd = (math.cos(_rad), math.sin(_rad))
                    _rgt = (-math.sin(_rad), math.cos(_rad))
                    _rx, _ry = robot_pos
                    _corners = np.array([
                        [_rx + _fwd[0]*_half_l - _rgt[0]*_half_w, _ry + _fwd[1]*_half_l - _rgt[1]*_half_w],
                        [_rx + _fwd[0]*_half_l + _rgt[0]*_half_w, _ry + _fwd[1]*_half_l + _rgt[1]*_half_w],
                        [_rx - _fwd[0]*_half_l + _rgt[0]*_half_w, _ry - _fwd[1]*_half_l + _rgt[1]*_half_w],
                        [_rx - _fwd[0]*_half_l - _rgt[0]*_half_w, _ry - _fwd[1]*_half_l - _rgt[1]*_half_w],
                    ], dtype=np.int32)
                    _body = display.copy()
                    cv2.fillPoly(_body, [_corners], (0, 55, 0))
                    cv2.addWeighted(_body, 0.45, display, 0.55, 0, display)
                    cv2.drawContours(display, [_corners], 0, (0, 220, 0), 2)
                    # Front face highlighted in bright green
                    cv2.line(display, tuple(_corners[0]), tuple(_corners[1]), (80, 255, 80), 3)
                    if locked_ball_pos is not None:
                        cv2.drawContours(display, [_corners], 0, (0, 215, 255), 1)
                    rlabel = "LOCKED" if locked_ball_pos is not None else "ROBOT"
                    cv2.putText(display, rlabel, (_rx + int(_half_w) + 4, _ry + 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 0), 1)

                # ── Top HUD bar ─────────────────────────────────────────────
                _hud = display.copy()
                cv2.rectangle(_hud, (0, 0), (fw, 38), (12, 12, 12), -1)
                cv2.addWeighted(_hud, 0.72, display, 0.28, 0, display)

                cv2.circle(display, (12, 13), 6, (220, 220, 220), -1)
                cv2.putText(display, "x{}".format(white_count),  (22, 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1)
                cv2.circle(display, (58, 13), 6, (0, 150, 255), -1)
                cv2.putText(display, "x{}".format(orange_count), (68, 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 150, 255), 1)

                if robot_pos is None:
                    b_col, b_txt = (25, 25, 120), "NO ROBOT"
                elif mission_ready:
                    b_col, b_txt = (10,  90,  10), "PLANNED "
                else:
                    b_col, b_txt = (10,  75, 100), "READY   "
                bx = fw // 2 - 44
                cv2.rectangle(display, (bx, 4), (bx + 88, 30), b_col, -1)
                cv2.putText(display, b_txt, (bx + 6, 23),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (210, 210, 210), 1)

                fc = (10, 90, 10) if field_detected else (90, 55, 10)
                ft = "FIELD OK" if field_detected else "NO FIELD"
                cv2.rectangle(display, (fw - 86, 4), (fw - 4, 30), fc, -1)
                cv2.putText(display, ft, (fw - 82, 23),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1)

                # ── Bottom bar (HSV + hints) ─────────────────────────────────
                _bot = display.copy()
                cv2.rectangle(_bot, (0, fh - 28), (fw, fh), (12, 12, 12), -1)
                cv2.addWeighted(_bot, 0.72, display, 0.28, 0, display)

                hsv_live = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mxc = max(0, min(mouse_pos[0], fw - 1))
                myc = max(0, min(mouse_pos[1], fh - 1))
                h_c, s_c, v_c = hsv_live[myc, mxc]
                if placing_hole:
                    bot_txt = "Click to place HOLE  |  H = cancel   HSV({},{}) H:{} S:{} V:{}".format(
                        mxc, myc, h_c, s_c, v_c)
                elif robot_pos is None:
                    bot_txt = "Left-click to set robot pos  |  H = place hole   HSV({},{}) H:{} S:{} V:{}".format(
                        mxc, myc, h_c, s_c, v_c)
                else:
                    bot_txt = ("SPACE=plan  D=path  V=sim  H=hole  C=cal  A=auto  M=masks  S=shot  Q=quit"
                               "   HSV H:{} S:{} V:{}".format(h_c, s_c, v_c))
                cv2.putText(display, bot_txt,
                            (6, fh - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (155, 155, 155), 1)
                cv2.drawMarker(display, (mxc, myc), (0, 255, 255), cv2.MARKER_CROSS, 14, 1)

                cv2.imshow('Ball Detection', display)

                if masks_visible:
                    cv2.imshow('WHITE mask (raw HSV)',  analysis['white_mask'])
                    cv2.imshow('ORANGE mask (raw HSV)', analysis['orange_mask'])
                    cv2.imshow('RED walls (raw HSV)',   analysis['red_walls']['mask'])

                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    break

                elif key == ord('m'):
                    masks_visible = not masks_visible
                    if not masks_visible:
                        for w in ('WHITE mask (raw HSV)', 'ORANGE mask (raw HSV)',
                                  'RED walls (raw HSV)'):
                            cv2.destroyWindow(w)

                elif key == ord('c'):
                    print("\nEntering calibration mode...")
                    self.detector._ball_tracks = {"WHITE": [], "ORANGE": []}
                    run_calibration(self.cap, self.color_ranges)
                    print("Returned to planning mode.")

                elif key == ord('a'):
                    print("\nEntering auto-calibration mode...")
                    self.detector._ball_tracks = {"WHITE": [], "ORANGE": []}
                    run_auto_calibration(self.cap, self.color_ranges)
                    print("Returned to planning mode.")

                elif key == ord('h'):
                    if hole_override is not None:
                        hole_override = None
                        placing_hole  = False
                        print("Hole reset to auto position.")
                    else:
                        placing_hole = not placing_hole
                        print("Hole placement mode {}.".format("ON - click to place" if placing_hole else "OFF"))

                elif key == ord(' '):
                    if robot_pos is None:
                        print("Click to set robot position first, then press SPACE.")
                        continue
                    analysis = self.detector.analyze_course(last_frame)
                    print("\nGenerating mission for {} balls...".format(len(analysis['balls'])))
                    self.mission_commands = self._plan_path(analysis, robot_pos, hole_override=hole_override)
                    self.save_mission()
                    mission_ready = True
                    if hasattr(self, '_last_planner'):
                        debug_vis = self._last_planner.debug_overlay(
                            last_frame,
                            self._last_path_segs,
                            robot_pos,
                            self._last_ball_positions,
                            self._last_dropoff,
                            self._last_ball_order,
                            skipped=getattr(self, '_last_skipped', None),
                        )
                    print("Mission generated. Ready to run on EV3.")

                elif key == ord('d') and debug_vis is not None:
                    cv2.imshow('Planned Path', debug_vis)

                elif key == ord('v'):
                    sim_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'tools', 'vision_simulator.py')
                    subprocess.Popen([sys.executable, sim_path])
                    print("Simulator launched.")

                elif key == ord('s'):
                    screenshot_num += 1
                    self._save_screenshot(screenshot_num, frame, analysis, display)

        finally:
            self.cleanup()

    # ── Path planning ─────────────────────────────────────────────────────────

    def _plan_path(self, analysis, robot_pos, capacity=6, hole_override=None):
        balls = analysis['balls']
        x_min, y_min, x_max, y_max = analysis['field_bounds']
        center_pos = analysis['center_pos']

        if robot_pos is None:
            robot_pos = center_pos

        wall_margin = analysis['wall_margin'] or WALL_MARGIN
        center_radius = analysis.get('center_radius') or CENTER_RADIUS

        # Compute intended hole position (may still be in obstacle zone)
        if hole_override is not None:
            raw_dropoff = hole_override
            dx = min(hole_override[0] - x_min, x_max - hole_override[0])
            dy = min(hole_override[1] - y_min, y_max - hole_override[1])
            face_deg = 180.0 if dx <= dy else (270.0 if hole_override[1] < (y_min + y_max) / 2 else 90.0)
        elif analysis['field_detected']:
            raw_dropoff = (x_min, (y_min + y_max) // 2)
            face_deg = 180.0  # hole is on the left wall
        else:
            raw_dropoff = (
                int(x_min + (x_max - x_min) * HOLE_FRAC_X),
                int(y_min + (y_max - y_min) * HOLE_FRAC_Y),
            )
            face_deg = 180.0 if HOLE_FRAC_X < 0.5 else 0.0

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

        # Snap to nearest navigable cell so display and path endpoint match exactly
        dropoff = planner.snap_to_navigable(*raw_dropoff)

        ball_positions = [(b['x'], b['y']) for b in balls]
        print("Planning {} ball(s), capacity {}, hole at {}...".format(
            len(balls), capacity, dropoff))
        print("  center={} wall_margin={}{}".format(
            center_pos, wall_margin,
            " (detected)" if analysis['wall_margin'] else " (config fallback)"))

        commands = planner.plan_trips(
            robot_pos=robot_pos,
            ball_positions=ball_positions,
            dropoff_pos=dropoff,
            capacity=capacity,
            initial_heading_deg=INITIAL_HEADING_DEG,
            face_deg=face_deg,
        )

        self._last_planner        = planner
        # Use the planner's REACHABLE list (what _debug_ball_order indexes into),
        # not the full list — otherwise skipped balls shift the labels.
        self._last_ball_positions = getattr(planner, '_debug_ball_positions', ball_positions)
        self._last_dropoff        = dropoff
        self._last_path_segs      = getattr(planner, '_debug_path_segs', [])
        self._last_ball_order     = getattr(planner, '_debug_ball_order', [])
        self._last_skipped        = getattr(planner, '_debug_skipped_balls', [])
        return commands

    # ── Persistence ───────────────────────────────────────────────────────────

    def save_mission(self):
        try:
            with open(self.mission_file, "w") as f:
                for cmd in self.mission_commands:
                    f.write(cmd + "\n")
            print("Mission saved to: {}".format(self.mission_file))
            for i, cmd in enumerate(self.mission_commands, 1):
                if not cmd.startswith("#"):
                    print("  {}: {}".format(i, cmd))
        except Exception as e:
            print("Error saving mission: {}".format(e))

    def _save_screenshot(self, number, frame, analysis, display_frame):
        folder = os.path.join(SCREENSHOT_DIR, "mask_{:03d}".format(number))
        os.makedirs(folder, exist_ok=True)

        wmask = analysis['white_mask'].astype(np.uint8)
        omask = analysis['orange_mask'].astype(np.uint8)
        rmask = analysis['red_walls']['mask'].astype(np.uint8)

        overlay = frame.copy()
        overlay[wmask > 0] = (255, 255, 255)
        overlay[omask > 0] = (0, 140, 255)
        overlay[rmask > 0] = (0, 0, 255)
        composite = cv2.addWeighted(overlay, 0.45, frame, 0.55, 0)

        base = "screenshot_{:03d}".format(number)
        cv2.imwrite(os.path.join(folder, base + "_raw.png"),        frame)
        cv2.imwrite(os.path.join(folder, base + ".png"),            composite)
        cv2.imwrite(os.path.join(folder, base + "_annotated.png"),  display_frame)
        cv2.imwrite(os.path.join(folder, "white_mask.png"),  wmask)
        cv2.imwrite(os.path.join(folder, "orange_mask.png"), omask)
        cv2.imwrite(os.path.join(folder, "red_mask.png"),    rmask)
        print("Screenshot saved: {}".format(folder))

    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()
        print("\nShutdown complete")

    def _next_screenshot_number(self):
        pattern  = os.path.join(SCREENSHOT_DIR, "mask_*", "screenshot_*.png")
        existing = glob.glob(pattern)
        if not existing:
            return 1
        numbers = [
            int(m.group(1))
            for f in existing
            for m in [re.search(r'screenshot_(\d+)\.png', os.path.basename(f))]
            if m
        ]
        return max(numbers) + 1 if numbers else 1


if __name__ == "__main__":
    VisionApp().run()
