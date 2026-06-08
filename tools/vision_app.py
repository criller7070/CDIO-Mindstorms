#!/usr/bin/env python3
"""
Main application loop — the interactive planning menu.
Ties together detection, calibration, path planning, and screenshots.

Run directly:  python vision_app.py
"""
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2
import numpy as np
import glob
import re
import subprocess
import sys

from tools_path_planner import FieldPlanner, FIELD_WIDTH_MM, FIELD_HEIGHT_MM
from vision_config import (
    CAMERA_INDEX, MISSION_FILE, SCREENSHOT_DIR, MASK_DIR,
    WALL_MARGIN, CENTER_RADIUS, INITIAL_HEADING_DEG,
    HOLE_FRAC_X, HOLE_FRAC_Y,
    load_color_ranges,
)
from vision_detector import BallDetector
from vision_calibration import run_calibration
from vision_auto_calibrate import run_auto_calibration


class VisionApp:
    def __init__(self, camera_index=CAMERA_INDEX, mission_file=MISSION_FILE):
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.mission_file      = mission_file
        self.mission_commands  = []

        os.makedirs(SCREENSHOT_DIR, exist_ok=True)
        os.makedirs(MASK_DIR,       exist_ok=True)

        # color_ranges is a shared dict — calibration mutates it in-place so
        # the detector picks up changes without any extra wiring.
        self.color_ranges = load_color_ranges()
        self.detector     = BallDetector(self.color_ranges)

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
        print("  S           : take screenshot")
        print("  M           : toggle mask windows")
        print("  Q           : quit")
        print()

        robot_pos       = None
        locked_ball_pos = None   # if set, robot_pos tracks the nearest white ball
        last_frame      = None
        debug_vis       = None
        masks_visible   = True
        screenshot_num  = self._next_screenshot_number() - 1
        mouse_pos       = [0, 0]
        last_balls      = []     # updated each frame, readable by mouse callback

        def on_mouse(event, x, y, flags, param):
            nonlocal robot_pos, locked_ball_pos
            mouse_pos[0], mouse_pos[1] = x, y
            if event == cv2.EVENT_LBUTTONDOWN:
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
                white_count = orange_count = 0

                for ball in balls:
                    colour = (255, 255, 255) if ball['color'] == "WHITE" else (0, 165, 255)
                    cv2.circle(display, (ball['x'], ball['y']), ball['radius'], colour, 2)
                    cv2.circle(display, (ball['x'], ball['y']), 3, colour, -1)
                    white_count  += ball['color'] == "WHITE"
                    orange_count += ball['color'] == "ORANGE"

                x0, y0, x1, y1 = bounds
                field_detected = analysis['field_detected']
                if field_detected:
                    red_cnts, _ = cv2.findContours(analysis['red_walls']['mask'],
                                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    hull = cv2.convexHull(np.vstack(red_cnts))
                    cv2.drawContours(display, [hull], -1, (0, 0, 200), 2)
                else:
                    cv2.rectangle(display, (x0, y0), (x1, y1), (0, 0, 200), 1)

                # Default to geometric centre; override with the red X if detected.
                cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
                if field_detected:
                    bw, bh = x1 - x0, y1 - y0
                    mx, my = bw * 0.25, bh * 0.25
                    inner = []
                    for cnt in red_cnts:
                        M = cv2.moments(cnt)
                        if M['m00'] == 0:
                            continue
                        ccx, ccy = M['m10'] / M['m00'], M['m01'] / M['m00']
                        if x0 + mx <= ccx <= x1 - mx and y0 + my <= ccy <= y1 - my:
                            inner.append(cnt)
                    if inner:
                        pts = np.vstack(inner)
                        cx = int(np.mean(pts[:, 0, 0]))
                        cy = int(np.mean(pts[:, 0, 1]))
                cv2.circle(display, (cx, cy), CENTER_RADIUS, (0, 0, 200), 1)

                hx = int(x0 + (x1 - x0) * HOLE_FRAC_X)
                hy = int(y0 + (y1 - y0) * HOLE_FRAC_Y)
                cv2.circle(display, (hx, hy), 10, (255, 0, 0), 2)
                cv2.putText(display, "HOLE", (hx + 12, hy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1)

                if robot_pos:
                    cv2.circle(display, robot_pos, 8, (0, 255, 0), -1)
                    cv2.putText(display, "ROBOT", (robot_pos[0] + 10, robot_pos[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

                if not robot_pos:
                    hint = "Click a white ball (or anywhere) to set robot pos"
                elif locked_ball_pos is not None:
                    hint = "LOCKED to ball | SPACE=plan D=debug V=sim S=screenshot Q=quit"
                else:
                    hint = "SPACE=plan D=debug V=sim M=masks S=screenshot Q=quit"
                cv2.putText(display,
                            "White:{} Orange:{} | {}".format(white_count, orange_count, hint),
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

                # Live HSV readout at mouse cursor
                hsv_live = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                fh, fw   = hsv_live.shape[:2]
                mx = max(0, min(mouse_pos[0], fw - 1))
                my = max(0, min(mouse_pos[1], fh - 1))
                h_c, s_c, v_c = hsv_live[my, mx]
                cv2.putText(display,
                            "HSV({},{})= H:{} S:{} V:{}".format(mx, my, h_c, s_c, v_c),
                            (10, fh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.drawMarker(display, (mx, my), (0, 255, 255), cv2.MARKER_CROSS, 14, 1)

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

                elif key == ord(' '):
                    if robot_pos is None:
                        print("Click to set robot position first, then press SPACE.")
                        continue
                    analysis = self.detector.analyze_course(last_frame)
                    print("\nGenerating mission for {} balls...".format(len(analysis['balls'])))
                    self.mission_commands = self._plan_path(analysis, robot_pos)
                    self.save_mission()
                    if hasattr(self, '_last_planner'):
                        debug_vis = self._last_planner.debug_overlay(
                            last_frame,
                            self._last_path_segs,
                            robot_pos,
                            self._last_ball_positions,
                            self._last_dropoff,
                            self._last_ball_order,
                        )
                    print("Mission generated. Ready to run on EV3.")

                elif key == ord('d') and debug_vis is not None:
                    cv2.imshow('Planned Path', debug_vis)

                elif key == ord('v'):
                    sim_path = os.path.join(os.path.dirname(__file__), 'vision_simulator.py')
                    subprocess.Popen([sys.executable, sim_path])
                    print("Simulator launched.")

                elif key == ord('s'):
                    screenshot_num += 1
                    self._save_screenshot(screenshot_num, frame, analysis, display)

        finally:
            self.cleanup()

    # ── Path planning ─────────────────────────────────────────────────────────

    def _plan_path(self, analysis, robot_pos, capacity=6):
        balls = analysis['balls']
        x_min, y_min, x_max, y_max = analysis['field_bounds']
        cx, cy = (x_min + x_max) / 2, (y_min + y_max) / 2

        if robot_pos is None:
            robot_pos = (cx, cy)

        dropoff = (
            int(x_min + (x_max - x_min) * HOLE_FRAC_X),
            int(y_min + (y_max - y_min) * HOLE_FRAC_Y),
        )

        planner = FieldPlanner(
            field_bounds=analysis['field_bounds'],
            center_pos=(int(cx), int(cy)),
            wall_margin=WALL_MARGIN,
            center_radius=CENTER_RADIUS,
            field_width_mm=FIELD_WIDTH_MM,
            field_height_mm=FIELD_HEIGHT_MM,
        )

        ball_positions = [(b['x'], b['y']) for b in balls]
        print("Planning {} ball(s), capacity {}, hole at {}...".format(
            len(balls), capacity, dropoff))

        commands = planner.plan_trips(
            robot_pos=robot_pos,
            ball_positions=ball_positions,
            dropoff_pos=dropoff,
            capacity=capacity,
            initial_heading_deg=INITIAL_HEADING_DEG,
        )

        self._last_planner        = planner
        self._last_ball_positions = ball_positions
        self._last_dropoff        = dropoff
        self._last_path_segs      = getattr(planner, '_debug_path_segs', [])
        self._last_ball_order     = getattr(planner, '_debug_ball_order', [])
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
