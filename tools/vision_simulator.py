#!/usr/bin/env python3
"""
Simulator — replay commands.txt on a 2-D field canvas.

Run directly:  python vision_simulator.py

Keys:
  SPACE        : step one command forward
  BACKSPACE    : step one command back
  A            : toggle animation playback
  E            : jump to end (show full path)
  0 / Home     : jump to start
  + / -        : increase / decrease animation speed
  R            : reload commands.txt and reset
  Q            : quit
Left-click inside the field to set the robot start position.
"""
import os
import math
import time
import cv2
import numpy as np

from vision_config import (
    MISSION_FILE,
    INITIAL_HEADING_DEG,
    HOLE_FRAC_X, HOLE_FRAC_Y,
)
from tools_path_planner import FIELD_WIDTH_MM, FIELD_HEIGHT_MM

# ── Display layout ────────────────────────────────────────────────────────────
CANVAS_W   = 1050
CANVAS_H   = 700
FIELD_PAD  = 40
SIDE_W     = 280

# Approximate wall margin and center obstacle in mm
# (camera pixel values from vision_config converted at ~0.30 px/mm)
WALL_MARGIN_MM   = 150
CENTER_RADIUS_MM = 200

# ── Colours (BGR) ─────────────────────────────────────────────────────────────
C_BG      = (30,  30,  30)
C_FIELD   = (55,  75,  45)
C_BORDER  = (40,  40, 180)
C_MARGIN  = (65,  65, 110)
C_OBST    = (55,  35,  35)
C_OBST_E  = (90,  55,  55)
C_PATH    = (0,  210, 210)
C_DIM     = (80,  80,  80)
C_ROBOT   = (0,  255,   0)
C_COLLECT = (0,  140, 255)
C_DEPOSIT = (50,  50, 255)
C_HOLE    = (60,  60, 255)
C_TEXT    = (210, 210, 210)
C_HI      = (0,  255, 255)
C_COMMENT = (110, 110,  55)


# ── Command parsing ───────────────────────────────────────────────────────────

def parse_commands(path):
    """Return list of (cmd_str, numeric_value_or_None) for every line."""
    result = []
    try:
        with open(path) as f:
            for raw in f:
                line = raw.strip()
                if not line:
                    continue
                if line.startswith('#'):
                    result.append(('COMMENT', line))
                    continue
                if ':' in line:
                    k, v = line.split(':', 1)
                    try:
                        result.append((k.strip(), float(v.strip())))
                    except ValueError:
                        result.append((k.strip(), v.strip()))
                else:
                    result.append((line, None))
    except FileNotFoundError:
        print("commands.txt not found: {}".format(path))
    return result


def parse_sim_metadata(path):
    """Extract SIM_* geometry hints written by the planner into commands.txt."""
    meta = {}
    try:
        with open(path) as f:
            for raw in f:
                line = raw.strip()
                if not line.startswith('# SIM_'):
                    continue
                content = line[2:]  # strip '# '
                key, _, val = content.partition(':')
                nums = [float(x) for x in val.split()]
                meta[key.strip()] = nums
    except FileNotFoundError:
        pass
    return meta


# ── Simulator class ───────────────────────────────────────────────────────────

class Simulator:

    def __init__(self, mission_file=MISSION_FILE):
        self.mission_file = mission_file

        # Scale field to fit available canvas area
        avail_w = CANVAS_W - SIDE_W - 2 * FIELD_PAD
        avail_h = CANVAS_H - 2 * FIELD_PAD
        self.scale = min(avail_w / FIELD_WIDTH_MM, avail_h / FIELD_HEIGHT_MM)

        # Top-left of field on canvas
        self.ox = float(FIELD_PAD)
        self.oy = FIELD_PAD + (avail_h - FIELD_HEIGHT_MM * self.scale) / 2.0

        # Defaults — overwritten by SIM_* metadata from commands.txt
        self.start_mm    = (FIELD_WIDTH_MM / 2.0, FIELD_HEIGHT_MM / 2.0)
        self.center_mm   = (FIELD_WIDTH_MM / 2.0, FIELD_HEIGHT_MM / 2.0)
        self.center_r_mm = CENTER_RADIUS_MM
        self.wall_mm     = WALL_MARGIN_MM
        self.hole_mm     = (FIELD_WIDTH_MM * HOLE_FRAC_X, FIELD_HEIGHT_MM * HOLE_FRAC_Y)
        self.hull_mm     = None   # list of (x_mm, y_mm) when hull is available
        self.init_heading = float(INITIAL_HEADING_DEG)

        self.commands  = []
        self.positions = []   # (x_mm, y_mm, heading_deg, event_tag)
        self.step_idx  = 0

        self.animating  = False
        self.anim_speed = 0.04   # seconds per step
        self._last_anim = 0.0

        self.reload()

    # ── Coordinate helpers ────────────────────────────────────────────────────

    def _mm_to_canvas(self, x, y):
        return (int(self.ox + x * self.scale),
                int(self.oy + y * self.scale))

    def _canvas_to_mm(self, cx, cy):
        return ((cx - self.ox) / self.scale,
                (cy - self.oy) / self.scale)

    # ── Simulation ────────────────────────────────────────────────────────────

    def reload(self):
        self.commands = parse_commands(self.mission_file)
        meta = parse_sim_metadata(self.mission_file)

        if 'SIM_CENTER' in meta:
            cx, cy = meta['SIM_CENTER']
            self.center_mm = (cx, cy)
        else:
            self.center_mm = (FIELD_WIDTH_MM / 2.0, FIELD_HEIGHT_MM / 2.0)

        if 'SIM_CENTER_R' in meta:
            self.center_r_mm = meta['SIM_CENTER_R'][0]
        else:
            self.center_r_mm = CENTER_RADIUS_MM

        if 'SIM_WALL' in meta:
            self.wall_mm = meta['SIM_WALL'][0]
        else:
            self.wall_mm = WALL_MARGIN_MM

        if 'SIM_START' in meta:
            sx, sy = meta['SIM_START']
            self.start_mm = (sx, sy)

        if 'SIM_HOLE' in meta:
            hx, hy = meta['SIM_HOLE']
            self.hole_mm = (hx, hy)
        else:
            self.hole_mm = (FIELD_WIDTH_MM * HOLE_FRAC_X, FIELD_HEIGHT_MM * HOLE_FRAC_Y)

        if 'SIM_HULL' in meta:
            pts = meta['SIM_HULL']
            self.hull_mm = [(pts[i], pts[i + 1]) for i in range(0, len(pts) - 1, 2)]
        else:
            self.hull_mm = None

        self.reset()

    def reset(self):
        self.step_idx = 0
        self.animating = False
        self._simulate_all()

    def _simulate_all(self):
        """Pre-run every command and record (x, y, heading, tag) per step."""
        x, y    = self.start_mm
        heading = self.init_heading
        self.positions = [(x, y, heading, 'START')]

        for cmd, val in self.commands:
            if cmd in ('COMMENT', 'SPEED'):
                tag = cmd
            elif cmd == 'TURN':
                heading = (heading + val + 180.0) % 360.0 - 180.0
                tag = 'TURN'
            elif cmd == 'FORWARD':
                rad = math.radians(heading)
                x  += val * math.cos(rad)
                y  += val * math.sin(rad)
                tag = 'FORWARD'
            elif cmd == 'LIFT_DOWN':
                tag = 'LIFT_DOWN'
            elif cmd == 'LIFT_UP':
                tag = 'LIFT_UP'
            elif cmd == 'STOP':
                tag = 'STOP'
            else:
                tag = cmd
            self.positions.append((x, y, heading, tag))

    # ── Drawing ───────────────────────────────────────────────────────────────

    def draw(self):
        canvas = np.zeros((CANVAS_H, CANVAS_W, 3), dtype=np.uint8)
        canvas[:] = C_BG

        self._draw_field(canvas)
        self._draw_path(canvas)
        self._draw_events(canvas)
        self._draw_robot(canvas)
        self._draw_panel(canvas)
        self._draw_status(canvas)

        return canvas

    def _draw_field(self, canvas):
        if self.hull_mm:
            hull_pts = np.array([self._mm_to_canvas(x, y) for x, y in self.hull_mm],
                                dtype=np.int32)
            cv2.fillPoly(canvas, [hull_pts], C_FIELD)
            cv2.polylines(canvas, [hull_pts], True, C_BORDER, 2)
        else:
            p0 = self._mm_to_canvas(0, 0)
            p1 = self._mm_to_canvas(FIELD_WIDTH_MM, FIELD_HEIGHT_MM)
            cv2.rectangle(canvas, p0, p1, C_FIELD, -1)
            cv2.rectangle(canvas, p0, p1, C_BORDER, 2)

        # Wall margin guide
        m = self.wall_mm
        cv2.rectangle(canvas,
                      self._mm_to_canvas(m, m),
                      self._mm_to_canvas(FIELD_WIDTH_MM - m, FIELD_HEIGHT_MM - m),
                      C_MARGIN, 1)

        # Centre obstacle
        cx, cy = self._mm_to_canvas(*self.center_mm)
        cr = int(self.center_r_mm * self.scale)
        cv2.circle(canvas, (cx, cy), cr, C_OBST, -1)
        cv2.circle(canvas, (cx, cy), cr, C_OBST_E, 1)
        cv2.putText(canvas, "X", (cx - 6, cy + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_OBST_E, 1)

        # Hole
        hx, hy = self._mm_to_canvas(*self.hole_mm)
        cv2.circle(canvas, (hx, hy), int(15 * self.scale + 3), C_HOLE, -1)
        cv2.circle(canvas, (hx, hy), int(15 * self.scale + 3), (120, 120, 255), 1)
        cv2.putText(canvas, "HOLE", (hx + int(15 * self.scale + 5), hy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, C_HOLE, 1)

    def _draw_path(self, canvas):
        for i in range(1, len(self.positions)):
            x0, y0, _, _ = self.positions[i - 1]
            x1, y1, _, _ = self.positions[i]
            p0 = self._mm_to_canvas(x0, y0)
            p1 = self._mm_to_canvas(x1, y1)
            if i <= self.step_idx:
                cv2.line(canvas, p0, p1, C_PATH, 2)
            else:
                cv2.line(canvas, p0, p1, C_DIM, 1)

    def _draw_events(self, canvas):
        for i, (x, y, _, ev) in enumerate(self.positions):
            if ev != 'LIFT_DOWN':
                continue
            px, py = self._mm_to_canvas(x, y)
            # Collect: LIFT_DOWN immediately followed by LIFT_UP
            is_collect = (i + 1 < len(self.positions)
                          and self.positions[i + 1][3] == 'LIFT_UP')
            color = C_COLLECT if is_collect else C_DEPOSIT
            radius = 5 if is_collect else 7
            filled = i <= self.step_idx
            cv2.circle(canvas, (px, py), radius, color, -1 if filled else 1)
            if not is_collect:
                cv2.putText(canvas, "DEP", (px + 8, py + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1)

    def _draw_robot(self, canvas):
        if not self.positions:
            return
        idx = min(self.step_idx, len(self.positions) - 1)
        rx, ry, rh, _ = self.positions[idx]
        rpx, rpy = self._mm_to_canvas(rx, ry)

        arrow = 22
        rad   = math.radians(rh)
        ax    = int(rpx + arrow * math.cos(rad))
        ay    = int(rpy + arrow * math.sin(rad))

        cv2.circle(canvas, (rpx, rpy), 9, C_ROBOT, -1)
        cv2.arrowedLine(canvas, (rpx, rpy), (ax, ay), C_ROBOT, 2, tipLength=0.45)

    def _draw_panel(self, canvas):
        panel_x = CANVAS_W - SIDE_W
        cv2.rectangle(canvas, (panel_x, 0), (CANVAS_W, CANVAS_H - 30), (18, 18, 18), -1)
        cv2.line(canvas, (panel_x, 0), (panel_x, CANVAS_H), (55, 55, 55), 1)

        cv2.putText(canvas, "COMMANDS  ({}/{})".format(
            self.step_idx, len(self.positions) - 1),
            (panel_x + 6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_HI, 1)

        visible  = 28
        half     = visible // 2
        last_exe = self.step_idx - 1   # index of last executed command
        start    = max(0, last_exe - half + 1)
        end      = min(len(self.commands), start + visible)

        for row, i in enumerate(range(start, end)):
            cmd, val = self.commands[i]
            ry = 30 + row * 23

            # Build display text
            if cmd == 'COMMENT':
                text  = (val or '#')[:30]
                color = C_COMMENT
            elif val is not None:
                iv    = int(val) if val == int(val) else val
                text  = "{}:{}".format(cmd, iv)
                color = C_TEXT
            else:
                text  = cmd
                color = C_TEXT

            # Colour coding
            if cmd == 'LIFT_DOWN':
                color = C_COLLECT
            elif cmd == 'LIFT_UP':
                color = (0, 180, 120)
            elif cmd == 'STOP':
                color = (80, 80, 200)

            # Highlight last executed
            if i == last_exe:
                cv2.rectangle(canvas,
                               (panel_x + 2, ry - 2),
                               (CANVAS_W - 2, ry + 18),
                               (35, 55, 35), -1)
                color = C_HI

            cv2.putText(canvas, text, (panel_x + 8, ry + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)

    def _draw_status(self, canvas):
        panel_x = CANVAS_W - SIDE_W
        cv2.rectangle(canvas, (0, CANVAS_H - 30), (CANVAS_W, CANVAS_H), (18, 18, 18), -1)
        cv2.line(canvas, (0, CANVAS_H - 30), (CANVAS_W, CANVAS_H - 30), (55, 55, 55), 1)

        speed_fps = int(round(1.0 / max(self.anim_speed, 0.001)))
        anim_txt  = "PLAYING  {}/s".format(speed_fps) if self.animating else "PAUSED"
        status    = ("SPACE=step  BKSP=back  A=play  E=end  0=start  +/-=speed  "
                     "R=reload  Q=quit  |  {}".format(anim_txt))
        cv2.putText(canvas, status, (6, CANVAS_H - 9),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, C_TEXT, 1)

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        print("Command Simulator")
        print("=" * 50)
        print("  SPACE        : step forward one command")
        print("  BACKSPACE    : step back one command")
        print("  A            : toggle animation")
        print("  E            : jump to end (full path)")
        print("  0            : jump to start")
        print("  + / -        : animation speed")
        print("  R            : reload commands.txt")
        print("  Left-click   : set robot start position")
        print("  Q            : quit")
        print()

        def on_mouse(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                xmm, ymm = self._canvas_to_mm(x, y)
                if 0 <= xmm <= FIELD_WIDTH_MM and 0 <= ymm <= FIELD_HEIGHT_MM:
                    self.start_mm = (xmm, ymm)
                    self.reset()
                    print("Start set to ({:.0f}, {:.0f}) mm".format(xmm, ymm))

        cv2.namedWindow('Simulator')
        cv2.setMouseCallback('Simulator', on_mouse)

        while True:
            # Animation tick
            if self.animating:
                now = time.time()
                if now - self._last_anim >= self.anim_speed:
                    self._last_anim = now
                    if self.step_idx < len(self.positions) - 1:
                        self.step_idx += 1
                    else:
                        self.animating = False

            cv2.imshow('Simulator', self.draw())

            key = cv2.waitKey(16)   # ~60 fps refresh
            ch  = key & 0xFF

            if ch == ord('q'):
                break
            elif ch == ord(' ') or key == 2555904:   # SPACE or RIGHT arrow
                self.animating = False
                self.step_idx  = min(self.step_idx + 1, len(self.positions) - 1)
            elif ch == 8 or key == 2424832:          # BACKSPACE or LEFT arrow
                self.animating = False
                self.step_idx  = max(self.step_idx - 1, 0)
            elif ch == ord('a'):
                self.animating    = not self.animating
                self._last_anim   = time.time()
            elif ch == ord('e'):
                self.animating = False
                self.step_idx  = len(self.positions) - 1
            elif ch in (ord('0'), 36):               # '0' or Home
                self.animating = False
                self.step_idx  = 0
            elif ch in (ord('+'), ord('=')):
                self.anim_speed = max(0.01, self.anim_speed - 0.01)
                print("Speed: {:.0f} steps/s".format(1.0 / self.anim_speed))
            elif ch == ord('-'):
                self.anim_speed = min(2.0, self.anim_speed + 0.02)
                print("Speed: {:.0f} steps/s".format(1.0 / self.anim_speed))
            elif ch == ord('r'):
                self.reload()
                print("Reloaded: {}".format(self.mission_file))

        cv2.destroyAllWindows()


if __name__ == "__main__":
    Simulator().run()
