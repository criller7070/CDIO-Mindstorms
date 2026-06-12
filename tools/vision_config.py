#!/usr/bin/env python3
"""
Shared constants and colour-range persistence for the vision system.
All other vision modules import from here.
"""
import os
import json
import copy

# ── Camera & paths ────────────────────────────────────────────────────────────
CAMERA_INDEX   = 1
MISSION_FILE   = os.path.join(os.path.dirname(__file__), "..", "robot", "commands.txt")
SCREENSHOT_DIR = os.path.join(os.path.dirname(__file__), "..", "screenshots")
MASK_DIR       = os.path.join(SCREENSHOT_DIR, "masks")

# ── Field layout ──────────────────────────────────────────────────────────────
WALL_MARGIN         = 45
CENTER_RADIUS       = 60
INITIAL_HEADING_DEG = -90
HOLE_FRAC_X         = 0.05   # left hole: 5 % from left edge
HOLE_FRAC_Y         = 0.50   # left hole: 50 % down

# ── Robot physical dimensions (mm) ────────────────────────────────────────────
ROBOT_WIDTH_MM  = 240   # side-to-side across tracks
ROBOT_LENGTH_MM = 370   # front bumper to back

# Turn pivot: the robot rotates about a point at 25% of its length from the BACK
# and 50% of its width (centred between the tracks).  This value is how far that
# pivot sits BEHIND the geometric centre (positive = toward the rear).  Command
# generation compensates for it so the robot centre still follows the path.
ROBOT_PIVOT_OFFSET_MM = ROBOT_LENGTH_MM * (0.5 - 0.25)   # 92.5 mm behind centre

# ── HSV calibration ───────────────────────────────────────────────────────────
COLOR_RANGES_FILE = os.path.join(os.path.dirname(__file__), "color_ranges.json")

# Shape gates default to OFF (0 thresholds, wide radius) so the raw HSV mask
# drives detection. Tighten values in color_ranges.json once HSV is calibrated.
DEFAULT_COLOR_RANGES = {
    "WHITE": {
        "lower": [0, 0, 175], "upper": [179, 50, 255],
        "radius_min": 1, "radius_max": 999,
        "circularity": 0.70, "fill_ratio": 0.50, "solidity": 0.75, "aspect_ratio": 1.6,
    },
    "ORANGE": {
        "lower": [5, 60, 80], "upper": [25, 255, 255],
        "mean_h_min": 0, "mean_h_max": 179, "mean_s_min": 0, "mean_v_min": 0,
        "radius_min": 1, "radius_max": 999,
        "circularity": 0.70, "fill_ratio": 0.50, "solidity": 0.75, "aspect_ratio": 1.6,
    },
    "RED": {
        "lower":  [0,   80, 70], "upper":  [10,  255, 255],
        "lower2": [170, 80, 70], "upper2": [179, 255, 255],
    },
}


def load_color_ranges():
    """Load calibration from JSON, merged on top of defaults."""
    try:
        with open(COLOR_RANGES_FILE, 'r') as f:
            saved = json.load(f)
        ranges = copy.deepcopy(DEFAULT_COLOR_RANGES)
        for color, vals in saved.items():
            if color in ranges:
                ranges[color].update(vals)
        print("Loaded calibration from: {}".format(COLOR_RANGES_FILE))
        return ranges
    except FileNotFoundError:
        print("No calibration file found – using defaults. Press C to calibrate.")
        return copy.deepcopy(DEFAULT_COLOR_RANGES)


def save_color_ranges(ranges):
    """Persist calibration to JSON."""
    with open(COLOR_RANGES_FILE, 'w') as f:
        json.dump(ranges, f, indent=2)
    print("Calibration saved to: {}".format(COLOR_RANGES_FILE))
