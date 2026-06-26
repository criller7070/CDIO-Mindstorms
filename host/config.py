#!/usr/bin/env python3
"""
shared constants and colour-range persistence for the vision system.
all other vision modules import from here.
"""
import os
import json
import copy

# CAMERA & PATHS
CAMERA_INDEX   = 1
MISSION_FILE   = os.path.join(os.path.dirname(__file__), "..", "robot", "commands.txt")
SCREENSHOT_DIR = os.path.join(os.path.dirname(__file__), "..", "screenshots")
MASK_DIR       = os.path.join(SCREENSHOT_DIR, "masks")
ARUCO_FROM_BACK_FRAC = 0.344 # 110 / 320

# FIELD LAYOUT
WALL_MARGIN         = 45
CENTER_RADIUS       = 60
INITIAL_HEADING_DEG = -90
HOLE_FRAC_X         = 0.05
HOLE_FRAC_Y         = 0.50

# ROBOT DIMENSIONS (mm)
ROBOT_WIDTH_MM  = 240
ROBOT_LENGTH_MM = 320
GATE_ARM_MM     = 90
ROBOT_PIVOT_OFFSET_MM = ROBOT_LENGTH_MM * (0.5 - 0.25) # 92.5 mm behind centre

_env_path = os.path.join(os.path.dirname(__file__), "..", ".env")
try:
    with open(_env_path) as _f:
        for _line in _f:
            _line = _line.strip()
            if _line and not _line.startswith("#") and "=" in _line:
                _k, _, _v = _line.partition("=")
                os.environ.setdefault(_k.strip(), _v.strip())
except FileNotFoundError:
    pass

# ROBOFLOW YOLO MODEL
ROBOFLOW_API_KEY   = os.environ.get("ROBOFLOW_API_KEY", "")
ROBOFLOW_API_URL   = "https://serverless.roboflow.com"
ROBOFLOW_MODEL_ID  = "ping-pong-finder-w6mxk/9"

# HSV CALIBRATION
COLOR_RANGES_FILE = os.path.join(os.path.dirname(__file__), "color_ranges.json")
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
    with open(COLOR_RANGES_FILE, 'w') as f:
        json.dump(ranges, f, indent=2)
    print("Calibration saved to: {}".format(COLOR_RANGES_FILE))

# CONTROL-LOOP TUNABLES
ARRIVE_PX                = 20.0
TURN_TOL_DEG             = 8.0
TURN_COMMIT_DEG          = 45.0
TURN_SLOPE               = 1.0
TURN_COAST_DEG           = 3.0
MAX_STEP_MM              = 20
MIN_STEP_MM              = 10
FORWARD_CMD_SCALE        = 3.2288
ACTUAL_PX_PER_MM         = 1.39
MAX_POSE_MISS            = 60
REPLAN_PX                = 150.0
REPLAN_EVERY_N           = 8
DENSIFY_GAP_PX           = 40.0
HEADING_LOOKAHEAD_PX     = 100.0
GATE_OPEN_DEG            = 0
GATE_CLOSE_DEG           = 90
GATE_DROPOFF_DEG         = 15
LIFT_DROPOFF_DEG         = 200
GATE_PRE_OPEN_PX         = 200
BALL_GATE_THRESHOLD_PX   = 70
CENTER_OBSTACLE_EXTRA_PX = 5