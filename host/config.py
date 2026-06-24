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

# FIELD LAYOUT
WALL_MARGIN         = 45
CENTER_RADIUS       = 60
INITIAL_HEADING_DEG = -90
HOLE_FRAC_X         = 0.05   # left hole: 5 % from left edge
HOLE_FRAC_Y         = 0.50   # left hole: 50 % down

# ROBOT DIMENSIONS (mm)
ROBOT_WIDTH_MM  = 240   # side-to-side across tracks
ROBOT_LENGTH_MM = 320   # front bumper to back (measured: 110 mm rear + 210 mm to front)
GATE_ARM_MM     = 90    # each gate arm length; when fully open (90°) each side extends this far

# ArUco marker position along the robot's length axis.
# 0% = rear bumper, 100% = front bumper.
# Measured: ArUco centre is 110 mm from rear, 210 mm from front → total 320 mm.
ARUCO_FROM_BACK_FRAC = 0.344   # 110 / 320

# Turn pivot: the robot rotates about a point at 25% of its length from the BACK
# and 50% of its width (centred between the tracks).  This value is how far that
# pivot sits BEHIND the geometric centre (positive = toward the rear).  Command
# generation compensates for it so the robot centre still follows the path.
ROBOT_PIVOT_OFFSET_MM = ROBOT_LENGTH_MM * (0.5 - 0.25)   # 92.5 mm behind centre

# load .env from repo root so ROBOFLOW_API_KEY and profile IPs are available
# without the caller having to export them manually.
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
# Leave empty to fall back to the HoughCircles detector.
ROBOFLOW_API_KEY   = os.environ.get("ROBOFLOW_API_KEY", "")
ROBOFLOW_API_URL   = "https://serverless.roboflow.com"
ROBOFLOW_MODEL_ID  = "ping-pong-finder-w6mxk/9"

# HSV CALIBRATION
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
    """load calibration from JSON, merged on top of defaults."""
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
    """persist calibration to JSON."""
    with open(COLOR_RANGES_FILE, 'w') as f:
        json.dump(ranges, f, indent=2)
    print("Calibration saved to: {}".format(COLOR_RANGES_FILE))


# CONTROL-LOOP TUNABLES
ARRIVE_PX       = 20.0   # waypoint counts as reached within this many pixels.
                         # With DENSIFY_GAP_PX=40px and ACTUAL_PX_PER_MM=1.39,
                         # one forward step = 20mm = 27.8px. Arrival at 20px lets
                         # the robot reach the waypoint area without tight looping.
TURN_TOL_DEG    = 8.0    # rotate only for heading errors larger than this.
                         # TURN_COAST_DEG=3° so 8° leaves 5° cmd headroom before
                         # coast clears the target - prevents limit-cycle oscillation.
TURN_COMMIT_DEG = 45.0   # after a turn, drive a forward step before turning again
                         # unless the heading error still exceeds this. Prevents
                         # turn-turn-turn oscillation from small overshoots.
# Measured EV3 follow-mode turn response: actual = TURN_SLOPE*commanded + coast.
# Compensate so a requested heading change actually lands on target:
#   command = (desired - TURN_COAST_DEG) / TURN_SLOPE
# Recalibrated from run log (3 observed turns at 45 deg/s):
#   cmd=104 → actual=108, cmd=61 → actual=63, cmd=156 → actual=158
#   Best fit: actual ≈ 1.0*cmd + 3  (coast much smaller than original 8 deg)
TURN_SLOPE      = 1.0
TURN_COAST_DEG  = 3.0
MAX_STEP_MM     = 20     # never drive more than this (physical mm) between observations
                         # CRITICAL: must satisfy MAX_STEP_MM * px_per_mm < ARRIVE_PX
                         # or the robot overshoots waypoints and spins back (oscillation)
MIN_STEP_MM     = 10     # smallest forward nudge worth sending
# robot/main.py executes FORWARD:v as straight(-v / 3.2288), i.e. the command
# value is ~3.2x the physical mm travelled. Scale the command so a requested
# physical step actually moves that far (otherwise the robot crawls ~1/3 speed).
FORWARD_CMD_SCALE      = 3.2288
# Measured camera scale: FORWARD:161 cmd = 50 mm actual = 69 px in image → 1.38 px/mm.
# Used for step sizing so close-approach steps don't overshoot the waypoint.
ACTUAL_PX_PER_MM       = 1.39
MAX_POSE_MISS          = 60     # give up after this many consecutive frames with no marker
REPLAN_PX              = 150.0  # re-plan when robot is >150px off its target
REPLAN_EVERY_N         = 8      # also force a replan after every N forward steps
DENSIFY_GAP_PX         = 40.0   # maximum pixel gap between consecutive waypoints
                                 # 40px ≈ 29mm. At 40px spacing, 5px positional noise
                                 # causes only 7° heading error - below TURN_TOL_DEG.
HEADING_LOOKAHEAD_PX   = 100.0  # pre-align to next waypoint's bearing when this close
                                 # > 2*ARRIVE_PX so it doesn't conflict with in_close_approach;
                                 # robot starts turning toward its post-arrival heading early
GATE_OPEN_DEG          = 0      # absolute target angle for GATE_OPEN (0 = fully open stop)
GATE_CLOSE_DEG         = 90     # absolute target angle for GATE_CLOSE (90 = manual closed position)
GATE_DROPOFF_DEG       = 45     # absolute target angle for dropoff partial-open (halfway)
LIFT_DROPOFF_DEG       = 45     # tray tip angle at dropoff: enough to roll balls out, not flip tray
GATE_PRE_OPEN_PX    = 144    # open gate when this many px from a ball so the robot
                              # drives INTO the ball with gate already open.
BALL_GATE_THRESHOLD_PX = 70  # close gate (capture) when within this many px of ball.
                              # trim offset = 135mm * 0.369px/mm = 49.8px, must be > 50px.
CENTER_OBSTACLE_EXTRA_PX = 5    # extra px added to X obstacle radius before A*
                                 # _build_grid already adds robot_half_width_px (~42px),
                                 # so keep this small - 10px compensates camera underestimate
                                 # without blocking reachable balls
