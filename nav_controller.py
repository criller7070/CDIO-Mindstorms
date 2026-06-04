#!/usr/bin/env python3
"""
Nav controller - screenshot mode.
Click on the robot in the image to plan and send the mission via Bluetooth.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "tools"))
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

import cv2
import numpy as np

from tools_path_planner import FieldPlanner, FIELD_WIDTH_MM, FIELD_HEIGHT_MM
from tools_vision_boundary_detection import (
    BallHuntingPlanner,
    WALL_MARGIN,
    CENTER_RADIUS,
    INITIAL_HEADING_DEG,
    SCREENSHOT_DIR,
    MISSION_FILE,
)
from tools_mission_sender import HostMissionSender

SCREENSHOT_PATH = os.path.join(
    SCREENSHOT_DIR, "063b8555-1005-435a-826e-66413a671d29.jpg"
)


def detect_balls(frame):
    d = BallHuntingPlanner.__new__(BallHuntingPlanner)
    d.white_ball_radius_range = (7, 16)
    d.orange_ball_radius_range = (7, 16)
    d.min_ball_area = 60
    d.border_margin = 25
    d.min_red_line_width = 5
    d.min_red_line_length = 50
    d._ball_tracks = {"WHITE": [], "ORANGE": []}
    d.ball_confirm_frames = 4
    d.ball_miss_frames = 1
    d.ball_match_distance = 18

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_white = cv2.inRange(hsv, np.array([0, 0, 150]), np.array([180, 50, 255]))
    mask_orange = cv2.inRange(hsv, np.array([5, 60, 80]), np.array([25, 255, 255]))
    bm = d.border_margin
    for mask in (mask_white, mask_orange):
        mask[:bm, :] = 0; mask[-bm:, :] = 0
        mask[:, :bm] = 0; mask[:, -bm:] = 0

    white = d._find_ball_centers(mask_white, "WHITE", frame.shape[:2], hsv=hsv, min_circularity=0.45)
    orange = d._find_ball_centers(mask_orange, "ORANGE", frame.shape[:2], hsv=hsv, min_circularity=0.45)
    field_bounds = d.detect_field_bounds(frame)
    return white + orange, field_bounds


def plan(balls, field_bounds, robot_pos):
    x_min, y_min, x_max, y_max = field_bounds
    cx, cy = (x_min + x_max) // 2, (y_min + y_max) // 2

    planner = FieldPlanner(
        field_bounds=field_bounds,
        center_pos=(cx, cy),
        wall_margin=WALL_MARGIN,
        center_radius=CENTER_RADIUS,
        field_width_mm=FIELD_WIDTH_MM,
        field_height_mm=FIELD_HEIGHT_MM,
    )

    ball_positions = [(b["x"], b["y"]) for b in balls]
    ball_order, path_segs = planner.optimal_route(robot_pos, ball_positions, robot_pos)
    if path_segs is None:
        return ["STOP"], planner, [], []

    visit_segs = path_segs[: len(ball_order)]
    commands = ["SPEED:300"]
    heading = float(INITIAL_HEADING_DEG)

    for seg in visit_segs:
        pts = FieldPlanner.simplify(seg, eps=8)
        for i in range(1, len(pts)):
            x0, y0 = pts[i - 1]
            x1, y1 = pts[i]
            dx, dy = x1 - x0, y1 - y0
            dist_px = (dx ** 2 + dy ** 2) ** 0.5
            if dist_px < planner.GRID_SCALE:
                continue
            target = float(np.degrees(np.arctan2(dy, dx)))
            turn = (target - heading + 180.0) % 360.0 - 180.0
            if abs(turn) > 2:
                commands.append("TURN:{}".format(int(round(turn))))
                heading = target
            dist_mm = max(1, int(dist_px / planner.px_per_mm))
            commands.append("FORWARD:{}".format(dist_mm))

    commands.append("STOP")
    return commands, planner, ball_order, visit_segs


def send(commands, mission_file):
    os.makedirs(os.path.dirname(os.path.abspath(mission_file)), exist_ok=True)
    with open(mission_file, "w") as f:
        for cmd in commands:
            f.write(cmd + "\n")
    print("Saved to {}".format(mission_file))

    sender = HostMissionSender()
    if not sender.connect_to_ev3():
        return
    try:
        sender.send_command("MISSION_START")
        for cmd in commands:
            sender.send_command(cmd)
        sender.send_command("MISSION_END")
        print("Sent {} commands via Bluetooth.".format(len(commands)))
    finally:
        sender.disconnect()


def run(screenshot_path=SCREENSHOT_PATH, mission_file=MISSION_FILE):
    frame = cv2.imread(screenshot_path)
    if frame is None:
        print("ERROR: Cannot load screenshot: {}".format(screenshot_path))
        return

    balls, field_bounds = detect_balls(frame)
    x_min, y_min, x_max, y_max = field_bounds
    cx, cy = (x_min + x_max) // 2, (y_min + y_max) // 2

    print("Detected {} ball(s): {} white, {} orange".format(
        len(balls),
        sum(1 for b in balls if b["color"] == "WHITE"),
        sum(1 for b in balls if b["color"] == "ORANGE"),
    ))
    print("Click on the robot to plan and send.")

    display = frame.copy()
    for ball in balls:
        color_bgr = (255, 255, 255) if ball["color"] == "WHITE" else (0, 165, 255)
        cv2.circle(display, (ball["x"], ball["y"]), ball["radius"], color_bgr, 2)
        cv2.circle(display, (ball["x"], ball["y"]), 3, color_bgr, -1)
    cv2.rectangle(display, (x_min, y_min), (x_max, y_max), (0, 0, 200), 1)
    cv2.circle(display, (cx, cy), CENTER_RADIUS, (0, 0, 200), 1)
    cv2.putText(display, "Click on robot to send mission",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

    def on_click(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        robot_pos = (x, y)
        print("Robot at ({}, {}) — planning...".format(x, y))
        commands, _, _, _ = plan(balls, field_bounds, robot_pos)
        print("Commands: {}".format(commands))
        send(commands, mission_file)

    cv2.namedWindow("Nav Controller")
    cv2.setMouseCallback("Nav Controller", on_click)
    cv2.imshow("Nav Controller", display)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else SCREENSHOT_PATH
    run(screenshot_path=path)
