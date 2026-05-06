#!/usr/bin/env python3
"""
Vision-based ball hunting navigator for EV3 robot.
Detects white and orange table tennis balls in a square course with X obstacle.
Generates autonomous path to visit all balls while avoiding walls and X structure.
"""

import cv2
import numpy as np
import time

from path_planner import FieldPlanner, FIELD_WIDTH_MM, FIELD_HEIGHT_MM

# Configuration - easily change these values
CAMERA_INDEX = 0
MISSION_FILE = "commands.txt"

# Wall margin (pixels) and center no-go radius (pixels) for the planner
WALL_MARGIN = 45
CENTER_RADIUS = 60

# Robot's initial heading in image-space degrees.
# -90 = facing toward top of frame (most common starting orientation).
INITIAL_HEADING_DEG = -90

# Left hole position as a fraction of the field interior (0.0-1.0).
# (0.05, 0.5) means 5% from left edge, 50% down — adjust to match your course.
HOLE_FRAC_X = 0.05
HOLE_FRAC_Y = 0.50


class BallHuntingPlanner:
    def __init__(self, camera_index=0, mission_file="commands.txt"):
        """Initialize camera and path planner"""
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        self.mission_file = mission_file
        self.mission_commands = []
        
        # Ball detection parameters
        self.white_ball_radius_range = (15, 50)
        self.orange_ball_radius_range = (15, 50)
        self.min_ball_area = 100
        
        # Red wall detection parameters
        self.min_red_line_width = 5
        self.min_red_line_length = 50
    
    def detect_balls(self, frame):
        """Detect white and orange table tennis balls"""
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Detect white balls (high value, low saturation)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        
        # Detect orange balls (orange hue range)
        lower_orange = np.array([5, 100, 100])
        upper_orange = np.array([25, 255, 255])
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
        
        white_balls = self._find_ball_centers(mask_white, "WHITE")
        orange_balls = self._find_ball_centers(mask_orange, "ORANGE")
        
        all_balls = white_balls + orange_balls
        
        return all_balls, mask_white, mask_orange
    
    def _find_ball_centers(self, mask, color_name):
        """Find ball centers in mask"""
        balls = []
        
        # Apply morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_ball_area:
                # Fit circle to contour
                (x, y), radius = cv2.minEnclosingCircle(contour)
                if radius > 8:
                    balls.append({
                        'x': int(x),
                        'y': int(y),
                        'radius': int(radius),
                        'area': area,
                        'color': color_name
                    })
        
        return balls
    
    def detect_obstacles(self, frame):
        """Detect the X structure and square boundary"""
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Detect dark obstacles (X structure and walls)
        lower_dark = np.array([0, 0, 0])
        upper_dark = np.array([180, 255, 100])
        mask_dark = cv2.inRange(hsv, lower_dark, upper_dark)
        
        return mask_dark
    
    def detect_red_walls(self, frame):
        """Detect red walls and X obstacle structure"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Red color in HSV: Hue wraps around 0/180
        # Lower red range (0-10)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        
        # Upper red range (170-180)
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        
        # Combine both red ranges
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        # Apply morphological operations to clean up noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        
        # Detect edges to find wall boundaries
        edges = cv2.Canny(mask_red, 50, 150)
        
        # Detect lines in the red areas
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=self.min_red_line_length, maxLineGap=10)
        
        return {
            'mask': mask_red,
            'edges': edges,
            'lines': lines
        }
    
    def detect_field_bounds(self, frame):
        """
        Return (x_min, y_min, x_max, y_max) pixel bounds of the red field rectangle.
        Falls back to a margin-inset of the full frame if walls are not detected.
        """
        red_info = self.detect_red_walls(frame)
        mask = red_info['mask']

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            all_pts = np.vstack(contours)
            x, y, w, h = cv2.boundingRect(all_pts)
            return (x, y, x + w, y + h)

        # Fallback: use full frame with a small inset
        fh, fw = frame.shape[:2]
        inset = 20
        return (inset, inset, fw - inset, fh - inset)

    def analyze_course(self, frame):
        """Full course analysis"""
        balls, mask_white, mask_orange = self.detect_balls(frame)
        obstacles = self.detect_obstacles(frame)
        red_walls = self.detect_red_walls(frame)
        field_bounds = self.detect_field_bounds(frame)

        h, w = frame.shape[:2]

        return {
            'balls': balls,
            'obstacles': obstacles,
            'white_mask': mask_white,
            'orange_mask': mask_orange,
            'red_walls': red_walls,
            'field_bounds': field_bounds,
            'frame_h': h,
            'frame_w': w,
        }

    def plan_path_to_balls(self, analysis, robot_pos=None, capacity=6):
        """
        Plan a capacity-aware multi-trip route to collect all balls and
        deposit them at the left hole.

        robot_pos: (x, y) pixel position of the robot. If None, uses the
                   field centre as the starting point.
        capacity:  maximum balls the robot can carry per trip.
        """
        balls = analysis['balls']
        field_bounds = analysis['field_bounds']
        x_min, y_min, x_max, y_max = field_bounds

        cx = (x_min + x_max) / 2
        cy = (y_min + y_max) / 2

        if robot_pos is None:
            robot_pos = (cx, cy)

        hole_x = x_min + (x_max - x_min) * HOLE_FRAC_X
        hole_y = y_min + (y_max - y_min) * HOLE_FRAC_Y
        dropoff = (int(hole_x), int(hole_y))

        planner = FieldPlanner(
            field_bounds=field_bounds,
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

        # Store for debug overlay (uses the last trip's segments if available)
        self._last_planner = planner
        self._last_ball_positions = ball_positions
        self._last_dropoff = dropoff

        return commands
    
    def run_planning_mode(self):
        """
        Interactive mode to analyze course and generate mission.

        Controls:
          LEFT-CLICK  - set robot starting position
          SPACE       - generate mission from current frame + robot position
          D           - show A* debug overlay (after generating a mission)
          Q           - quit
        """
        print("Ball Hunting Navigator")
        print("=" * 50)
        print("  LEFT-CLICK  : set robot start position")
        print("  SPACE       : analyse frame and generate mission")
        print("  D           : show planned path overlay")
        print("  Q           : quit")
        print()

        robot_pos = None       # set by mouse click
        last_frame = None      # freeze frame used for planning
        debug_vis = None       # path overlay image

        def on_mouse(event, x, y, flags, param):
            nonlocal robot_pos
            if event == cv2.EVENT_LBUTTONDOWN:
                robot_pos = (x, y)
                print("Robot position set to ({}, {})".format(x, y))

        cv2.namedWindow('Ball Detection')
        cv2.setMouseCallback('Ball Detection', on_mouse)

        try:
            frame_count = 0
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to read frame")
                    break

                frame = cv2.flip(frame, 1)
                frame_count += 1
                last_frame = frame.copy()

                analysis = self.analyze_course(frame)
                balls = analysis['balls']
                bounds = analysis['field_bounds']

                display_frame = frame.copy()
                white_count = orange_count = 0

                for ball in balls:
                    color_bgr = (255, 255, 255) if ball['color'] == "WHITE" else (0, 165, 255)
                    cv2.circle(display_frame, (ball['x'], ball['y']), ball['radius'], color_bgr, 2)
                    cv2.circle(display_frame, (ball['x'], ball['y']), 3, color_bgr, -1)
                    white_count += ball['color'] == "WHITE"
                    orange_count += ball['color'] == "ORANGE"

                # Draw detected field boundary
                x0, y0, x1, y1 = bounds
                cv2.rectangle(display_frame, (x0, y0), (x1, y1), (0, 0, 200), 1)

                # Draw center obstacle zone
                cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
                cv2.circle(display_frame, (cx, cy), CENTER_RADIUS, (0, 0, 200), 1)

                # Draw left hole
                hx = int(x0 + (x1 - x0) * HOLE_FRAC_X)
                hy = int(y0 + (y1 - y0) * HOLE_FRAC_Y)
                cv2.circle(display_frame, (hx, hy), 10, (255, 0, 0), 2)
                cv2.putText(display_frame, "HOLE", (hx + 12, hy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1)

                # Draw robot position if set
                if robot_pos:
                    cv2.circle(display_frame, robot_pos, 8, (0, 255, 0), -1)
                    cv2.putText(display_frame, "ROBOT", (robot_pos[0] + 10, robot_pos[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

                status = "White:{} Orange:{} | {}".format(
                    white_count, orange_count,
                    "Click to set robot pos" if not robot_pos else "SPACE=plan D=debug Q=quit"
                )
                cv2.putText(display_frame, status, (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

                cv2.imshow('Ball Detection', display_frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord(' '):
                    if robot_pos is None:
                        print("Click to set robot position first, then press SPACE.")
                        continue
                    analysis = self.analyze_course(last_frame)
                    print("\nGenerating A* mission for {} balls...".format(len(analysis['balls'])))
                    self.mission_commands = self.plan_path_to_balls(analysis, robot_pos)
                    self.save_mission()
                    # Build debug overlay if planner data is available
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

        finally:
            self.cleanup()
    
    def save_mission(self):
        """Save mission commands to file"""
        try:
            with open(self.mission_file, "w") as f:
                for cmd in self.mission_commands:
                    f.write(cmd + "\n")
            print("Mission saved to: {}".format(self.mission_file))
            print("\nGenerated commands:")
            for i, cmd in enumerate(self.mission_commands, 1):
                if not cmd.startswith("#"):
                    print("  {}: {}".format(i, cmd))
        except Exception as e:
            print("Error saving mission: {}".format(e))
    
    def cleanup(self):
        """Clean up resources"""
        self.cap.release()
        cv2.destroyAllWindows()
        print("\nShutdown complete")


if __name__ == "__main__":
    planner = BallHuntingPlanner(camera_index=CAMERA_INDEX, mission_file=MISSION_FILE)
    planner.run_planning_mode()
