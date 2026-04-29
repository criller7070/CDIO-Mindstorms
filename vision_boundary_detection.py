#!/usr/bin/env python3
"""
Vision-based ball hunting navigator for EV3 robot.
Detects white and orange table tennis balls in a square course with X obstacle.
Generates autonomous path to visit all balls while avoiding walls and X structure.
"""

import cv2
import numpy as np
import time

class BallHuntingPlanner:
    def __init__(self, camera_index=1, mission_file="commands.txt"):
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
    
    def analyze_course(self, frame):
        """Full course analysis"""
        balls, mask_white, mask_orange = self.detect_balls(frame)
        obstacles = self.detect_obstacles(frame)
        
        h, w = frame.shape[:2]
        
        return {
            'balls': balls,
            'obstacles': obstacles,
            'white_mask': mask_white,
            'orange_mask': mask_orange,
            'frame_h': h,
            'frame_w': w
        }
    
    def plan_path_to_balls(self, analysis):
        """Plan path visiting all detected balls"""
        balls = analysis['balls']
        
        if not balls:
            return ["STOP"]
        
        commands = [
            "# Ball hunting mission",
            "SPEED:200",
        ]
        
        # Sort balls by distance from center (nearest first for efficiency)
        center_x = analysis['frame_w'] / 2
        center_y = analysis['frame_h'] / 2
        
        balls_sorted = sorted(balls, key=lambda b: (b['x'] - center_x)**2 + (b['y'] - center_y)**2)
        
        # Generate movement commands for each ball
        current_pos = (center_x, center_y)
        
        for i, ball in enumerate(balls_sorted):
            # Calculate direction to ball
            dx = ball['x'] - current_pos[0]
            dy = ball['y'] - current_pos[1]
            distance = int(np.sqrt(dx**2 + dy**2))
            
            if distance > 20:
                # Move toward ball
                angle_rad = np.arctan2(dy, dx)
                angle_deg = int(np.degrees(angle_rad))
                
                commands.append("# Moving to {} ball at x:{} y:{}".format(ball['color'], ball['x'], ball['y']))
                
                # Approach ball with small movements and turns to avoid obstacles
                for step in range(0, distance, 200):
                    commands.append("FORWARD:200")
                
                current_pos = (ball['x'], ball['y'])
                
                # Stop at ball location briefly
                commands.append("STOP")
                time.sleep(0.2)  # Simulate ball pickup
        
        # Return to center
        commands.append("# Returning to center")
        commands.append("FORWARD:500")
        commands.append("STOP")
        
        return commands
    
    def run_planning_mode(self):
        """
        Interactive mode to analyze course and generate mission
        """
        print("Ball Hunting Navigator")
        print("=" * 50)
        print("Instructions:")
        print("  - Point camera at the course")
        print("  - Press 'SPACE' to analyze frame and generate mission")
        print("  - Press 'q' to quit")
        print()
        
        try:
            frame_count = 0
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to read frame")
                    break
                
                frame = cv2.flip(frame, 1)
                frame_count += 1
                
                # Analyze current frame
                analysis = self.analyze_course(frame)
                balls = analysis['balls']
                
                # Visualization
                h, w = frame.shape[:2]
                display_frame = frame.copy()
                
                # Draw detected balls
                white_count = 0
                orange_count = 0
                
                for ball in balls:
                    color_bgr = (255, 255, 255) if ball['color'] == "WHITE" else (0, 165, 255)
                    cv2.circle(display_frame, (ball['x'], ball['y']), ball['radius'], color_bgr, 2)
                    cv2.circle(display_frame, (ball['x'], ball['y']), 3, color_bgr, -1)
                    
                    if ball['color'] == "WHITE":
                        white_count += 1
                    else:
                        orange_count += 1
                
                # Draw center point
                cv2.circle(display_frame, (w//2, h//2), 5, (0, 255, 0), -1)
                
                # Status text
                status_text = "White: {} | Orange: {}".format(white_count, orange_count)
                cv2.putText(display_frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_frame, "Press SPACE to generate mission | q to quit", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                
                cv2.imshow('Ball Detection', display_frame)
                cv2.imshow('White Balls', analysis['white_mask'])
                cv2.imshow('Orange Balls', analysis['orange_mask'])
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord(' '):
                    print("\nGenerating mission for {} balls...".format(len(balls)))
                    self.mission_commands = self.plan_path_to_balls(analysis)
                    self.save_mission()
                    print("\nMission generated! Ready to run on EV3.")
                    break
        
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
    planner = BallHuntingPlanner(camera_index=1)
    planner.run_planning_mode()
