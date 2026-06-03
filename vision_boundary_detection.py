#!/usr/bin/env python3
"""
Vision-based ball hunting navigator for EV3 robot.
Detects white and orange table tennis balls in a square course with X obstacle.
Generates autonomous path to visit all balls while avoiding walls and X structure.
"""
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2
import numpy as np
import time
import glob
import re

from path_planner import FieldPlanner, FIELD_WIDTH_MM, FIELD_HEIGHT_MM

# Configuration - easily change these values
CAMERA_INDEX = 1
MISSION_FILE = "commands.txt"
SCREENSHOT_DIR = "screenshots"
MASK_DIR = os.path.join(SCREENSHOT_DIR, "masks")

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

        os.makedirs(SCREENSHOT_DIR, exist_ok=True)
        os.makedirs(MASK_DIR, exist_ok=True)
        
        # Ball detection parameters (table tennis balls have fixed size)
        # The balls are small in this camera view, so keep the radius range tight.
        self.white_ball_radius_range = (7, 16)
        self.orange_ball_radius_range = (7, 16)
        self.min_ball_area = 60
        self.border_margin = 25
        self.ball_confirm_frames = 4
        self.ball_miss_frames = 1
        self.ball_match_distance = 18
        self._ball_tracks = {"WHITE": [], "ORANGE": []}
        
        # Red wall detection parameters
        self.min_red_line_width = 5
        self.min_red_line_length = 50
    
    def detect_balls(self, frame):
        """Detect white and orange table tennis balls"""
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Detect white balls (any hue, but very low saturation = nearly white/gray)
        lower_white = np.array([0, 0, 150])
        upper_white = np.array([180, 50, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        
        # Detect orange balls. The prior range was too narrow for lighting variation,
        # so keep the hue centered on orange but broaden saturation/value.
        lower_orange = np.array([5, 60, 80])
        upper_orange = np.array([25, 255, 255])
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)

        # Suppress edge glare from the frame border before contouring.
        mask_white[:self.border_margin, :] = 0
        mask_white[-self.border_margin:, :] = 0
        mask_white[:, :self.border_margin] = 0
        mask_white[:, -self.border_margin:] = 0

        mask_orange[:self.border_margin, :] = 0
        mask_orange[-self.border_margin:, :] = 0
        mask_orange[:, :self.border_margin] = 0
        mask_orange[:, -self.border_margin:] = 0
        
        # Pass the HSV frame into the per-contour checks so we can validate
        # color consistency inside each detected contour rather than discarding
        # detections solely by position.
        white_candidates = self._find_ball_centers(mask_white, "WHITE", frame.shape[:2], hsv=hsv, min_circularity=0.45)
        orange_candidates = self._find_ball_centers(mask_orange, "ORANGE", frame.shape[:2], hsv=hsv, min_circularity=0.45)

        white_balls = self._update_stable_ball_tracks(white_candidates, "WHITE")
        orange_balls = self._update_stable_ball_tracks(orange_candidates, "ORANGE")

        # Build filtered masks that contain only accepted ball detections.
        filtered_white = np.zeros_like(mask_white)
        filtered_orange = np.zeros_like(mask_orange)
        for ball in white_balls:
            cv2.circle(filtered_white, (ball['x'], ball['y']), ball['radius'], 255, -1)
        for ball in orange_balls:
            cv2.circle(filtered_orange, (ball['x'], ball['y']), ball['radius'], 255, -1)
        
        all_balls = white_balls + orange_balls
        
        return all_balls, filtered_white, filtered_orange

    def _update_stable_ball_tracks(self, detections, color_name):
        """Keep only detections that stay consistent across frames."""
        tracks = self._ball_tracks[color_name]
        updated_tracks = []
        matched_track_indices = set()

        for detection in detections:
            best_index = None
            best_distance = None

            for index, track in enumerate(tracks):
                if index in matched_track_indices:
                    continue

                distance = np.hypot(detection['x'] - track['x'], detection['y'] - track['y'])
                if distance <= self.ball_match_distance and (best_distance is None or distance < best_distance):
                    best_index = index
                    best_distance = distance

            if best_index is not None:
                track = tracks[best_index].copy()
                track['x'] = int(round(track['x'] * 0.6 + detection['x'] * 0.4))
                track['y'] = int(round(track['y'] * 0.6 + detection['y'] * 0.4))
                track['radius'] = int(round(track['radius'] * 0.6 + detection['radius'] * 0.4))
                track['area'] = detection['area']
                track['circularity'] = detection['circularity']
                track['fill_ratio'] = detection['fill_ratio']
                track['solidity'] = detection['solidity']
                track['hits'] += 1
                track['streak'] = track.get('streak', 0) + 1
                track['misses'] = 0
                matched_track_indices.add(best_index)
                updated_tracks.append(track)
            else:
                updated_tracks.append({
                    'x': detection['x'],
                    'y': detection['y'],
                    'radius': detection['radius'],
                    'area': detection['area'],
                    'color': color_name,
                    'circularity': detection['circularity'],
                    'fill_ratio': detection['fill_ratio'],
                    'solidity': detection['solidity'],
                    'hits': 1,
                    'streak': 1,
                    'misses': 0,
                })

        for index, track in enumerate(tracks):
            if index in matched_track_indices:
                continue

            track = track.copy()
            track['misses'] += 1
            track['streak'] = 0
            if track['misses'] <= self.ball_miss_frames:
                updated_tracks.append(track)

        self._ball_tracks[color_name] = updated_tracks

        stable_balls = []
        for track in updated_tracks:
            if track['streak'] >= self.ball_confirm_frames and track['misses'] == 0:
                stable_balls.append({
                    'x': track['x'],
                    'y': track['y'],
                    'radius': track['radius'],
                    'area': track['area'],
                    'color': track['color'],
                    'circularity': track['circularity'],
                    'fill_ratio': track['fill_ratio'],
                    'solidity': track['solidity'],
                })

        return stable_balls
    
    def _find_ball_centers(self, mask, color_name, frame_shape, hsv=None, min_circularity=0.5):
        """Find ball centers in mask using circularity, size, and border filtering"""
        balls = []
        frame_h, frame_w = frame_shape
        
        # Apply morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Determine which size range to use
        if color_name == "WHITE":
            min_radius, max_radius = self.white_ball_radius_range
        else:  # ORANGE
            min_radius, max_radius = self.orange_ball_radius_range
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_ball_area:
                # Fit circle to contour
                (x, y), radius = cv2.minEnclosingCircle(contour)

                # Ignore bright blobs that touch the frame edge; these are usually glare.
                x0, y0, bw, bh = cv2.boundingRect(contour)
                if (
                    x0 <= self.border_margin or
                    y0 <= self.border_margin or
                    x0 + bw >= frame_w - self.border_margin or
                    y0 + bh >= frame_h - self.border_margin
                ):
                    continue
                
                # Check if radius is within ball size range (fixed size balls)
                if min_radius <= radius <= max_radius:
                    # Calculate circularity: 4π*area / perimeter²
                    perimeter = cv2.arcLength(contour, True)
                    if perimeter > 0:
                        circularity = (4 * np.pi * area) / (perimeter * perimeter)
                    else:
                        circularity = 0

                    circle_area = np.pi * radius * radius
                    fill_ratio = area / circle_area if circle_area > 0 else 0
                    hull = cv2.convexHull(contour)
                    hull_area = cv2.contourArea(hull)
                    solidity = area / hull_area if hull_area > 0 else 0
                    
                    # Only accept if circularity is high enough
                    # Real balls are compact and fairly full circles; glare tends to be thin or ragged.
                    if color_name == "ORANGE":
                        # Validate mean HSV inside the contour to avoid rim/X reflections
                        if hsv is not None:
                            contour_mask = np.zeros(mask.shape, dtype=np.uint8)
                            cv2.drawContours(contour_mask, [contour], -1, 255, -1)
                            mean_h, mean_s, mean_v, _ = cv2.mean(hsv, mask=contour_mask)
                            # Hue in OpenCV is 0-179. Orange ~5-25. Require decent saturation/value.
                            if not (4 <= mean_h <= 28 and mean_s >= 60 and mean_v >= 70):
                                continue
                        if circularity >= min_circularity and fill_ratio >= 0.35 and solidity >= 0.75:
                            balls.append({
                                'x': int(x),
                                'y': int(y),
                                'radius': int(radius),
                                'area': area,
                                'color': color_name,
                                'circularity': circularity,
                                'fill_ratio': fill_ratio,
                                'solidity': solidity
                            })
                    else:
                        if circularity >= min_circularity and fill_ratio >= 0.45 and solidity >= 0.85:
                            balls.append({
                                'x': int(x),
                                'y': int(y),
                                'radius': int(radius),
                                'area': area,
                                'color': color_name,
                                'circularity': circularity,
                                'fill_ratio': fill_ratio,
                                'solidity': solidity
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
        
        # Red wraps around the HSV hue axis, so combine low-hue and high-hue bands.
        lower_red_1 = np.array([0, 80, 70])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([170, 80, 70])
        upper_red_2 = np.array([179, 255, 255])
        mask_red_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask_red_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)
        
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
          S           - take screenshot (saved with unique name)
          Q           - quit
        """
        print("Ball Hunting Navigator")
        print("=" * 50)
        print("  LEFT-CLICK  : set robot start position")
        print("  SPACE       : analyse frame and generate mission")
        print("  D           : show planned path overlay")
        print("  S           : take screenshot")
        print("  M           : toggle mask windows")
        print("  Q           : quit")
        print("  (Mask windows show color detection - WHITE, ORANGE, RED)")
        print()

        robot_pos = None       # set by mouse click
        last_frame = None      # freeze frame used for planning
        debug_vis = None       # path overlay image
        masks_visible = True   # toggle mask windows on/off
        screenshot_count = self._get_next_screenshot_number() - 1  # counter for unique screenshot names

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
                    "Click to set robot pos" if not robot_pos else "SPACE=plan D=debug M=masks S=screenshot Q=quit"
                )
                cv2.putText(display_frame, status, (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

                cv2.imshow('Ball Detection', display_frame)
                
                # Show color detection masks for debugging (toggle with M key)
                if masks_visible:
                    cv2.imshow('WHITE mask', analysis['white_mask'])
                    cv2.imshow('ORANGE mask', analysis['orange_mask'])
                    cv2.imshow('RED walls', analysis['red_walls']['mask'])

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('m'):
                    masks_visible = not masks_visible
                    if masks_visible:
                        print("Mask windows enabled")
                    else:
                        cv2.destroyWindow('WHITE mask')
                        cv2.destroyWindow('ORANGE mask')
                        cv2.destroyWindow('RED walls')
                        print("Mask windows disabled")
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
                elif key == ord('s'):
                    # Increment screenshot counter and create grouped mask folder
                    screenshot_count += 1
                    screenshot_base = "screenshot_{:03d}".format(screenshot_count)

                    # Create a mask folder per screenshot for grouped masks
                    mask_folder_name = "mask_{:03d}".format(screenshot_count)
                    mask_folder = os.path.join(SCREENSHOT_DIR, mask_folder_name)
                    os.makedirs(mask_folder, exist_ok=True)

                    # Files are saved only inside the per-screenshot mask folder
                    screenshot_file = os.path.join(mask_folder, screenshot_base + ".png")
                    white_mask_file = os.path.join(mask_folder, "white_mask.png")
                    orange_mask_file = os.path.join(mask_folder, "orange_mask.png")
                    red_mask_file = os.path.join(mask_folder, "red_mask.png")

                    # Build a composite image: original frame blended with colored mask overlays
                    # so the saved screenshot shows the masks visually but without UI text.
                    composite = frame.copy()
                    # Ensure masks are uint8 single-channel
                    wmask = analysis['white_mask'] if analysis['white_mask'].dtype == np.uint8 else analysis['white_mask'].astype(np.uint8)
                    omask = analysis['orange_mask'] if analysis['orange_mask'].dtype == np.uint8 else analysis['orange_mask'].astype(np.uint8)
                    rmask = analysis['red_walls']['mask'] if analysis['red_walls']['mask'].dtype == np.uint8 else analysis['red_walls']['mask'].astype(np.uint8)

                    overlay = composite.copy()
                    # White mask -> brightened area (use white color)
                    overlay[wmask > 0] = (255, 255, 255)
                    # Orange mask -> orange tint
                    overlay[omask > 0] = (0, 140, 255)
                    # Red mask -> red tint
                    overlay[rmask > 0] = (0, 0, 255)

                    # Blend overlay onto composite with alpha
                    alpha = 0.45
                    composite = cv2.addWeighted(overlay, alpha, composite, 1 - alpha, 0)

                    # Save the composite screenshot (frame combined with masks)
                    cv2.imwrite(screenshot_file, composite)

                    # Save the annotated display (with overlays/UI) inside the mask folder
                    annotated_in_mask = os.path.join(mask_folder, screenshot_base + "_annotated.png")
                    cv2.imwrite(annotated_in_mask, display_frame)

                    # Save the masks
                    cv2.imwrite(white_mask_file, analysis['white_mask'])
                    cv2.imwrite(orange_mask_file, analysis['orange_mask'])
                    cv2.imwrite(red_mask_file, analysis['red_walls']['mask'])

                    print("Saved screenshots and masks in folder: {}".format(mask_folder))

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
    
    def _get_next_screenshot_number(self):
        """Find the highest existing screenshot number and return the next one"""
        # Look for existing screenshot files inside mask folders (mask_***/screenshot_***.png)
        pattern = os.path.join(SCREENSHOT_DIR, "mask_*", "screenshot_*.png")
        existing = glob.glob(pattern)
        if not existing:
            return 1

        # Extract numbers from filenames
        numbers = []
        for f in existing:
            match = re.search(r'screenshot_(\d+)\.png', os.path.basename(f))
            if match:
                numbers.append(int(match.group(1)))

        return max(numbers) + 1 if numbers else 1


if __name__ == "__main__":
    planner = BallHuntingPlanner(camera_index=CAMERA_INDEX, mission_file=MISSION_FILE)
    planner.run_planning_mode()
