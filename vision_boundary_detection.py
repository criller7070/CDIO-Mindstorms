#!/usr/bin/env python3
"""
Vision-based boundary detection for EV3 robot using Logitech camera.
Detects walls and obstacles, sends navigation commands to EV3 via Bluetooth.
"""

import cv2
import numpy as np
import socket
import threading
import time
from collections import deque

class BoundaryDetector:
    def __init__(self, camera_index=0):
        """
        Initialize the boundary detector with camera
        
        Args:
            camera_index: Camera device index (0 for Logitech camera)
        """
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Bluetooth connection
        self.ev3_address = None
        self.sock = None
        self.connected = False
        
        # Detection parameters
        self.wall_threshold = 50          # Distance to wall in pixels from frame edge
        self.min_contour_area = 500       # Minimum contour area to consider as obstacle
        self.frame_history = deque(maxlen=5)
        
        # Color ranges for different wall types (BGR format)
        # Adjust these based on your actual wall colors
        self.lower_wall = np.array([0, 0, 0])      # Black/dark walls
        self.upper_wall = np.array([100, 100, 100])
        
        self.lower_red = np.array([0, 0, 100])     # Red boundaries
        self.upper_red = np.array([50, 50, 255])
        
        self.lower_blue = np.array([100, 0, 0])    # Blue boundaries
        self.upper_blue = np.array([255, 50, 50])
        
    def find_ev3(self):
        """Find EV3 Bluetooth address from Windows registry"""
        import subprocess
        import re
        
        try:
            result = subprocess.run(
                ["reg", "query", "HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Services\\BTHPORT\\Parameters\\Devices"],
                capture_output=True,
                text=True
            )
            addresses = re.findall(r'[0-9A-Fa-f]{12}', result.stdout)
            if addresses:
                # Format as MAC address
                formatted = []
                for addr in addresses:
                    fmt = ':'.join([addr[i:i+2] for i in range(0, 12, 2)])
                    print(f"Found Bluetooth device: {fmt}")
                    formatted.append(fmt)
                return formatted
        except Exception as e:
            print(f"Error finding EV3: {e}")
        
        return []
    
    def connect_to_ev3(self, address):
        """Connect to EV3 via Bluetooth using Windows socket"""
        try:
            # Use Windows native Bluetooth socket (no pybluez needed)
            self.sock = socket.socket(socket.AF_BTH, socket.SOCK_STREAM)
            self.sock.connect((address, 1))
            self.connected = True
            print(f"Connected to EV3 at {address}")
            return True
        except Exception as e:
            print(f"Failed to connect to EV3: {e}")
            self.connected = False
            return False
    
    def send_command(self, command):
        """Send command to EV3"""
        if self.connected and self.sock:
            try:
                self.sock.send(command.encode())
            except Exception as e:
                print(f"Error sending command: {e}")
                self.connected = False
    
    def detect_obstacles(self, frame):
        """
        Detect obstacles and walls in frame
        
        Returns:
            obstacle_info: Dictionary with obstacle positions and distances
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Detect dark walls/obstacles
        mask_dark = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 100]))
        
        # Detect red boundaries
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
        
        # Combine masks
        mask = cv2.bitwise_or(mask_dark, mask_red)
        
        # Apply morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        h, w = frame.shape[:2]
        obstacle_info = {
            'left': False,
            'right': False,
            'center': False,
            'front': False,
            'obstacles': []
        }
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_contour_area:
                x, y, cw, ch = cv2.boundingRect(contour)
                
                # Determine position relative to frame
                center_x = x + cw // 2
                center_y = y + ch // 2
                
                obstacle_info['obstacles'].append({
                    'x': center_x,
                    'y': center_y,
                    'width': cw,
                    'height': ch,
                    'area': area
                })
                
                # Check which regions are blocked
                left_third = w // 3
                right_third = w - w // 3
                
                if center_x < left_third:
                    obstacle_info['left'] = True
                elif center_x > right_third:
                    obstacle_info['right'] = True
                else:
                    obstacle_info['center'] = True
                
                # Front detection (bottom of frame)
                if center_y > h - self.wall_threshold:
                    obstacle_info['front'] = True
        
        return obstacle_info, mask
    
    def calculate_steering_command(self, obstacle_info):
        """
        Calculate steering command based on obstacle positions
        
        Returns:
            command: String command to send to EV3
        """
        if obstacle_info['front']:
            if obstacle_info['left'] and not obstacle_info['right']:
                return "TURN_RIGHT"
            elif obstacle_info['right'] and not obstacle_info['left']:
                return "TURN_LEFT"
            else:
                return "STOP"
        elif obstacle_info['left']:
            return "TURN_RIGHT"
        elif obstacle_info['right']:
            return "TURN_LEFT"
        else:
            return "FORWARD"
    
    def run(self):
        """Main vision processing loop"""
        print("Boundary Detection System Starting...")
        
        # Find and connect to EV3
        addresses = self.find_ev3()
        if addresses:
            self.connect_to_ev3(addresses[0])
        else:
            print("Warning: No EV3 found. Running in visualization-only mode.")
        
        if not self.cap.isOpened():
            print("Error: Cannot open camera")
            return
        
        frame_count = 0
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to read frame")
                    break
                
                frame_count += 1
                
                # Flip frame horizontally for natural mirror view
                frame = cv2.flip(frame, 1)
                
                # Detect obstacles
                obstacle_info, mask = self.detect_obstacles(frame)
                
                # Calculate command every 5 frames to avoid too frequent updates
                if frame_count % 5 == 0:
                    command = self.calculate_steering_command(obstacle_info)
                    print(f"Frame {frame_count}: {command} | Obstacles: L={obstacle_info['left']} C={obstacle_info['center']} R={obstacle_info['right']} F={obstacle_info['front']}")
                    
                    # Send command to EV3
                    if self.connected:
                        self.send_command(command)
                
                # Visualization
                h, w = frame.shape[:2]
                
                # Draw frame divided into regions
                cv2.line(frame, (w // 3, 0), (w // 3, h), (0, 255, 0), 2)
                cv2.line(frame, (2 * w // 3, 0), (2 * w // 3, h), (0, 255, 0), 2)
                cv2.line(frame, (0, h - self.wall_threshold), (w, h - self.wall_threshold), (0, 255, 255), 2)
                
                # Draw detected obstacles
                for obs in obstacle_info['obstacles']:
                    x, y, cw, ch = obs['x'], obs['y'], obs['width'], obs['height']
                    cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
                    cv2.rectangle(frame, (x - cw // 2, y - ch // 2), (x + cw // 2, y + ch // 2), (0, 255, 0), 2)
                
                # Status text
                status = "CONNECTED" if self.connected else "DISCONNECTED"
                cv2.putText(frame, f"Status: {status}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Obstacles: {len(obstacle_info['obstacles'])}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Show frames
                cv2.imshow('Boundary Detection', frame)
                cv2.imshow('Mask', mask)
                
                # Exit on 'q' key
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        self.cap.release()
        cv2.destroyAllWindows()
        print("Shutdown complete")


if __name__ == "__main__":
    detector = BoundaryDetector(camera_index=0)
    detector.run()
