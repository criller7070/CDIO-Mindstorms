#!/usr/bin/env python3
"""
Generate a printable ArUco marker to mount on the robot for closed-loop tracking.

Usage:
    python generate_aruco_marker.py [marker_id] [pixels]

Defaults: marker_id=0, pixels=600.  Uses DICT_4X4_1000 (matches vision_detector
and chev.me's "4x4 (1000)" generator).
Print it, mount it FLAT on top of the robot with the marker's TOP edge pointing
toward the robot's FRONT, and keep the white border (quiet zone) intact — the
detector needs it.  A bigger physical marker = more reliable detection.
"""
import os
import sys
import cv2
import numpy as np


def generate(marker_id=0, size=600):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    img = cv2.aruco.generateImageMarker(d, marker_id, size)

    # Add a white quiet-zone border. ArUco needs at least one marker module of
    # white around the pattern to isolate it from the background; a DICT_4X4
    # marker is 6 modules across (4 data + 2 black border), so one module is
    # size/6. We use size/4 (~1.5 modules) for robust detection against any
    # field background.
    b = size // 4
    canvas = np.full((size + 2 * b, size + 2 * b), 255, np.uint8)
    canvas[b:b + size, b:b + size] = img

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "robot_marker_{}.png".format(marker_id))
    cv2.imwrite(out, canvas)
    return out


if __name__ == "__main__":
    mid  = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    size = int(sys.argv[2]) if len(sys.argv) > 2 else 600
    path = generate(mid, size)
    print("Wrote {} (id={}, {}px). Mount TOP edge toward robot front.".format(
        path, mid, size))
