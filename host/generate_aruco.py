#!/usr/bin/env python3
"""
Generate a printable ArUco marker to put on the robot 
"""
import os
import sys
import cv2 # this is where the magic happens

def generate(marker_id=0, size=600):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    img = cv2.aruco.generateImageMarker(d, marker_id, size)
    # Adds a border because QR readers need a border
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
