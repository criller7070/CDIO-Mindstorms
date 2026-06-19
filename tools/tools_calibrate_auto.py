#!/usr/bin/env python3
"""Run automatic HSV calibration — no clicking required.

Usage:  python tools/tools_calibrate_auto.py
"""
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
from host.config import load_color_ranges, CAMERA_INDEX
from host.calibration_auto import run_auto_calibration

cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cr = load_color_ranges()
run_auto_calibration(cap, cr)
cap.release()
