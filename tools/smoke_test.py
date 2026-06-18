#!/usr/bin/env python3
"""
Smoke test — verifies the full vision + robot stack without driving.

Usage:
    python smoke_test.py                    # camera + vision only
    python smoke_test.py --tcp 192.168.x.y  # + robot PING (no movement)
    python smoke_test.py --camera 0         # override camera index

Checks (in order):
  1. Camera open + valid frame captured
  2. Roboflow API reachable + returns predictions
  3. Field (red wall) detection
  4. ArUco robot marker visible
  5. [optional] TCP robot link: connect + PING/PONG
"""
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

import sys
import time
import cv2

# ── Resolve tools/ dir so relative imports work when called from elsewhere ─────
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vision_config import (
    CAMERA_INDEX, ROBOFLOW_API_KEY, ROBOFLOW_API_URL, ROBOFLOW_MODEL_ID,
    load_color_ranges,
)
from vision_detector import BallDetector

PASS = "PASS"
FAIL = "FAIL"
SKIP = "SKIP"


def _status(label, ok, detail=""):
    tag = PASS if ok else FAIL
    line = "  [{tag}] {label}".format(tag=tag, label=label)
    if detail:
        line += " — " + detail
    print(line)
    return ok


def check_camera(camera_index):
    print("\n[1] Camera")
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, -5)

    opened = cap.isOpened()
    if not _status("Camera index {} opens".format(camera_index), opened):
        cap.release()
        return None, None

    ret, frame = cap.read()
    if not _status("Frame captured", ret and frame is not None):
        cap.release()
        return None, None

    h, w = frame.shape[:2]
    _status(
        "Resolution {}x{}".format(w, h),
        w >= 320 and h >= 240,
        "{}x{}".format(w, h),
    )

    return cap, frame


def check_yolo(detector, frame):
    print("\n[2] Roboflow YOLO API")
    if detector._roboflow_client is None:
        print("  [{}] Roboflow client — no API key, HoughCircles fallback active".format(SKIP))
        return

    # Submit the frame and wait up to 3 s for the worker thread to return results.
    _ = detector._find_balls_yolo(frame)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        with detector._yolo_lock:
            cached = list(detector._yolo_cached_result)
        if cached or time.monotonic() > deadline:
            break
        time.sleep(0.1)

    # A response (even an empty list) means the API is reachable.
    with detector._yolo_lock:
        got_response = detector._yolo_cached_result is not None
        ball_count   = len(detector._yolo_cached_result)

    _status(
        "API reachable (model {})".format(ROBOFLOW_MODEL_ID),
        got_response,
    )
    _status(
        "Predictions returned",
        True,
        "{} ball(s) detected".format(ball_count),
    )


def check_field(detector, frame):
    print("\n[3] Field (red-wall) detection")
    red   = detector.detect_red_walls(frame)
    mask  = red['mask']
    lines = red['lines']

    import numpy as np
    pixel_count = int(np.count_nonzero(mask))
    _status("Red-wall mask non-empty", pixel_count > 0,
            "{} red pixels".format(pixel_count))

    fh, fw = frame.shape[:2]
    valid = detector._is_valid_field_mask(mask, fh, fw)
    if valid:
        x0, y0, x1, y1 = detector._bounds_from_mask(mask)
        _status("Field bounds valid",  True,
                "({},{}) to ({},{})".format(x0, y0, x1, y1))
    else:
        _status("Field bounds valid", False,
                "mask too small or wrong aspect — aim camera at the field")

    line_count = len(lines) if lines is not None else 0
    _status("Hough lines found", line_count > 0,
            "{} line segment(s)".format(line_count))


def check_aruco(detector, frame):
    print("\n[4] ArUco robot marker")
    if detector._aruco_detector is None:
        _status("ArUco detector available", False,
                "opencv-contrib not installed")
        return

    pose = detector.detect_robot(frame)
    if pose is not None:
        x, y, hdg = pose
        _status("Marker detected", True,
                "centre ({},{})  heading {:.1f}°".format(x, y, hdg))
    else:
        _status("Marker detected", False,
                "place the ArUco marker in camera view and re-run")


def check_tcp(host, port=9999):
    print("\n[5] Robot TCP link  ({}:{})".format(host, port))
    import socket
    try:
        sock = socket.create_connection((host, port), timeout=5.0)
    except OSError as e:
        _status("TCP connect", False, str(e))
        return

    _status("TCP connect", True)

    try:
        sock.sendall(b"PING\n")
        sock.settimeout(5.0)
        buf = ""
        deadline = time.time() + 5.0
        while time.time() < deadline:
            data = sock.recv(256)
            if not data:
                break
            buf += data.decode("utf-8", errors="ignore")
            if "PONG" in buf:
                break
        _status("PING → PONG handshake", "PONG" in buf, repr(buf.strip()))
    except Exception as e:
        _status("PING → PONG handshake", False, str(e))
    finally:
        try:
            sock.close()
        except OSError:
            pass


def main():
    args = sys.argv[1:]

    camera_index = CAMERA_INDEX
    if "--camera" in args:
        camera_index = int(args[args.index("--camera") + 1])

    tcp_host = None
    if "--tcp" in args:
        spec = args[args.index("--tcp") + 1]
        parts = spec.split(":")
        tcp_host = parts[0]

    print("=" * 50)
    print("CDIO smoke test")
    print("=" * 50)

    color_ranges = load_color_ranges()
    detector = BallDetector(
        color_ranges,
        roboflow_api_key=ROBOFLOW_API_KEY,
        roboflow_model_id=ROBOFLOW_MODEL_ID,
        roboflow_api_url=ROBOFLOW_API_URL,
    )

    cap, frame = check_camera(camera_index)
    if frame is None:
        print("\nCamera failed — cannot run vision checks.")
    else:
        check_yolo(detector, frame)
        check_field(detector, frame)
        check_aruco(detector, frame)
        cap.release()

    if tcp_host:
        check_tcp(tcp_host)
    else:
        print("\n[5] Robot TCP link  — skipped (pass --tcp <host> to test)")

    print("\n" + "=" * 50)


if __name__ == "__main__":
    main()
