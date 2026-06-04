#!/usr/bin/env python3
"""
Live HSV calibration tool.
Call run_calibration(cap, color_ranges) to open the trackbar UI.
Modifies color_ranges in-place so the detector sees changes immediately.
"""
import cv2
import numpy as np
from vision_config import save_color_ranges

COLORS = ['WHITE', 'ORANGE', 'RED']
TINTS  = {'WHITE': (255, 255, 255), 'ORANGE': (0, 140, 255), 'RED': (0, 0, 255)}

CAL_WIN  = 'HSV Calibration'
MASK_WIN = 'Calibration Mask'


def run_calibration(cap, color_ranges):
    """
    Open a trackbar window to tune HSV ranges live.

    Controls:
      TAB  – cycle  WHITE → ORANGE → RED
      S    – save to color_ranges.json (detector picks it up instantly)
      Q    – exit calibration
    """
    color_idx = [0]

    cv2.namedWindow(CAL_WIN)
    cv2.namedWindow(MASK_WIN)

    def nothing(_):
        pass

    def _positions_for(color):
        """Return (h_min, h_max, s_min, s_max, v_min, v_max) from saved ranges."""
        cr = color_ranges[color]
        if color == 'RED':
            return (
                min(cr['upper'][0],  179),   # H min = top of low band
                min(cr['lower2'][0], 179),   # H max = bottom of high band
                cr['lower'][1], cr['upper'][1],
                cr['lower'][2], cr['upper'][2],
            )
        lo, hi = cr['lower'], cr['upper']
        return (min(lo[0], 179), min(hi[0], 179), lo[1], hi[1], lo[2], hi[2])

    # Create trackbars at the saved starting values for the first color
    init = _positions_for(COLORS[0])
    for (name, max_val), val in zip(
        [('H min', 179), ('H max', 179),
         ('S min', 255), ('S max', 255),
         ('V min', 255), ('V max', 255)],
        init,
    ):
        cv2.createTrackbar(name, CAL_WIN, int(val), max_val, nothing)

    def _load_trackbars(color):
        h_min, h_max, s_min, s_max, v_min, v_max = _positions_for(color)
        for name, val in [('H min', h_min), ('H max', h_max),
                          ('S min', s_min), ('S max', s_max),
                          ('V min', v_min), ('V max', v_max)]:
            cv2.setTrackbarPos(name, CAL_WIN, val)

    def _get_trackbars():
        return {k: cv2.getTrackbarPos(k, CAL_WIN)
                for k in ('H min', 'H max', 'S min', 'S max', 'V min', 'V max')}

    print("\n--- HSV Calibration ---")
    print("  TAB  : cycle WHITE / ORANGE / RED")
    print("  S    : save to color_ranges.json")
    print("  Q    : exit calibration")

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.flip(frame, 1)

        color = COLORS[color_idx[0]]
        tb    = _get_trackbars()
        h_min, h_max = tb['H min'], tb['H max']
        s_min, s_max = tb['S min'], tb['S max']
        v_min, v_max = tb['V min'], tb['V max']

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if color == 'RED':
            # Red wraps: H min = upper bound of low band, H max = lower bound of high band
            m1   = cv2.inRange(hsv, np.array([0,     s_min, v_min]),
                                    np.array([h_min, s_max, v_max]))
            m2   = cv2.inRange(hsv, np.array([h_max, s_min, v_min]),
                                    np.array([179,   s_max, v_max]))
            mask = cv2.bitwise_or(m1, m2)
            h_label = "H:0-{}  and  {}-179".format(h_min, h_max)
        else:
            mask    = cv2.inRange(hsv,
                                  np.array([h_min, s_min, v_min]),
                                  np.array([h_max, s_max, v_max]))
            h_label = "H:{}-{}".format(h_min, h_max)

        overlay = frame.copy()
        overlay[mask > 0] = TINTS[color]
        display = cv2.addWeighted(overlay, 0.5, frame, 0.5, 0)

        fh = display.shape[0]
        cv2.putText(display,
                    "CALIBRATE: {} | TAB=switch  S=save  Q=quit".format(color),
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(display,
                    "{}  S:{}-{}  V:{}-{}".format(h_label, s_min, s_max, v_min, v_max),
                    (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1)
        if color == 'RED':
            cv2.putText(display,
                        "RED: H min = top of low band   H max = bottom of high band",
                        (10, fh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1)

        cv2.imshow(CAL_WIN, display)
        cv2.imshow(MASK_WIN, mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == 9:  # TAB
            color_idx[0] = (color_idx[0] + 1) % len(COLORS)
            _load_trackbars(COLORS[color_idx[0]])
            print("Switched to: {}".format(COLORS[color_idx[0]]))
        elif key == ord('s'):
            _save(color, color_ranges, h_min, h_max, s_min, s_max, v_min, v_max)

    cv2.destroyWindow(CAL_WIN)
    cv2.destroyWindow(MASK_WIN)


def _save(color, color_ranges, h_min, h_max, s_min, s_max, v_min, v_max):
    if color == 'RED':
        color_ranges['RED']['lower']  = [0,     s_min, v_min]
        color_ranges['RED']['upper']  = [h_min, s_max, v_max]
        color_ranges['RED']['lower2'] = [h_max, s_min, v_min]
        color_ranges['RED']['upper2'] = [179,   s_max, v_max]
    else:
        color_ranges[color]['lower'] = [h_min, s_min, v_min]
        color_ranges[color]['upper'] = [h_max, s_max, v_max]
        if color == 'ORANGE':
            color_ranges['ORANGE']['mean_h_min'] = max(0,   h_min - 2)
            color_ranges['ORANGE']['mean_h_max'] = min(179, h_max + 5)

    save_color_ranges(color_ranges)
    print("Saved {} → lower={} upper={}".format(
        color, color_ranges[color]['lower'], color_ranges[color]['upper']))
