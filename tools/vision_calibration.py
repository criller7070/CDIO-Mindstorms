#!/usr/bin/env python3
"""
Live HSV calibration tool.
Call run_calibration(cap, color_ranges) to open the trackbar UI.
Modifies color_ranges in-place so the detector sees changes immediately.

Controls panel is tkinter (avoids OpenCV/GTK label rendering bugs on Linux).
Camera preview still uses cv2.imshow.
"""
import cv2
import numpy as np
import tkinter as tk
from vision_config import save_color_ranges

COLORS = ['WHITE', 'ORANGE', 'RED']
TINTS  = {'WHITE': (255, 255, 255), 'ORANGE': (0, 140, 255), 'RED': (0, 0, 255)}

CAL_WIN  = 'HSV Calibration'
MASK_WIN = 'Calibration Mask'


def _positions_for(color, color_ranges):
    """Return (h_min, h_max, s_min, s_max, v_min, v_max) from saved ranges."""
    cr = color_ranges[color]
    if color == 'RED':
        return (
            min(cr['upper'][0],  179),
            min(cr['lower2'][0], 179),
            cr['lower'][1], cr['upper'][1],
            cr['lower'][2], cr['upper'][2],
        )
    lo, hi = cr['lower'], cr['upper']
    return (min(lo[0], 179), min(hi[0], 179), lo[1], hi[1], lo[2], hi[2])


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


def run_calibration(cap, color_ranges):
    """
    Open a tkinter slider panel + OpenCV preview windows to tune HSV ranges live.

    Controls:
      N / Next Color button  – cycle WHITE → ORANGE → RED
      S / Save button        – save to color_ranges.json
      Q / Quit button        – exit calibration
    """
    color_idx = [0]
    running   = [True]

    root = tk.Tk()
    root.title("HSV Controls")
    root.resizable(False, False)
    # Keep the window on top so it doesn't hide behind the camera preview
    root.attributes('-topmost', True)

    # ── Color indicator ────────────────────────────────────────────────────────
    color_label = tk.Label(root, text=COLORS[0], font=('Arial', 15, 'bold'),
                           fg='#2266cc', pady=6)
    color_label.pack()

    # ── Sliders ────────────────────────────────────────────────────────────────
    slider_defs = [
        ('H min', 179), ('H max', 179),
        ('S min', 255), ('S max', 255),
        ('V min', 255), ('V max', 255),
    ]
    sliders = {}
    for name, max_val in slider_defs:
        row = tk.Frame(root)
        row.pack(fill='x', padx=14, pady=2)
        tk.Label(row, text=name, width=7, anchor='w',
                 font=('Arial', 10)).pack(side='left')
        var = tk.IntVar()
        sliders[name] = var
        sc = tk.Scale(row, variable=var, from_=0, to=max_val,
                      orient='horizontal', length=300, showvalue=True,
                      font=('Arial', 9))
        sc.pack(side='left', fill='x', expand=True)

    # ── Red-channel hint ───────────────────────────────────────────────────────
    hint_var = tk.StringVar()
    tk.Label(root, textvariable=hint_var, font=('Arial', 8), fg='gray').pack(pady=(0, 4))

    # ── Buttons ────────────────────────────────────────────────────────────────
    btn_frame = tk.Frame(root)
    btn_frame.pack(pady=8)

    def _get_values():
        return (sliders['H min'].get(), sliders['H max'].get(),
                sliders['S min'].get(), sliders['S max'].get(),
                sliders['V min'].get(), sliders['V max'].get())

    def _load_sliders(color):
        vals = _positions_for(color, color_ranges)
        for (name, _), v in zip(slider_defs, vals):
            sliders[name].set(v)
        hint_var.set("H min = top of low band   H max = bottom of high band"
                     if color == 'RED' else "")

    def do_next():
        color = COLORS[color_idx[0]]
        _save(color, color_ranges, *_get_values())
        color_idx[0] = (color_idx[0] + 1) % len(COLORS)
        color = COLORS[color_idx[0]]
        color_label.config(text=color)
        _load_sliders(color)
        print("Switched to:", color)

    def do_save():
        _save(COLORS[color_idx[0]], color_ranges, *_get_values())

    def do_quit():
        running[0] = False
        root.quit()

    tk.Button(btn_frame, text="Next Color  (N)", command=do_next,
              width=16).pack(side='left', padx=5)
    tk.Button(btn_frame, text="Save  (S)", command=do_save,
              width=10).pack(side='left', padx=5)
    tk.Button(btn_frame, text="Quit  (Q)", command=do_quit,
              width=10).pack(side='left', padx=5)

    # Bind keys on the root window (works even when a slider has focus)
    for key in ('<n>', '<N>'): root.bind(key, lambda e: do_next())
    for key in ('<s>', '<S>'): root.bind(key, lambda e: do_save())
    for key in ('<q>', '<Q>'): root.bind(key, lambda e: do_quit())

    # ── Camera loop via tkinter after() ───────────────────────────────────────
    _load_sliders(COLORS[0])
    print("\n--- HSV Calibration ---")
    print("  N or button : cycle WHITE / ORANGE / RED  (auto-saves current)")
    print("  S or button : save to color_ranges.json")
    print("  Q or button : exit calibration\n")

    def camera_tick():
        if not running[0]:
            return
        ret, frame = cap.read()
        if not ret:
            root.after(30, camera_tick)
            return

        frame = cv2.flip(frame, 1)
        color = COLORS[color_idx[0]]
        h_min, h_max, s_min, s_max, v_min, v_max = _get_values()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if color == 'RED':
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
                    "CALIBRATE: {}".format(color),
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
        cv2.waitKey(1)   # pump OpenCV's event queue so imshow renders

        root.after(30, camera_tick)

    root.after(30, camera_tick)
    root.mainloop()

    cv2.destroyWindow(CAL_WIN)
    cv2.destroyWindow(MASK_WIN)
