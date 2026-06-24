#!/usr/bin/env python3
"""
Live HSV calibration - library module.
Call run_calibration(cap, color_ranges) to open the trackbar UI.
Modifies color_ranges in-place so the detector sees changes immediately.

Controls panel is customtkinter (modern look on Linux + Windows).
Camera preview still uses cv2.imshow.
"""
import cv2
import numpy as np
import customtkinter as ctk

from config import save_color_ranges

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

_CLAHE = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))


def _to_hsv_norm(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    v = _CLAHE.apply(v)
    return cv2.merge([h, s, v])

COLORS = ['WHITE', 'ORANGE', 'RED']
TINTS  = {'WHITE': (255, 255, 255), 'ORANGE': (0, 140, 255), 'RED': (0, 0, 255)}

CAL_WIN  = 'HSV Calibration'
MASK_WIN = 'Calibration Mask'

COLOR_FG = {'WHITE': '#cccccc', 'ORANGE': '#ff8c00', 'RED': '#ff4444'}


def _positions_for(color, color_ranges):
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
    Open a customtkinter slider panel + OpenCV preview windows to tune HSV ranges live.

    Controls:
      N / Next Color button  – cycle WHITE → ORANGE → RED
      S / Save button        – save to color_ranges.json
      Q / Quit button        – exit calibration
    """
    color_idx  = [0]
    running    = [True]
    after_id   = [None]

    root = ctk.CTk()
    root.title("HSV Controls")
    root.resizable(False, False)
    root.attributes('-topmost', True)

    # ── Color indicator ────────────────────────────────────────────────────────
    color_label = ctk.CTkLabel(root, text=COLORS[0],
                               font=ctk.CTkFont(size=18, weight='bold'),
                               text_color=COLOR_FG[COLORS[0]])
    color_label.pack(pady=(14, 6))

    # ── Sliders ────────────────────────────────────────────────────────────────
    slider_defs = [
        ('H min', 179), ('H max', 179),
        ('S min', 255), ('S max', 255),
        ('V min', 255), ('V max', 255),
    ]
    sliders     = {}
    value_vars  = {}

    slider_frame = ctk.CTkFrame(root)
    slider_frame.pack(fill='x', padx=16, pady=4)

    for name, max_val in slider_defs:
        row = ctk.CTkFrame(slider_frame, fg_color='transparent')
        row.pack(fill='x', padx=4, pady=3)

        ctk.CTkLabel(row, text=name, width=52, anchor='w',
                     font=ctk.CTkFont(size=12)).pack(side='left')

        val_label = ctk.CTkLabel(row, text='0', width=34,
                                 font=ctk.CTkFont(size=12))
        val_label.pack(side='right')

        slider = ctk.CTkSlider(row, from_=0, to=max_val, width=280,
                               number_of_steps=max_val)
        slider.pack(side='left', fill='x', expand=True, padx=(4, 4))

        def _on_change(v, lbl=val_label):
            lbl.configure(text=str(int(v)))
        slider.configure(command=_on_change)

        sliders[name] = slider

    # ── Red-channel hint ───────────────────────────────────────────────────────
    hint_label = ctk.CTkLabel(root, text='', font=ctk.CTkFont(size=10),
                              text_color='gray')
    hint_label.pack(pady=(2, 4))

    # ── Buttons ────────────────────────────────────────────────────────────────
    btn_frame = ctk.CTkFrame(root, fg_color='transparent')
    btn_frame.pack(pady=10)

    def _get_values():
        return tuple(int(sliders[n].get()) for n, _ in slider_defs)

    def _load_sliders(color):
        vals = _positions_for(color, color_ranges)
        for (name, _), v in zip(slider_defs, vals):
            sliders[name].set(v)
            sliders[name]._command(v)  # noqa: SLF001
        hint_label.configure(
            text="H min = top of low band   H max = bottom of high band"
            if color == 'RED' else "")

    def do_next():
        color = COLORS[color_idx[0]]
        _save(color, color_ranges, *_get_values())
        color_idx[0] = (color_idx[0] + 1) % len(COLORS)
        color = COLORS[color_idx[0]]
        color_label.configure(text=color, text_color=COLOR_FG[color])
        _load_sliders(color)
        print("Switched to:", color)

    def do_save():
        _save(COLORS[color_idx[0]], color_ranges, *_get_values())

    def do_quit():
        running[0] = False
        if after_id[0] is not None:
            root.after_cancel(after_id[0])
        root.destroy()

    ctk.CTkButton(btn_frame, text="Next Color  (N)", command=do_next,
                  width=140).pack(side='left', padx=6)
    ctk.CTkButton(btn_frame, text="Save  (S)", command=do_save,
                  width=90).pack(side='left', padx=6)
    ctk.CTkButton(btn_frame, text="Quit  (Q)", command=do_quit,
                  width=90, fg_color='#6b2020', hover_color='#8b3030').pack(side='left', padx=6)

    root.bind('<n>', lambda e: do_next())
    root.bind('<N>', lambda e: do_next())
    root.bind('<s>', lambda e: do_save())
    root.bind('<S>', lambda e: do_save())
    root.bind('<q>', lambda e: do_quit())
    root.bind('<Q>', lambda e: do_quit())

    # ── Camera loop via after() ────────────────────────────────────────────────
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
            after_id[0] = root.after(30, camera_tick)
            return

        frame = cv2.flip(frame, 1)
        color = COLORS[color_idx[0]]
        h_min, h_max, s_min, s_max, v_min, v_max = _get_values()

        hsv = _to_hsv_norm(frame)
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
        cv2.putText(display, "CALIBRATE: {}".format(color),
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
        cv2.waitKey(1)

        after_id[0] = root.after(30, camera_tick)

    after_id[0] = root.after(30, camera_tick)
    root.mainloop()

    cv2.destroyWindow(CAL_WIN)
    cv2.destroyWindow(MASK_WIN)
