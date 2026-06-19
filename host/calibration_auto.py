#!/usr/bin/env python3
"""
Fully automatic HSV calibration — library module.

Algorithm:
  1. RED walls detected with existing range → field boundary established.
     Range is refined with a floor constraint so it can never be worse than
     the current values.
  2. ORANGE balls: existing range as seed + circularity filter inside field.
  3. WHITE balls:  brightness seed + stricter circularity (rejects glare).

Temporal consistency: each colour builds a spatial hit-count map across all
frames. Only pixels in areas detected consistently (>= HIT_THRESHOLD of frames)
contribute to the final range. Single-frame glare and reflections are suppressed.

Press ESC during countdown to abort without saving.
"""
import cv2
import numpy as np

from config import save_color_ranges

# ── Tuning ────────────────────────────────────────────────────────────────────
WHITE_SEED = {'lo': [0, 0, 180], 'hi': [179, 55, 255]}  # wider after CLAHE

# CLAHE normalises V locally — must match the detector (detection._to_hsv).
_CLAHE = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

BALL_MIN_CIRC       = 0.55
BALL_MIN_CIRC_WHITE = 0.70   # stricter — glare passes the brightness seed
BALL_MAX_ASPECT     = 2.0
BALL_MIN_AREA       = 40     # px²
WALL_MIN_AREA       = 300    # px²

H_TOL, S_TOL, V_TOL = 8, 20, 20   # tighter — more frames compensate

COUNTDOWN_SEC  = 3
CAPTURE_FRAMES = 1000

# Pixel location is "reliable" when detected in at least this fraction of frames
HIT_THRESHOLD = 0.15

WIN = 'Auto Calibration'


# ── Mask helpers ──────────────────────────────────────────────────────────────

def _to_hsv_norm(frame):
    """Same normalization as BallDetector._to_hsv — CLAHE on V channel."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    v = _CLAHE.apply(v)
    return cv2.merge([h, s, v])


def _mask_from_range(hsv, cr):
    m = cv2.inRange(hsv, np.array(cr['lower']), np.array(cr['upper']))
    if 'lower2' in cr:
        m = cv2.bitwise_or(m, cv2.inRange(hsv,
                                           np.array(cr['lower2']),
                                           np.array(cr['upper2'])))
    return m


def _white_seed_mask(hsv):
    return cv2.inRange(hsv, np.array(WHITE_SEED['lo']), np.array(WHITE_SEED['hi']))


def _morph(mask, close=5, open_=5):
    k_c = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close, close))
    k_o = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_,  open_))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_c)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k_o)
    return mask


def _circular_blobs(mask, min_circ=BALL_MIN_CIRC):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < BALL_MIN_AREA:
            continue
        perim = cv2.arcLength(c, True)
        circ  = (4 * np.pi * area / perim ** 2) if perim > 0 else 0
        if circ < min_circ:
            continue
        _, _, bw, bh = cv2.boundingRect(c)
        if max(bw, bh) / max(min(bw, bh), 1) > BALL_MAX_ASPECT:
            continue
        out.append(c)
    return out


def _large_blobs(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return [c for c in contours if cv2.contourArea(c) >= WALL_MIN_AREA]


def _hull_from_contours(contours):
    if not contours:
        return None
    return cv2.convexHull(np.vstack(contours))


def _inside_hull(hull, x, y, inset=5):
    if hull is None:
        return True
    return cv2.pointPolygonTest(hull, (float(x), float(y)), True) >= inset


def _contours_inside_hull(contours, hull, inset=5):
    out = []
    for c in contours:
        M = cv2.moments(c)
        if M['m00'] == 0:
            continue
        if _inside_hull(hull, int(M['m10'] / M['m00']), int(M['m01'] / M['m00']), inset):
            out.append(c)
    return out


# ── Range computation ─────────────────────────────────────────────────────────

def _compute_range(pixels, color_name):
    H = pixels[:, 0].astype(float)
    S = pixels[:, 1].astype(float)
    V = pixels[:, 2].astype(float)

    s_lo = int(max(0,   np.percentile(S,  5) - S_TOL))
    s_hi = int(min(255, np.percentile(S, 95) + S_TOL))
    v_lo = int(max(0,   np.percentile(V,  5) - V_TOL))
    v_hi = int(min(255, np.percentile(V, 95) + V_TOL))

    if color_name == 'WHITE':
        return {'lower': [0, s_lo, v_lo], 'upper': [179, s_hi, v_hi]}

    if color_name == 'RED':
        lo_h   = H[H <= 90]
        hi_h   = H[H  > 90]
        h_low  = int(min(30,  np.percentile(lo_h, 95) + H_TOL)) if len(lo_h)  else 10
        h_high = int(max(150, np.percentile(hi_h,  5) - H_TOL)) if len(hi_h)  else 170
        return {
            'lower':  [0,      s_lo, v_lo],
            'upper':  [h_low,  s_hi, v_hi],
            'lower2': [h_high, s_lo, v_lo],
            'upper2': [179,    s_hi, v_hi],
        }

    # ORANGE
    h_lo = int(max(0,   np.percentile(H,  5) - H_TOL))
    h_hi = int(min(179, np.percentile(H, 95) + H_TOL))
    return {'lower': [h_lo, s_lo, v_lo], 'upper': [h_hi, s_hi, v_hi]}


# ── Per-frame analysis ────────────────────────────────────────────────────────

def _analyse_frame(frame, hit_maps, hsv_pools, color_ranges, hsv=None):
    fh, fw = frame.shape[:2]
    if hsv is None:
        hsv = _to_hsv_norm(frame)

    # RED — boundary + accumulate
    red_mask   = _morph(_mask_from_range(hsv, color_ranges['RED']), close=7, open_=5)
    red_cnts   = _large_blobs(red_mask)
    field_hull = _hull_from_contours(red_cnts)
    hit_maps['RED'] += (red_mask > 0).astype(np.float32)
    hsv_pools['RED'].append(hsv[red_mask > 0])

    # ORANGE
    or_mask = _morph(_mask_from_range(hsv, color_ranges['ORANGE']), close=5, open_=5)
    or_mask = cv2.bitwise_and(or_mask, cv2.bitwise_not(red_mask))
    or_cnts = _circular_blobs(or_mask)
    or_cnts = _contours_inside_hull(or_cnts, field_hull, inset=5)
    if or_cnts:
        cmask = np.zeros((fh, fw), np.uint8)
        cv2.drawContours(cmask, or_cnts, -1, 255, -1)
        hit_maps['ORANGE'] += (cmask > 0).astype(np.float32)
        hsv_pools['ORANGE'].append(hsv[cmask > 0])

    # WHITE
    wh_mask = _morph(_white_seed_mask(hsv), close=5, open_=5)
    wh_cnts = _circular_blobs(wh_mask, min_circ=BALL_MIN_CIRC_WHITE)
    wh_cnts = _contours_inside_hull(wh_cnts, field_hull, inset=5)
    if wh_cnts:
        cmask = np.zeros((fh, fw), np.uint8)
        cv2.drawContours(cmask, wh_cnts, -1, 255, -1)
        hit_maps['WHITE'] += (cmask > 0).astype(np.float32)
        hsv_pools['WHITE'].append(hsv[cmask > 0])

    return red_mask


# ── Main entry point ──────────────────────────────────────────────────────────

def run_auto_calibration(cap, color_ranges, headless=False):
    if not headless:
        cv2.namedWindow(WIN)

    import time
    deadline = time.time() + COUNTDOWN_SEC
    print("\n--- Auto Calibration ---")
    print("Keep ALL objects visible. Calibrating in {}s...".format(COUNTDOWN_SEC))

    # ── Countdown preview ────────────────────────────────────────────────────
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame   = cv2.flip(frame, 1)
        remaining = max(0.0, deadline - time.time())
        if not headless:
            preview = frame.copy()
            hsv = _to_hsv_norm(frame)
            preview[_mask_from_range(hsv, color_ranges['RED'])    > 0] = (80,  80,  255)
            preview[_mask_from_range(hsv, color_ranges['ORANGE']) > 0] = (0,  140,  255)
            preview[_white_seed_mask(hsv)                         > 0] = (200, 200, 200)
            cv2.putText(preview,
                        "Auto-calibrating in {:.0f}s — keep field in view | ESC=abort".format(
                            remaining),
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow(WIN, cv2.addWeighted(preview, 0.5, frame, 0.5, 0))
            if cv2.waitKey(1) & 0xFF == 27:
                cv2.destroyWindow(WIN)
                print("Aborted.")
                return {}
        else:
            print("  {:.0f}s remaining...".format(remaining), end='\r')
        if time.time() >= deadline:
            break

    # ── Capture ───────────────────────────────────────────────────────────────
    ret, probe = cap.read()
    if not ret:
        cv2.destroyWindow(WIN)
        return {}
    fh, fw = probe.shape[:2]

    hit_maps     = {c: np.zeros((fh, fw), np.float32) for c in ('RED', 'ORANGE', 'WHITE')}
    hsv_pools    = {c: [] for c in ('RED', 'ORANGE', 'WHITE')}
    sampled_hsvs = []
    SAMPLE_EVERY = 20   # 50 samples across 1000 frames ≈ 45 MB

    for i in range(CAPTURE_FRAMES):
        ret, frame = cap.read()
        if not ret:
            break
        frame    = cv2.flip(frame, 1)
        hsv      = _to_hsv_norm(frame)
        red_mask = _analyse_frame(frame, hit_maps, hsv_pools, color_ranges, hsv)

        if i % SAMPLE_EVERY == 0:
            sampled_hsvs.append(hsv)

        pct = int((i + 1) / CAPTURE_FRAMES * 100)
        if not headless:
            preview = frame.copy()
            preview[red_mask > 0] = (80, 80, 255)
            cv2.putText(preview, "Analysing... {}%  (temporal filtering active)".format(pct),
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow(WIN, cv2.addWeighted(preview, 0.5, frame, 0.5, 0))
            cv2.waitKey(1)
        elif pct % 10 == 0:
            print("  {}% captured...".format(pct))

    # ── Temporal filter + compute ranges ─────────────────────────────────────
    threshold_count = HIT_THRESHOLD * CAPTURE_FRAMES
    confirmed = {}
    sanity    = {'ORANGE': (0, 50), 'WHITE': (0, 179)}

    for color_name in ('RED', 'ORANGE', 'WHITE'):
        raw_pool = hsv_pools[color_name]
        if not raw_pool:
            print("  {} : nothing detected — keeping existing range".format(color_name))
            continue

        reliable = (hit_maps[color_name] >= threshold_count).astype(np.uint8)
        reliable_pixel_count = int(reliable.sum())
        if reliable_pixel_count < 50:
            print("  {} : too few reliable pixels ({}) — keeping existing".format(
                color_name, reliable_pixel_count))
            continue

        if sampled_hsvs:
            px_parts  = [h[reliable > 0] for h in sampled_hsvs]
            stable_px = np.vstack([p for p in px_parts if len(p) > 0])
        else:
            cutoff    = max(1, len(raw_pool) * 3 // 4)
            stable_px = np.vstack(raw_pool[cutoff:]) if len(raw_pool) > 4 else np.vstack(raw_pool)

        if len(stable_px) < 50:
            print("  {} : too few stable pixels — keeping existing".format(color_name))
            continue

        new_range = _compute_range(stable_px, color_name)

        if color_name in sanity:
            h_c = (new_range['lower'][0] + new_range['upper'][0]) / 2
            lo, hi = sanity[color_name]
            if not (lo <= h_c <= hi):
                print("  {} : sanity FAIL (H={:.0f}) — keeping existing".format(
                    color_name, h_c))
                continue

        if color_name == 'RED':
            ex = color_ranges['RED']
            new_range['lower'][1] = max(new_range['lower'][1], ex['lower'][1])
            new_range['lower'][2] = max(new_range['lower'][2], ex['lower'][2])
            if 'lower2' in ex and 'lower2' in new_range:
                new_range['lower2'][1] = max(new_range['lower2'][1], ex['lower2'][1])
                new_range['lower2'][2] = max(new_range['lower2'][2], ex['lower2'][2])

        confirmed[color_name] = new_range
        print("  {} : lower={}  upper={}  ({} reliable px)".format(
            color_name, new_range['lower'], new_range['upper'], len(stable_px)))

    # ── Apply & save ──────────────────────────────────────────────────────────
    if confirmed:
        for color_name, ranges in confirmed.items():
            if color_name not in color_ranges:
                color_ranges[color_name] = {}
            for k, v in ranges.items():
                color_ranges[color_name][k] = v
        save_color_ranges(color_ranges)
        print("Calibration saved: {}".format(list(confirmed.keys())))
    else:
        print("Nothing detected — calibration unchanged.")

    # ── Result display ────────────────────────────────────────────────────────
    if not headless:
        ret, frame = cap.read()
        if ret:
            frame  = cv2.flip(frame, 1)
            result = frame.copy()
            hsv    = _to_hsv_norm(frame)
            tints  = {'RED': (80,80,255), 'ORANGE': (0,140,255), 'WHITE': (200,200,200)}
            for color_name, ranges in confirmed.items():
                m = cv2.inRange(hsv, np.array(ranges['lower']), np.array(ranges['upper']))
                if 'lower2' in ranges:
                    m = cv2.bitwise_or(m, cv2.inRange(
                        hsv, np.array(ranges['lower2']), np.array(ranges['upper2'])))
                result[m > 0] = tints.get(color_name, (255, 255, 255))
            msg = "Done: {}  |  Press any key".format(', '.join(confirmed.keys())) \
                  if confirmed else "Nothing found  |  Press any key"
            cv2.putText(result, msg, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow(WIN, cv2.addWeighted(result, 0.5, frame, 0.5, 0))
            cv2.waitKey(0)
        cv2.destroyWindow(WIN)
    return confirmed
