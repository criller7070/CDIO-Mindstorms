#!/usr/bin/env python3
"""
Ball and field detection engine. No UI code lives here.
Receives a shared color_ranges dict; mutations to it (e.g. from calibration)
are reflected immediately because Python dicts are passed by reference.
"""
import cv2
import numpy as np


class BallDetector:
    def __init__(self, color_ranges):
        self.color_ranges = color_ranges

        # Detection parameters (all shape gates are in color_ranges)
        self.min_ball_area      = 10
        self.border_margin      = 0
        self.ball_confirm_frames = 7
        self.ball_miss_frames   = 6
        self.ball_match_distance = 18
        self._ball_tracks       = {"WHITE": [], "ORANGE": []}
        self.min_red_line_length = 50

    # ── Public API ────────────────────────────────────────────────────────────

    # pixels a ball centre must be inside the red-wall bounding box.
    # Raises it enough to exclude detections sitting ON the wall surface.
    FIELD_INSET = 10

    def analyze_course(self, frame):
        """Run all detections and return a single result dict."""
        balls, mask_white, mask_orange = self.detect_balls(frame)
        red_walls    = self.detect_red_walls(frame)
        field_bounds = self.detect_field_bounds(frame)
        fh, fw = frame.shape[:2]

        # Keep only balls whose centre is at least FIELD_INSET pixels inside the
        # convex hull of all red wall pixels.  pointPolygonTest returns a signed
        # distance: positive = inside, negative = outside.  Using the hull (not a
        # bounding rect) means the check works at any camera angle and handles
        # gaps in the red-wall detection more gracefully than a rectangle would.
        red_cnts, _ = cv2.findContours(red_walls['mask'],
                                        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if red_cnts:
            hull = cv2.convexHull(np.vstack(red_cnts))
            balls = [b for b in balls
                     if cv2.pointPolygonTest(
                         hull, (float(b['x']), float(b['y'])), True) >= self.FIELD_INSET]

        return {
            'balls':        balls,
            'white_mask':   mask_white,
            'orange_mask':  mask_orange,
            'red_walls':    red_walls,
            'field_bounds': field_bounds,
            'frame_h': fh,
            'frame_w': fw,
        }

    def detect_balls(self, frame):
        """Return (balls, raw_white_mask, raw_orange_mask)."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        cr_w = self.color_ranges['WHITE']
        mask_white  = cv2.inRange(hsv, np.array(cr_w['lower']), np.array(cr_w['upper']))

        cr_o = self.color_ranges['ORANGE']
        mask_orange = cv2.inRange(hsv, np.array(cr_o['lower']), np.array(cr_o['upper']))

        # Save raw masks before any modification — these match the calibration view.
        raw_white  = mask_white.copy()
        raw_orange = mask_orange.copy()

        if self.border_margin > 0:
            m = self.border_margin
            for mask in (mask_white, mask_orange):
                mask[:m, :] = 0;  mask[-m:, :] = 0
                mask[:, :m] = 0;  mask[:, -m:] = 0

        white_balls  = self._update_stable_ball_tracks(
            self._find_ball_centers(mask_white,  "WHITE",  frame.shape[:2], hsv), "WHITE")
        orange_balls = self._update_stable_ball_tracks(
            self._find_ball_centers(mask_orange, "ORANGE", frame.shape[:2], hsv), "ORANGE")

        return white_balls + orange_balls, raw_white, raw_orange

    def detect_red_walls(self, frame):
        """Return dict with mask, edges, and Hough lines for the red boundary."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cr  = self.color_ranges['RED']
        m1  = cv2.inRange(hsv, np.array(cr['lower']),  np.array(cr['upper']))
        m2  = cv2.inRange(hsv, np.array(cr['lower2']), np.array(cr['upper2']))
        mask = cv2.bitwise_or(m1, m2)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50,
                                minLineLength=self.min_red_line_length, maxLineGap=10)
        return {'mask': mask, 'edges': edges, 'lines': lines}

    def detect_field_bounds(self, frame):
        """Return (x_min, y_min, x_max, y_max) pixel bounds of the red field rectangle."""
        mask = self.detect_red_walls(frame)['mask']
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            x, y, w, h = cv2.boundingRect(np.vstack(contours))
            return (x, y, x + w, y + h)
        fh, fw = frame.shape[:2]
        return (20, 20, fw - 20, fh - 20)

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _find_ball_centers(self, mask, color_name, frame_shape, hsv=None):
        frame_h, frame_w = frame_shape

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cr           = self.color_ranges[color_name]
        min_radius   = cr.get('radius_min',   1)
        max_radius   = cr.get('radius_max',   999)
        min_circ     = cr.get('circularity',  0.0)
        min_fill     = cr.get('fill_ratio',   0.0)
        min_solid    = cr.get('solidity',     0.0)
        max_aspect   = cr.get('aspect_ratio', 999)  # max(w/h, h/w); 999 = disabled

        balls = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_ball_area:
                continue

            (x, y), radius = cv2.minEnclosingCircle(contour)
            x0, y0, bw, bh = cv2.boundingRect(contour)

            if self.border_margin > 0:
                if (x0 <= self.border_margin or y0 <= self.border_margin or
                        x0 + bw >= frame_w - self.border_margin or
                        y0 + bh >= frame_h - self.border_margin):
                    continue

            if not (min_radius <= radius <= max_radius):
                continue

            # Aspect ratio: how rectangular is the bounding box?
            # A circle → 1.0.  A wall segment → much larger.
            aspect = max(bw, bh) / max(min(bw, bh), 1)
            if aspect > max_aspect:
                continue

            perimeter   = cv2.arcLength(contour, True)
            circularity = (4 * np.pi * area) / (perimeter ** 2) if perimeter > 0 else 0
            fill_ratio  = area / (np.pi * radius ** 2)          if radius   > 0 else 0
            hull_area   = cv2.contourArea(cv2.convexHull(contour))
            solidity    = area / hull_area                       if hull_area > 0 else 0

            if circularity < min_circ or fill_ratio < min_fill or solidity < min_solid:
                continue

            if color_name == "ORANGE" and hsv is not None:
                cmask = np.zeros(mask.shape, dtype=np.uint8)
                cv2.drawContours(cmask, [contour], -1, 255, -1)
                mean_h, mean_s, mean_v, _ = cv2.mean(hsv, mask=cmask)
                if not (cr['mean_h_min'] <= mean_h <= cr['mean_h_max']
                        and mean_s >= cr['mean_s_min']
                        and mean_v >= cr['mean_v_min']):
                    continue

            balls.append({
                'x': int(x), 'y': int(y), 'radius': int(radius),
                'area': area, 'color': color_name,
                'circularity': circularity, 'fill_ratio': fill_ratio, 'solidity': solidity,
            })

        return balls

    def _update_stable_ball_tracks(self, detections, color_name):
        tracks  = self._ball_tracks[color_name]
        updated = []
        matched = set()

        for det in detections:
            best_i, best_d = None, None
            for i, t in enumerate(tracks):
                if i in matched:
                    continue
                d = np.hypot(det['x'] - t['x'], det['y'] - t['y'])
                if d <= self.ball_match_distance and (best_d is None or d < best_d):
                    best_i, best_d = i, d

            if best_i is not None:
                t = tracks[best_i].copy()
                t['x']           = int(round(t['x']      * 0.6 + det['x']      * 0.4))
                t['y']           = int(round(t['y']      * 0.6 + det['y']      * 0.4))
                t['radius']      = int(round(t['radius'] * 0.6 + det['radius'] * 0.4))
                t['area']        = det['area']
                t['circularity'] = det['circularity']
                t['fill_ratio']  = det['fill_ratio']
                t['solidity']    = det['solidity']
                t['hits']        = t.get('hits', 0) + 1
                t['streak']      = t.get('streak', 0) + 1
                t['misses']      = 0
                matched.add(best_i)
                updated.append(t)
            else:
                updated.append({**det, 'hits': 1, 'streak': 1, 'misses': 0})

        for i, t in enumerate(tracks):
            if i in matched:
                continue
            t = t.copy()
            t['misses'] += 1
            t['streak']  = 0
            if t['misses'] <= self.ball_miss_frames:
                updated.append(t)

        self._ball_tracks[color_name] = updated

        return [
            {k: t[k] for k in ('x', 'y', 'radius', 'area', 'color',
                                'circularity', 'fill_ratio', 'solidity')}
            for t in updated
            if t['streak'] >= self.ball_confirm_frames and t['misses'] == 0
        ]
