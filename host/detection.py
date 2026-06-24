#!/usr/bin/env python3
"""
ball and field detection engine. no UI code lives here.
receives a shared color_ranges dict; mutations (e.g. from calibration)
are reflected immediately because Python dicts are passed by reference.
"""
import base64
import cv2
import numpy as np
import threading
import time
import requests


class _RoboflowClient:
    """minimal Roboflow serverless inference client using plain HTTP (no inference_sdk)."""

    def __init__(self, api_url, api_key):
        self._api_url = api_url.rstrip('/')
        self._api_key = api_key

    def infer(self, frame, model_id):
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        b64 = base64.b64encode(buf.tobytes()).decode('ascii')
        url = "{}/{}?api_key={}".format(self._api_url, model_id, self._api_key)
        resp = requests.post(url, data=b64,
                             headers={"Content-Type": "application/x-www-form-urlencoded"},
                             timeout=10)
        resp.raise_for_status()
        return resp.json()


class BallDetector:
    def __init__(self, color_ranges, roboflow_api_key="", roboflow_model_id="",
                 roboflow_api_url="https://serverless.roboflow.com"):
        self.color_ranges = color_ranges
        self._roboflow_api_key  = roboflow_api_key
        self._roboflow_model_id = roboflow_model_id
        self._roboflow_client   = None
        if roboflow_api_key:
            self._roboflow_client = _RoboflowClient(
                api_url=roboflow_api_url,
                api_key=roboflow_api_key,
            )
            print("Roboflow YOLO backend active ({})".format(roboflow_model_id))
            # background thread so the main loop never blocks on network I/O.
            self._yolo_latest_frame  = None   # frame waiting to be processed
            self._yolo_cached_result = []     # last good prediction list
            self._yolo_lock  = threading.Lock()
            self._yolo_event = threading.Event()
            t = threading.Thread(target=self._yolo_worker, daemon=True)
            t.start()

        # detection parameters (all shape gates are in color_ranges)
        self.min_ball_area      = 10
        self.border_margin      = 0
        self.ball_confirm_frames = 4
        self.ball_miss_frames   = 6
        self.ball_match_distance = 18
        self._ball_tracks       = {"WHITE": [], "ORANGE": []}
        self.min_red_line_length = 50
        self._center_ema        = None   # smoothed centre position (x, y)
        self._center_r_ema      = None   # smoothed centre marker radius (px)

        # CLAHE normalises the V channel locally (8×8 tiles) so that one
        # dark corner and one bright corner of the field produce similar V
        # values for the same physical colour.  One instance, reused every
        # frame to avoid repeated allocation.
        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # ROBOT POSE (ArUco marker)
        # the closed-loop controller reads the robot's live pose from a single
        # ArUco marker mounted flat on top of the robot. mount it so the
        # marker's TOP edge points toward the robot's FRONT; set
        # robot_heading_offset_deg for any other mounting rotation.
        self.robot_marker_id          = None   # None = use first marker found
        self.robot_heading_offset_deg = 0.0
        try:
            # DICT_4X4_1000 matches the marker generator and chev.me "4x4 (1000)".
            # (low ids like 0 are identical across all 4X4 dicts, so older 4x4_50
            #  markers still work too.)
            _adict  = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
            _aparams = cv2.aruco.DetectorParameters()
            self._aruco_detector = cv2.aruco.ArucoDetector(_adict, _aparams)
        except Exception:
            # opencv-python (non-contrib) or an old API: robot detection disabled.
            self._aruco_detector = None

    def _to_hsv(self, frame):
        """BGR → HSV with CLAHE on V so thresholds work across uneven lighting."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v = self._clahe.apply(v)
        return cv2.merge([h, s, v])

    # PUBLIC API

    # pixels a ball centre must be inside the red-wall bounding box.
    # raised enough to exclude detections sitting ON the wall surface.
    FIELD_INSET = 10

    # pixels around the robot marker centre to exclude from ball detection.
    ROBOT_EXCLUSION_R = 50

    def analyze_course(self, frame):
        """run all detections and return a single result dict."""
        balls, mask_white, mask_orange = self.detect_balls(frame)
        red_walls    = self.detect_red_walls(frame)
        fh, fw = frame.shape[:2]

        red_cnts, _ = cv2.findContours(red_walls['mask'],
                                        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        field_detected = (red_cnts is not None and len(red_cnts) > 0
                          and self._is_valid_field_mask(red_walls['mask'], fh, fw))

        if field_detected:
            hull = cv2.convexHull(np.vstack(red_cnts))
            field_bounds = self._bounds_from_mask(red_walls['mask'])
            fx0, fy0, fx1, fy1 = field_bounds
            fi = self.FIELD_INSET
            balls = [b for b in balls
                     if fx0 + fi <= b['x'] <= fx1 - fi
                     and fy0 + fi <= b['y'] <= fy1 - fi]
            field_hull   = hull
        else:
            field_bounds = (20, 20, fw - 20, fh - 20)
            field_hull   = None

        # remove any "ball" whose centre falls inside the robot body - the
        # white ArUco marker frame creates circular Hough artefacts.
        robot_pose = self.detect_robot(frame)
        if robot_pose is not None:
            rx, ry, _ = robot_pose
            er2 = self.ROBOT_EXCLUSION_R ** 2
            balls = [b for b in balls
                     if (b['x'] - rx) ** 2 + (b['y'] - ry) ** 2 > er2]

        center_pos, center_radius = self._detect_center(
            red_cnts, field_bounds, field_detected)
        wall_margin  = (self._detect_wall_margin(red_walls['mask'], field_bounds)
                        if field_detected else None)

        return {
            'balls':          balls,
            'white_mask':     mask_white,
            'orange_mask':    mask_orange,
            'red_walls':      red_walls,
            'field_bounds':   field_bounds,
            'field_detected': field_detected,
            'field_hull':     field_hull,
            'center_pos':     center_pos,
            'center_radius':  center_radius,
            'wall_margin':    wall_margin,
            'frame_h': fh,
            'frame_w': fw,
        }

    def detect_robot(self, frame):
        """detect robot pose from ArUco marker. returns (x, y, heading_deg) or None."""
        if self._aruco_detector is None:
            return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._aruco_detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            return None

        ids = ids.flatten()
        idx = 0
        if self.robot_marker_id is not None:
            matches = [i for i, mid in enumerate(ids)
                       if int(mid) == self.robot_marker_id]
            if not matches:
                return None
            idx = matches[0]

        # corners are ordered TL, TR, BR, BL (clockwise) per OpenCV.
        quad = corners[idx].reshape(4, 2)
        cx = float(quad[:, 0].mean())
        cy = float(quad[:, 1].mean())

        top_mid = (quad[0] + quad[1]) / 2.0      # midpoint of the TL–TR edge
        fx, fy  = top_mid[0] - cx, top_mid[1] - cy
        heading = float(np.degrees(np.arctan2(fy, fx))) + self.robot_heading_offset_deg
        heading = (heading + 180.0) % 360.0 - 180.0

        # marker perimeter in pixels - larger = marker is close/well-lit and readable.
        perimeter = float(np.sum(np.linalg.norm(np.diff(quad, axis=0, append=quad[:1]), axis=1)))
        self._last_marker_perimeter = perimeter

        return (int(round(cx)), int(round(cy)), heading)

    # EMA weight for centre smoothing (0 = frozen, 1 = no smoothing).
    CENTER_SMOOTHING = 0.3

    def _detect_center(self, red_cnts, field_bounds, field_detected):
        """find the red X centre marker among red contours. returns ((cx, cy), radius), EMA-smoothed."""
        x0, y0, x1, y1 = field_bounds
        fcx, fcy = (x0 + x1) / 2.0, (y0 + y1) / 2.0

        raw = None
        raw_r = None
        if field_detected and red_cnts:
            bw, bh = x1 - x0, y1 - y0
            mfx, mfy = bw * 0.25, bh * 0.25
            field_area = max(bw * bh, 1)
            best_d = None
            for cnt in red_cnts:
                M = cv2.moments(cnt)
                area = M['m00']
                if area == 0:
                    continue
                # skip the field-boundary contour and tiny noise specks.
                if area > 0.15 * field_area or area < 40:
                    continue
                ccx = M['m10'] / area
                ccy = M['m01'] / area
                if not (x0 + mfx <= ccx <= x1 - mfx
                        and y0 + mfy <= ccy <= y1 - mfy):
                    continue
                d = (ccx - fcx) ** 2 + (ccy - fcy) ** 2
                if best_d is None or d < best_d:
                    best_d = d
                    raw = (ccx, ccy)
                    raw_r = cv2.minEnclosingCircle(cnt)[1]

        if raw is None:
            raw = (fcx, fcy)

        a = self.CENTER_SMOOTHING
        if self._center_ema is None:
            self._center_ema = raw
        else:
            self._center_ema = (self._center_ema[0] * (1 - a) + raw[0] * a,
                                self._center_ema[1] * (1 - a) + raw[1] * a)

        radius = None
        if raw_r is not None:
            if self._center_r_ema is None:
                self._center_r_ema = raw_r
            else:
                self._center_r_ema = self._center_r_ema * (1 - a) + raw_r * a
            radius = int(round(self._center_r_ema))

        return ((int(round(self._center_ema[0])),
                 int(round(self._center_ema[1]))), radius)

    @staticmethod
    def _detect_wall_margin(red_mask, field_bounds):
        """estimate wall thickness by scanning inward along 5 evenly-spaced lines per edge."""
        x0, y0, x1, y1 = field_bounds
        h, w = red_mask.shape[:2]
        thicknesses = []

        for frac in (0.25, 0.375, 0.50, 0.625, 0.75):
            # horizontal row: measure left- and right-wall thickness
            y = max(0, min(h - 1, int(y0 + (y1 - y0) * frac)))
            row = red_mask[y, max(0, x0):min(w, x1)]
            zeros_l = np.where(row == 0)[0]
            if len(zeros_l):
                thicknesses.append(int(zeros_l[0]))
            zeros_r = np.where(row[::-1] == 0)[0]
            if len(zeros_r):
                thicknesses.append(int(zeros_r[0]))

            # vertical column: measure top- and bottom-wall thickness
            x = max(0, min(w - 1, int(x0 + (x1 - x0) * frac)))
            col = red_mask[max(0, y0):min(h, y1), x]
            zeros_t = np.where(col == 0)[0]
            if len(zeros_t):
                thicknesses.append(int(zeros_t[0]))
            zeros_b = np.where(col[::-1] == 0)[0]
            if len(zeros_b):
                thicknesses.append(int(zeros_b[0]))

        if len(thicknesses) >= 4:
            return max(8, int(np.median(thicknesses)))
        return None

    def detect_balls(self, frame):
        """return (balls, raw_white_mask, raw_orange_mask)."""
        hsv = self._to_hsv(frame)

        # keep raw HSV masks for calibration visualisation - not used for detection.
        cr_w = self.color_ranges['WHITE']
        cr_o = self.color_ranges['ORANGE']
        raw_white  = cv2.inRange(hsv, np.array(cr_w['lower']), np.array(cr_w['upper']))
        raw_orange = cv2.inRange(hsv, np.array(cr_o['lower']), np.array(cr_o['upper']))

        if self._roboflow_client:
            candidates = self._find_balls_yolo(frame)
        else:
            candidates = self._find_balls_hough(hsv, frame.shape[:2])

        white_cands  = [b for b in candidates if b['color'] == 'WHITE']
        orange_cands = [b for b in candidates if b['color'] == 'ORANGE']

        white_balls  = self._update_stable_ball_tracks(white_cands,  'WHITE')
        orange_balls = self._update_stable_ball_tracks(orange_cands, 'ORANGE')

        return sorted(white_balls + orange_balls, key=lambda b: (b['x'], b['y'])), raw_white, raw_orange

    # minimum seconds between Roboflow API calls
    YOLO_CALL_INTERVAL = 0.25

    def _yolo_worker(self):
        """background thread: process the latest queued frame and cache result."""
        last_call = 0.0
        while True:
            self._yolo_event.wait()
            self._yolo_event.clear()
            now = time.monotonic()
            wait = self.YOLO_CALL_INTERVAL - (now - last_call)
            if wait > 0:
                time.sleep(wait)
            with self._yolo_lock:
                frame = self._yolo_latest_frame
            if frame is None:
                continue
            last_call = time.monotonic()
            try:
                result = self._roboflow_client.infer(frame, model_id=self._roboflow_model_id)
                preds  = result.get('predictions', [])
            except Exception:
                continue
            balls = []
            for p in preds:
                cls = p.get('class', '')
                if cls == 'Ball white':
                    color = 'WHITE'
                elif cls == 'Ball orange':
                    color = 'ORANGE'
                else:
                    continue
                cx = int(round(p['x']))
                cy = int(round(p['y']))
                r  = max(1, int(round(min(p['width'], p['height']) / 2)))
                balls.append({
                    'x': cx, 'y': cy, 'radius': r,
                    'area': float(np.pi * r * r), 'color': color,
                    'circularity': 1.0, 'fill_ratio': 1.0, 'solidity': 1.0,
                    'confidence': round(float(p.get('confidence', 0.0)), 2),
                })
            with self._yolo_lock:
                self._yolo_cached_result = balls

    def _find_balls_yolo(self, frame):
        """submit frame to background worker; return last cached result immediately."""
        with self._yolo_lock:
            self._yolo_latest_frame = frame
            cached = list(self._yolo_cached_result)
        self._yolo_event.set()
        return cached

    def _find_balls_hough(self, hsv, frame_shape):
        """detect balls via HoughCircles on the V channel, classify by mean HSV."""
        fh, fw = frame_shape
        cr_w = self.color_ranges['WHITE']
        cr_o = self.color_ranges['ORANGE']

        r_min = min(cr_w.get('radius_min', 5), cr_o.get('radius_min', 5))
        r_max = max(cr_w.get('radius_max', 22), cr_o.get('radius_max', 22))

        _, _, v = cv2.split(hsv)
        v_blur = cv2.GaussianBlur(v, (5, 5), 1.5)

        circles = cv2.HoughCircles(
            v_blur,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=35,
            param1=60,
            param2=10,
            minRadius=r_min,
            maxRadius=r_max,
        )
        if circles is None:
            return []

        balls = []
        for cx, cy, cr in np.round(circles[0]).astype(int):
            if not (0 <= cx < fw and 0 <= cy < fh):
                continue
            cmask = np.zeros((fh, fw), dtype=np.uint8)
            cv2.circle(cmask, (cx, cy), max(cr - 1, 1), 255, -1)
            mh, ms, mv, _ = cv2.mean(hsv, mask=cmask)

            if (cr_w['lower'][1] <= ms <= cr_w['upper'][1]
                    and mv >= cr_w['lower'][2]):
                color = 'WHITE'
            elif (cr_o['lower'][0] <= mh <= cr_o['upper'][0]
                  and ms >= cr_o['lower'][1]
                  and mv >= cr_o['lower'][2]
                  and cr_o['mean_h_min'] <= mh <= cr_o['mean_h_max']):
                color = 'ORANGE'
            else:
                continue

            balls.append({
                'x': int(cx), 'y': int(cy), 'radius': int(cr),
                'area': float(np.pi * cr * cr), 'color': color,
                'circularity': 1.0, 'fill_ratio': 1.0, 'solidity': 1.0,
            })
        return balls

    def detect_hole_markers(self, frame, field_bounds=None):
        """Detect dropoff-hole ArUco markers in the frame.

        Returns a list of dicts, one per detected non-robot marker:
            {'id': int, 'x': int, 'y': int, 'heading_deg': float, 'approach_deg': float}

        'approach_deg' is the heading the robot should face when arriving at the
        hole — opposite to the direction the marker faces into the field.  Assumes
        each hole marker is mounted with its TOP edge pointing inward (into the
        field), so the robot approaches from inside the field facing the marker.

        field_bounds: (x0, y0, x1, y1) pixel bounds of the field.  When provided,
        markers that are NOT near the left or right wall are silently dropped —
        this filters out the robot's own ArUco marker without requiring robot_marker_id
        to be set.
        """
        if self._aruco_detector is None:
            return []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._aruco_detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            return []

        ids_flat = ids.flatten()
        holes = []
        for i, mid in enumerate(ids_flat):
            mid = int(mid)
            if mid not in (1, 2):
                continue
            quad = corners[i].reshape(4, 2)
            cx = float(quad[:, 0].mean())
            cy = float(quad[:, 1].mean())

            # When we know the field bounds, filter to markers near the left or
            # right wall (within 20 % of the field width from each side).  This
            # rejects the robot's marker — which is somewhere in the middle —
            # without needing robot_marker_id to be explicitly configured.
            if field_bounds is not None:
                x0, y0, x1, y1 = field_bounds
                wall_zone = (x1 - x0) * 0.20
                if not (cx < x0 + wall_zone or cx > x1 - wall_zone):
                    continue

            top_mid = (quad[0] + quad[1]) / 2.0
            fx, fy = top_mid[0] - cx, top_mid[1] - cy
            heading = float(np.degrees(np.arctan2(fy, fx)))
            heading = (heading + 180.0) % 360.0 - 180.0

            # Robot faces the marker from inside the field, i.e. opposite direction.
            approach = (heading + 180.0) % 360.0
            if approach > 180.0:
                approach -= 360.0

            holes.append({
                'id': mid,
                'x': int(round(cx)),
                'y': int(round(cy)),
                'heading_deg': heading,
                'approach_deg': approach,
            })
        return holes

    def detect_red_walls(self, frame):
        """return dict with mask, edges, and Hough lines for the red boundary."""
        hsv = self._to_hsv(frame)
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

    @staticmethod
    def _bounds_from_mask(mask):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        x, y, w, h = cv2.boundingRect(np.vstack(cnts))
        return (x, y, x + w, y + h)

    @staticmethod
    def _is_valid_field_mask(mask, frame_h, frame_w):
        """true when the red mask looks like a real field boundary, not noise."""
        if int(np.count_nonzero(mask)) < 500:
            return False
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return False
        _, _, w, h = cv2.boundingRect(np.vstack(cnts))
        frame_area = frame_h * frame_w
        bbox_area  = w * h
        if not (0.15 * frame_area <= bbox_area <= 0.97 * frame_area):
            return False
        aspect = max(w, h) / max(min(w, h), 1)
        return aspect <= 4.0

    # INTERNAL HELPERS

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
