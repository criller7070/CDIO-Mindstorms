import os, sys, time
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2
sys.path.insert(0, 'c:/Users/crill/Documents/GitHub/CDIO-Mindstorms/host')

_env = os.path.join(os.path.dirname(__file__), '..', '.env')
if os.path.exists(_env):
    for line in open(_env):
        line = line.strip()
        if '=' in line and not line.startswith('#'):
            k, v = line.split('=', 1)
            os.environ.setdefault(k.strip(), v.strip())

from detection import BallDetector
from pathfinding import FieldPlanner
from config import (load_color_ranges, ROBOFLOW_API_KEY, ROBOFLOW_API_URL, ROBOFLOW_MODEL_ID,
                    WALL_MARGIN, CENTER_RADIUS, ROBOT_WIDTH_MM, ROBOT_LENGTH_MM,
                    ROBOT_PIVOT_OFFSET_MM, GATE_ARM_MM, ARUCO_FROM_BACK_FRAC,
                    CENTER_OBSTACLE_EXTRA_PX, HOLE_FRAC_X, HOLE_FRAC_Y)

det = BallDetector(
    load_color_ranges(),
    roboflow_api_key=ROBOFLOW_API_KEY,
    roboflow_model_id=ROBOFLOW_MODEL_ID,
    roboflow_api_url=ROBOFLOW_API_URL,
)
cap = cv2.VideoCapture(1)
time.sleep(1.5)

last_plan_time = 0
cached_vis = None

while True:
    ret, frame = cap.read()
    if not ret:
        break

    analysis = det.analyze_course(frame)
    vis = frame.copy()

    # Field bounds
    if not analysis['field_detected']:
        cv2.putText(vis, "No field", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("Ball Route - Q to quit", vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue

    x0, y0, x1, y1 = analysis['field_bounds']
    cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 200, 0), 1)

    balls = analysis['balls']
    ball_positions = [(b['x'], b['y']) for b in balls]

    # Draw all balls first (unordered)
    for b in balls:
        color = (0, 165, 255) if b['color'] == 'ORANGE' else (200, 200, 200)
        cv2.circle(vis, (b['x'], b['y']), b.get('radius', 8), color, 2)

    # Plan route if we have balls and a robot pose
    robot_pose = det.detect_robot(frame)
    robot_pos = (robot_pose[0], robot_pose[1]) if robot_pose is not None else None
    now = time.monotonic()
    if ball_positions and robot_pos and now - last_plan_time > 1.0:
        last_plan_time = now
        try:
            center_pos = analysis['center_pos']
            wall_margin = analysis['wall_margin'] or WALL_MARGIN
            detected_r = analysis.get('center_radius') or CENTER_RADIUS
            center_radius = min(detected_r, CENTER_RADIUS) + CENTER_OBSTACLE_EXTRA_PX

            hole_markers = det.detect_hole_markers(frame, analysis['field_bounds'])
            if hole_markers:
                best = sorted(hole_markers, key=lambda m: m['x'])[0]
                dropoff_raw = (best['x'], best['y'])
                face_deg = best['approach_deg']
            else:
                dropoff_raw = (int(x0 + (x1 - x0) * HOLE_FRAC_X),
                               int(y0 + (y1 - y0) * HOLE_FRAC_Y))
                face_deg = 180.0

            planner = FieldPlanner(
                field_bounds=analysis['field_bounds'],
                center_pos=center_pos,
                wall_margin=wall_margin,
                center_radius=center_radius,
                field_hull=analysis.get('field_hull'),
                robot_width_mm=ROBOT_WIDTH_MM,
                robot_length_mm=ROBOT_LENGTH_MM,
                pivot_offset_mm=ROBOT_PIVOT_OFFSET_MM,
                gate_open=False,
                gate_arm_mm=GATE_ARM_MM,
                aruco_from_back_frac=ARUCO_FROM_BACK_FRAC,
            )
            import math as _math
            _fr = _math.radians(face_deg)
            approach_wp = (dropoff_raw[0] - 100.0 * _math.cos(_fr),
                           dropoff_raw[1] - 100.0 * _math.sin(_fr))
            dropoff = dropoff_raw
            planner.plan_trips(
                robot_pos=robot_pos,
                ball_positions=ball_positions,
                dropoff_pos=approach_wp,
                capacity=8,
                initial_heading_deg=-90,
            )

            ball_order = getattr(planner, '_debug_ball_order', [])
            ball_positions_used = getattr(planner, '_debug_ball_positions', ball_positions)
            skipped = getattr(planner, '_debug_skipped_balls', [])

            # Draw obstacle grid (red tint over blocked cells)
            gs = planner.GRID_SCALE
            for gx in range(planner.gw):
                for gy in range(planner.gh):
                    if planner.grid[gx][gy]:
                        px2 = planner.x0 + gx * gs
                        py2 = planner.y0 + gy * gs
                        cv2.rectangle(vis, (px2, py2), (px2 + gs, py2 + gs), (0, 0, 80), -1)

            # Mark skipped balls in red
            for sx, sy in skipped:
                cv2.circle(vis, (int(sx), int(sy)), 12, (0, 0, 255), 2)
                cv2.putText(vis, "X", (int(sx) - 4, int(sy) + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Draw actual A* path segments (shows perpendicular approach)
            debug_segs = [list(s) for s in getattr(planner, '_debug_path_segs', [])]
            # Append clean approach_wp → dropoff segment (no snap, no kink)
            if debug_segs:
                debug_segs.append([approach_wp, dropoff])
            # Last 2 segs are the dropoff (A*→approach_wp + approach_wp→wall)
            n_collect = max(0, len(debug_segs) - 2)
            for leg, seg in enumerate(debug_segs):
                color = (0, 255, 255) if leg < n_collect else (0, 100, 255)
                for i in range(1, len(seg)):
                    p0 = (int(seg[i-1][0]), int(seg[i-1][1]))
                    p1 = (int(seg[i][0]), int(seg[i][1]))
                    cv2.line(vis, p0, p1, color, 2)

            # Number each ball in visit order
            for step, idx in enumerate(ball_order):
                bx, by = int(ball_positions_used[idx][0]), int(ball_positions_used[idx][1])
                cv2.circle(vis, (bx, by), 12, (0, 255, 0), 2)
                cv2.putText(vis, str(step + 1), (bx - 5, by + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Dropoff marker
            cv2.circle(vis, (int(dropoff[0]), int(dropoff[1])), 10, (0, 100, 255), -1)
            cv2.putText(vis, "DROP", (int(dropoff[0]) + 5, int(dropoff[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 2)

            # Console route printout
            print("\nRoute: robot", end="")
            for step, idx in enumerate(ball_order):
                bx, by = int(ball_positions_used[idx][0]), int(ball_positions_used[idx][1])
                col = balls[idx]['color'] if idx < len(balls) else '?'
                print(" -> #{} {}({},{})".format(step + 1, col[0], bx, by), end="")
            print(" -> DROP({},{})".format(int(dropoff[0]), int(dropoff[1])))

            cached_vis = vis.copy()
        except Exception as e:
            print("Plan error:", e)

    # Overlay cached route lines on fresh frame
    if cached_vis is not None:
        vis = cached_vis.copy()

    white = sum(1 for b in balls if b['color'] == 'WHITE')
    orange = sum(1 for b in balls if b['color'] == 'ORANGE')
    cv2.putText(vis, "W:{} O:{} total:{}".format(white, orange, len(balls)),
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    cv2.imshow("Ball Route - Q to quit", vis)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
