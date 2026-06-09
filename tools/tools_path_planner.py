#!/usr/bin/env python3
"""
A* path planner for CDIO golf field.

Finds the optimal order to collect all balls and deliver them to the left hole
while staying inside the wall margin and away from the center obstacle.

Outputs EV3-compatible TURN/FORWARD/LIFT_DOWN/LIFT_UP commands.

Coordinate system (image space):
  - x increases left → right
  - y increases top  → bottom
  - heading 0° = facing right, -90° = facing up, 90° = facing down, 180° = facing left
  - TURN positive = clockwise (matches pybricks DriveBase.turn)
"""

import heapq
import cv2
import numpy as np
from itertools import permutations


# -----------------------------------------------------------------------------
# - Calibration - set FIELD_WIDTH_MM and FIELD_HEIGHT_MM to real measurements -
# -----------------------------------------------------------------------------
FIELD_WIDTH_MM = 1670   # field width:  167.0 cm
FIELD_HEIGHT_MM = 1215  # field height: 121.5 cm


class FieldPlanner:
    """Grid-based A* planner with obstacle avoidance for the golf field."""

    GRID_SCALE = 5  # pixels per grid cell (lower = finer but slower)

    def __init__(self,
                 field_bounds,
                 center_pos,
                 wall_margin=45,
                 center_radius=60,
                 field_width_mm=FIELD_WIDTH_MM,
                 field_height_mm=FIELD_HEIGHT_MM,
                 field_hull=None,
                 robot_width_mm=0,
                 robot_length_mm=0):
        """
        field_bounds:    (x_min, y_min, x_max, y_max) pixel coords of red rectangle
        center_pos:      (cx, cy) pixel coords of center crosshair
        wall_margin:     pixels to keep away from walls
        center_radius:   pixel radius of center no-go zone
        field_width_mm:  real-world field width in mm (for command distances)
        field_height_mm: real-world field height in mm
        field_hull:      optional (N,1,2) cv2 contour of the actual wall shape;
                         cells outside it are blocked regardless of wall_margin
        """
        self.bounds = field_bounds
        self.center = center_pos
        self.wall_margin = wall_margin
        self.center_radius = center_radius
        self._field_hull = field_hull

        x_min, y_min, x_max, y_max = field_bounds
        self.x0, self.y0 = x_min, y_min
        field_px_w = x_max - x_min
        field_px_h = y_max - y_min

        # pixels per mm (average of both axes)
        px_per_mm_x = field_px_w / field_width_mm
        px_per_mm_y = field_px_h / field_height_mm
        self.px_per_mm = (px_per_mm_x + px_per_mm_y) / 2.0

        # Robot body clearance (added to obstacle margins so the body stays clear)
        self.robot_half_width_px   = (robot_width_mm  / 2.0) * self.px_per_mm
        self.robot_half_length_mm  = robot_length_mm  / 2.0

        self.gw = field_px_w // self.GRID_SCALE + 1
        self.gh = field_px_h // self.GRID_SCALE + 1

        self.grid = self._build_grid()

    # ------------------------------------------------------------------
    # Grid construction
    # ------------------------------------------------------------------

    def _build_grid(self):
        gs = self.GRID_SCALE
        margin = max(1, int(self.wall_margin + self.robot_half_width_px) // gs)
        cx = (self.center[0] - self.x0) // gs
        cy = (self.center[1] - self.y0) // gs
        cr = max(1, int(self.center_radius + self.robot_half_width_px) // gs)

        grid = [[False] * self.gh for _ in range(self.gw)]
        for gx in range(self.gw):
            for gy in range(self.gh):
                # Outside actual field hull (non-rectangular walls)
                if self._field_hull is not None:
                    px, py = self._to_px(gx, gy)
                    if cv2.pointPolygonTest(
                            self._field_hull, (float(px), float(py)), False) < 0:
                        grid[gx][gy] = True
                        continue
                # Rectangular wall buffer (fallback or extra margin)
                if gx < margin or gx >= self.gw - margin:
                    grid[gx][gy] = True
                elif gy < margin or gy >= self.gh - margin:
                    grid[gx][gy] = True
                # Center obstacle (circular)
                elif (gx - cx) ** 2 + (gy - cy) ** 2 <= cr ** 2:
                    grid[gx][gy] = True
        return grid

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    def _to_grid(self, px, py):
        gx = max(0, min(self.gw - 1, (int(px) - self.x0) // self.GRID_SCALE))
        gy = max(0, min(self.gh - 1, (int(py) - self.y0) // self.GRID_SCALE))
        return gx, gy

    def _is_in_obstacle(self, px, py):
        gx, gy = self._to_grid(px, py)
        return self.grid[gx][gy]

    def _to_px(self, gx, gy):
        return gx * self.GRID_SCALE + self.x0, gy * self.GRID_SCALE + self.y0

    def _nearest_free(self, gx, gy):
        """Return nearest obstacle-free grid cell to (gx, gy)."""
        if not self.grid[gx][gy]:
            return gx, gy
        for r in range(1, max(self.gw, self.gh)):
            for dx in range(-r, r + 1):
                for dy in ([-r, r] if abs(dx) < r else range(-r, r + 1)):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.gw and 0 <= ny < self.gh and not self.grid[nx][ny]:
                        return nx, ny
        return gx, gy

    # ------------------------------------------------------------------
    # A* search
    # ------------------------------------------------------------------

    def astar(self, start_px, end_px):
        """
        Find shortest path from start_px to end_px in pixel space.
        Returns list of (px, py) tuples, or None if unreachable.
        """
        s = self._nearest_free(*self._to_grid(*start_px))
        e = self._nearest_free(*self._to_grid(*end_px))

        DIRS = [
            (-1,  0, 1.0), (1,  0, 1.0), (0, -1, 1.0), (0,  1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1,  1, 1.414),
        ]

        heap = [(0.0, s)]
        came_from = {}
        g = {s: 0.0}

        while heap:
            _, cur = heapq.heappop(heap)
            if cur == e:
                path = []
                while cur in came_from:
                    path.append(self._to_px(*cur))
                    cur = came_from[cur]
                path.append(self._to_px(*s))
                path.reverse()
                return path

            for dx, dy, cost in DIRS:
                nx, ny = cur[0] + dx, cur[1] + dy
                if not (0 <= nx < self.gw and 0 <= ny < self.gh):
                    continue
                if self.grid[nx][ny]:
                    continue
                ng = g[cur] + cost
                nb = (nx, ny)
                if ng < g.get(nb, float('inf')):
                    came_from[nb] = cur
                    g[nb] = ng
                    h = ((nx - e[0]) ** 2 + (ny - e[1]) ** 2) ** 0.5
                    heapq.heappush(heap, (ng + h, nb))

        return None

    # ------------------------------------------------------------------
    # K-Means++ clustering (capacity-aware multi-trip)
    # ------------------------------------------------------------------

    def cluster_balls(self, ball_positions, capacity):
        """
        Partition ball_positions into groups of ≤ capacity using K-Means++.
        Returns list of lists of indices into ball_positions.
        """
        n = len(ball_positions)
        k = max(1, -(-n // capacity))  # ceil(n / capacity)
        if k == 1:
            return [list(range(n))]

        pts = np.array(ball_positions, float)
        assignments = self._kmeans_plus_plus(pts, k)
        assignments = self._balance_clusters(pts, assignments, k, capacity)

        clusters = [[] for _ in range(k)]
        for i, a in enumerate(assignments):
            clusters[a].append(i)
        return [c for c in clusters if c]

    def _kmeans_plus_plus(self, pts, k):
        """K-Means++ initialisation + iteration. Returns integer assignment array."""
        n = len(pts)
        rng = np.random.default_rng(42)  # fixed seed → reproducible routes

        # K-Means++ center selection
        ci = [int(rng.integers(n))]
        for _ in range(k - 1):
            dists = np.array([min(np.sum((p - pts[c]) ** 2) for c in ci) for p in pts])
            probs = dists / dists.sum()
            ci.append(int(rng.choice(n, p=probs)))
        centers = pts[ci].copy()

        # Iterate until convergence
        assignments = np.zeros(n, dtype=int)
        for _ in range(100):
            dists_sq = np.sum((pts[:, None, :] - centers[None, :, :]) ** 2, axis=2)
            new_assignments = np.argmin(dists_sq, axis=1)
            new_centers = np.array([
                pts[new_assignments == i].mean(axis=0) if np.any(new_assignments == i)
                else centers[i]
                for i in range(k)
            ])
            if np.array_equal(new_assignments, assignments) and np.allclose(centers, new_centers):
                break
            assignments, centers = new_assignments, new_centers

        return assignments

    def _balance_clusters(self, pts, assignments, k, capacity):
        """Reassign overflow balls to the nearest under-capacity cluster."""
        assignments = assignments.copy()
        for _ in range(len(pts)):
            counts = np.bincount(assignments, minlength=k)
            over = np.where(counts > capacity)[0]
            if len(over) == 0:
                break
            centers = np.array([
                pts[assignments == i].mean(axis=0) if np.any(assignments == i)
                else np.zeros(2)
                for i in range(k)
            ])
            for c in over:
                members = np.where(assignments == c)[0]
                for m in members:
                    counts = np.bincount(assignments, minlength=k)
                    if counts[c] <= capacity:
                        break
                    under = np.where(counts < capacity)[0]
                    if len(under) == 0:
                        break
                    nearest = under[np.argmin(np.sum((centers[under] - pts[m]) ** 2, axis=1))]
                    assignments[m] = nearest
        return assignments

    # ------------------------------------------------------------------
    # Optimal route (TSP brute-force for ≤ ~8 balls)
    # ------------------------------------------------------------------

    def optimal_route(self, robot_pos, ball_positions, dropoff_pos):
        """
        Try every permutation of ball_positions and return the route with
        the shortest total A* path length.

        Returns:
            ball_order  - list of indices into ball_positions
            path_segs   - list of pixel-path segments:
                          [robot→b0, b0→b1, ..., bN→dropoff]
        """
        n = len(ball_positions)
        if n == 0:
            path = self.astar(robot_pos, dropoff_pos)
            return [], ([path] if path else [])

        best_order, best_segs, best_cost = None, None, float('inf')

        for perm in permutations(range(n)):
            waypoints = ([robot_pos]
                         + [ball_positions[i] for i in perm]
                         + [dropoff_pos])
            segs = []
            cost = 0.0
            ok = True
            for i in range(len(waypoints) - 1):
                p0 = (int(waypoints[i][0]), int(waypoints[i][1]))
                p1 = (int(waypoints[i + 1][0]), int(waypoints[i + 1][1]))
                seg = self.astar(p0, p1)
                if seg is None:
                    ok = False
                    break
                segs.append(seg)
                cost += self._path_length(seg)
            if ok and cost < best_cost:
                best_cost = cost
                best_order = list(perm)
                best_segs = segs

        return (best_order or list(range(n))), best_segs

    @staticmethod
    def _path_length(path):
        total = 0.0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            total += (dx ** 2 + dy ** 2) ** 0.5
        return total

    # ------------------------------------------------------------------
    # Path simplification (Ramer-Douglas-Peucker)
    # ------------------------------------------------------------------

    @staticmethod
    def simplify(pts, eps=8):
        """Remove near-collinear points to reduce command count."""
        if len(pts) <= 2:
            return pts
        start = np.array(pts[0], float)
        end = np.array(pts[-1], float)
        seg = end - start
        seg_len = float(np.linalg.norm(seg))
        if seg_len == 0:
            return [pts[0], pts[-1]]
        max_d, max_i = 0.0, 0
        for i in range(1, len(pts) - 1):
            d = float(np.linalg.norm(np.cross(seg, start - np.array(pts[i], float)))) / seg_len
            if d > max_d:
                max_d, max_i = d, i
        if max_d > eps:
            left = FieldPlanner.simplify(pts[:max_i + 1], eps)
            right = FieldPlanner.simplify(pts[max_i:], eps)
            return left[:-1] + right
        return [pts[0], pts[-1]]

    # ------------------------------------------------------------------
    # Command generation
    # ------------------------------------------------------------------

    def paths_to_commands(self, path_segs, initial_heading_deg=0):
        """
        Convert route path segments to EV3 command strings.

        path_segs:           output of optimal_route
        initial_heading_deg: robot's starting heading in image-space degrees
                             (-90 = facing up/north, 0 = facing right, 90 = facing down)

        Returns list of command strings for commands.txt.
        """
        commands = ["SPEED:300"]
        heading = float(initial_heading_deg)
        n_collect = len(path_segs) - 1  # last segment ends at dropoff

        for leg, seg in enumerate(path_segs):
            is_collect = leg < n_collect
            pts = self.simplify(seg, eps=8)

            for i in range(1, len(pts)):
                x0, y0 = pts[i - 1]
                x1, y1 = pts[i]
                dx, dy = x1 - x0, y1 - y0
                dist_px = (dx ** 2 + dy ** 2) ** 0.5
                if dist_px < self.GRID_SCALE:
                    continue

                target_heading = float(np.degrees(np.arctan2(dy, dx)))
                turn = (target_heading - heading + 180.0) % 360.0 - 180.0
                if abs(turn) > 2:
                    commands.append("TURN:{}".format(int(round(turn))))
                    heading = target_heading

                dist_mm = max(1, int(dist_px / self.px_per_mm))
                commands.append("FORWARD:{}".format(dist_mm))

            if not is_collect:
                commands.append("STOP")

        return commands

    def _sim_metadata(self, robot_pos, dropoff_pos):
        """Comment lines that embed geometry so the simulator can draw it correctly."""
        cx_mm = (self.center[0] - self.x0) / self.px_per_mm
        cy_mm = (self.center[1] - self.y0) / self.px_per_mm
        cr_mm = self.center_radius / self.px_per_mm
        wm_mm = self.wall_margin / self.px_per_mm
        rx_mm = (robot_pos[0] - self.x0) / self.px_per_mm
        ry_mm = (robot_pos[1] - self.y0) / self.px_per_mm
        hx_mm = (dropoff_pos[0] - self.x0) / self.px_per_mm
        hy_mm = (dropoff_pos[1] - self.y0) / self.px_per_mm
        meta = [
            "# SIM_CENTER: {:.0f} {:.0f}".format(cx_mm, cy_mm),
            "# SIM_CENTER_R: {:.0f}".format(cr_mm),
            "# SIM_WALL: {:.0f}".format(wm_mm),
            "# SIM_START: {:.0f} {:.0f}".format(rx_mm, ry_mm),
            "# SIM_HOLE: {:.0f} {:.0f}".format(hx_mm, hy_mm),
        ]
        if self._field_hull is not None:
            pts = self._field_hull.reshape(-1, 2)
            pairs = " ".join(
                "{:.0f} {:.0f}".format((px - self.x0) / self.px_per_mm,
                                       (py - self.y0) / self.px_per_mm)
                for px, py in pts
            )
            meta.append("# SIM_HULL: {}".format(pairs))
        return meta

    # ------------------------------------------------------------------
    # Multi-trip planning
    # ------------------------------------------------------------------

    def _segs_to_commands(self, path_segs, is_last_trip, heading):
        """
        Convert one trip's path segments to EV3 commands.
        Tracks and returns the robot's final heading so the next trip
        can continue without a spurious TURN at the join.
        """
        commands = []
        n_collect = len(path_segs) - 1

        for leg, seg in enumerate(path_segs):
            is_collect = leg < n_collect
            pts = self.simplify(seg, eps=8)

            leg_cmds = []
            for i in range(1, len(pts)):
                x0, y0 = pts[i - 1]
                x1, y1 = pts[i]
                dx, dy = x1 - x0, y1 - y0
                dist_px = (dx ** 2 + dy ** 2) ** 0.5
                if dist_px < self.GRID_SCALE:
                    continue

                target_heading = float(np.degrees(np.arctan2(dy, dx)))
                turn = (target_heading - heading + 180.0) % 360.0 - 180.0
                if abs(turn) > 2:
                    leg_cmds.append("TURN:{}".format(int(round(turn))))
                    heading = target_heading

                dist_mm = max(1, int(dist_px / self.px_per_mm))
                leg_cmds.append("FORWARD:{}".format(dist_mm))

            # Shorten last FORWARD so the robot's front face (not centre) reaches the ball
            if is_collect and self.robot_half_length_mm > 0:
                for j in range(len(leg_cmds) - 1, -1, -1):
                    if leg_cmds[j].startswith("FORWARD:"):
                        old_val = int(leg_cmds[j].split(":")[1])
                        new_val = max(1, old_val - int(round(self.robot_half_length_mm)))
                        leg_cmds[j] = "FORWARD:{}".format(new_val)
                        break

            commands.extend(leg_cmds)
            if not is_collect and is_last_trip:
                commands.append("STOP")

        return commands, heading

    def plan_trips(self, robot_pos, ball_positions, dropoff_pos,
                   capacity=6, initial_heading_deg=0):
        """
        Capacity-aware multi-trip planner. Handles any number of balls.

        - Balls are clustered into trips of ≤ capacity using K-Means++.
        - The best cluster to do first (closest to robot_pos) is chosen by
          trying all k candidates and picking the lowest total A* cost.
        - Remaining trips all start and end at dropoff_pos; their internal
          order is optimised independently by brute-force permutation (≤ capacity!).
        - Heading is threaded across all trips so TURN commands are correct.

        Returns a list of EV3 command strings ready to write to commands.txt.
        """
        meta = self._sim_metadata(robot_pos, dropoff_pos)

        reachable = [bp for bp in ball_positions if not self._is_in_obstacle(*bp)]
        skipped = len(ball_positions) - len(reachable)
        if skipped:
            print("Skipping {} ball(s) inside obstacles/walls.".format(skipped))
        ball_positions = reachable

        n = len(ball_positions)
        if n == 0:
            path = self.astar(
                (int(robot_pos[0]), int(robot_pos[1])),
                (int(dropoff_pos[0]), int(dropoff_pos[1])),
            )
            cmds = ["SPEED:300"] + meta
            if path:
                trip_cmds, _ = self._segs_to_commands(
                    [path], True, initial_heading_deg)
                cmds += trip_cmds
            return cmds

        clusters = self.cluster_balls(ball_positions, capacity)
        k = len(clusters)
        print("Planned {} trip(s) for {} balls (capacity {})".format(k, n, capacity))

        # Precompute hole→cluster→hole routes for every cluster
        hole_segs = []
        hole_ball_orders = []
        for cluster in clusters:
            pts = [ball_positions[j] for j in cluster]
            local_order, segs = self.optimal_route(dropoff_pos, pts, dropoff_pos)
            hole_segs.append(segs)
            hole_ball_orders.append([cluster[i] for i in local_order])

        # Choose the best first trip (try all k clusters as the first)
        best_cost = float('inf')
        best_first = 0
        best_first_segs = None
        best_first_order = None

        for fi in range(k):
            pts = [ball_positions[j] for j in clusters[fi]]
            local_order, segs = self.optimal_route(robot_pos, pts, dropoff_pos)
            if segs is None:
                continue
            cost = sum(self._path_length(s) for s in segs)
            for i in range(k):
                if i != fi and hole_segs[i]:
                    cost += sum(self._path_length(s) for s in hole_segs[i])
            if cost < best_cost:
                best_cost = cost
                best_first = fi
                best_first_segs = segs
                best_first_order = [clusters[fi][i] for i in local_order]

        if best_first_segs is None:
            print("WARNING: could not plan any trip – check field bounds / obstacles.")
            return ["STOP"]

        # Build commands, threading heading across all trips
        commands = ["SPEED:300"] + meta
        heading = float(initial_heading_deg)

        # Trip 1 starts from robot_pos
        trip_cmds, heading = self._segs_to_commands(
            best_first_segs, k == 1, heading)
        commands += ["# Trip 1/{} – cluster {}".format(k, best_first)] + trip_cmds

        # Remaining trips start from the hole
        trip_num = 2
        debug_segs = list(best_first_segs)
        debug_order = list(best_first_order or [])
        for i in range(k):
            if i == best_first:
                continue
            segs = hole_segs[i]
            if segs:
                is_last = (trip_num == k)
                trip_cmds, heading = self._segs_to_commands(
                    segs, is_last, heading)
                commands += ["# Trip {}/{} – cluster {}".format(trip_num, k, i)] + trip_cmds
                debug_segs.extend(segs)
                debug_order.extend(hole_ball_orders[i])
            trip_num += 1

        self._debug_path_segs = debug_segs
        self._debug_ball_order = debug_order
        return commands

    # ------------------------------------------------------------------
    # Debug visualization
    # ------------------------------------------------------------------

    def debug_overlay(self, frame, path_segs, robot_pos, ball_positions,
                      dropoff_pos, ball_order):
        """Draw the planned route on a copy of frame for visual inspection."""
        import cv2
        vis = frame.copy()
        colors = [(0, 255, 255), (255, 128, 0), (128, 0, 255)]

        for i, seg in enumerate(path_segs):
            color = colors[i % len(colors)]
            for j in range(1, len(seg)):
                cv2.line(vis, seg[j - 1], seg[j], color, 2)

        # Obstacle grid overlay (thin)
        for gx in range(self.gw):
            for gy in range(self.gh):
                if self.grid[gx][gy]:
                    px, py = self._to_px(gx, gy)
                    cv2.rectangle(vis, (px, py), (px + self.GRID_SCALE, py + self.GRID_SCALE),
                                  (50, 50, 50), -1)

        cv2.circle(vis, (int(robot_pos[0]), int(robot_pos[1])), 8, (0, 255, 0), -1)
        cv2.putText(vis, "START", (int(robot_pos[0]) + 10, int(robot_pos[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        for rank, idx in enumerate(ball_order):
            bx, by = int(ball_positions[idx][0]), int(ball_positions[idx][1])
            cv2.circle(vis, (bx, by), 6, (0, 165, 255), -1)
            cv2.putText(vis, str(rank + 1), (bx + 8, by),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        cv2.circle(vis, (int(dropoff_pos[0]), int(dropoff_pos[1])), 10, (255, 0, 0), 2)
        cv2.putText(vis, "HOLE", (int(dropoff_pos[0]) + 12, int(dropoff_pos[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        return vis
