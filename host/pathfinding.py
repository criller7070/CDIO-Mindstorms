#!/usr/bin/env python3
"""
A* path planner for CDIO golf field.

Coordinate system (image space):
  - x increases left → right
  - y increases top  → bottom
  - heading 0° = facing right, -90° = facing up, 90° = facing down, 180° = facing left
  - TURN positive = clockwise (matches pybricks DriveBase.turn)
"""

import heapq
import math
import cv2
import numpy as np
from itertools import permutations


# CALIBRATION - set FIELD_WIDTH_MM and FIELD_HEIGHT_MM to real measurements
FIELD_WIDTH_MM = 1670   # field width:  167.0 cm
FIELD_HEIGHT_MM = 1215  # field height: 121.5 cm

# Straight approach segment injected before each ball pickup.  Must exceed the
# nose_to_aruco trim distance (248 mm) so the trimmed endpoint lands inside the
# guaranteed-straight segment and the robot's final approach is always direct.
BALL_APPROACH_MM = 400


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
                 robot_length_mm=0,
                 pivot_offset_mm=0.0,
                 gate_open=True,
                 gate_arm_mm=0.0,
                 aruco_from_back_frac=0.5):
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

        # Robot body dimensions in pixels.
        self.robot_half_width_px  = (robot_width_mm  / 2.0) * self.px_per_mm
        self.robot_half_length_px = (robot_length_mm / 2.0) * self.px_per_mm
        self.robot_half_length_mm = robot_length_mm / 2.0

        # The nav loop tracks the ArUco marker, not the geometric centre.
        # To land the nose on a ball, trim back by the ArUco-to-nose distance.
        nose_to_aruco_mm = robot_length_mm * (1.0 - aruco_from_back_frac)
        self.nose_to_aruco_px = nose_to_aruco_mm * self.px_per_mm

        # Effective lateral half-width including open gate arms.
        gate_extra_px = (gate_arm_mm * self.px_per_mm) if gate_open else 0.0
        self.effective_half_width_px = self.robot_half_width_px + gate_extra_px

        # Center-obstacle clearance: the robot can approach the cross from any
        # direction, so use whichever robot dimension is larger (half-length when
        # approaching head-on, effective half-width when approaching from the side).
        self.center_clearance_px = max(self.effective_half_width_px,
                                       self.robot_half_length_px)

        # Turn pivot offset (mm behind centre → px).  The robot rotates about
        # this point, not its centre, so command generation compensates for it.
        self.pivot_offset_mm = pivot_offset_mm
        self.pivot_offset_px = pivot_offset_mm * self.px_per_mm

        self.gw = field_px_w // self.GRID_SCALE + 1
        self.gh = field_px_h // self.GRID_SCALE + 1

        self.grid = self._build_grid()

    # GRID CONSTRUCTION

    def _build_grid(self):
        gs = self.GRID_SCALE
        # Wall margin: keep robot center far enough that the widest part of the
        # body (including open gate arms) clears the wall.
        margin = max(1, int(self.wall_margin + self.effective_half_width_px) // gs)
        cx = (self.center[0] - self.x0) // gs
        cy = (self.center[1] - self.y0) // gs
        # Center obstacle: use the larger of effective half-width and half-length
        # so the body clears the cross regardless of approach angle.
        cr = max(1, int(self.center_radius + self.center_clearance_px) // gs)

        grid = [[False] * self.gh for _ in range(self.gw)]
        for gx in range(self.gw):
            for gy in range(self.gh):
                # Outside actual field hull (non-rectangular walls).  Keep the
                # robot centre at least effective-half-width inside the wall.
                if self._field_hull is not None:
                    px, py = self._to_px(gx, gy)
                    if cv2.pointPolygonTest(
                            self._field_hull, (float(px), float(py)), True) \
                            < self.effective_half_width_px:
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

    # COORDINATE HELPERS

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

    def snap_to_navigable(self, px, py):
        """Return pixel coords of the nearest obstacle-free cell to (px, py)."""
        gx, gy = self._to_grid(px, py)
        free_gx, free_gy = self._nearest_free(gx, gy)
        return self._to_px(free_gx, free_gy)

    # A* SEARCH

    def astar(self, start_px, end_px):
        """shortest path from start_px to end_px. returns list of (px, py) or None."""
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

    # K-MEANS++ CLUSTERING (capacity-aware multi-trip)

    def cluster_balls(self, ball_positions, capacity):
        """split balls into groups of ≤ capacity using K-Means++. returns list of index lists."""
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

    # BALL PICKUP ROUTING

    def _forced_approach_dir(self, ball_pos):
        """
        Return a forced (ux, uy) approach direction based on the ball's field position,
        or None for open-field balls (caller uses A* walkback instead).

        (ux, uy) is the direction the robot MOVES when arriving at the ball:
            approach_point = ball - approach_px * (ux, uy)
        so the approach_point is in the open space the robot comes FROM.

        Priority order:
          1. Corner (near two walls): bisect diagonally from the open-field quadrant.
          2. Single wall: approach perpendicular from field toward wall.
          3. Centre obstacle proximity: approach radially inward (from open space
             toward the obstacle), so the gate sweeps cleanly through the ball.
          4. Open field: return None.
        """
        bx, by = float(ball_pos[0]), float(ball_pos[1])
        x0, y0, x1, y1 = self.bounds
        # only force approach direction for balls genuinely hugging a wall.
        # robot half-width was inflating this to ~240mm, catching open-field balls.
        threshold = self.wall_margin

        near_left   = bx - x0 < threshold
        near_right  = x1 - bx < threshold
        near_top    = by - y0 < threshold
        near_bottom = y1 - by < threshold

        _s2 = 1.0 / math.sqrt(2)
        # Corners: diagonal from the open-field quadrant into the corner
        if near_left  and near_top:    return (-_s2, -_s2)
        if near_right and near_top:    return ( _s2, -_s2)
        if near_left  and near_bottom: return (-_s2,  _s2)
        if near_right and near_bottom: return ( _s2,  _s2)

        # Single wall: perpendicular from field toward wall
        if near_left:   return (-1.0,  0.0)
        if near_right:  return ( 1.0,  0.0)
        if near_top:    return ( 0.0, -1.0)
        if near_bottom: return ( 0.0,  1.0)

        # Centre obstacle: if ball is within a small buffer beyond the obstacle
        # edge, force inward radial approach (from open space toward obstacle).
        # 80 px ≈ 57 mm — enough to catch balls sitting beside the obstacle
        # without swallowing open-field balls far from the centre.
        if self.center is not None and self.center_clearance_px > 0:
            cx, cy = float(self.center[0]), float(self.center[1])
            ddx, ddy = bx - cx, by - cy
            dist = math.hypot(ddx, ddy)
            ctr_threshold = self.center_clearance_px + 80.0
            if 0 < dist < ctr_threshold:
                return ((cx - bx) / dist, (cy - by) / dist)

        return None

    # Keep old name as alias so any external callers still work.
    _wall_approach_dir = _forced_approach_dir

    def _pickup_seg(self, from_pos, ball_pos):
        """A* path to ball_pos with a guaranteed straight BALL_APPROACH_MM final segment."""
        bx, by = float(ball_pos[0]), float(ball_pos[1])
        fx, fy = float(from_pos[0]), float(from_pos[1])
        approach_px = BALL_APPROACH_MM * self.px_per_mm

        # Wall, corner, or centre-adjacent: use forced direction.
        wall_dir = self._forced_approach_dir(ball_pos)
        if wall_dir is not None:
            ux, uy = wall_dir
            apx = bx - approach_px * ux
            apy = by - approach_px * uy
            if not self._is_in_obstacle(apx, apy):
                transit = self.astar((int(fx), int(fy)), (int(apx), int(apy)))
                if transit is not None:
                    return transit[:-1] + [(apx, apy), (bx, by)]
            # Fallback for wall balls
            seg = self.astar((int(fx), int(fy)), (int(bx), int(by)))
            if seg:
                seg[-1] = (bx, by)
            return seg

        # Open-field ball: A* to ball, walk back approach_px along path.
        seg = self.astar((int(fx), int(fy)), (int(bx), int(by)))
        if seg is None:
            return None
        seg[-1] = (bx, by)  # exact endpoint

        # Walk backward from the ball end to find the approach_point.
        remaining = approach_px
        for i in range(len(seg) - 1, 0, -1):
            p0x, p0y = float(seg[i - 1][0]), float(seg[i - 1][1])
            p1x, p1y = float(seg[i][0]),     float(seg[i][1])
            step = math.hypot(p1x - p0x, p1y - p0y)
            if remaining <= step:
                t = remaining / step
                apx = p1x - t * (p1x - p0x)
                apy = p1y - t * (p1y - p0y)
                return seg[:i] + [(apx, apy), (bx, by)]
            remaining -= step

        # Path shorter than approach_px: return as-is (ball very close).
        return seg

    # OPTIMAL ROUTE (TSP brute-force for ≤ ~8 balls)

    # Brute-force n! is fine on a cost matrix (no A* per permutation).
    # Cap at 9 to keep factorial under ~360k iterations; use 2-opt above.
    _BRUTE_FORCE_LIMIT = 9

    def optimal_route(self, robot_pos, ball_positions, dropoff_pos):
        """best visit order + A* path segments. brute-force ≤ 9 balls, greedy + 2-opt otherwise."""
        n = len(ball_positions)
        if n == 0:
            path = self.astar(robot_pos, dropoff_pos)
            return [], ([path] if path else [])

        # nodes: 0=robot, 1..n=balls, n+1=dropoff
        nodes = [robot_pos] + list(ball_positions) + [dropoff_pos]
        N = len(nodes)

        # Pre-compute all pairwise segments and costs once.
        seg_cache = [[None] * N for _ in range(N)]
        cost_cache = [[float('inf')] * N for _ in range(N)]
        for i in range(N):
            for j in range(N):
                if i == j:
                    cost_cache[i][j] = 0.0
                    continue
                is_ball_dest = 1 <= j <= n
                if is_ball_dest:
                    seg = self._pickup_seg(nodes[i], nodes[j])
                else:
                    seg = self.astar(
                        (int(nodes[i][0]), int(nodes[i][1])),
                        (int(nodes[j][0]), int(nodes[j][1])))
                seg_cache[i][j] = seg
                cost_cache[i][j] = self._path_length(seg) if seg else float('inf')

        def route_cost(order):
            # order: list of ball node indices (1-based into nodes)
            total = cost_cache[0][order[0]]
            for k in range(len(order) - 1):
                total += cost_cache[order[k]][order[k + 1]]
            total += cost_cache[order[-1]][n + 1]
            return total

        # Ball node indices are 1..n
        ball_nodes = list(range(1, n + 1))

        if n <= self._BRUTE_FORCE_LIMIT:
            best_order, best_cost = None, float('inf')
            for perm in permutations(ball_nodes):
                c = route_cost(perm)
                if c < best_cost:
                    best_cost = c
                    best_order = list(perm)
        else:
            # Nearest-neighbour seed
            remaining = list(ball_nodes)
            best_order = []
            cur = 0
            while remaining:
                nxt = min(remaining, key=lambda j: cost_cache[cur][j])
                best_order.append(nxt)
                cur = nxt
                remaining.remove(nxt)

            # 2-opt improvement
            improved = True
            while improved:
                improved = False
                for i in range(len(best_order) - 1):
                    for j in range(i + 2, len(best_order)):
                        before = (cost_cache[best_order[i - 1] if i > 0 else 0][best_order[i]]
                                  + cost_cache[best_order[j]][best_order[j + 1] if j + 1 < len(best_order) else n + 1])
                        after  = (cost_cache[best_order[i - 1] if i > 0 else 0][best_order[j]]
                                  + cost_cache[best_order[i]][best_order[j + 1] if j + 1 < len(best_order) else n + 1])
                        if after < before - 1e-6:
                            best_order[i:j + 1] = best_order[i:j + 1][::-1]
                            improved = True

        if best_order is None:
            best_order = ball_nodes

        # Reconstruct segments in best order
        segs = []
        prev = 0
        for node_idx in best_order:
            seg = seg_cache[prev][node_idx]
            if seg is None:
                seg = [nodes[prev], nodes[node_idx]]
            segs.append(seg)
            prev = node_idx
        end_seg = seg_cache[prev][n + 1]
        if end_seg:
            segs.append(end_seg)

        # Convert node indices back to ball_positions indices (0-based)
        ball_order = [idx - 1 for idx in best_order]
        return ball_order, segs

    @staticmethod
    def _path_length(path):
        total = 0.0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            total += (dx ** 2 + dy ** 2) ** 0.5
        return total

    # PATH SIMPLIFICATION (Ramer-Douglas-Peucker)

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

    # COMMAND GENERATION

    def paths_to_commands(self, path_segs, initial_heading_deg=0):
        """convert route segments to EV3 command strings for commands.txt."""
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
        # Use the effective obstacle margin (wall keepout + robot body clearance)
        # so the simulator's margin band matches the actual navigable boundary.
        wm_mm = (self.wall_margin + self.effective_half_width_px) / self.px_per_mm
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
            # Camera pixels-per-mm so the simulator can render at the exact
            # same scale as the debug overlay (same physical mm → same screen px).
            "# SIM_PXPERMM: {:.5f}".format(self.px_per_mm),
            # Turn pivot offset (mm behind centre) so the simulator swings the
            # robot about the same point the commands were compensated for.
            "# SIM_PIVOT: {:.1f}".format(self.pivot_offset_mm),
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

    # MULTI-TRIP PLANNING

    def _segs_to_commands(self, path_segs, is_last_trip, heading, cur_pos,
                          face_deg=None):
        """convert one trip's segments to commands, trimming collect legs to nose-stop.

        returns (commands, heading, cur_pos, driven_segs) so the next trip starts
        from the real stopped position, not the segment's nominal endpoint.
        """
        commands = []
        driven_segs = []
        n_collect = len(path_segs) - 1
        # Trim so the nose overshoots the ball by ~75 mm.
        # Measured ArUco-to-nose = 210 mm. Capture zone = 0..90 mm past nose.
        # T=135 mm → nose at ball + (210-135) = ball + 75 mm.  Mid-capture-zone.
        TRIM_MM = 135.0
        half_len_px = TRIM_MM * self.px_per_mm

        for leg, seg in enumerate(path_segs):
            is_collect = leg < n_collect
            if is_collect and len(seg) >= 2:
                # Preserve the last 2 points of collect segs as-is (no RDP).
                # _pickup_seg stores [exact_approach_pt, exact_ball_center] there,
                # so the approach direction and endpoint are never altered by
                # simplification.
                transit_raw = list(seg[:-2]) if len(seg) > 2 else []
                transit = self.simplify(transit_raw, eps=8) if transit_raw else []
                pts = transit + [tuple(seg[-2]), tuple(seg[-1])]
            else:
                pts = self.simplify(seg, eps=8)
            if not pts:
                continue

            # Start this leg from where the robot ACTUALLY is.  The segment's
            # nominal start (pts[0]) is the previous waypoint (a ball centre),
            # but after a front-intake collect the robot stopped half a body
            # length short of it.  Overriding pts[0] with the real pose makes
            # the first move re-join the planned path cleanly instead of
            # translating the whole leg by the collect shortfall.
            pts = [tuple(cur_pos)] + [tuple(p) for p in pts[1:]]

            # Front-intake collection: the intake sits in the middle of the
            # front face, so the ball must end up JUST in front of the robot.
            # Trim the planned path by half a body length of ARC length from the
            # end, so the robot's CENTRE stops there and its NOSE lands on the
            # ball - driving the centre onto the ball would shove it away.
            # Walking back along the polyline (not just the last segment) keeps
            # the nose on the ball even when the final hop is shorter than the
            # half-length.  Pivot compensation in the move loop then lands the
            # centre on this trimmed endpoint automatically - no post-hoc command
            # surgery needed.  (Wire a LIFT_DOWN / intake command in right after
            # this leg once collection is implemented.)
            if is_collect and half_len_px > 0:
                remaining = half_len_px
                while len(pts) >= 2:
                    ax, ay = pts[-2]
                    bx, by = pts[-1]
                    seg_len = ((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5
                    if seg_len <= 1e-6:
                        pts.pop()
                        continue
                    if seg_len >= remaining:
                        t = (seg_len - remaining) / seg_len
                        pts[-1] = (ax + (bx - ax) * t, ay + (by - ay) * t)
                        break
                    remaining -= seg_len
                    pts.pop()

            driven_segs.append(list(pts))

            leg_cmds = []
            for i in range(1, len(pts)):
                x0, y0 = pts[i - 1]
                x1, y1 = pts[i]
                dx, dy = x1 - x0, y1 - y0
                dist_px = (dx ** 2 + dy ** 2) ** 0.5
                if dist_px < self.GRID_SCALE:
                    continue

                # Off-centre pivot compensation.  The robot rotates about a point
                # L px behind its centre, so a turn swings the centre.  To still
                # land the centre on the next waypoint, aim along
                #   V = (waypoint - centre) + L * heading_unit
                # and drive |V| - L.  With L = 0 this reduces to the old maths.
                L = self.pivot_offset_px
                hr = np.radians(heading)
                vx = dx + L * np.cos(hr)
                vy = dy + L * np.sin(hr)
                target_heading = float(np.degrees(np.arctan2(vy, vx)))
                turn = (target_heading - heading + 180.0) % 360.0 - 180.0
                if abs(turn) > 2:
                    leg_cmds.append("TURN:{}".format(int(round(turn))))
                    heading = target_heading

                dist_px_eff = (vx ** 2 + vy ** 2) ** 0.5 - L
                dist_mm = max(1, int(dist_px_eff / self.px_per_mm))
                leg_cmds.append("FORWARD:{}".format(dist_mm))

            commands.extend(leg_cmds)
            # The move loop lands the centre on pts[-1]; for a collect leg that
            # is already the pulled-back nose-stop, so this is the real pose.
            cur_pos = pts[-1]

            if not is_collect and is_last_trip:
                if face_deg is not None:
                    turn = (face_deg - heading + 180.0) % 360.0 - 180.0
                    if abs(turn) > 2:
                        commands.append("TURN:{}".format(int(round(turn))))
                        heading = face_deg
                commands.append("STOP")

        return commands, heading, cur_pos, driven_segs

    def plan_trips(self, robot_pos, ball_positions, dropoff_pos,
                   capacity=6, initial_heading_deg=0, face_deg=None):
        """multi-trip planner: clusters balls, picks best first trip, routes each optimally."""
        meta = self._sim_metadata(robot_pos, dropoff_pos)

        # A ball is reachable if:
        #   - Its center is outside the obstacle zone (A* fallback can reach it), OR
        #   - Its center is inside the obstacle (e.g. wall-adjacent) but its
        #     wall-perpendicular approach point is navigable.
        approach_px = BALL_APPROACH_MM * self.px_per_mm
        def _approach_reachable(bp):
            bx, by = float(bp[0]), float(bp[1])
            if not self._is_in_obstacle(bx, by):
                return True  # ball outside obstacle — A* can reach it directly
            # Ball is inside obstacle zone; check approach point.
            wall_dir = self._forced_approach_dir(bp)
            if wall_dir is not None:
                ux, uy = wall_dir
            else:
                dx, dy = bx - float(robot_pos[0]), by - float(robot_pos[1])
                dist = math.hypot(dx, dy)
                ux, uy = (dx / dist, dy / dist) if dist > 1e-6 else (1.0, 0.0)
            apx = bx - approach_px * ux
            apy = by - approach_px * uy
            return not self._is_in_obstacle(apx, apy)

        reachable = [bp for bp in ball_positions if _approach_reachable(bp)]
        skipped_balls = [bp for bp in ball_positions if not _approach_reachable(bp)]
        if skipped_balls:
            print("Skipping {} ball(s) with unreachable approach point.".format(len(skipped_balls)))
        ball_positions = reachable
        # Expose the ACTUAL routed list + skipped balls so the overlay can label
        # them correctly.  _debug_ball_order indexes into this reachable list,
        # not the caller's original (which still includes the skipped balls).
        self._debug_ball_positions = ball_positions
        self._debug_skipped_balls = skipped_balls

        n = len(ball_positions)
        if n == 0:
            path = self.astar(
                (int(robot_pos[0]), int(robot_pos[1])),
                (int(dropoff_pos[0]), int(dropoff_pos[1])),
            )
            cmds = ["SPEED:300"] + meta
            if path:
                trip_cmds, _, _, driven = self._segs_to_commands(
                    [path], True, initial_heading_deg,
                    (float(robot_pos[0]), float(robot_pos[1])), face_deg=face_deg)
                cmds += trip_cmds
                self._debug_path_segs = driven
                self._debug_ball_order = []
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

        # Build commands, threading heading AND real pose across all trips
        commands = ["SPEED:300"] + meta
        heading = float(initial_heading_deg)
        cur_pos = (float(robot_pos[0]), float(robot_pos[1]))

        # Trip 1 starts from robot_pos
        trip_cmds, heading, cur_pos, driven = self._segs_to_commands(
            best_first_segs, k == 1, heading, cur_pos,
            face_deg=face_deg if k == 1 else None)
        commands += ["# Trip 1/{} – cluster {}".format(k, best_first)] + trip_cmds

        # Remaining trips start from the hole.  debug_segs collects the ACTUAL
        # driven polylines (trimmed to the nose-stops) so the overlay matches
        # the commands, not the raw A* paths to the ball centres.
        trip_num = 2
        debug_segs = list(driven)
        debug_order = list(best_first_order or [])
        for i in range(k):
            if i == best_first:
                continue
            segs = hole_segs[i]
            if segs:
                is_last = (trip_num == k)
                trip_cmds, heading, cur_pos, driven = self._segs_to_commands(
                    segs, is_last, heading, cur_pos,
                    face_deg=face_deg if is_last else None)
                commands += ["# Trip {}/{} – cluster {}".format(trip_num, k, i)] + trip_cmds
                debug_segs.extend(driven)
                debug_order.extend(hole_ball_orders[i])
            trip_num += 1

        self._debug_path_segs = debug_segs
        self._debug_ball_order = debug_order
        return commands

    # DEBUG VISUALIZATION

    def debug_overlay(self, frame, path_segs, robot_pos, ball_positions,
                      dropoff_pos, ball_order, skipped=None):
        """draw the planned route on a copy of frame for visual inspection."""
        import cv2
        vis = frame.copy()
        colors = [(0, 255, 255), (255, 128, 0), (128, 0, 255)]

        def _ipt(p):
            return (int(round(p[0])), int(round(p[1])))

        dropoff_i = _ipt(dropoff_pos)
        for i, seg in enumerate(path_segs):
            color = colors[i % len(colors)]
            for j in range(1, len(seg)):
                cv2.line(vis, _ipt(seg[j - 1]), _ipt(seg[j]), color, 2)
            # Mark where the robot actually stops at the end of each leg.  Legs
            # that end at the hole are the delivery legs; everything else is a
            # collect leg whose endpoint is the nose-stop in front of a ball.
            if seg:
                end = _ipt(seg[-1])
                if abs(end[0] - dropoff_i[0]) > 12 or abs(end[1] - dropoff_i[1]) > 12:
                    cv2.circle(vis, end, 5, (0, 255, 0), 2)
                    cv2.putText(vis, "stop", (end[0] + 6, end[1] - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)

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

        # Skipped balls (unreachable) - greyed out so it's clear the route
        # ignores them.  Drawn first so routed balls sit on top.
        for sx, sy in (skipped or []):
            sx, sy = int(sx), int(sy)
            cv2.circle(vis, (sx, sy), 6, (120, 120, 120), 1)
            cv2.line(vis, (sx - 5, sy - 5), (sx + 5, sy + 5), (120, 120, 120), 1)
            cv2.line(vis, (sx - 5, sy + 5), (sx + 5, sy - 5), (120, 120, 120), 1)
            cv2.putText(vis, "skip", (sx + 8, sy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)

        for rank, idx in enumerate(ball_order):
            bx, by = int(ball_positions[idx][0]), int(ball_positions[idx][1])
            cv2.circle(vis, (bx, by), 6, (0, 165, 255), -1)
            cv2.putText(vis, str(rank + 1), (bx + 8, by),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        cv2.circle(vis, (int(dropoff_pos[0]), int(dropoff_pos[1])), 10, (255, 0, 0), 2)
        cv2.putText(vis, "HOLE", (int(dropoff_pos[0]) + 12, int(dropoff_pos[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        return vis
