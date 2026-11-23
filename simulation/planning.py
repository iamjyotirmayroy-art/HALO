"""
Planning: zone splitting, lawn-mower coverage, A* flight paths and
hazard-aware rescuer routes.
"""
import heapq
import numpy as np
from scipy.optimize import linear_sum_assignment
import config as C
from world import to_cell, cell_center

NB8 = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]


def astar(blocked, start, goal, cost=None):
    """Grid A*. blocked: bool grid. cost: optional extra cost per cell. Returns list of cells or None."""
    n = blocked.shape[0]
    if blocked[goal]:
        return None
    openq = [(0.0, 0.0, start, None)]
    came, g = {}, {start: 0.0}
    while openq:
        _, gc, cur, par = heapq.heappop(openq)
        if cur in came:
            continue
        came[cur] = par
        if cur == goal:
            path = [cur]
            while came[path[-1]] is not None:
                path.append(came[path[-1]])
            return path[::-1]
        for dx, dy in NB8:
            nx, ny = cur[0] + dx, cur[1] + dy
            if not (0 <= nx < n and 0 <= ny < n) or blocked[nx, ny]:
                continue
            if dx and dy and (blocked[cur[0] + dx, cur[1]] or blocked[cur[0], cur[1] + dy]):
                continue  # no corner cutting
            step = 1.4142 if dx and dy else 1.0
            if cost is not None:
                step += cost[nx, ny] * (1.4142 if dx and dy else 1.0)
            ng = gc + step
            if ng < g.get((nx, ny), 1e18):
                g[(nx, ny)] = ng
                h = np.hypot(goal[0] - nx, goal[1] - ny)
                heapq.heappush(openq, (ng + h, ng, (nx, ny), cur))
    return None


def inflate(mask, r=1):
    out = mask.copy()
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            out |= np.roll(np.roll(mask, dx, 0), dy, 1)
    return out


def line_clear(blocked, a, b):
    d = np.linalg.norm(b - a)
    for t in np.linspace(0, 1, max(2, int(d / (C.CELL / 2)))):
        q = a + t * (b - a)
        if 0 <= q[0] < C.AREA and 0 <= q[1] < C.AREA and blocked[to_cell(q)]:
            return False
    return True


def flight_path(fly_blocked, a, b):
    """Waypoints from a to b avoiding tall buildings (handles points outside the grid, e.g. the base)."""
    if line_clear(fly_blocked, a, b):
        return [b]
    ca = to_cell(np.clip(a, 0, C.AREA - 1e-3))
    cb = to_cell(np.clip(b, 0, C.AREA - 1e-3))
    blk = fly_blocked.copy()
    blk[ca] = False
    blk[cb] = False
    path = astar(blk, ca, cb)
    if path is None:
        return [b]
    pts = [cell_center(c) for c in path[1:-1]]
    # string-pulling to drop unneeded waypoints
    out, anchor = [], a
    for i, q in enumerate(pts):
        nxt = pts[i + 1] if i + 1 < len(pts) else b
        if not line_clear(fly_blocked, anchor, nxt):
            out.append(q)
            anchor = q
    return out + [b]


# ---------------- zone splitting ----------------
def split_zones(cells, drone_xy):
    """
    Evenly split a set of cells among drones: sort into vertical strips with equal
    cell counts, then match strips to drones by distance (Hungarian algorithm).
    Returns list of cell-lists aligned with drone_xy.
    """
    k = len(drone_xy)
    if k == 0 or not cells:
        return [[] for _ in range(k)]
    cells = sorted(cells, key=lambda c: (c[0], c[1]))
    chunks = [list(ch) for ch in np.array_split(np.array(cells), k)]
    chunks = [[tuple(c) for c in ch] for ch in chunks]
    cent = [np.mean([cell_center(c) for c in ch], axis=0) if ch else np.zeros(2) for ch in chunks]
    cost = np.array([[np.linalg.norm(d - ce) for ce in cent] for d in drone_xy])
    rows, cols = linear_sum_assignment(cost)
    out = [[] for _ in range(k)]
    for r, c in zip(rows, cols):
        out[r] = chunks[c]
    return out


def coverage_waypoints(cells, start_xy):
    """Lawn-mower (boustrophedon) waypoints over a zone, lanes LANE metres apart."""
    if not cells:
        return []
    arr = np.array(cells)
    step = max(1, int(round(C.LANE / C.CELL)))
    x_lo, x_hi = arr[:, 0].min(), arr[:, 0].max()
    lanes = list(range(x_lo + step // 2, x_hi + 1, step))
    if not lanes or x_hi - lanes[-1] > step // 2:
        lanes.append(x_hi)
    # start from the lane end nearest the drone
    if abs(cell_center((lanes[0], 0))[0] - start_xy[0]) > abs(cell_center((lanes[-1], 0))[0] - start_xy[0]):
        lanes = lanes[::-1]
    wps, up = [], None
    for ln in lanes:
        ys = arr[np.abs(arr[:, 0] - ln) <= step // 2 + 0][:, 1]
        if len(ys) == 0:
            continue
        y0, y1 = ys.min(), ys.max()
        if up is None:
            up = abs(cell_center((0, y0))[1] - start_xy[1]) < abs(cell_center((0, y1))[1] - start_xy[1])
        seq = range(y0, y1 + 1, 2) if up else range(y1, y0 - 1, -2)
        lane_pts = [cell_center((ln, y)) for y in seq]
        if lane_pts:
            wps.append(lane_pts[0]); wps.extend(lane_pts[1:-1]); wps.append(lane_pts[-1])
        up = not up
    return wps


# ---------------- rescuer routes ----------------
def rescuer_route(world, goal_xy):
    """
    Compare the direct (shortest walkable) route with the hazard-aware safe route.
    Rescuers can't walk through tall buildings or a sealed collapse; the safe route also
    avoids the unstable building and keeps radiation dose under the limit.
    """
    start, goal = to_cell(C.ENTRY), to_cell(goal_xy)
    walls = world.tall | world.sealed
    target_note = ""
    if walls[goal] or world.weak[goal]:
        # victim inside a collapsed / unstable building: route to the nearest safe perimeter cell
        cand = [(i, j) for i in range(C.N) for j in range(C.N)
                if not walls[i, j] and not world.weak[i, j]]
        goal = min(cand, key=lambda c: np.hypot(c[0] - goal[0], c[1] - goal[1]))
        target_note = "victim inside collapsed structure: route ends at safest perimeter point, heavy-rescue team needed"
    direct = astar(walls, start, goal)
    hazard_cost = world.radiation * 6.0 + world.rubble * 0.6
    safe_blocked = walls | world.weak
    safe = astar(safe_blocked, start, goal, cost=hazard_cost)

    def stats(path):
        if path is None:
            return None
        length = sum(np.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(path, path[1:])) * C.CELL
        dose = float(sum(world.radiation[c] for c in path) * C.CELL)
        weak = int(sum(world.weak[c] for c in path))
        return dict(length_m=round(length, 1), dose=round(dose, 2), unstable_cells=weak)

    sd, ss = stats(direct), stats(safe)
    direct_unsafe = sd is not None and (sd["dose"] > C.RAD_DOSE_LIMIT or sd["unstable_cells"] > 0)
    status = "SAFE ROUTE" if ss and ss["dose"] <= C.RAD_DOSE_LIMIT else "NO SAFE ROUTE - specialist team"
    reason = ""
    if ss and direct_unsafe:
        status = "REROUTED (direct route unsafe)"
        reason = " + ".join(r for r, bad in [(f"radiation dose {sd['dose']} > {C.RAD_DOSE_LIMIT}", sd["dose"] > C.RAD_DOSE_LIMIT),
                                             ("crosses an unstable structure", sd["unstable_cells"] > 0)] if bad)
    return dict(direct=sd, safe=ss, direct_unsafe=direct_unsafe, status=status, note=target_note, reason=reason,
                safe_path=[cell_center(c).round(2).tolist() for c in safe] if safe else [],
                direct_path=[cell_center(c).round(2).tolist() for c in direct] if direct else [])
