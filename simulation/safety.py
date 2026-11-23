"""
Anti-crash safety filter for rescuer-flown drones.

The rescuer's stick input is only a *desired* velocity. Before it reaches the flight
controller it is smoothed (speed + acceleration limits), then capped so the drone can
never close in on a building, debris or another drone, or leave the geofence.
"""
import numpy as np
import pybullet as p

_A = np.linspace(0, 2 * np.pi, 16, endpoint=False)
RAYS = np.vstack([np.c_[np.cos(_A), np.sin(_A), np.zeros(16)], [[0, 0, 1], [0, 0, -1]]])  # 16 around + up + down
RANGE, START = 2.0, 0.08    # rangefinder reach; rays start just outside the airframe (m)
MARGIN, SEP = 0.4, 0.5      # min gap to obstacles / to other drones (m)
GAIN = 1.5                  # allowed closing speed per metre of free space (1/s)
V_XY, V_Z, ACCEL = 0.9, 0.5, 2.0


def ranges(client, pos):
    """Simulated rangefinders: distance to the nearest surface along each ray (RANGE if clear)."""
    hits = p.rayTestBatch((pos + RAYS * START).tolist(), (pos + RAYS * RANGE).tolist(), physicsClientId=client)
    return np.array([START + h[2] * (RANGE - START) if h[0] >= 0 else RANGE for h in hits])


def _cap(v, u, gap, v_other=np.zeros(3)):
    """Limit the closing speed along unit vector u so it reaches zero at the safety gap (negative gap pushes away)."""
    excess = (v - v_other) @ u - gap * GAIN
    return v - u * excess if excess > 0 else v


def safe_velocity(v_des, pos, v_prev, dist, neighbours, bounds, dt):
    """Filter a desired velocity into a safe one. Returns (velocity, True if the filter had to step in)."""
    v = np.asarray(v_des, float).copy()
    sp = np.linalg.norm(v[:2])
    if sp > V_XY:
        v[:2] *= V_XY / sp
    v[2] = np.clip(v[2], -V_Z, V_Z)
    dv = v - v_prev
    n = np.linalg.norm(dv)
    if n > ACCEL * dt:
        v = v_prev + dv * (ACCEL * dt / n)
    wanted = v.copy()
    for u, d in zip(RAYS[dist < RANGE], dist[dist < RANGE]):
        v = _cap(v, u, d - MARGIN)
    for q, qv in neighbours:
        r = q - pos
        d = np.linalg.norm(r)
        if 1e-6 < d < 1.5:
            v = _cap(v, r / d, d - SEP, qv)
    v = np.clip(v, (bounds[0] - pos) * GAIN, (bounds[1] - pos) * GAIN)
    return v, bool(np.linalg.norm(v - wanted) > 0.05)
