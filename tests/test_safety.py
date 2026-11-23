import numpy as np
import pybullet as p
import pytest
import config as C
import safety
from world import poly_mask


@pytest.fixture
def wall():
    cl = p.connect(p.DIRECT)
    shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 2, 1], physicsClientId=cl)
    p.createMultiBody(0, shape, basePosition=[3, 0, 1], physicsClientId=cl)      # wall face at x = 2.9
    yield cl
    p.disconnect(cl)


def fly(cl, v_des, pos, neighbours=(), secs=8, dt=1 / 48):
    bounds = (np.array([-9, -9, 0.3]), np.array([9, 9, 2.5]))
    v, closest, stepped = np.zeros(3), 9.0, False
    for _ in range(int(secs / dt)):
        v, s = safety.safe_velocity(v_des, pos, v, safety.ranges(cl, pos), neighbours, bounds, dt)
        pos = pos + v * dt
        stepped |= s
        closest = min(closest, 2.9 - pos[0])
    return pos, closest, stepped


def test_full_speed_at_a_wall_stops_at_the_margin(wall):
    pos, closest, stepped = fly(wall, [1, 0, 0], np.array([0.0, 0, 1]))
    assert stepped and closest >= safety.MARGIN - 0.02 and closest < safety.MARGIN + 0.1


def test_sliding_along_the_wall_is_still_allowed(wall):
    pos, closest, _ = fly(wall, [1, 1, 0], np.array([0.0, -1.5, 1]), secs=5)
    assert closest >= safety.MARGIN - 0.02 and pos[1] > 1.0


def test_never_closes_in_on_another_drone(wall):
    other = (np.array([1.5, 0, 1]), np.zeros(3))
    pos, _, stepped = fly(wall, [1, 0, 0], np.array([0.0, 0, 1]), neighbours=[other])
    assert stepped and np.linalg.norm(pos - other[0]) >= safety.SEP - 0.02


def test_geofence_holds_altitude(wall):
    pos, _, _ = fly(wall, [0, 0, -1], np.array([0.0, -5, 1]), secs=4)
    assert pos[2] >= 0.3 - 0.02


def test_drawn_polygon_selects_cells_inside():
    m = poly_mask([[0, 0], [12, 0], [0, 12]])                # lower-left triangle
    assert m[0, 0] and not m[-1, -1] and 0.4 < m.mean() < 0.6
    assert poly_mask(None).all()


def test_base_snaps_outside_the_nearest_edge():
    C.set_base((20, 3))
    try:
        assert np.allclose(C.NORMAL, [1, 0]) and np.allclose(C.BASE, [13.5, 3])
        assert np.allclose(C.ENTRY, [11.75, 3]) and np.allclose(C.stage(C.pad(0, 1)), [11.6, 3])
    finally:
        C.set_base((-1.5, 6.0))
