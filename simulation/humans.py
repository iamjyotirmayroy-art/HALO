"""
Human victims built from body parts (head, hair, torso, arms, legs, shoes), in four kinds the detector
is trained on: a whole person lying down, or only a hand, a foot or a head showing out of the rubble.
Used by the disaster scene and by the synthetic training-data generator (train_detector.py).
"""
import numpy as np
import pybullet as p

SKIN = [(0.94, 0.78, 0.66), (0.87, 0.67, 0.53), (0.72, 0.52, 0.38), (0.55, 0.38, 0.26), (0.38, 0.26, 0.18)]
HAIR = [(0.1, 0.07, 0.05), (0.3, 0.2, 0.1), (0.55, 0.4, 0.2), (0.15, 0.15, 0.15), (0.7, 0.68, 0.62)]
CLOTH = [(0.75, 0.15, 0.15), (0.2, 0.45, 0.8), (0.2, 0.25, 0.45), (0.25, 0.25, 0.25), (0.15, 0.5, 0.3), (0.9, 0.75, 0.2),
         (0.85, 0.85, 0.82), (0.5, 0.3, 0.6), (0.6, 0.4, 0.25), (0.9, 0.5, 0.15)]
KINDS = ("person", "hand", "foot", "head")
DEBRIS = [(0.45, 0.38, 0.30), (0.55, 0.52, 0.48), (0.62, 0.45, 0.32), (0.4, 0.4, 0.42)]


def style(rng):
    pick = lambda pal: (*pal[rng.integers(len(pal))], 1)
    return dict(skin=pick(SKIN), hair=pick(HAIR), shirt=pick(CLOTH), pants=pick(CLOTH), shoe=(0.12, 0.1, 0.08, 1))


def build(cl, kind, xy, yaw, rng, st=None):
    """Spawn one victim at xy. Returns (body-part ids, debris ids). Debris hides the buried part of partial victims."""
    st = st or style(rng)
    x, y = xy
    c, s = np.cos(yaw), np.sin(yaw)
    W = lambda px, py, z: [x + c * px - s * py, y + s * px + c * py, z]
    Q = lambda r=0.0, pt=0.0, yw=0.0: p.getQuaternionFromEuler([r, pt, yaw + yw])

    def part(shape, pos, orn, rgba, **k):
        return p.createMultiBody(0, -1, p.createVisualShape(shape, rgbaColor=rgba, physicsClientId=cl, **k), pos, orn, physicsClientId=cl)

    def debris(px, py, hx, hy, hz, tilt=0.0):
        he = [hx, hy, hz]
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=he, physicsClientId=cl)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=he, physicsClientId=cl, rgbaColor=(*DEBRIS[rng.integers(len(DEBRIS))], 1))
        return p.createMultiBody(0, col, vis, W(px, py, hz), Q(tilt, 0, rng.uniform(-0.4, 0.4)), physicsClientId=cl)

    head = lambda px, z: [part(p.GEOM_SPHERE, W(px, 0, z), Q(), st["skin"], radius=0.065),
                          part(p.GEOM_SPHERE, W(px + 0.025, 0, z + 0.015), Q(), st["hair"], radius=0.06)]
    if kind == "person":
        a = rng.uniform(0.2, 0.9)                                         # how far the arms are spread
        ids = head(0.36, 0.07) + [part(p.GEOM_BOX, W(0.17, 0, 0.06), Q(), st["shirt"], halfExtents=[0.13, 0.09, 0.045]),
                                  part(p.GEOM_BOX, W(-0.02, 0, 0.055), Q(), st["pants"], halfExtents=[0.06, 0.085, 0.04])]
        for sd in (1, -1):
            ids += [part(p.GEOM_CAPSULE, W(0.24 - 0.06 * np.cos(a), sd * (0.1 + 0.06 * np.sin(a)), 0.045), Q(0, np.pi / 2, sd * a), st["shirt"], radius=0.032, length=0.1),
                    part(p.GEOM_CAPSULE, W(0.12 - 0.16 * np.cos(a), sd * (0.12 + 0.17 * np.sin(a)), 0.04), Q(0, np.pi / 2, sd * a), st["skin"], radius=0.027, length=0.13),
                    part(p.GEOM_CAPSULE, W(-0.24, sd * 0.055, 0.045), Q(0, np.pi / 2, sd * 0.06), st["pants"], radius=0.042, length=0.3),
                    part(p.GEOM_BOX, W(-0.43, sd * 0.06, 0.04), Q(), st["shoe"], halfExtents=[0.04, 0.03, 0.035])]
        return ids, ([debris(-0.25, 0, 0.14, 0.16, 0.05, 0.1)] if rng.random() < 0.4 else [])
    if kind == "hand":                                                    # forearm and hand reaching out of a rubble pile
        t = rng.uniform(0.3, 0.8)
        ids = [part(p.GEOM_CAPSULE, W(0.05 * np.sin(t), 0, 0.14), Q(0, t), st["skin"], radius=0.028, length=0.16),
               part(p.GEOM_CAPSULE, W(-0.03 * np.sin(t), 0, 0.05), Q(0, t), st["shirt"], radius=0.034, length=0.06),
               part(p.GEOM_BOX, W(0.13 * np.sin(t) + 0.02, 0, 0.24), Q(0, t), st["skin"], halfExtents=[0.035, 0.03, 0.012]),
               part(p.GEOM_CAPSULE, W(0.13 * np.sin(t) + 0.06, 0, 0.26), Q(0, t + 0.3), st["skin"], radius=0.012, length=0.05)]
        return ids, [debris(rng.uniform(-0.18, -0.1), rng.uniform(-0.1, 0.1), 0.12, 0.1, 0.06), debris(-0.05, 0.14, 0.09, 0.08, 0.05),
                     debris(-0.06, -0.15, 0.08, 0.09, 0.045)]
    if kind == "foot":                                                    # lower leg and shoe sticking out from under a slab
        ids = [part(p.GEOM_CAPSULE, W(0.04, 0, 0.045), Q(0, np.pi / 2), st["pants"], radius=0.042, length=0.14),
               part(p.GEOM_BOX, W(0.16, 0, 0.045), Q(), st["shoe"], halfExtents=[0.045, 0.032, 0.04])]
        return ids, [debris(-0.2, 0, 0.16, 0.17, 0.07, rng.uniform(-0.1, 0.1))]
    ids = head(0.0, 0.08) + [part(p.GEOM_BOX, W(-0.11, 0, 0.05), Q(), st["shirt"], halfExtents=[0.05, 0.1, 0.04])]   # head out of the rubble
    return ids, [debris(-0.28, 0, 0.16, 0.18, 0.08, rng.uniform(-0.1, 0.1)), debris(-0.08, 0.17, 0.08, 0.07, 0.06)]


def decoy(cl, kind, xy, rng):
    """Warm things that are not people (the detector must ignore them): smouldering debris or an animal."""
    x, y = xy
    yaw = p.getQuaternionFromEuler([0, 0, rng.uniform(0, 6.28)])
    vis = lambda shape, rgba, **k: p.createVisualShape(shape, rgbaColor=rgba, physicsClientId=cl, **k)
    if kind == "fire":
        return [p.createMultiBody(0, -1, vis(p.GEOM_BOX, (0.15, 0.15, 0.15, 1), halfExtents=[0.2, 0.2, 0.075]), [x, y, 0.075], yaw, physicsClientId=cl),
                p.createMultiBody(0, -1, vis(p.GEOM_CYLINDER, (1.0, 0.45, 0.05, 1), radius=0.12, length=0.35), [x, y, 0.325], physicsClientId=cl)]
    return [p.createMultiBody(0, -1, vis(p.GEOM_BOX, (0.55, 0.35, 0.15, 1), halfExtents=[0.18, 0.07, 0.07]), [x, y, 0.07], yaw, physicsClientId=cl),
            p.createMultiBody(0, -1, vis(p.GEOM_SPHERE, (0.5, 0.32, 0.14, 1), radius=0.06), [x + 0.2, y, 0.09], physicsClientId=cl)]
