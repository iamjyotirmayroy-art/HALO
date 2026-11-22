"""
Disaster scene: buildings, rubble, a damaged building, hazards, victims, decoys
and phones. Holds both the PyBullet bodies (3D) and the grid maps the AI uses.
"""
from dataclasses import dataclass, field
import numpy as np
import pybullet as p
import config as C


def to_cell(xy):
    return (int(np.clip(xy[0] // C.CELL, 0, C.N - 1)), int(np.clip(xy[1] // C.CELL, 0, C.N - 1)))


def cell_center(c):
    return np.array([(c[0] + 0.5) * C.CELL, (c[1] + 0.5) * C.CELL])


@dataclass
class HeatSource:
    id: str
    kind: str              # person | hand | foot | head | buried | fire | animal
    pos: np.ndarray
    temp_c: float
    is_human: bool
    has_phone: bool = False
    visible: bool = True   # False = fully buried (no thermal / camera view)
    body_id: int = -1
    reported: bool = False


@dataclass
class World:
    client: int
    tall: np.ndarray = field(default=None)      # intact buildings: block flight
    rubble: np.ndarray = field(default=None)    # low debris: slows rescuers
    weak: np.ndarray = field(default=None)      # damaged / unstable structure
    sealed: np.ndarray = field(default=None)    # collapsed: no way in or out
    radiation: np.ndarray = field(default=None) # dose rate per cell
    sources: list = field(default_factory=list)
    weak_box: tuple = (4.0, 8.5, 6.5, 11.0)     # damaged building footprint (x0,y0,x1,y1)
    collapsed: bool = False

    # ------------------------------------------------------------------
    def build(self):
        cl = self.client
        n = C.N
        self.tall = np.zeros((n, n), bool)
        self.rubble = np.zeros((n, n), bool)
        self.weak = np.zeros((n, n), bool)
        self.sealed = np.zeros((n, n), bool)

        p.changeVisualShape(0, -1, textureUniqueId=-1, rgbaColor=[0.47, 0.45, 0.41, 1], physicsClientId=cl)  # dusty ground
        # roads (asphalt strips) and a lighter search-area pad so the block reads as a city street grid
        self._box((-0.3, -0.3, C.AREA + 0.3, C.AREA + 0.3), 0.004, [0.56, 0.54, 0.49, 1])
        for r in [(3.4, -0.3, 4.4, C.AREA + 0.3), (7.0, -0.3, 8.0, C.AREA + 0.3),
                  (-0.3, 3.6, C.AREA + 0.3, 4.6), (-0.3, 7.0, C.AREA + 0.3, 7.9), (-2.6, 5.0, 0.0, 7.0)]:
            self._box(r, 0.008, [0.30, 0.30, 0.32, 1])

        # intact tall buildings (x0, y0, x1, y1, height)
        for (x0, y0, x1, y1, h) in [(1.0, 8.0, 3.0, 11.0, 2.2), (8.0, 1.0, 11.0, 3.0, 2.0),
                                    (5.0, 5.0, 6.5, 6.5, 1.9), (8.5, 8.0, 9.5, 9.5, 2.4)]:
            self._box((x0, y0, x1, y1), h, [0.78, 0.74, 0.66, 1])
            self._box((x0 + 0.08, y0 + 0.08, x1 - 0.08, y1 - 0.08), h + 0.06, [0.55, 0.52, 0.48, 1])  # roof parapet
            for k in range(1, int(h / 0.45) + 1):          # window bands
                z = k * 0.45 - 0.12
                self._band((x0, y0, x1, y1), z, [0.25, 0.35, 0.45, 1])
            self._mark(self.tall, (x0, y0, x1, y1))

        # damaged building: low broken walls, open roof, unstable
        x0, y0, x1, y1 = self.weak_box
        t = 0.12
        for wall in [(x0, y0, x1, y0 + t), (x0, y1 - t, x1, y1), (x0, y0, x0 + t, y1 - 1.2),
                     (x1 - t, y0 + 0.9, x1, y1)]:
            self._box(wall, 0.6, [0.80, 0.55, 0.30, 1])
        self._mark(self.weak, self.weak_box)

        # rubble piles (low)
        rng = np.random.default_rng(C.SEED)
        piles = [(5.6, 2.3, 0.8), (1.6, 4.8, 0.9), (3.3, 6.3, 0.7), (7.2, 7.8, 0.6),
                 (11.0, 11.2, 0.7), (6.8, 3.8, 0.5), (2.6, 1.0, 0.5), (10.2, 6.6, 0.6)]
        for (cx, cy, r) in piles:
            for _ in range(7):
                dx, dy = rng.uniform(-r, r, 2) * 0.7
                s = rng.uniform(0.12, 0.25)
                self._box((cx + dx - s, cy + dy - s, cx + dx + s, cy + dy + s),
                          rng.uniform(0.08, 0.28), [0.45, 0.38, 0.30, 1], yaw=rng.uniform(0, 3))
            self._mark(self.rubble, (cx - r, cy - r, cx + r, cy + r))

        # radiation field (damaged industrial unit) -- shown as a yellow marker drum
        xs = (np.arange(n) + 0.5) * C.CELL
        X, Y = np.meshgrid(xs, xs, indexing="ij")
        self.rad_center = np.array([8.6, 5.6])
        self.radiation = 3.0 * np.exp(-((X - 8.6) ** 2 + (Y - 5.6) ** 2) / (2 * 1.1 ** 2))
        self._cyl(self.rad_center, 0.18, 0.4, [0.95, 0.85, 0.1, 1])

        # charging pads at base
        for i in range(C.NUM_DRONES):
            pad = C.BASE + np.array([0, (i - (C.NUM_DRONES - 1) / 2) * C.PAD_SPACING])
            self._box((pad[0] - 0.2, pad[1] - 0.2, pad[0] + 0.2, pad[1] + 0.2), 0.02, [0.1, 0.6, 0.9, 1])
        self._box((C.BASE[0] - 1.9, C.BASE[1] - 2.0, C.BASE[0] - 1.5, C.BASE[1] + 2.0), 0.5, [0.9, 0.2, 0.2, 1])  # rescue truck

        # ---- heat sources: 20 people (some only partly visible, 3 fully buried) + 3 decoys ----
        S = HeatSource
        P_, H_, F_, HD, B_ = "person", "hand", "foot", "head", "buried"
        spec = [  # id, kind, x, y, temp, phone
            ("V1", P_, 2.5, 2.2, 36.4, True), ("V2", H_, 5.9, 2.6, 33.1, False),
            ("V3", P_, 9.6, 5.0, 35.2, True), ("V4", F_, 3.4, 6.6, 32.0, False),
            ("V5", P_, 5.3, 9.8, 34.8, True), ("V6", HD, 11.0, 11.1, 34.0, False),
            ("V7", P_, 7.4, 11.3, 36.0, True), ("V8", B_, 1.7, 4.9, 35.0, True),
            ("V9", P_, 0.8, 0.8, 35.9, False), ("V10", H_, 4.2, 0.9, 32.6, False),
            ("V11", P_, 7.0, 0.6, 36.2, True), ("V12", HD, 11.6, 0.5, 33.5, False),
            ("V13", P_, 11.4, 4.2, 35.5, False), ("V14", B_, 7.0, 4.4, 34.9, True),
            ("V15", F_, 0.6, 7.2, 31.8, False), ("V16", P_, 3.6, 11.5, 36.1, False),
            ("V17", H_, 7.6, 9.2, 33.0, False), ("V18", P_, 10.4, 7.2, 35.7, True),
            ("V19", B_, 0.5, 11.5, 35.2, True), ("V20", P_, 4.6, 4.2, 36.3, False),
        ]
        self.sources = [S(i, k, np.array([x, y]), t, True, has_phone=ph, visible=(k != B_))
                        for i, k, x, y, t, ph in spec]
        self.sources += [S("H1", "fire", np.array([7.0, 7.6]), 180.0, False),
                         S("H2", "animal", np.array([3.2, 3.6]), 38.5, False),
                         S("H3", "fire", np.array([9.8, 11.6]), 95.0, False)]
        for s in self.sources:
            s.body_id = self._spawn_source(s)

    # ------------------------------------------------------------------
    def _spawn_source(self, s):
        x, y = s.pos
        skin = [0.87, 0.67, 0.53, 1]
        if s.kind == "person":       # lying person: torso + head
            b = self._box((x - 0.28, y - 0.09, x + 0.28, y + 0.09), 0.1, [0.2, 0.3, 0.7, 1])
            self._sphere((x + 0.36, y), 0.08, skin, z=0.08)
            return b
        if s.kind == "hand":         # arm sticking out of rubble
            self._box((x - 0.04, y - 0.04, x + 0.04, y + 0.04), 0.32, skin)
            return self._sphere((x, y), 0.06, skin, z=0.34)
        if s.kind == "foot":
            return self._box((x - 0.12, y - 0.05, x + 0.12, y + 0.05), 0.12, [0.25, 0.2, 0.15, 1])
        if s.kind == "head":
            return self._sphere((x, y), 0.09, skin, z=0.22)
        if s.kind == "fire":
            self._box((x - 0.2, y - 0.2, x + 0.2, y + 0.2), 0.15, [0.15, 0.15, 0.15, 1])
            return self._cyl(s.pos, 0.12, 0.35, [1.0, 0.45, 0.05, 1], z0=0.15)
        if s.kind == "animal":
            return self._box((x - 0.18, y - 0.07, x + 0.18, y + 0.07), 0.14, [0.55, 0.35, 0.15, 1])
        return -1                    # buried: nothing visible

    def _box(self, r, h, rgba, yaw=0.0):
        x0, y0, x1, y1 = r
        he = [(x1 - x0) / 2, (y1 - y0) / 2, h / 2]
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=he, physicsClientId=self.client)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=he, rgbaColor=rgba, physicsClientId=self.client)
        return p.createMultiBody(0, col, vis, [(x0 + x1) / 2, (y0 + y1) / 2, h / 2],
                                 p.getQuaternionFromEuler([0, 0, yaw]), physicsClientId=self.client)

    def _band(self, r, z, rgba):
        x0, y0, x1, y1 = r
        he = [(x1 - x0) / 2 + 0.01, (y1 - y0) / 2 + 0.01, 0.06]
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=he, rgbaColor=rgba, physicsClientId=self.client)
        p.createMultiBody(0, -1, vis, [(x0 + x1) / 2, (y0 + y1) / 2, z], physicsClientId=self.client)

    def _sphere(self, xy, r, rgba, z=0.0):
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=r, rgbaColor=rgba, physicsClientId=self.client)
        return p.createMultiBody(0, -1, vis, [xy[0], xy[1], z], physicsClientId=self.client)

    def _cyl(self, xy, r, h, rgba, z0=0.0):
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=h, rgbaColor=rgba, physicsClientId=self.client)
        col = p.createCollisionShape(p.GEOM_CYLINDER, radius=r, height=h, physicsClientId=self.client)
        return p.createMultiBody(0, col, vis, [xy[0], xy[1], z0 + h / 2], physicsClientId=self.client)

    def _mark(self, grid, r):
        x0, y0, x1, y1 = r
        for i in range(C.N):
            for j in range(C.N):
                cx, cy = (i + 0.5) * C.CELL, (j + 0.5) * C.CELL
                if x0 - 0.15 <= cx <= x1 + 0.15 and y0 - 0.15 <= cy <= y1 + 0.15:
                    grid[i, j] = True

    # ------------------------------------------------------------------
    def inside_weak(self, xy, margin=0.0):
        x0, y0, x1, y1 = self.weak_box
        return x0 + margin < xy[0] < x1 - margin and y0 + margin < xy[1] < y1 - margin

    def aftershock_collapse(self):
        """Damaged building collapses: roof slab falls, walls rise -> sealed."""
        if self.collapsed:
            return
        self.collapsed = True
        x0, y0, x1, y1 = self.weak_box
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[(x1 - x0) / 2, (y1 - y0) / 2, 0.06], physicsClientId=self.client)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[(x1 - x0) / 2, (y1 - y0) / 2, 0.06],
                                  rgbaColor=[0.5, 0.35, 0.2, 1], physicsClientId=self.client)
        p.createMultiBody(0, col, vis, [(x0 + x1) / 2, (y0 + y1) / 2, 0.82],
                          p.getQuaternionFromEuler([0.04, -0.03, 0.02]), physicsClientId=self.client)
        for wall in [(x0, y0, x0 + 0.12, y1), (x1 - 0.12, y0, x1, y1)]:
            self._box(wall, 0.8, [0.70, 0.48, 0.28, 1])
        self.sealed |= self.weak
