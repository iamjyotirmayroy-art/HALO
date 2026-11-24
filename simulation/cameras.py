"""
Live drone cameras, rendered in background processes so that watching drones never slows the mission.

Each worker builds its own identical copy of the scene (same seed), receives the drones' poses a few
times a second, renders the watched drones' cameras (photo + thermal, with the AI's boxes) to
results/cams/drone<N>[_thermal].jpg and reports what it saw. A busy worker drops stale frames.
"""
import multiprocessing as mp
import os, queue
from importlib.resources import files
import cv2
import pybullet as p
import pybullet_data
import config as C
import perception as P
from world import World, poly_mask


def _worker(cfg, jobs, results):
    C.AREA_POLY = cfg["poly"]
    C.set_base(cfg["base"])
    cl = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=cl)
    p.loadURDF("plane.urdf", physicsClientId=cl)
    urdf = str(files("gym_pybullet_drones") / "assets" / "cf2x.urdf")
    drones = [p.loadURDF(urdf, [0, 0, -5], useFixedBase=True, physicsClientId=cl) for _ in range(cfg["n"])]
    beacons = [p.createMultiBody(0, -1, p.createVisualShape(p.GEOM_SPHERE, radius=0.16, physicsClientId=cl,
                                 rgbaColor=[c / 255 for c in C.DRONE_COLORS[i % len(C.DRONE_COLORS)]] + [1]),
                                 [0, 0, -5], physicsClientId=cl) for i in range(cfg["n"])]
    world = World(cl, poly_mask(C.AREA_POLY))
    world.build(cfg["n"])
    while True:
        job = jobs.get()
        if job is None:
            return
        if job["collapsed"]:
            world.aftershock_collapse()
        for b, m, (pos, h) in zip(drones, beacons, job["poses"]):
            p.resetBasePositionAndOrientation(b, pos, p.getQuaternionFromEuler([0, 0, h]), physicsClientId=cl)
            p.resetBasePositionAndOrientation(m, [pos[0], pos[1], pos[2] + 0.12], [0, 0, 0, 1], physicsClientId=cl)
        for i in job["render"]:
            rgb, seg = P.camera(cl, *P.drone_view(*job["poses"][i]))
            dets = P.detect(rgb, seg, world.sources)
            th, hot = P.thermal(rgb, seg, world.sources)
            for name, img in ((f"drone{i}", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)), (f"drone{i}_thermal", th)):
                tmp = os.path.join(cfg["out"], name + ".tmp.jpg")
                cv2.imwrite(tmp, P.draw_boxes(img, dets), [cv2.IMWRITE_JPEG_QUALITY, 80])
                os.replace(tmp, os.path.join(cfg["out"], name + ".jpg"))
            results.put((i, dict(t=job["t"], heat=hot, people=[label for label, _, _ in dets])))


class CameraPool:
    """Workers start with a dashboard mission (they take a few seconds to boot) or on first use."""
    def __init__(self, n_drones, out):
        self.cfg = dict(n=n_drones, out=out, poly=C.AREA_POLY, base=C.BASE.tolist())
        self.workers, self.jobs = [], []
        self.results = None
        os.makedirs(out, exist_ok=True)

    def start(self):
        ctx = mp.get_context("spawn")
        self.results = ctx.Queue()
        for _ in range(max(1, min(3, (os.cpu_count() or 2) - 2))):
            q = ctx.Queue(maxsize=1)
            w = ctx.Process(target=_worker, args=(self.cfg, q, self.results), daemon=True)
            w.start()
            self.workers.append(w)
            self.jobs.append(q)

    def render(self, sim, ids):
        """Ask the workers for new frames of drones `ids` (skipped by a worker still busy with the last ones)."""
        if not self.workers:
            self.start()
        poses = [(d.pos.tolist(), d.heading) for d in sim.drones]
        for k, q in enumerate(self.jobs):
            mine = [i for i in ids if i % len(self.jobs) == k]
            if mine:
                try:
                    q.put_nowait(dict(t=round(sim.t, 1), poses=poses, render=mine, collapsed=sim.world.collapsed))
                except queue.Full:
                    pass

    def collect(self):
        """What the cameras saw since last time: {drone: {t, heat, people}}."""
        out = {}
        while self.results:
            try:
                i, meta = self.results.get_nowait()
            except queue.Empty:
                break
            out[i] = meta
        return out

    def close(self):
        for q in self.jobs:
            try:
                q.put_nowait(None)
            except queue.Full:
                pass
        for w in self.workers:
            w.join(timeout=2)
            if w.is_alive():
                w.terminate()
