"""
Train HALO's victim detector (YOLOv8n) on synthetic drone photos rendered in PyBullet.

Every image is a random rubble scene with people, partial body parts (hand/arm, foot/leg, head)
and look-alike decoys (fires, animals, debris), shot from random drone heights, angles and light,
with random skin, clothing, ground and rubble colours (domain randomisation). Boxes come straight
from PyBullet's segmentation mask, so the labels are exact, occlusion included.

    python train_vision.py                      # generate data + train -> models/halo_yolov8n.pt
    python train_vision.py --images 1200 --epochs 40

On real hardware, fine-tune the same model on real disaster imagery.
"""
import argparse, os, shutil
import numpy as np
import pybullet as p
import cv2
from perception import camera

CLASSES = ["person", "body_part"]
ROOT = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(ROOT, "vision_data")
MODEL = os.path.join(ROOT, "models", "halo_yolov8n.pt")


def box(cl, c, he, rgba, yaw=0.0):
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=he, rgbaColor=rgba, physicsClientId=cl)
    return p.createMultiBody(0, -1, vis, c, p.getQuaternionFromEuler([0, 0, yaw]), physicsClientId=cl)


def ball(cl, c, r, rgba):
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=r, rgbaColor=rgba, physicsClientId=cl)
    return p.createMultiBody(0, -1, vis, c, physicsClientId=cl)


def cyl(cl, c, r, h, rgba):
    vis = p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=h, rgbaColor=rgba, physicsClientId=cl)
    return p.createMultiBody(0, -1, vis, c, physicsClientId=cl)


def spawn(cl, kind, x, y, rng):
    """Same shapes as world.py, with random colours, size and heading. Returns (class or None, body ids)."""
    skin = list(np.clip(np.array([0.87, 0.67, 0.53]) * rng.uniform(0.45, 1.15), 0, 1)) + [1]
    cloth = list(rng.uniform(0.05, 0.9, 3)) + [1]
    k, yaw = rng.uniform(0.8, 1.25), rng.uniform(0, np.pi)
    u = np.array([np.cos(yaw), np.sin(yaw)])
    if kind == "person":
        hx, hy = (np.array([x, y]) + u * 0.36 * k)
        return 0, [box(cl, [x, y, 0.05 * k], [0.28 * k, 0.09 * k, 0.05 * k], cloth, yaw), ball(cl, [hx, hy, 0.08 * k], 0.08 * k, skin)]
    if kind == "hand":
        h = rng.uniform(0.15, 0.34)
        return 1, [box(cl, [x, y, h / 2], [0.04 * k, 0.04 * k, h / 2], skin, yaw), ball(cl, [x, y, h + 0.02], 0.06 * k, skin)]
    if kind == "foot":
        return 1, [box(cl, [x, y, 0.06 * k], [0.12 * k, 0.05 * k, 0.06 * k], list(rng.uniform(0.1, 0.5, 3)) + [1], yaw)]
    if kind == "head":
        return 1, [ball(cl, [x, y, rng.uniform(0.1, 0.22)], 0.09 * k, skin)]
    if kind == "fire":
        return None, [box(cl, [x, y, 0.075], [0.2, 0.2, 0.075], [0.15, 0.15, 0.15, 1], yaw),
                      cyl(cl, [x, y, 0.32], 0.12 * k, 0.35 * k, [1.0, rng.uniform(0.3, 0.6), 0.05, 1])]
    if kind == "animal":
        return None, [box(cl, [x, y, 0.07], [0.18 * k, 0.07 * k, 0.07 * k], list(rng.uniform(0.3, 0.6, 3) * [1, 0.7, 0.4]) + [1], yaw)]
    return None, [box(cl, [x, y, 0.06], list(rng.uniform(0.05, 0.2, 3)), list(rng.uniform(0.3, 0.8, 3)) + [1], yaw)]   # debris


def scene(cl, rng):
    p.resetSimulation(physicsClientId=cl)
    g = list(rng.uniform(0.3, 0.65) * np.array([1, rng.uniform(0.9, 1), rng.uniform(0.8, 1)])) + [1]
    box(cl, [0, 0, -0.01], [4, 4, 0.01], g)
    for _ in range(rng.integers(8, 30)):                       # rubble field
        s = rng.uniform(0.03, 0.18, 3)
        box(cl, [*rng.uniform(-1.6, 1.6, 2), s[2]], list(s), list(rng.uniform(0.25, 0.7) * rng.uniform(0.8, 1, 3)) + [1], rng.uniform(0, 3))
    objs = []
    for kind in rng.choice(["person", "hand", "foot", "head", "fire", "animal", "debris"], rng.integers(1, 5),
                           p=[.22, .2, .14, .14, .1, .1, .1]):
        cls, bodies = spawn(cl, kind, *rng.uniform(-1.0, 1.0, 2), rng)
        objs.append((cls, bodies))
    return objs


def generate(n, rng):
    shutil.rmtree(DATA, ignore_errors=True)
    cl = p.connect(p.DIRECT)
    for i in range(n):
        split = "val" if i % 6 == 0 else "train"
        os.makedirs(os.path.join(DATA, "images", split), exist_ok=True)
        os.makedirs(os.path.join(DATA, "labels", split), exist_ok=True)
        objs = scene(cl, rng)
        focus = [b for c, b in objs if c is not None] or [objs[0][1]]
        tx, ty = np.array(p.getBasePositionAndOrientation(focus[rng.integers(len(focus))][0], physicsClientId=cl)[0][:2]) + rng.uniform(-.25, .25, 2)
        h = rng.uniform(0.35, 1.4)
        eye = [tx + rng.uniform(-.6, .6) * h, ty + rng.uniform(-.6, .6) * h, h]
        rgb, seg = camera(cl, eye, [tx, ty, 0], 320, 320)
        rgb = np.clip(rgb.astype(np.float32) * rng.uniform(0.7, 1.25) + rng.normal(0, 4, rgb.shape), 0, 255).astype(np.uint8)
        lines = []
        for cls, bodies in objs:
            if cls is None:
                continue
            ys, xs = np.nonzero(np.isin(seg, bodies))
            if len(xs) < 25:
                continue                                       # too small / hidden to label
            x0, x1, y0, y1 = xs.min(), xs.max() + 1, ys.min(), ys.max() + 1
            lines.append(f"{cls} {(x0 + x1) / 640:.5f} {(y0 + y1) / 640:.5f} {(x1 - x0) / 320:.5f} {(y1 - y0) / 320:.5f}")
        cv2.imwrite(os.path.join(DATA, "images", split, f"{i:05d}.jpg"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        with open(os.path.join(DATA, "labels", split, f"{i:05d}.txt"), "w") as f:
            f.write("\n".join(lines))
    p.disconnect(cl)
    with open(os.path.join(DATA, "halo.yaml"), "w") as f:
        f.write(f"path: {DATA}\ntrain: images/train\nval: images/val\nnames: {dict(enumerate(CLASSES))}\n")


def train(epochs, imgsz):
    from ultralytics import YOLO
    model = YOLO("yolov8n.pt")                                 # COCO-pretrained start (already knows "person")
    model.train(data=os.path.join(DATA, "halo.yaml"), epochs=epochs, imgsz=imgsz, batch=16, device="cpu",
                workers=2, project=os.path.join(DATA, "runs"), name="halo", exist_ok=True, plots=False, verbose=False)
    os.makedirs(os.path.dirname(MODEL), exist_ok=True)
    shutil.copy(os.path.join(DATA, "runs", "halo", "weights", "best.pt"), MODEL)
    return YOLO(MODEL).val(data=os.path.join(DATA, "halo.yaml"), imgsz=imgsz, device="cpu", plots=False, verbose=False)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", type=int, default=900)
    ap.add_argument("--epochs", type=int, default=25)
    ap.add_argument("--imgsz", type=int, default=320)
    ap.add_argument("--skip-data", action="store_true")
    a = ap.parse_args()
    if not a.skip_data:
        generate(a.images, np.random.default_rng(0))
    m = train(a.epochs, a.imgsz)
    print(f"mAP50 {m.box.map50:.3f}  mAP50-95 {m.box.map:.3f}  per class mAP50 "
          + ", ".join(f"{c} {v:.3f}" for c, v in zip(CLASSES, m.box.ap50)))
