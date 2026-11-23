"""
Train HALO's victim detector: Ultralytics YOLOv8 fine-tuned on synthetic drone photos from this simulator.

    python train_detector.py                         # render the dataset, fine-tune yolov8n.pt -> models/halo_yolov8n.pt
    python train_detector.py --images 3000 --epochs 60

Each image is a randomised disaster scene (ground colour, rubble, heat decoys, 1-4 victims of the four kinds with random
clothes, skin tone, pose and burial) seen from a random drone height, tilt and heading under random light. Boxes come from
the renderer's segmentation mask, so labels are pixel-exact. Stock COCO YOLOv8 does not recognise these rendered people
(domain gap), which is why the detector is fine-tuned. On real drones, fine-tune again on real rubble imagery.
"""
import argparse, os, shutil
import cv2
import numpy as np
import pybullet as p
import pybullet_data
import humans

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, "models")
SIZE = 320                                               # training and inference image size (px)


def scene(cl, rng):
    """One random scene. Returns [(class index, body ids)] for the victims in it."""
    p.resetSimulation(physicsClientId=cl)
    p.loadURDF("plane.urdf", physicsClientId=cl)
    p.changeVisualShape(0, -1, textureUniqueId=-1, rgbaColor=[*rng.uniform(0.3, 0.62, 3), 1], physicsClientId=cl)
    for _ in range(rng.integers(6, 22)):                 # loose rubble
        he = rng.uniform([0.04, 0.04, 0.02], [0.22, 0.22, 0.14])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=he, rgbaColor=(*humans.DEBRIS[rng.integers(4)], 1), physicsClientId=cl)
        p.createMultiBody(0, -1, vis, [*rng.uniform(-1.6, 1.6, 2), he[2]], p.getQuaternionFromEuler([0, 0, rng.uniform(0, 3)]), physicsClientId=cl)
    for _ in range(rng.integers(0, 3)):
        humans.decoy(cl, ("fire", "animal")[rng.integers(2)], rng.uniform(-1.3, 1.3, 2), rng)
    victims = []
    for _ in range(rng.integers(1, 5)):
        k = rng.integers(len(humans.KINDS))
        victims.append((k, humans.build(cl, humans.KINDS[k], rng.uniform(-1.1, 1.1, 2), rng.uniform(0, 6.28), rng)[0]))
    return victims


def shot(cl, rng, victims):
    """Random drone view near a victim: image + YOLO labels from the segmentation mask."""
    _, ids = victims[rng.integers(len(victims))]
    tgt = np.array(p.getBasePositionAndOrientation(ids[0], physicsClientId=cl)[0]) + [*rng.uniform(-0.25, 0.25, 2), 0]
    tgt[2] = 0.05
    alt, tilt, hdg = rng.uniform(0.35, 1.3), rng.uniform(0, 0.9), rng.uniform(0, 6.28)
    eye = tgt + [-np.cos(hdg) * alt * np.tan(tilt), -np.sin(hdg) * alt * np.tan(tilt), alt]
    up = [np.cos(hdg), np.sin(hdg), 0] if tilt < 0.05 else [0, 0, 1]
    view = p.computeViewMatrix(eye.tolist(), tgt.tolist(), up, physicsClientId=cl)
    proj = p.computeProjectionMatrixFOV(rng.uniform(55, 80), 1, 0.05, 20, physicsClientId=cl)
    img = p.getCameraImage(SIZE, SIZE, view, proj, renderer=p.ER_TINY_RENDERER, shadow=int(rng.random() < 0.7),
                           lightDirection=[*rng.uniform(-4, 4, 2), rng.uniform(3, 8)], physicsClientId=cl)
    rgb = np.reshape(img[2], (SIZE, SIZE, 4))[:, :, :3].astype(np.uint8)
    seg = np.reshape(img[4], (SIZE, SIZE))
    rows = []
    for k, ids in victims:
        ys, xs = np.nonzero(np.isin(seg, ids))
        if len(xs) >= 30:                                # skip slivers too small to recognise
            x0, x1, y0, y1 = xs.min(), xs.max() + 1, ys.min(), ys.max() + 1
            rows.append(f"{k} {(x0 + x1) / 2 / SIZE:.5f} {(y0 + y1) / 2 / SIZE:.5f} {(x1 - x0) / SIZE:.5f} {(y1 - y0) / SIZE:.5f}")
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), rows


def make_dataset(root, n, seed):
    cl = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=cl)
    rng = np.random.default_rng(seed)
    for i in range(n):
        if i % 6 == 0:
            victims = scene(cl, rng)
        split = "val" if i % 10 == 0 else "train"
        img, rows = shot(cl, rng, victims)
        for sub in ("images", "labels"):
            os.makedirs(os.path.join(root, sub, split), exist_ok=True)
        cv2.imwrite(os.path.join(root, "images", split, f"{i:05d}.jpg"), img)
        with open(os.path.join(root, "labels", split, f"{i:05d}.txt"), "w") as f:
            f.write("\n".join(rows))
    p.disconnect(cl)
    yaml = os.path.join(root, "data.yaml")
    with open(yaml, "w") as f:
        f.write(f"path: {root}\ntrain: images/train\nval: images/val\nnames:\n" + "".join(f"  {i}: {k}\n" for i, k in enumerate(humans.KINDS)))
    return yaml


if __name__ == "__main__":
    from ultralytics import YOLO
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", type=int, default=2400)
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    root = os.path.join(HERE, "datasets", "halo_victims")
    shutil.rmtree(root, ignore_errors=True)
    yaml = make_dataset(root, a.images, a.seed)
    print(f"dataset: {a.images} images in {root}")
    model = YOLO(os.path.join(MODELS, "yolov8n.pt"))     # official pretrained weights (downloaded on first use)
    res = model.train(data=yaml, epochs=a.epochs, imgsz=SIZE, batch=32, device="cpu", workers=2, seed=a.seed,
                      project=os.path.join(HERE, "datasets", "runs"), name="halo", exist_ok=True, plots=False, verbose=False)
    shutil.copy(os.path.join(res.save_dir, "weights", "best.pt"), os.path.join(MODELS, "halo_yolov8n.pt"))
    print("saved", os.path.join(MODELS, "halo_yolov8n.pt"))
