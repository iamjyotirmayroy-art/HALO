"""
Victim detector: Ultralytics YOLOv8.

Uses models/halo_yolov8n.pt, YOLOv8n fine-tuned on this simulator's drone photos (train_detector.py) with four classes:
person, hand, foot and head. Without it, falls back to the official COCO yolov8n.pt (whole people only, downloaded on
first use), which barely recognises rendered people. On real drones: fine-tune on real rubble imagery and drop it in here.
"""
import os
from functools import lru_cache

MODELS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
TUNED = os.path.join(MODELS, "halo_yolov8n.pt")
NAMES = {"person": "person", "hand": "body part: hand/arm", "foot": "body part: foot/leg", "head": "body part: head"}


@lru_cache(maxsize=1)
def model():
    from ultralytics import YOLO
    return YOLO(TUNED if os.path.exists(TUNED) else os.path.join(MODELS, "yolov8n.pt"))


def name():
    return "YOLOv8n · HALO-tuned" if os.path.exists(TUNED) else "YOLOv8n · COCO"


def detect(bgr, conf=0.25):
    """[(label, confidence, (x0, y0, x1, y1))] for every person or body part in the image, most confident first."""
    r = model().predict(bgr, imgsz=320, conf=conf, device="cpu", verbose=False)[0]
    out = [(NAMES[r.names[int(c)]], float(s), tuple(int(v) for v in b))
           for c, s, b in zip(r.boxes.cls, r.boxes.conf, r.boxes.xyxy.tolist()) if r.names[int(c)] in NAMES]
    return sorted(out, key=lambda d: -d[1])
