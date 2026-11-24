"""
Real victim detection: HALO's YOLOv8n (trained by train_vision.py) on the drone's camera image.
Finds whole people and partial body parts. If the trained model or ultralytics is missing,
detect() returns None and perception falls back to its stand-in.
"""
import os

MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "halo_yolov8n.pt")
NAMES = {0: "person", 1: "body part"}
_model = False


def _load():
    global _model
    if _model is False:
        try:
            from ultralytics import YOLO
            _model = YOLO(MODEL) if os.path.exists(MODEL) else None
        except ImportError:
            _model = None
    return _model


def available():
    return _load() is not None


def detect(rgb, conf=0.1):
    """rgb: HxWx3 uint8. Returns [(label, confidence, (x0, y0, x1, y1)), ...] best first, or None without a model."""
    m = _load()
    if m is None:
        return None
    r = m.predict(rgb[:, :, ::-1], imgsz=320, conf=conf, device="cpu", verbose=False)[0]
    out = [(NAMES[int(c)], float(s), tuple(int(v) for v in b))
           for b, c, s in zip(r.boxes.xyxy.tolist(), r.boxes.cls.tolist(), r.boxes.conf.tolist())]
    return sorted(out, key=lambda d: -d[1])
