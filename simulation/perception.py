"""
Perception: thermal scan, photo capture + AI victim check, phone-signal locating,
and victim priority scoring.

AI check: Ultralytics YOLOv8 (vision.py) runs on every rendered drone photo, fine-tuned on this simulator's
images to find whole people and partial body parts (hand, foot, head). The decision policy is the same on hardware.
"""
import os
import numpy as np
import pybullet as p
import cv2
import config as C
import vision


# ---------------- thermal ----------------
def thermal_scan(world, drone_xy, alt, rng):
    """Heat sources inside the thermal footprint. Fully buried people give no thermal signal."""
    radius = C.THERMAL_RADIUS * np.clip(alt / C.SEARCH_ALT, 0.5, 1.3)
    hits = []
    for s in world.sources:
        if not s.visible or s.reported:
            continue
        d = np.linalg.norm(s.pos - drone_xy)
        if d <= radius:
            measured = s.temp_c + rng.normal(0, 0.6)
            if measured > 30.0:          # anything warmer than the surroundings is "unusual"
                hits.append((s, measured))
    return hits


# ---------------- camera ----------------
def camera(client, eye, target, w=320, h=240, fov=60):
    """Render an RGB frame + segmentation mask (PyBullet body id per pixel) from eye looking at target."""
    down = np.hypot(target[0] - eye[0], target[1] - eye[1]) < 0.05
    view = p.computeViewMatrix(list(eye), list(target), [0, 1, 0] if down else [0, 0, 1], physicsClientId=client)
    proj = p.computeProjectionMatrixFOV(fov=fov, aspect=w / h, nearVal=0.05, farVal=20, physicsClientId=client)
    img = p.getCameraImage(w, h, view, proj, renderer=p.ER_TINY_RENDERER, physicsClientId=client)
    return np.reshape(img[2], (h, w, 4))[:, :, :3].astype(np.uint8), np.reshape(img[4], (h, w))


def drone_view(pos, heading):
    """Drone camera pose: just under the airframe, looking 1 m ahead along its heading, tilted down."""
    return [pos[0], pos[1], pos[2] - 0.03], [pos[0] + np.cos(heading), pos[1] + np.sin(heading), 0.0]


def thermal(rgb, seg, sources):
    """False-colour thermal frame: cool ambient rubble, warm glow on every heat-source body in view.
    Returns (BGR image, hottest temperature seen or None)."""
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY).astype(np.float32) / 255.0
    temp = 18.0 + 6.0 * gray                               # ambient rubble ~ 18-24 C
    hot = None
    for s in sources:
        m = np.isin(seg, s.bodies)
        if m.any():
            temp[m] = s.temp_c
            hot = max(hot or 0.0, s.temp_c)
    glow = cv2.GaussianBlur(temp, (0, 0), 4)
    img = np.clip((np.maximum(temp, glow) - 15.0) / 30.0, 0, 1)
    out = cv2.applyColorMap((img * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
    if hot:
        cv2.putText(out, f"{hot:.1f} C", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return out, hot


def draw_boxes(bgr, dets):
    """Draw YOLOv8 detections (label, confidence, box) on a BGR image."""
    for label, conf, (x0, y0, x1, y1) in dets:
        cv2.rectangle(bgr, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2.putText(bgr, f"{label.replace('body part: ', '')} {conf:.0%}", (x0, max(12, y0 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
    return bgr


def take_photo(client, eye, target, sources, path, size=320):
    """Photo + matching thermal frame for the rescuer. YOLOv8 runs on the photo; its boxes are drawn on it.
    Returns the detections."""
    rgb, seg = camera(client, eye, target, size, size, fov=75)
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    dets = vision.detect(bgr)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cv2.imwrite(path, draw_boxes(bgr, dets))
    cv2.imwrite(path.replace(".png", "_thermal.png"), thermal(rgb, seg, sources)[0])
    return dets


# ---------------- AI victim check ----------------
def ai_check(dets, measured_temp):
    """
    What YOLOv8 found in the photo, and what goes to the rescuer.
    Policy: NO heat signal is ever discarded. Low-confidence or empty results are still sent, marked for
    the rescuer to verify. A missed victim costs far more than a false alarm.
    """
    label, conf = dets[0][:2] if dets else ("no person detected", 0.0)
    if conf >= 0.8:
        status = "CONFIRMED VICTIM"
    elif conf >= 0.5:
        status = "LIKELY VICTIM - please verify"
    else:
        status, label = "POSSIBLE VICTIM - please verify", f"unclear ({label}?)" if dets else "unclear heat source"
    return dict(label=label, confidence=round(conf, 2), status=status, body_temp_like=28.0 <= measured_temp <= 39.5,
                measured_temp=round(measured_temp, 1), model=vision.name(),
                detections=[dict(label=d[0], conf=round(d[1], 2)) for d in dets])


# ---------------- phone signals ----------------
def phone_rssi(world, drone_xy, rng):
    out = []
    for s in world.sources:
        if not s.has_phone:
            continue
        d = max(np.linalg.norm(s.pos - drone_xy), 0.3)
        if not hasattr(s, "n_true"):        # each victim's rubble attenuates differently
            s.n_true = rng.uniform(*C.RSSI_N_TRUE)
        rssi = C.RSSI_P0 - 10 * s.n_true * np.log10(d) + rng.normal(0, C.RSSI_NOISE)
        if rssi >= C.PHONE_RANGE_DBM:
            out.append((s.id, rssi))
    return out


def locate_phone(samples):
    """samples: list of (drone_xy, rssi). Least-squares fit over a fine grid."""
    if len(samples) < 8:
        return None, None
    pts = np.array([s[0] for s in samples]); rs = np.array([s[1] for s in samples])
    if np.ptp(pts[:, 0]) < 1.0 and np.ptp(pts[:, 1]) < 1.0:
        return None, None
    g = np.arange(0, C.AREA, 0.1)
    X, Y = np.meshgrid(g, g, indexing="ij")
    err = np.zeros_like(X)
    for (px, py), r in zip(pts, rs):
        d = np.maximum(np.hypot(X - px, Y - py), 0.3)
        err += (r - (C.RSSI_P0 - 10 * C.RSSI_N * np.log10(d))) ** 2
    i = np.unravel_index(np.argmin(err), err.shape)
    return np.array([X[i], Y[i]]), float(np.sqrt(err[i] / len(rs)))


# ---------------- priority ----------------
def priority_score(report, world):
    """0-100 urgency: certainty it's a person, how trapped they are, hazards nearby, cold body temp."""
    from world import to_cell
    c = to_cell(report["pos"])
    score = 40 * report.get("confidence", 0.5)
    if report.get("trapped"):
        score += 20
    if world.radiation[c] > 0.5 or world.weak[c]:
        score += 20
    t = report.get("measured_temp")
    if t is not None and t < 34.0:
        score += 15                     # possible hypothermia / blood loss
    if report.get("source") == "phone":
        score += 10                     # buried with no visual: hardest to reach
    return int(min(100, round(score)))
