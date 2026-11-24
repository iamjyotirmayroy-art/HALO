"""
Perception: thermal scan, photo capture + AI victim check, phone-signal locating,
and victim priority scoring.

THE AI CHECK: the photo is a rendered drone camera image, checked by HALO's YOLOv8n detector
(vision.py, trained by train_vision.py on synthetic rubble scenes) for whole people AND partial
body parts. If the trained model isn't installed, a calibrated stand-in gives the scores instead.
The decision policy is the same either way: nothing is ever discarded.
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


def detect(rgb, seg, sources):
    """People / body parts in a frame as [(label, confidence, box)]: YOLOv8 if installed, else the stand-in
    (a box around every human body visible in the segmentation mask)."""
    dets = vision.detect(rgb)
    if dets is not None:
        return dets
    out = []
    for s in sources:
        if s.is_human and s.bodies:
            ys, xs = np.nonzero(np.isin(seg, s.bodies))
            if len(xs):
                out.append((LABELS[s.kind].replace("body part: ", ""), CONF_MEAN[s.kind],
                            (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()))))
    return out


def draw_boxes(bgr, dets):
    for label, conf, (x0, y0, x1, y1) in dets:
        cv2.rectangle(bgr, (x0 - 3, y0 - 3), (x1 + 3, y1 + 3), (0, 255, 0), 2)
        cv2.putText(bgr, f"{label} {conf:.0%}", (x0 - 3, max(12, y0 - 7)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 255, 0), 1, cv2.LINE_AA)
    return bgr


def take_photo(client, eye, target, sources, path, size=320):
    """Photo + matching thermal frame for the rescuer, with the AI's boxes drawn on the photo. Returns the detections."""
    rgb, seg = camera(client, eye, target, size, size)
    dets = detect(rgb, seg, sources)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cv2.imwrite(path, draw_boxes(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), dets))
    cv2.imwrite(path.replace(".png", "_thermal.png"), thermal(rgb, seg, sources)[0])
    return dets


# ---------------- AI victim check ----------------
LABELS = {"person": "person", "hand": "body part: hand/arm", "foot": "body part: foot/leg",
          "head": "body part: head", "fire": "hot debris / fire", "animal": "animal"}
CONF_MEAN = {"person": 0.93, "hand": 0.68, "foot": 0.61, "head": 0.72, "fire": 0.06, "animal": 0.24}


def ai_check(source, measured_temp, rng, dets=None):
    """
    Returns what the AI thinks it sees and what goes to the rescuer: YOLOv8's best detection in the
    drone's photo, or the stand-in score when no trained model is installed.
    Policy: NO heat signal is ever discarded. Low-confidence results are still
    sent, marked for the rescuer to verify. A missed victim costs far more than a false alarm.
    """
    if vision.available():
        label, conf = (dets[0][0], dets[0][1]) if dets else ("no person seen", 0.05)
        if label == "body part":
            label = "body part: " + {"hand": "hand/arm", "foot": "foot/leg", "head": "head"}.get(source.kind, "partial")
    else:
        conf = float(np.clip(rng.normal(CONF_MEAN[source.kind], 0.06), 0.01, 0.99))
        label = LABELS[source.kind]
    body_temp_like = 28.0 <= measured_temp <= 39.5
    if conf >= 0.85:
        status, label_out = "CONFIRMED VICTIM", label
    elif conf >= 0.5:
        status, label_out = "LIKELY VICTIM - please verify", label
    else:
        status = "POSSIBLE VICTIM - please verify"
        label_out = "unclear - no person seen" if label == "no person seen" else f"unclear ({label}?)"
    return dict(label=label_out, confidence=round(conf, 2), status=status,
                body_temp_like=body_temp_like, measured_temp=round(measured_temp, 1))


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
