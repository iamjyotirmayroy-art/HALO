"""
Perception: thermal scan, photo capture + AI victim check, phone-signal locating,
and victim priority scoring.

NOTE ON THE AI CHECK: in this simulation the photo is a real rendered camera image,
but the person/body-part classifier is a calibrated stand-in that returns confidence
scores with realistic noise for each kind of heat source. On hardware it is replaced
by a detector (e.g. YOLOv8) trained on whole people AND partial body parts (hands,
arms, feet, heads) in rubble. The decision policy below is the same either way.
"""
import os
import numpy as np
import pybullet as p
import cv2
import config as C


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
def take_photo(client, cam_xyz, target_xy, path, size=320):
    view = p.computeViewMatrix(cameraEyePosition=list(cam_xyz),
                               cameraTargetPosition=[target_xy[0], target_xy[1], 0.0],
                               cameraUpVector=[0, 1, 0], physicsClientId=client)
    proj = p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.05, farVal=20, physicsClientId=client)
    img = p.getCameraImage(size, size, view, proj, renderer=p.ER_TINY_RENDERER, physicsClientId=client)
    rgb = np.reshape(np.array(img[2], dtype=np.uint8), (size, size, 4))[:, :, :3]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cv2.imwrite(path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    return rgb


def thermal_image(rgb, temp_c, kind, path, rng):
    """False-colour thermal frame matching the photo: cool scene, hot blob where the heat source is."""
    h, w = rgb.shape[:2]
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY).astype(np.float32) / 255.0
    base = 0.18 + 0.22 * gray                          # ambient rubble ~ 18-24 C
    Y, X = np.mgrid[0:h, 0:w]
    sx, sy = {"person": (70, 26), "hand": (16, 16), "foot": (24, 14), "head": (18, 18),
              "fire": (34, 34), "animal": (40, 18)}.get(kind, (24, 24))
    heat = np.clip((temp_c - 20) / 25.0, 0.3, 1.0)
    blob = heat * np.exp(-(((X - w / 2) / sx) ** 2 + ((Y - h / 2) / sy) ** 2))
    img = np.clip(base + blob + rng.normal(0, 0.015, base.shape), 0, 1)
    out = cv2.applyColorMap((img * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
    cv2.putText(out, f"{temp_c:.1f} C", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.imwrite(path, out)


# ---------------- AI victim check ----------------
LABELS = {"person": "person", "hand": "body part: hand/arm", "foot": "body part: foot/leg",
          "head": "body part: head", "fire": "hot debris / fire", "animal": "animal"}
CONF_MEAN = {"person": 0.93, "hand": 0.68, "foot": 0.61, "head": 0.72, "fire": 0.06, "animal": 0.24}


def ai_check(source, measured_temp, rng):
    """
    Returns what the AI thinks it sees and what goes to the rescuer.
    Policy: NO heat signal is ever discarded. Low-confidence results are still
    sent, marked for the rescuer to verify. A missed victim costs far more than a false alarm.
    """
    conf = float(np.clip(rng.normal(CONF_MEAN[source.kind], 0.06), 0.01, 0.99))
    label = LABELS[source.kind]
    body_temp_like = 28.0 <= measured_temp <= 39.5
    if conf >= 0.85:
        status, label_out = "CONFIRMED VICTIM", label
    elif conf >= 0.5:
        status, label_out = "LIKELY VICTIM - please verify", label
    else:
        status = "POSSIBLE VICTIM - please verify"
        label_out = f"unclear ({label}?)"
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
