"""Video frames: 3D PyBullet view + live 2D rescuer map with event feed."""
import numpy as np
import pybullet as p
import cv2
import config as C

W3, H = 720, 720          # 3D view size
WM = 720                  # map panel size
PX = 40                   # pixels per metre on the map
OX, OY = 140, 36          # map origin offset (px) - leaves room for the base on the left

LVL_COL = {"info": (200, 200, 200), "warn": (0, 170, 255), "alarm": (60, 60, 255), "victim": (90, 220, 90),
           "verify": (0, 220, 255), "heat": (120, 160, 255), "net": (255, 190, 120), "ok": (90, 220, 90)}


def m2px(xy):
    return int(OX + xy[0] * PX), int(OY + (C.AREA - xy[1]) * PX)


def render_3d(client, t):
    yaw = 25 + 40 * np.sin(t / 60.0)
    view = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[5.4, 6.0, 0.0], distance=12.5,
                                               yaw=yaw, pitch=-48, roll=0, upAxisIndex=2, physicsClientId=client)
    proj = p.computeProjectionMatrixFOV(fov=60, aspect=W3 / H, nearVal=0.1, farVal=50, physicsClientId=client)
    img = p.getCameraImage(W3, H, view, proj, renderer=p.ER_TINY_RENDERER, shadow=1, lightDirection=[-3, -2, 6],
                           lightAmbientCoeff=0.55, lightDiffuseCoeff=0.6, lightSpecularCoeff=0.1, physicsClientId=client)
    rgb = np.reshape(np.array(img[2], dtype=np.uint8), (H, W3, 4))[:, :, :3]
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def base_map(world):
    img = np.full((H, WM, 3), 30, np.uint8)
    for i in range(C.N):
        for j in range(C.N):
            x0, y1 = m2px((i * C.CELL, j * C.CELL))
            x1, y0 = m2px(((i + 1) * C.CELL, (j + 1) * C.CELL))
            col = (58, 58, 58)
            r = world.radiation[i, j]
            if r > 0.15:
                k = min(1.0, r / 3.0)
                col = (int(58 * (1 - k)), int(58 + 150 * k), int(58 + 180 * k))
            if world.rubble[i, j]:
                col = tuple(int(0.6 * a + 0.4 * b) for a, b in zip(col, (60, 90, 120)))
            if world.weak[i, j]:
                col = (40, 110, 200)
            if world.tall[i, j]:
                col = (150, 150, 150)
            cv2.rectangle(img, (x0, y0), (x1 - 1, y1 - 1), col, -1)
    cv2.circle(img, m2px(C.DEAD_ZONE_C), int(C.DEAD_ZONE_R * PX), (180, 120, 255), 1, cv2.LINE_AA)
    cv2.putText(img, "NO SIGNAL", (m2px(C.DEAD_ZONE_C)[0] - 40, m2px(C.DEAD_ZONE_C)[1] - int(C.DEAD_ZONE_R * PX) - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 120, 255), 1, cv2.LINE_AA)
    cv2.putText(img, "RAD", m2px(world.rad_center + np.array([-0.3, -0.1])), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (0, 0, 0), 1, cv2.LINE_AA)
    bx, by = m2px(C.BASE)
    hx, hy = int(22 + 48 * C.TANGENT[0]), int(22 + 48 * C.TANGENT[1])     # pads run along the base edge
    cv2.rectangle(img, (bx - hx, by - hy), (bx + hx, by + hy), (150, 90, 20), 2)
    cv2.putText(img, "BASE", (bx - 20, by + hy + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 120), 1)
    cv2.rectangle(img, m2px((0, C.AREA)), m2px((C.AREA, 0)), (255, 255, 255), 1)
    return img


def render_map(sim, base):
    img = base.copy()
    # covered cells tinted with the colour of the drone that searched them
    over = img.copy()
    idx = np.argwhere(sim.covered)
    for i, j in idx:
        owner = sim.cover_owner[i, j]
        r, g, b = C.DRONE_COLORS[owner % len(C.DRONE_COLORS)]
        x0, y1 = m2px((i * C.CELL, j * C.CELL)); x1, y0 = m2px(((i + 1) * C.CELL, (j + 1) * C.CELL))
        cv2.rectangle(over, (x0, y0), (x1 - 1, y1 - 1), (b, g, r), -1)
    img = cv2.addWeighted(over, 0.35, img, 0.65, 0)
    if sim.world.collapsed:
        x0, y0, x1, y1 = sim.world.weak_box
        cv2.rectangle(img, m2px((x0, y1)), m2px((x1, y0)), (0, 0, 255), 2)
        cv2.putText(img, "COLLAPSED", m2px((x0 + 0.2, y1 - 0.4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

    # rescuer routes
    for vid, r in sim.routes.items():
        v = sim.command.victims.get(vid, {})
        if v.get("confidence", 1) < 0.3:
            continue
        if r.get("direct_unsafe") and r["direct_path"]:
            pts = np.array([m2px(q) for q in r["direct_path"]], np.int32)
            for a, b in zip(pts[:-1:2], pts[1::2]):
                cv2.line(img, tuple(a), tuple(b), (60, 60, 230), 1, cv2.LINE_AA)
        if r["safe_path"]:
            pts = np.array([m2px(q) for q in r["safe_path"]], np.int32)
            cv2.polylines(img, [pts], False, (80, 230, 80), 2, cv2.LINE_AA)

    # comm links
    for a, b in sim.links:
        cv2.line(img, m2px(a), m2px(b), (255, 190, 120), 1, cv2.LINE_AA)

    # victims reported so far
    for vid, v in sim.command.victims.items():
        x, y = m2px(v["pos"])
        conf = v["status"].startswith("CONFIRMED")
        col = (80, 230, 80) if conf else (0, 220, 255)
        if v.get("source") == "phone":
            col = (255, 120, 255)
        cv2.circle(img, (x, y), 9, col, 2, cv2.LINE_AA)
        cv2.putText(img, f"{vid} P{v['priority']}", (x + 10, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)
    for s in sim.sos:
        x, y = m2px(s["pos"])
        rr = 8 + int(6 * (1 + np.sin(sim.t * 8)))
        cv2.circle(img, (x, y), rr, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(img, "SOS", (x - 14, y + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # drones
    for d in sim.drones:
        r, g, b = C.DRONE_COLORS[d.id % len(C.DRONE_COLORS)]
        if len(d.path) > 1:
            pts = np.array([m2px(q) for q in d.path[-400:]], np.int32)
            cv2.polylines(img, [pts], False, (b, g, r), 1, cv2.LINE_AA)
        x, y = m2px(d.xy)
        cv2.circle(img, (x, y), 7, (b, g, r), -1, cv2.LINE_AA)
        if not d.online:
            cv2.circle(img, (x, y), 11, (180, 120, 255), 1, cv2.LINE_AA)
        cv2.putText(img, str(d.id), (x - 4, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 0), 1)

    # header: drone status cards
    y0 = H - 168
    cv2.rectangle(img, (0, y0 - 8), (WM, H), (20, 20, 20), -1)
    for k, d in enumerate(sim.drones):
        r, g, b = C.DRONE_COLORS[d.id % len(C.DRONE_COLORS)]
        x = 6 + k * 119
        cv2.putText(img, f"D{d.id} {d.state[:10]}", (x, y0 + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (b, g, r), 1, cv2.LINE_AA)
        bw = int(100 * d.battery / 100)
        cv2.rectangle(img, (x, y0 + 16), (x + 100, y0 + 24), (80, 80, 80), 1)
        bc = (80, 200, 80) if d.battery > 40 else (0, 170, 255) if d.battery > 20 else (0, 0, 255)
        cv2.rectangle(img, (x, y0 + 16), (x + bw, y0 + 24), bc, -1)
        net = "ONLINE" if d.online else "OFFLINE-AUTO"
        if d.online and d.hops > 1:
            net = f"MESH x{d.hops - 1}"
        cv2.putText(img, net, (x, y0 + 38), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (200, 200, 200), 1, cv2.LINE_AA)
    # event feed
    for k, (t, msg, lvl) in enumerate(sim.events[-6:]):
        cv2.putText(img, f"{t:5.1f}s {msg[:92]}", (8, y0 + 60 + k * 17), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                    LVL_COL.get(lvl, (220, 220, 220)), 1, cv2.LINE_AA)
    # title
    cov = 100 * sim.coverage()
    found = sum(1 for v in sim.command.victims.values() if v["is_human"])
    cv2.putText(img, f"HALO Rescuer Map   t={sim.t:5.1f}s   searched {cov:4.1f}%   victims found {found}",
                (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def compose(img3d, imgmap, t):
    frame = np.hstack([img3d, imgmap])
    cv2.putText(frame, "HALO: AI-Powered Autonomous Rescue Drone Swarm  |  PyBullet simulation",
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    return frame
