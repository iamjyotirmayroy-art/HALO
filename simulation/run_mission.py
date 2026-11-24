"""
HALO mission runner.

    python run_mission.py                 # headless run, writes results/ + demo video
    python run_mission.py --gui           # watch it live in the PyBullet window
    python run_mission.py --drones 1      # single-drone baseline for comparison
    python run_mission.py --no-video      # fastest: metrics only
    python run_mission.py --area area.json --live   # started by the dashboard: rescuer-drawn area, real time
"""
import argparse, json, os, time
import numpy as np
import pybullet as p
import cv2

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

import config as C
from world import World, to_cell, cell_center, poly_mask
from planning import inflate, rescuer_route
from agents import DroneAgent, CommandAI, SEARCH, INVESTIGATE, RETURN, CHARGING, TRAPPED, DONE, IDLE, MANUAL
import perception as P
import render as R
import safety
import pilot_rl
from rescue import Dispatcher
from rescuer_link import RescuerLink
from cameras import CameraPool
import dashboard_export as DX


class Mission:
    def __init__(self, n_drones, n_teams, gui, out, video, aftershock=True, realtime=False):
        self.aftershock = aftershock
        self.pilot = pilot_rl.LearnedPilot() if C.PILOT == "learned" and pilot_rl.available() else None
        P.vision.available()             # load the victim detector now, so the first photo doesn't stall the mission
        self.pilot_steps = self.shield_steps = self.crashes = self.cruise_steps = 0
        self.contacts = set()
        self.realtime = realtime or gui
        self.out = out
        os.makedirs(out, exist_ok=True)
        self.rng = np.random.default_rng(C.SEED)
        self.t = 0.0
        self.events, self.sos, self.links, self.claimed = [], [], [], set()
        self.routes, self.phone_victims = {}, {}
        self.heat_zones = {}
        self.zone_version = 0
        self.video = video

        pads = [C.pad(i, n_drones) for i in range(n_drones)]
        init = np.array([[q[0], q[1], 0.05] for q in pads])
        self.env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=n_drones, initial_xyzs=init,
                              physics=Physics.PYB, pyb_freq=C.PYB_FREQ, ctrl_freq=C.CTRL_FREQ,
                              gui=gui, record=False, obstacles=False, user_debug_gui=False)
        self.client = self.env.CLIENT
        self.obs, _ = self.env.reset()
        if gui:
            p.resetDebugVisualizerCamera(13, 35, -50, [5, 6, 0], physicsClientId=self.client)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client)
        self.world = World(self.client, poly_mask(C.AREA_POLY))
        self.world.build(n_drones)
        # big coloured beacons above each drone so they're visible in the wide 3D shot
        self.markers = []
        for i in range(n_drones):
            col = [c / 255 for c in C.DRONE_COLORS[i % len(C.DRONE_COLORS)]] + [1]
            vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.16, rgbaColor=col, physicsClientId=self.client)
            self.markers.append(p.createMultiBody(0, -1, vis, [0, 0, -5], physicsClientId=self.client))
        self.n_humans = sum(s.is_human for s in self.world.sources)

        self.ctrl = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(n_drones)]
        batt = C.START_BATTERY if n_drones == len(C.START_BATTERY) else [100] * n_drones
        self.drones = [DroneAgent(i, pads[i], batt[i]) for i in range(n_drones)]
        self.command = CommandAI()
        self.dispatch = Dispatcher(self, n_teams)
        self.exporter = DX.Exporter(self)
        self.link = RescuerLink(self)
        self.cams = CameraPool(n_drones, os.path.join(out, "cams"))
        if realtime:
            self.cams.start()                      # dashboard mission: cameras ready before anyone opens them
        self.cam_meta, self.snapshots = {}, []
        corners = np.vstack([[0, 0], [C.AREA, C.AREA], pads])
        self.bounds = (np.r_[corners.min(0) - 0.5, 0.3], np.r_[corners.max(0) + 0.5, 2.5])   # geofence

        self.fly_blocked = inflate(self.world.tall, 1)
        self.free_cells = [tuple(c) for c in np.argwhere(~self.world.tall & self.world.area)]
        self.covered = np.zeros((C.N, C.N), bool)
        self.cover_owner = np.zeros((C.N, C.N), int)
        g = (np.arange(C.N) + 0.5) * C.CELL
        self.GX, self.GY = np.meshgrid(g, g, indexing="ij")
        self.gui = gui
        self.labels = {}
        self.rings, self.rescued_marked = {}, set()
        self.search_done = False
        self.phones_reported = set()

    # ---------------- helpers used by agents ----------------
    def event(self, msg, lvl="info"):
        self.events.append((self.t, msg, lvl))
        print(f"[{self.t:6.1f}s] {msg}")

    def covered_around(self, xy, r=0.75):
        m = np.hypot(self.GX - xy[0], self.GY - xy[1]) <= r
        return bool(self.covered[m & ~self.world.tall & self.world.area].all())

    def snap_free(self, xy):
        c = to_cell(xy)
        if not self.fly_blocked[c]:
            return xy
        free = np.argwhere(~self.fly_blocked)
        k = np.argmin(np.hypot(free[:, 0] - c[0], free[:, 1] - c[1]))
        return cell_center(tuple(free[k]))

    def coverage(self):
        tot = [c for c in self.free_cells if not self.world.sealed[c]]
        return sum(self.covered[c] for c in tot) / len(tot)

    # ---------------- comms (mesh relay) ----------------
    def update_comms(self):
        def no_signal(xy):
            return np.linalg.norm(xy - C.DEAD_ZONE_C) < C.DEAD_ZONE_R
        nodes = [d for d in self.drones if d.state not in (DONE,)]
        hop = {}
        frontier = []
        self.links = []
        for d in nodes:
            if not no_signal(d.xy) and np.linalg.norm(d.xy - C.BASE) <= C.BASE_RANGE:
                hop[d.id] = 1; frontier.append(d)
                if d.state not in (CHARGING,) and d.pos[2] > 0.2:
                    self.links.append((C.BASE, d.xy.copy()))
        while frontier:
            nxt = []
            for a in frontier:
                for b in nodes:
                    if b.id in hop or no_signal(b.xy) or no_signal(a.xy):
                        continue
                    if np.linalg.norm(a.xy - b.xy) <= C.MESH_RANGE:
                        hop[b.id] = hop[a.id] + 1; nxt.append(b)
                        self.links.append((a.xy.copy(), b.xy.copy()))
            frontier = nxt
        for d in self.drones:
            was = d.online
            linked = d.id in hop or d.state in (DONE, CHARGING)
            d.hops = hop.get(d.id, 0)
            if linked:
                d.last_link = self.t
                d.online = True
            elif self.t - getattr(d, "last_link", 0.0) >= C.OFFLINE_GRACE:
                d.online = False
            if was and not d.online:
                d.offline_since = getattr(d, "last_link", self.t)
                self.event(f"Drone {d.id} lost signal - continuing autonomously (edge AI)", "net")
            if d.online and not was:
                n = len(d.outbox)
                self.event(f"Drone {d.id} back online after {self.t - d.offline_since:.1f}s"
                           + (f" - synced {n} buffered report(s)" if n else ""), "net")
            if d.online and d.outbox:
                for msg in d.outbox:
                    if d.hops > 1:
                        msg["relayed_hops"] = d.hops - 1
                    self.command.receive(self, msg)
                d.outbox = []

    # ---------------- main loop ----------------
    def run(self):
        self.command.rebalance(self, "start")
        for d in self.drones:
            d.state = SEARCH
        self.event(f"Rescuer marked a {len(self.free_cells) * (C.CELL * C.MAP_SCALE) ** 2:,.0f} m² search area. "
                   f"Swarm of {len(self.drones)} drones split it into {len(self.drones)} equal zones", "info")
        dt = 1.0 / C.CTRL_FREQ
        writer, frame_t, base = None, 0.0, R.base_map(self.world)
        if self.video:
            raw = os.path.join(self.out, "_raw.mp4")
            writer = cv2.VideoWriter(raw, cv2.VideoWriter_fourcc(*"mp4v"), 15, (R.W3 + R.WM, R.H))
        step, slow_every, link_every, next_cam = 0, int(C.CTRL_FREQ / 4), C.CTRL_FREQ // 10, 0.0
        t_wall = time.time()                          # real-time pacing: wall-clock time of the next step
        finished_at = None
        while self.t < C.MAX_TIME:
            # sense: true drone states from the physics engine
            for i, d in enumerate(self.drones):
                st = self.obs[i]
                newpos = st[0:3].copy()
                d.distance += np.linalg.norm(newpos[:2] - d.pos[:2])
                d.pos = newpos
                if d.state == MANUAL:
                    d.heading = d.yaw
                elif np.hypot(*st[10:12]) > 0.15:
                    d.heading = float(np.arctan2(st[11], st[10]))
            # coverage marking
            for d in self.drones:
                if d.state in (SEARCH, INVESTIGATE) and d.pos[2] > 0.4:
                    m = (np.hypot(self.GX - d.pos[0], self.GY - d.pos[1]) <= C.THERMAL_RADIUS) & ~self.covered
                    self.covered |= m
                    self.cover_owner[m] = d.id
            # slower loop (4 Hz): comms, phone scans, trails, battery-aware thinking
            if step % slow_every == 0:
                self.count_crashes()
                self.update_comms()
                for d in self.drones:
                    if d.state in (SEARCH, INVESTIGATE, RETURN) and d.pos[2] > 0.3:
                        for pid, rssi in P.phone_rssi(self.world, d.xy, self.rng):
                            d.rssi_log.setdefault(pid, []).append((d.xy.copy(), rssi))
                    if not d.path or np.linalg.norm(d.xy - d.path[-1]) > 0.15:
                        d.path.append(d.xy.copy())   # flown trail, thinned for the map
                if step % (slow_every * 20) == 0:
                    self.phone_check()
                self.exporter.tick()
            if step % link_every == 0:
                self.link.poll(self)
            if self.t >= next_cam:
                next_cam = self.t + 1.0 / C.CAM_FPS
                self.update_cams()
            # think + battery
            for d in self.drones:
                d.think(self, dt)
                if d.state not in (CHARGING, DONE, TRAPPED, IDLE) and d.pos[2] > 0.1:
                    v = np.linalg.norm(self.obs[d.id][10:12])
                    d.battery -= (C.BATTERY_DRAIN_HOVER + C.BATTERY_DRAIN_MOVE * v) * dt
            # collision avoidance: vertical deconfliction between close drones
            targets = [d.target.copy() for d in self.drones]
            for a in self.drones:
                for b in self.drones:
                    near_pads = np.linalg.norm(b.xy - b.pad) < 0.35 and b.state == RETURN
                    if a.id < b.id and a.pos[2] > 0.3 and b.pos[2] > 0.3 and b.state not in (TRAPPED, MANUAL) \
                            and not near_pads and np.linalg.norm(a.xy - b.xy) < C.SEPARATION:
                        targets[b.id][2] = max(targets[b.id][2], a.pos[2] + 0.4)
            # smooth set-points: never ask the controller for a jump bigger than it can fly safely
            for i, d in enumerate(self.drones):
                if d.state == MANUAL:
                    targets[i] = self.manual_target(d, dt)
                    continue
                dz = np.clip(targets[i][2] - d.pos[2], -0.25, 0.3)
                dxy = targets[i][:2] - d.xy
                n = np.linalg.norm(dxy)
                if n > 0.4:
                    dxy = dxy / n * 0.4
                targets[i] = np.array([d.xy[0] + dxy[0], d.xy[1] + dxy[1], d.pos[2] + dz])
            # act: PID -> motor RPMs -> physics
            action = np.zeros((len(self.drones), 4))
            for i, d in enumerate(self.drones):
                if d.state == TRAPPED and d.pos[2] < 0.12:
                    continue          # motors off, beacon on
                if d.state in (CHARGING, DONE) and d.pos[2] < 0.12:
                    continue
                rpm, _, _ = self.ctrl[i].computeControlFromState(
                    control_timestep=dt, state=self.obs[i], target_pos=targets[i], target_rpy=np.array([0, 0, d.yaw]),
                    target_vel=d.vel if d.state == MANUAL else np.zeros(3))
                action[i] = rpm
            self.obs, _, _, _, _ = self.env.step(action)
            self.t += dt; step += 1
            self.dispatch.update(self, dt)

            if self.gui:
                self.gui_labels()
                if step % 4 == 0:
                    self.sync_3d()
            if self.realtime:
                t_wall = max(t_wall + dt, time.time() - 0.5)  # catch up after a slow step, never by more than 0.5 s
                time.sleep(max(0.0, t_wall - time.time()))
            if writer is not None and self.t >= frame_t:
                self.sync_3d()
                frame_t += C.FRAME_DT if not self.search_done else C.FRAME_DT * 3   # fast-forward rescue phase
                writer.write(R.compose(R.render_3d(self.client, self.t), R.render_map(self, base), self.t))

            no_work = not self.command.remaining_work(self)
            drones_done = all(d.state in (DONE, TRAPPED) or (d.state == CHARGING and no_work) for d in self.drones)
            if drones_done and not self.search_done:
                self.search_done = True
                self.event("All drones home. Search complete - rescue teams finishing", "ok")
                self.phone_check(final=True)
            if finished_at is None and self.search_done and not self.dispatch.busy():
                finished_at = self.t
                self.event("Mission complete: every victim reached by a rescue team", "ok")
                self.post_mission()
                end_hold = self.t + 3.0
            if finished_at is not None and self.t >= end_hold:
                break
        if finished_at is None:
            self.event("Time limit reached", "warn"); self.post_mission(); finished_at = self.t
        self.exporter.tick(force=True, final=True)
        if writer is not None:
            for _ in range(30):   # hold final frame 2 s
                writer.write(R.compose(R.render_3d(self.client, self.t), R.render_map(self, base), self.t))
            writer.release()
            os.system(f"ffmpeg -loglevel error -y -i {raw} -c:v libx264 -preset veryfast -pix_fmt yuv420p "
                      f"{os.path.join(self.out, 'halo_demo.mp4')} && rm {raw}")
        cv2.imwrite(os.path.join(self.out, "final_map.png"), R.render_map(self, base))
        self.cams.close()
        self.env.close()
        return self.report(finished_at)

    # ---------------- rescuer commands (dashboard) ----------------
    def manual_target(self, d, dt):
        """Rescuer stick -> safety filter -> position set-point. Auto-release after 2 s without input or on signal loss."""
        if self.t - self.link.ctrl_t > 2.0 or not d.online:
            self.release(d, "no input for 2 s" if d.online else "lost signal")
            return d.pos.copy()
        c = self.link.stick(d.id, self.t)
        v_des = np.zeros(3)
        if c:
            d.yaw += float(c["yaw_rate"]) * dt
            cy, sy = np.cos(d.yaw), np.sin(d.yaw)
            v_des = np.array([c["vx"] * cy - c["vy"] * sy, c["vx"] * sy + c["vy"] * cy, c["vz"]], float)
        others = [(o.pos, self.obs[o.id][10:13]) for o in self.drones if o is not d]
        d.vel, d.blocked = safety.safe_velocity(v_des, d.pos, d.vel, safety.ranges(self.client, d.pos),
                                                others, self.bounds, dt)
        step = d.hold + d.vel * dt - d.pos
        n = np.linalg.norm(step)
        d.hold = d.pos + (step if n <= 0.25 else step * 0.25 / n)   # no wind-up when the filter holds it back
        return d.hold.copy()

    def in_buffer(self, xy):
        """Inside the 0.5 m safety buffer around a building (inflated map), where the learned pilot hands over."""
        return 0 <= xy[0] < C.AREA and 0 <= xy[1] < C.AREA and bool(self.fly_blocked[to_cell(xy)])

    def count_crashes(self):
        """A crash = an airborne drone touching anything (building, debris, another drone). Trapped drones excluded."""
        now = set()
        for d, body in zip(self.drones, self.env.DRONE_IDS):
            if d.state != TRAPPED and d.pos[2] > 0.2:
                now |= {(d.id, c[2]) for c in p.getContactPoints(bodyA=body, physicsClientId=self.client)
                        if c[2] != self.env.PLANE_ID}
        for i, body in now - self.contacts:
            other = next((f"drone {o.id}" for o, b in zip(self.drones, self.env.DRONE_IDS) if b == body), "an obstacle")
            self.event(f"Drone {i} touched {other} at ({self.drones[i].pos[0]:.1f}, {self.drones[i].pos[1]:.1f}, "
                       f"{self.drones[i].pos[2]:.1f}) - {self.drones[i].state}", "warn")
        self.crashes += len(now - self.contacts)
        self.contacts = now

    def _abandon(self, d):
        """Drop a half-done photo check so the swarm searches that spot again."""
        if d.inv and not d.inv["source"].reported:
            s = d.inv["source"]
            self.claimed.discard(s.id)
            self.heat_zones.pop(s.id, None)
            self.covered[np.hypot(self.GX - s.pos[0], self.GY - s.pos[1]) <= C.THERMAL_RADIUS] = False
        d.inv = None

    def takeover(self, d):
        if d.state == TRAPPED or d.battery <= d.battery_needed_home() + 5:
            self.event(f"Drone {d.id} can't be flown manually: {'trapped' if d.state == TRAPPED else 'battery too low'}", "warn")
            return
        for o in self.drones:
            if o.state == MANUAL:
                self.release(o)                    # one rescuer-flown drone at a time
        self._abandon(d)
        d.state, d.route, d.wps, d.zone, d.landing = MANUAL, [], [], [], False
        d.hold, d.vel, d.yaw, d.blocked = d.pos.copy(), np.zeros(3), d.heading, False
        self.link.ctrl_t, self.link.control = self.t + 3.0, None    # 5 s to the first stick input
        self.event(f"Rescuer took manual control of drone {d.id} (anti-crash safety stays on)", "warn")
        self.command.rebalance(self, f"drone {d.id} under manual control")

    def release(self, d, why="released by rescuer"):
        if d.state != MANUAL:
            return
        rejoin = bool(self.command.remaining_work(self)) and not d.recalled
        d.state = SEARCH if rejoin else RETURN
        d.route = [] if rejoin else d.home_route(self)
        self.event(f"Drone {d.id} back to autonomous mode ({why})", "info")
        if rejoin:
            self.command.rebalance(self, f"drone {d.id} rejoined after manual control")

    def recall(self, d):
        if d.state in (DONE, TRAPPED):
            self.event(f"Drone {d.id} can't be recalled: {'already at base' if d.state == DONE else 'trapped'}", "warn")
            return
        d.recalled = True
        if d.state == CHARGING:
            d.state = DONE
        else:
            self._abandon(d)
            d.state, d.route, d.wps, d.zone = RETURN, d.home_route(self), [], []
            self.command.rebalance(self, f"drone {d.id} recalled")
        self.event(f"Rescuer recalled drone {d.id} to base", "warn")

    def snapshot(self, d):
        """Rescuer pressed 'Snapshot + AI check': high-res photo from the drone camera, AI check of what's in view."""
        eye, target = P.drone_view(d.pos, d.heading)
        rgb, seg = P.camera(self.client, eye, target, 640, 480)
        dets = P.detect(rgb, seg, self.world.sources)
        seen = [s for s in self.world.sources if s.bodies and np.isin(seg, s.bodies).any()]
        path = os.path.join(self.out, "snapshots", f"drone{d.id}_{self.t:.0f}s.jpg")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        cv2.imwrite(path, P.draw_boxes(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), dets))
        new = next((s for s in seen if s.id not in self.claimed and not s.reported), None)
        if new:
            self.claimed.add(new.id)
            d.photograph(self, new, new.temp_c + self.rng.normal(0, 0.6), eye, target)
            text = f"new heat source {new.id} found - AI result sent to the victim list"
        elif seen:
            text = "already reported: " + ", ".join(f"{s.id} ({P.LABELS[s.kind]})" for s in seen)
        else:
            text = "no person or heat source in view"
        self.snapshots.append(dict(drone=d.id, t=round(self.t, 1), img=os.path.relpath(path, self.out), text=text))
        self.event(f"Snapshot from drone {d.id}: {text}", "info")

    def update_cams(self):
        """Live camera frames, only for drones someone is watching (plus the one being flown). An offline
        drone sends nothing, so the dashboard keeps its last frame."""
        ids = [i for i in self.link.watch | {d.id for d in self.drones if d.state == MANUAL} if self.drones[i].online]
        if ids:
            self.cams.render(self, ids)
        self.cam_meta.update(self.cams.collect())

    def sync_3d(self):
        for d, mk in zip(self.drones, self.markers):
            p.resetBasePositionAndOrientation(mk, [d.pos[0], d.pos[1], d.pos[2] + 0.12], [0, 0, 0, 1],
                                              physicsClientId=self.client)
        for t in self.dispatch.teams:
            t.sync_3d()
        for vid, v in self.command.victims.items():
            if vid not in self.rings:
                col = [0.15, 0.95, 0.3, 1] if v["confidence"] >= 0.5 or v.get("source") == "phone" else [1, 0.8, 0, 1]
                vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.006], rgbaColor=col,
                                          physicsClientId=self.client)
                self.rings[vid] = p.createMultiBody(0, -1, vis, [v["pos"][0], v["pos"][1], 0.014],
                                                    physicsClientId=self.client)
            if v.get("reached_t") and vid not in self.rescued_marked:
                self.rescued_marked.add(vid)
                p.changeVisualShape(self.rings[vid], -1, rgbaColor=[0.1, 0.5, 1.0, 1], physicsClientId=self.client)
            if v["status"].startswith("DISMISSED") and vid not in self.rescued_marked:
                self.rescued_marked.add(vid)
                p.changeVisualShape(self.rings[vid], -1, rgbaColor=[0.4, 0.4, 0.4, 1], physicsClientId=self.client)

    def phone_check(self, final=False):
        """Pool all drones' phone readings. A phone with no heat/visual match in an already-searched
        spot = buried victim."""
        pooled = {}
        for d in self.drones:
            for pid, s in d.rssi_log.items():
                pooled.setdefault(pid, []).extend(s)
        thermal = [np.array(v["pos"]) for v in self.command.victims.values() if v.get("source") != "phone"]
        for pid, samples in pooled.items():
            if pid in self.phones_reported:
                continue
            est, rms = P.locate_phone(samples)
            if est is None:
                continue
            truth = next(s for s in self.world.sources if s.id == pid)
            self.phone_victims[pid] = dict(est=est.round(2).tolist(),
                                           error_m=round(float(np.linalg.norm(est - truth.pos)), 2),
                                           samples=len(samples))
            near = [np.linalg.norm(est - q) for q in thermal]
            if near and min(near) <= 1.5:
                self.phones_reported.add(pid)     # phone belongs to a victim already found
                continue
            if not (final or self.covered_around(est, 1.5)):
                continue                          # area not fully searched yet - wait
            self.phones_reported.add(pid)
            self.command.receive(self, dict(
                type="victim", id=pid, pos=est.round(2).tolist(), source="phone", trapped=True,
                label="phone signal only - likely BURIED (no thermal/visual)", confidence=0.6,
                status="LIKELY VICTIM - phone signal", measured_temp=None, is_human=True, kind="buried",
                t=round(self.t, 1), drone=None, photo=None))

    def gui_labels(self):
        for d in self.drones:
            txt = f"D{d.id} {d.state} {d.battery:.0f}%" + ("" if d.online else " OFFLINE")
            col = [c / 255 for c in C.DRONE_COLORS[d.id % len(C.DRONE_COLORS)]]
            pos = [d.pos[0], d.pos[1], d.pos[2] + 0.25]
            if d.id in self.labels:
                self.labels[d.id] = p.addUserDebugText(txt, pos, col, 1.1, replaceItemUniqueId=self.labels[d.id],
                                                       physicsClientId=self.client)
            else:
                self.labels[d.id] = p.addUserDebugText(txt, pos, col, 1.1, physicsClientId=self.client)

    # ---------------- after the search ----------------
    def post_mission(self):
        for vid, v in self.command.victims.items():
            if vid not in self.routes:
                r = rescuer_route(self.world, np.array(v["pos"]))
                self.routes[vid] = r
                v.setdefault("route", {k: r[k] for k in ("status", "direct", "safe", "note")})

    def report(self, finished_at):
        V = self.command.victims
        humans = [v for v in V.values() if v["is_human"]]
        decoys = [v for v in V.values() if not v["is_human"]]
        rep = dict(
            drones=len(self.drones),
            mission_time_s=round(finished_at, 1),
            area_searched_pct=round(100 * self.coverage(), 1),
            victims_total=self.n_humans,
            victims_found=len(humans),
            found_by_thermal_camera=sum(v["source"] != "phone" for v in humans),
            found_by_phone_signal_only=sum(v["source"] == "phone" for v in humans),
            partial_body_victims_found=sum(v.get("kind") in ("hand", "foot", "head") for v in humans),
            heat_signals_discarded=0,
            decoys_sent_for_verification=len(decoys),
            time_first_victim_s=min((v["t"] for v in humans if v["source"] != "phone"), default=None),
            time_all_thermal_victims_s=max((v["t"] for v in humans if v["source"] != "phone"), default=None),
            phone_location_error_m={k: v["error_m"] for k, v in self.phone_victims.items()},
            rerouted_routes=sum(1 for vid, r in self.routes.items() if r["direct_unsafe"] and V[vid]["is_human"]),
            rescue_teams=len(self.dispatch.teams),
            victims_reached_by_rescuers=sum(1 for v in humans if v.get("reached_t")),
            mean_detection_to_rescuer_s=round(float(np.mean([v["response_s"] for v in humans if v.get("response_s")])), 1)
                if any(v.get("response_s") for v in humans) else None,
            false_alarms_dismissed_by_rescuer=sum(1 for v in decoys if v["status"].startswith("DISMISSED")),
            ai_detector="YOLOv8n (HALO, trained on synthetic rubble scenes)" if P.vision.available() else "stand-in scores",
            ai_people_detected=f"{sum(v.get('confidence', 0) >= 0.5 for v in humans if v.get('photo'))}/{sum(1 for v in humans if v.get('photo'))}",
            ai_decoys_called_person=sum(v.get("confidence", 0) >= 0.5 for v in decoys),
            pilot="learned RL policy + anti-crash shield" if self.pilot else "classic path follower",
            learned_pilot_share_pct=round(100 * self.pilot_steps / max(1, self.cruise_steps), 1),
            shield_interventions_pct=round(100 * self.shield_steps / max(1, self.pilot_steps), 2),
            crashes=self.crashes,
            sos_events=self.sos,
            total_flight_distance_m=round(sum(d.distance for d in self.drones), 1),
            victims=sorted(V.values(), key=lambda v: -v["priority"]),
            timeline=[dict(t=round(t, 1), level=l, event=m) for t, m, l in self.events],
        )
        with open(os.path.join(self.out, "mission_report.json"), "w") as f:
            json.dump(rep, f, indent=2, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
        return rep


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--gui", action="store_true", help="open the PyBullet 3D window")
    ap.add_argument("--drones", type=int, default=C.NUM_DRONES)
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--out", default="results")
    ap.add_argument("--no-aftershock", action="store_true", help="disable the scripted building collapse")
    ap.add_argument("--max-time", type=float, default=C.MAX_TIME)
    ap.add_argument("--teams", type=int, default=C.NUM_RESCUERS)
    ap.add_argument("--area", help="JSON from the dashboard: poly + base (sim metres), origin (lat, lon), scale")
    ap.add_argument("--pilot", choices=["learned", "classic"], default=C.PILOT, help="cruise flight controller")
    ap.add_argument("--live", action="store_true",
                    help="real-time pace for the dashboard: rescuer verifies photos and can fly drones")
    a = ap.parse_args()
    C.MAX_TIME, C.PILOT = a.max_time, a.pilot
    if a.area:
        with open(a.area) as f:
            spec = json.load(f)
        C.AREA_POLY, C.ORIGIN, C.MAP_SCALE = spec["poly"], tuple(spec["origin"]), spec["scale"]
        if spec.get("base"):
            C.set_base(spec["base"])
    if a.live:
        C.VERIFY_DELAY, C.MAX_TIME = 45.0, max(a.max_time, 1800.0)   # give the rescuer time to look at photos
    m = Mission(a.drones, a.teams, a.gui, a.out, video=not a.no_video, aftershock=not a.no_aftershock,
                realtime=a.live)
    r = m.run()
    print("\n===== HALO MISSION SUMMARY =====")
    for k in ["drones", "mission_time_s", "area_searched_pct", "victims_found", "victims_total",
              "found_by_thermal_camera", "found_by_phone_signal_only", "partial_body_victims_found",
              "heat_signals_discarded", "decoys_sent_for_verification", "time_first_victim_s",
              "time_all_thermal_victims_s", "rerouted_routes", "rescue_teams", "victims_reached_by_rescuers",
              "mean_detection_to_rescuer_s", "false_alarms_dismissed_by_rescuer", "ai_detector", "ai_people_detected",
              "ai_decoys_called_person", "pilot", "learned_pilot_share_pct", "shield_interventions_pct", "crashes", "phone_location_error_m"]:
        print(f"{k:32s} {r[k]}")
    print(f"{'sos_events':32s} {len(r['sos_events'])}")
