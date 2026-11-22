"""
HALO mission runner.

    python run_mission.py                 # headless run, writes results/ + demo video
    python run_mission.py --gui           # watch it live in the PyBullet window
    python run_mission.py --drones 1      # single-drone baseline for comparison
    python run_mission.py --no-video      # fastest: metrics only
"""
import argparse, json, os, time
import numpy as np
import pybullet as p
import cv2

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

import config as C
from world import World, to_cell, cell_center
from planning import inflate, rescuer_route
from agents import DroneAgent, CommandAI, SEARCH, INVESTIGATE, RETURN, CHARGING, TRAPPED, DONE, IDLE
import perception as P
import render as R


class Mission:
    def __init__(self, n_drones, gui, out, video, aftershock=True):
        self.aftershock = aftershock
        self.out = out
        os.makedirs(out, exist_ok=True)
        self.rng = np.random.default_rng(C.SEED)
        self.t = 0.0
        self.events, self.sos, self.links, self.claimed = [], [], [], set()
        self.routes, self.phone_victims = {}, {}
        self.zone_version = 0
        self.video = video

        pads = [C.BASE + np.array([0, (i - (C.NUM_DRONES - 1) / 2) * C.PAD_SPACING]) for i in range(n_drones)]
        init = np.array([[q[0], q[1], 0.05] for q in pads])
        self.env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=n_drones, initial_xyzs=init,
                              physics=Physics.PYB, pyb_freq=C.PYB_FREQ, ctrl_freq=C.CTRL_FREQ,
                              gui=gui, record=False, obstacles=False, user_debug_gui=False)
        self.client = self.env.CLIENT
        self.obs, _ = self.env.reset()
        if gui:
            p.resetDebugVisualizerCamera(13, 35, -50, [5, 6, 0], physicsClientId=self.client)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client)
        self.world = World(self.client)
        self.world.build()
        # big coloured beacons above each drone so they're visible in the wide 3D shot
        self.markers = []
        for i in range(n_drones):
            col = [c / 255 for c in C.DRONE_COLORS[i % len(C.DRONE_COLORS)]] + [1]
            vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.13, rgbaColor=col, physicsClientId=self.client)
            self.markers.append(p.createMultiBody(0, -1, vis, [0, 0, -5], physicsClientId=self.client))
        self.n_humans = sum(s.is_human for s in self.world.sources)

        self.ctrl = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(n_drones)]
        batt = C.START_BATTERY if n_drones == len(C.START_BATTERY) else [100] * n_drones
        self.drones = [DroneAgent(i, pads[i], batt[i]) for i in range(n_drones)]
        self.command = CommandAI()

        self.fly_blocked = inflate(self.world.tall, 1)
        self.free_cells = [(i, j) for i in range(C.N) for j in range(C.N) if not self.world.tall[i, j]]
        self.covered = np.zeros((C.N, C.N), bool)
        self.cover_owner = np.zeros((C.N, C.N), int)
        g = (np.arange(C.N) + 0.5) * C.CELL
        self.GX, self.GY = np.meshgrid(g, g, indexing="ij")
        self.gui = gui
        self.labels = {}

    # ---------------- helpers used by agents ----------------
    def event(self, msg, lvl="info"):
        self.events.append((self.t, msg, lvl))
        print(f"[{self.t:6.1f}s] {msg}")

    def covered_around(self, xy, r=0.75):
        m = np.hypot(self.GX - xy[0], self.GY - xy[1]) <= r
        return bool(self.covered[m & ~self.world.tall].all())

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
        self.event(f"Rescuer marked a {C.AREA:.0f}x{C.AREA:.0f} m search area. Swarm of {len(self.drones)} "
                   f"drones split it into {len(self.drones)} equal zones", "info")
        dt = 1.0 / C.CTRL_FREQ
        writer, frame_t, base = None, 0.0, R.base_map(self.world)
        if self.video:
            raw = os.path.join(self.out, "_raw.mp4")
            writer = cv2.VideoWriter(raw, cv2.VideoWriter_fourcc(*"mp4v"), 15, (R.W3 + R.WM, R.H))
        step, slow_every = 0, int(C.CTRL_FREQ / 4)
        t_wall = time.time()
        finished_at = None
        while self.t < C.MAX_TIME:
            # sense: true drone states from the physics engine
            for i, d in enumerate(self.drones):
                st = self.obs[i]
                newpos = st[0:3].copy()
                d.distance += np.linalg.norm(newpos[:2] - d.pos[:2])
                d.pos = newpos
            # coverage marking
            for d in self.drones:
                if d.state in (SEARCH, INVESTIGATE) and d.pos[2] > 0.4:
                    m = (np.hypot(self.GX - d.pos[0], self.GY - d.pos[1]) <= C.THERMAL_RADIUS) & ~self.covered
                    self.covered |= m
                    self.cover_owner[m] = d.id
            # slower loop (4 Hz): comms, phone scans, trails, battery-aware thinking
            if step % slow_every == 0:
                self.update_comms()
                for d in self.drones:
                    if d.state in (SEARCH, INVESTIGATE, RETURN) and d.pos[2] > 0.3:
                        for pid, rssi in P.phone_rssi(self.world, d.xy, self.rng):
                            d.rssi_log.setdefault(pid, []).append((d.xy.copy(), rssi))
                    d.path.append(d.xy.copy())
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
                    if a.id < b.id and a.pos[2] > 0.3 and b.pos[2] > 0.3 and b.state != TRAPPED \
                            and not near_pads and np.linalg.norm(a.xy - b.xy) < C.SEPARATION:
                        targets[b.id][2] = max(targets[b.id][2], a.pos[2] + 0.4)
            # smooth set-points: never ask the controller for a jump bigger than it can fly safely
            for i, d in enumerate(self.drones):
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
                rpm, _, _ = self.ctrl[i].computeControlFromState(control_timestep=dt, state=self.obs[i],
                                                                 target_pos=targets[i])
                action[i] = rpm
            self.obs, _, _, _, _ = self.env.step(action)
            self.t += dt; step += 1

            if self.gui:
                self.gui_labels()
                for d, mk in zip(self.drones, self.markers):
                    p.resetBasePositionAndOrientation(mk, [d.pos[0], d.pos[1], d.pos[2] + 0.12], [0, 0, 0, 1],
                                                      physicsClientId=self.client)
                time.sleep(max(0, dt - (time.time() - t_wall))); t_wall = time.time()
            if writer is not None and self.t >= frame_t:
                for d, mk in zip(self.drones, self.markers):
                    p.resetBasePositionAndOrientation(mk, [d.pos[0], d.pos[1], d.pos[2] + 0.12], [0, 0, 0, 1],
                                                      physicsClientId=self.client)
                frame_t += C.FRAME_DT
                writer.write(R.compose(R.render_3d(self.client, self.t), R.render_map(self, base), self.t))

            no_work = not self.command.remaining_work(self)
            if finished_at is None and all(d.state in (DONE, TRAPPED) or (d.state == CHARGING and no_work)
                                           for d in self.drones):
                finished_at = self.t
                self.event("All drones home. Search complete", "ok")
                self.post_mission()
                end_hold = self.t + 4.0
            if finished_at is not None and self.t >= end_hold:
                break
        if finished_at is None:
            self.event("Time limit reached", "warn"); self.post_mission(); finished_at = self.t
        if writer is not None:
            for _ in range(30):   # hold final frame 2 s
                writer.write(R.compose(R.render_3d(self.client, self.t), R.render_map(self, base), self.t))
            writer.release()
            os.system(f"ffmpeg -loglevel error -y -i {raw} -c:v libx264 -preset veryfast -pix_fmt yuv420p "
                      f"{os.path.join(self.out, 'halo_demo.mp4')} && rm {raw}")
        cv2.imwrite(os.path.join(self.out, "final_map.png"), R.render_map(self, base))
        self.env.close()
        return self.report(finished_at)

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
        # phone-signal locating (pool every drone's readings)
        pooled = {}
        for d in self.drones:
            for pid, s in d.rssi_log.items():
                pooled.setdefault(pid, []).extend(s)
        thermal = [np.array(v["pos"]) for v in self.command.victims.values()]
        for pid, samples in pooled.items():
            est, rms = P.locate_phone(samples)
            if est is None:
                continue
            truth = next(s for s in self.world.sources if s.id == pid)
            err = float(np.linalg.norm(est - truth.pos))
            self.phone_victims[pid] = dict(est=est.round(2).tolist(), error_m=round(err, 2), samples=len(samples))
            near = [np.linalg.norm(est - q) for q in thermal]
            if not near or min(near) > 1.5:
                vid = f"{pid}"
                rep = dict(type="victim", id=vid, pos=est.round(2).tolist(), source="phone", trapped=True,
                           label="phone signal only - likely BURIED (no thermal/visual)", confidence=0.6,
                           status="LIKELY VICTIM - please verify", measured_temp=None, is_human=True,
                           kind="buried", t=round(self.t, 1), drone=None, photo=None)
                self.command.receive(self, rep)
        # safe rescuer routes to every reported human-like signal
        for vid, v in self.command.victims.items():
            r = rescuer_route(self.world, np.array(v["pos"]))
            self.routes[vid] = r
            v["route"] = {k: r[k] for k in ("status", "direct", "safe", "note")}
            if r["direct_unsafe"]:
                self.event(f"Route to {vid}: direct path unsafe -> rerouted "
                           f"({r['direct']['length_m']} m -> {r['safe']['length_m']} m, "
                           f"dose {r['direct']['dose']} -> {r['safe']['dose']})", "ok")

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
            rerouted_routes=sum(1 for r in self.routes.values() if r["direct_unsafe"]),
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
    a = ap.parse_args()
    C.MAX_TIME = a.max_time
    m = Mission(a.drones, a.gui, a.out, video=not a.no_video, aftershock=not a.no_aftershock)
    r = m.run()
    print("\n===== HALO MISSION SUMMARY =====")
    for k in ["drones", "mission_time_s", "area_searched_pct", "victims_found", "victims_total",
              "found_by_thermal_camera", "found_by_phone_signal_only", "partial_body_victims_found",
              "heat_signals_discarded", "decoys_sent_for_verification", "time_first_victim_s",
              "time_all_thermal_victims_s", "rerouted_routes", "phone_location_error_m"]:
        print(f"{k:32s} {r[k]}")
    print(f"{'sos_events':32s} {len(r['sos_events'])}")
