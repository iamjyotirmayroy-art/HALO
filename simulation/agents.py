"""
Multi-agent AI: one autonomous agent per drone + a central HALO Command AI.

Each DroneAgent decides on its own (search, investigate, return to charge,
avoid others, call SOS). The Command AI splits zones, re-balances the swarm
after any event and talks to the rescuer. If a drone loses radio contact it
keeps working from its own plan and buffers reports until it reconnects.
"""
import numpy as np
import config as C
from planning import split_zones, coverage_waypoints, flight_path, line_clear
import perception as P

# drone states (MANUAL = flown by a rescuer from the dashboard, through the safety filter)
IDLE, SEARCH, INVESTIGATE, RETURN, CHARGING, TRAPPED, DONE, MANUAL = (
    "IDLE", "SEARCH", "INVESTIGATE", "RETURN", "CHARGING", "TRAPPED", "DONE", "MANUAL")


class DroneAgent:
    def __init__(self, i, pad_xy, battery):
        self.id = i
        self.pad = pad_xy
        self.alt = C.SEARCH_ALT + C.ALT_LAYER * i
        self.battery = float(battery)
        self.state = IDLE
        self.pos = np.array([pad_xy[0], pad_xy[1], 0.05])
        self.target = self.pos.copy()
        self.route = []          # pending flight waypoints (xy)
        self.zone = []           # cells assigned
        self.wps = []            # coverage waypoints remaining
        self.online = True
        self.hops = 0
        self.outbox = []         # reports waiting for a radio link
        self.offline_since = None
        self.inv = None          # current investigation
        self.inspect_inside_t = None
        self.rssi_log = {}       # phone id -> [(xy, rssi)]
        self.path = []           # flown trail (for the map)
        self.distance = 0.0
        self.yaw = 0.0           # commanded yaw (rescuer control)
        self.heading = 0.0       # direction the camera faces
        self.vel = np.zeros(3)   # last safe velocity (rescuer control)
        self.hold = self.pos.copy()   # position set-point while a rescuer flies it
        self.blocked = False     # safety filter overrode the rescuer's input
        self.recalled = False    # rescuer recalled it: land and stay

    @property
    def xy(self):
        return self.pos[:2]

    def assign(self, cells, fly_blocked):
        self.zone = list(cells)
        self.wps = coverage_waypoints(self.zone, self.xy)
        self.route = []
        if self.state in (IDLE, SEARCH):
            self.state = SEARCH

    def battery_needed_home(self):
        stage = C.stage(self.pad)
        d = np.linalg.norm(self.xy - stage) + np.linalg.norm(stage - self.pad) + 1.0
        t = d / C.SPEED
        return t * (C.BATTERY_DRAIN_HOVER + C.BATTERY_DRAIN_MOVE * C.SPEED) + C.BATTERY_RESERVE

    # ------------------------------------------------------------------
    def think(self, sim, dt):
        """Autonomous decision step (runs on the drone, no link needed)."""
        w = sim.world
        if self.state in (TRAPPED, DONE, IDLE):
            return
        if self.state == CHARGING:
            self.battery = min(100.0, self.battery + C.CHARGE_RATE * dt)
            if self.battery >= 99.5:
                if sim.command.remaining_work(sim):
                    sim.event(f"Drone {self.id} recharged to 100% - rejoining the search", "info")
                    self.state = SEARCH
                    sim.command.rebalance(sim, reason=f"drone {self.id} rejoined")
                else:
                    self.state = DONE
            return

        if self.state == MANUAL:
            if self.battery <= self.battery_needed_home():     # failsafe overrides the rescuer
                self.state, self.route = RETURN, self.home_route(sim)
                sim.event(f"Drone {self.id} battery {self.battery:.0f}% - failsafe ends manual control, "
                          f"returning home", "warn")
            return

        # low battery -> fly home (agent decides this itself)
        if self.state in (SEARCH, INVESTIGATE) and self.battery <= self.battery_needed_home():
            if self.state == INVESTIGATE and self.inv is not None:
                self.inv = None
            self.state = RETURN
            self.route = self.home_route(sim)
            sim.event(f"Drone {self.id} battery {self.battery:.0f}% - returning to charge. "
                      f"Swarm re-splitting its zone", "warn")
            sim.command.rebalance(sim, reason=f"drone {self.id} low battery")
            return

        if self.state == SEARCH:
            hits = P.thermal_scan(w, self.xy, self.pos[2], sim.rng)
            hits = [h for h in hits if h[0].id not in sim.claimed]
            if hits:
                s, temp = hits[0]
                sim.claimed.add(s.id)
                self.inv = dict(source=s, temp=temp, t0=sim.t, phase="dive")
                self.state = INVESTIGATE
                self.route = []
                self.report(sim, dict(type="heat", id=s.id, pos=s.pos.round(2).tolist(), temp=round(temp, 1),
                                      text=f"Drone {self.id}: unusual heat signal "
                                      f"{temp:.1f} C at ({s.pos[0]:.1f}, {s.pos[1]:.1f}) - taking photo"))
                return
            if not self.route:
                if not self.wps:
                    # zone finished: ask Command AI for a share of whatever is left (work stealing)
                    if sim.command.remaining_work(sim) and sim.t - getattr(self, "_last_ask", -9) > 2.0:
                        self._last_ask = sim.t
                        sim.command.rebalance(sim, reason=f"drone {self.id} finished its zone", quiet=True)
                        if self.wps:
                            return
                    self.state = RETURN
                    self.route = self.home_route(sim)
                    return
                nxt = self.wps.pop(0)
                # skip waypoints already covered by someone else
                while self.wps and sim.covered_around(nxt):
                    nxt = self.wps.pop(0)
                nxt = sim.snap_free(nxt)
                self.route = flight_path(sim.fly_blocked, self.xy, nxt)
            self._follow(self.alt, sim)

        elif self.state == INVESTIGATE:
            inv = self.inv
            s = inv["source"]
            over = np.array([s.pos[0] - 0.05, s.pos[1]])
            if inv["phase"] == "dive":
                self.target = np.array([over[0], over[1], C.INSPECT_ALT])
                if np.linalg.norm(self.pos - self.target) < 0.12:
                    inv["phase"] = "photo"; inv["t_photo"] = sim.t
            elif inv["phase"] == "photo":
                if sim.t - inv["t_photo"] > 0.5:
                    self.photograph(sim, s, inv["temp"], self.pos - np.array([0.0, 0.0, 0.04]), [*s.pos, 0.0])
                    inv["phase"] = "climb"
                    if w.inside_weak(self.xy, margin=0.1):
                        self.inspect_inside_t = sim.t
            elif inv["phase"] == "climb":
                # scripted aftershock while this drone is inside the damaged building
                if self.inspect_inside_t is not None and not w.collapsed and sim.aftershock:
                    if sim.t - self.inspect_inside_t >= C.AFTERSHOCK_DELAY:
                        w.aftershock_collapse()
                        sim.event("AFTERSHOCK! Damaged building at (5.2, 9.8) collapsed", "alarm")
                        self._trapped(sim)
                        return
                    self.target = np.array([self.pos[0], self.pos[1], C.INSPECT_ALT])
                    return
                self.target = np.array([self.pos[0], self.pos[1], self.alt])
                if abs(self.pos[2] - self.alt) < 0.1:
                    self.inv = None
                    self.state = SEARCH

        elif self.state == RETURN:
            if not self.route:
                self.route = [self.pad]
            self._follow(self.alt, sim)
            if getattr(self, "landing", False) or np.linalg.norm(self.xy - self.pad) < 0.3:
                self.landing = True
                self.route = []
                self.target = np.array([self.pad[0], self.pad[1], 0.05])
                if self.pos[2] < 0.15:
                    self.landing = False
                    if self.recalled:
                        self.state = DONE
                        sim.event(f"Drone {self.id} landed at base (recalled by rescuer)", "info")
                    elif self.battery < 95:
                        self.state = CHARGING
                        sim.event(f"Drone {self.id} docked on charging pad", "info")
                    else:
                        self.state = DONE

    def photograph(self, sim, s, temp, eye, target):
        """Photo + thermal of heat source s, AI check, report to Command AI (buffered if offline)."""
        path = f"{sim.out}/photos/{s.id}_drone{self.id}.png"
        P.take_photo(sim.client, eye, target, sim.world.sources, path)
        s.reported = True
        trapped = s.kind in ("hand", "foot", "head") or sim.world.inside_weak(s.pos)
        self.report(sim, dict(type="victim", id=s.id, pos=s.pos.round(2).tolist(), photo=path,
                              thermal=path.replace(".png", "_thermal.png"), drone=self.id, t=round(sim.t, 1),
                              trapped=trapped, source="thermal+camera", is_human=s.is_human, kind=s.kind,
                              **P.ai_check(s, temp, sim.rng)))

    def home_route(self, sim):
        return flight_path(sim.fly_blocked, self.xy, C.stage(self.pad)) + [self.pad]

    def _trapped(self, sim):
        self.state = TRAPPED
        self.route, self.wps = [], []
        self.target = np.array([self.pos[0], self.pos[1], 0.05])
        sim.sos.append(dict(drone=self.id, pos=self.xy.round(2).tolist(), t=round(sim.t, 1)))
        self.report(sim, dict(type="sos", text=f"SOS! Drone {self.id} TRAPPED inside collapsed building - "
                              f"no exit path. Last position ({self.xy[0]:.2f}, {self.xy[1]:.2f}). Beacon active"))
        sim.command.rebalance(sim, reason=f"drone {self.id} trapped")

    def _follow(self, alt, sim=None):
        if not self.route:
            self.target = np.array([self.pos[0], self.pos[1], alt])
            return
        goal = self.route[0]
        # re-validate: if something now blocks the straight line to the next waypoint, re-plan
        if sim is not None and not line_clear(sim.fly_blocked, self.xy, goal):
            self.route = flight_path(sim.fly_blocked, self.xy, self.route[-1])
            goal = self.route[0]
        d = goal - self.xy
        dist = np.linalg.norm(d)
        if dist < 0.3 or (len(self.route) == 1 and dist < 0.2):
            self.route.pop(0)
            return
        # carrot: a point SPEED*lookahead ahead along the path
        near_pad = np.linalg.norm(self.xy - self.pad) < 1.2
        step = min(dist, 0.15 if near_pad else 0.35)
        carrot = self.xy + d / dist * step
        self.target = np.array([carrot[0], carrot[1], alt])

    def report(self, sim, msg):
        msg = dict(msg); msg.setdefault("drone", self.id); msg["t_sent"] = round(sim.t, 1)
        self.outbox.append(msg)


class CommandAI:
    """Central coordinator that talks to the rescuer."""
    def __init__(self):
        self.victims = {}
        self.inbox = []

    def remaining_work(self, sim):
        return [c for c in sim.free_cells if not sim.covered[c] and not sim.world.sealed[c]]

    def rebalance(self, sim, reason="start", quiet=False):
        """Evenly re-split everything not yet searched among the drones that can search and are online."""
        searchers = [d for d in sim.drones if d.state in (SEARCH, IDLE, INVESTIGATE) and d.online]
        offline = [d for d in sim.drones if d.state in (SEARCH, INVESTIGATE) and not d.online]
        reserved = set()
        for d in offline:                     # offline drones keep working their own plan
            reserved.update(d.zone)
        cells = [c for c in self.remaining_work(sim) if c not in reserved]
        if not searchers:
            return
        parts = split_zones(cells, [d.xy for d in searchers])
        for d, part in zip(searchers, parts):
            d.assign(part, sim.fly_blocked)
        # investigating drones get a share too once done: include them by giving an empty plan now
        sim.zone_version += 1
        if reason != "start" and not quiet:
            sim.event(f"Command AI re-split {len(cells)} unsearched cells across "
                      f"{len(searchers)} drones ({reason})", "info")

    def receive(self, sim, msg):
        self.inbox.append(msg)
        if msg["type"] == "victim" and msg.get("source") == "phone":
            msg["priority"] = P.priority_score(msg, sim.world)
            self.victims[msg["id"]] = msg
            sim.event(f"PHONE SIGNAL with no heat/visual at ({msg['pos'][0]:.1f}, {msg['pos'][1]:.1f}): "
                      f"likely BURIED victim, priority {msg['priority']}", "verify")
            sim.dispatch.new_report(sim, msg)
        elif msg["type"] == "victim":
            msg["priority"] = P.priority_score(msg, sim.world)
            self.victims[msg["id"]] = msg
            lvl = "victim" if msg["status"].startswith("CONFIRMED") else "verify"
            sim.event(f"{msg['status']}: {msg['label']} ({msg['confidence']*100:.0f}% AI conf.) at "
                      f"({msg['pos'][0]:.1f}, {msg['pos'][1]:.1f}) - photo sent, priority {msg['priority']}", lvl)
            sim.dispatch.new_report(sim, msg)
        elif msg["type"] == "sos":
            sim.event(msg["text"], "alarm")
        elif msg["type"] == "heat":
            sim.heat_zones[msg["id"]] = dict(pos=msg["pos"], temp=msg["temp"], t=round(sim.t, 1), drone=msg["drone"])
            sim.event(msg["text"], "heat")
