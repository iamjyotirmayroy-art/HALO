"""
Ground rescue teams.

The Command AI hands each free team the highest-priority victim, along the
hazard-aware safe route. Uncertain detections ("please verify") are first
checked by a rescuer at base looking at the drone's photo: real people are
dispatched, non-human heat sources are dismissed. Nothing is dropped by the AI.
"""
import numpy as np
import pybullet as p
import config as C
from planning import rescuer_route


class RescueTeam:
    def __init__(self, i, n, client):
        self.id = i
        self.name = f"R{i + 1}"
        self.home = C.ENTRY + C.TANGENT * (i - (n - 1) / 2) * 0.5
        self.pos = self.home.copy()
        self.state = "READY"          # READY | EN_ROUTE | ON_SITE | RETURNING
        self.path, self.victim, self.t_site = [], None, 0.0
        self.trail = []
        self.rescued = []
        col = p.createVisualShape(p.GEOM_CAPSULE, radius=0.09, length=0.3,
                                  rgbaColor=[1.0, 0.55, 0.0, 1], physicsClientId=client)
        helmet = p.createVisualShape(p.GEOM_SPHERE, radius=0.08, rgbaColor=[1, 1, 1, 1], physicsClientId=client)
        self.body = p.createMultiBody(0, -1, col, [self.pos[0], self.pos[1], 0.24], physicsClientId=client)
        self.head = p.createMultiBody(0, -1, helmet, [self.pos[0], self.pos[1], 0.5], physicsClientId=client)
        self.client = client

    def sync_3d(self):
        p.resetBasePositionAndOrientation(self.body, [self.pos[0], self.pos[1], 0.24], [0, 0, 0, 1],
                                          physicsClientId=self.client)
        p.resetBasePositionAndOrientation(self.head, [self.pos[0], self.pos[1], 0.5], [0, 0, 0, 1],
                                          physicsClientId=self.client)

    def _walk(self, dt):
        step = C.RESCUER_SPEED * dt
        while self.path and step > 0:
            d = np.array(self.path[0]) - self.pos
            n = np.linalg.norm(d)
            if n <= step:
                self.pos = np.array(self.path.pop(0), float); step -= n
            else:
                self.pos = self.pos + d / n * step; step = 0
        return not self.path

    def update(self, sim, dt):
        if self.state == "EN_ROUTE":
            if self._walk(dt):
                v = self.victim
                v["rescue_team"] = self.name
                v["reached_t"] = round(sim.t, 1)
                v["response_s"] = round(sim.t - v["t_detected"], 1)
                where = "perimeter (heavy-rescue requested)" if v.get("route", {}).get("note") else "victim"
                sim.event(f"Rescue team {self.name} reached {v['id']} ({where}) - "
                          f"{v['response_s']:.0f}s after detection", "ok")
                self.state, self.t_site = "ON_SITE", sim.t
                self.rescued.append(v["id"])
        elif self.state == "ON_SITE":
            if sim.t - self.t_site >= C.RESCUER_ON_SITE:
                self.state = "RETURNING"
                self.path = list(reversed(self.victim["route_path"]))[1:] + [self.home.tolist()]
        elif self.state == "RETURNING":
            if self._walk(dt):
                self.state, self.victim = "READY", None
        if int(sim.t * 4) != int((sim.t - dt) * 4):
            self.trail.append(self.pos.copy())


class Dispatcher:
    """Part of the Command AI: verification queue + team assignment."""
    def __init__(self, sim, n_teams):
        self.teams = [RescueTeam(i, n_teams, sim.client) for i in range(n_teams)]
        self.queue = []           # victims waiting for a team
        self.verify = []          # (due_time, victim) waiting for a human look at the photo

    def new_report(self, sim, v):
        v["t_detected"] = sim.t
        if v.get("source") == "phone" or v["confidence"] >= 0.5:
            self.queue.append(v)
        else:
            self.verify.append((sim.t + C.VERIFY_DELAY, v))

    def busy(self):
        return bool(self.queue or self.verify or any(t.state != "READY" for t in self.teams))

    def decide(self, sim, v, person):
        """A rescuer's verdict on a victim photo: dispatch a team, or dismiss it."""
        self.verify = [it for it in self.verify if it[1] is not v]
        if person:
            v["status"] = "VERIFIED BY RESCUER"
            sim.event(f"Rescuer confirmed {v['id']} is a person ({v['label']}) - dispatching", "victim")
            if v not in self.queue and not v.get("route_path"):
                self.queue.append(v)
        else:
            v["status"] = "DISMISSED - not a person"
            if v in self.queue:
                self.queue.remove(v)
            sim.event(f"Rescuer marked {v['id']} as not a person - dismissed", "info")

    def send(self, sim, v):
        """Rescuer asked for a team now: verified, and first in the queue."""
        if v.get("route_path"):
            sim.event(f"Team already assigned to {v['id']}", "info")
            return
        if v not in self.queue:
            self.decide(sim, v, True)
        v["urgent"] = True
        sim.event(f"Rescuer: send the next free team to {v['id']}", "info")

    def update(self, sim, dt):
        # no answer from the rescuer in time: the base team reviews uncertain photos itself
        for due, v in list(self.verify):
            if sim.t >= due:
                self.decide(sim, v, v["is_human"])
        # assign free teams to the most urgent victims (rescuer requests first)
        self.queue.sort(key=lambda v: (not v.get("urgent"), -v["priority"]))
        for team in self.teams:
            if team.state != "READY" or not self.queue:
                continue
            v = self.queue.pop(0)
            r = rescuer_route(sim.world, np.array(v["pos"]))
            sim.routes[v["id"]] = r
            v["route"] = {k: r[k] for k in ("status", "direct", "safe", "note")}
            if not r["safe_path"]:
                v["status"] += " | NO SAFE ROUTE - specialist team"
                sim.event(f"No safe route to {v['id']} - flagged for specialist team", "alarm")
                continue
            v["route_path"] = [C.ENTRY.tolist()] + r["safe_path"]
            team.victim, team.state = v, "EN_ROUTE"
            team.path = list(v["route_path"])
            msg = f"Team {team.name} dispatched to {v['id']} (priority {v['priority']})"
            if r["direct_unsafe"]:
                msg += f" - direct route unsafe, rerouted ({r['direct']['length_m']}m -> {r['safe']['length_m']}m)"
            sim.event(msg, "info")
        for team in self.teams:
            team.update(sim, dt)
