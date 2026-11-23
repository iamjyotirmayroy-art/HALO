"""
Writes the mission state for the rescuer dashboard, stored locally:
  results/live.json    - latest state, rewritten twice a second (the dashboard polls it live)
  results/replay.json  - every state of the mission, so the dashboard can replay it afterwards
No cloud needed. Swap the file writes for a database/API later for multi-team use.
"""
import json, os
import numpy as np
import config as C


def _j(o):
    return o.item() if hasattr(o, "item") else (o.tolist() if hasattr(o, "tolist") else str(o))


def _f(x):
    return [round(float(v), 2) for v in x]


class Exporter:
    def __init__(self, sim, every=0.5):
        self.sim, self.every, self.next_t = sim, every, 0.0
        self.frames = []
        os.makedirs(sim.out, exist_ok=True)
        self.static = None

    def _static(self):
        s, w = self.sim, self.sim.world
        rects = lambda g: [[int(i), int(j)] for i, j in np.argwhere(g)]
        return dict(area=C.AREA, cell=C.CELL, n=C.N, base=_f(C.BASE), entry=_f(C.ENTRY),
                    pads=[_f(d.pad) for d in s.drones], poly=C.AREA_POLY, origin=list(C.ORIGIN),
                    area_m2=round(len(s.free_cells) * (C.CELL * C.MAP_SCALE) ** 2),
                    tall=rects(w.tall), weak=rects(w.weak), rubble=rects(w.rubble),
                    radiation=[[round(float(v), 2) for v in row] for row in w.radiation],
                    dead_zone=dict(c=_f(C.DEAD_ZONE_C), r=C.DEAD_ZONE_R), weak_box=list(w.weak_box),
                    n_victims=s.n_humans, n_drones=len(s.drones), n_teams=len(s.dispatch.teams),
                    map_scale=C.MAP_SCALE)

    def _state(self, final=False):
        s = self.sim
        V = []
        for v in s.command.victims.values():
            V.append(dict(id=v["id"], pos=v["pos"], label=v["label"], confidence=v["confidence"],
                          status=v["status"], priority=v["priority"], source=v.get("source"),
                          photo=(os.path.relpath(v["photo"], s.out) if v.get("photo") else None),
                          thermal=(os.path.relpath(v["thermal"], s.out) if v.get("thermal") else None),
                          kind=v.get("kind"), trapped=v.get("trapped"), t=v.get("t"),
                          temp=v.get("measured_temp"), team=v.get("rescue_team"),
                          reached_t=v.get("reached_t"), response_s=v.get("response_s"),
                          route_status=(v.get("route") or {}).get("status"),
                          route_note=(v.get("route") or {}).get("note"), model=v.get("model"),
                          detections=v.get("detections")))
        routes = {k: dict(safe=r["safe_path"], direct=r["direct_path"] if r["direct_unsafe"] else [],
                          status=r["status"], reason=r["reason"]) for k, r in s.routes.items()}
        zones = np.full((C.N, C.N), ".")
        for d in s.drones:
            if d.zone:
                zones[tuple(np.array(d.zone).T)] = chr(48 + d.id)
        return dict(
            t=round(s.t, 1), final=final, collapsed=s.world.collapsed,
            coverage=round(100 * s.coverage(), 1),
            covered="".join("1" if c else "0" for c in s.covered.flatten()),
            zones="".join(zones.flatten()),
            drones=[dict(id=d.id, pos=_f(d.pos), state=d.state, battery=round(d.battery, 1), online=d.online,
                         hops=d.hops, heading=round(d.heading, 2), blocked=d.blocked, recalled=d.recalled)
                    for d in s.drones],
            trails=[[_f(q) for q in d.path] for d in s.drones],
            cams=s.cam_meta, snapshots=s.snapshots[-10:], ack=s.link.ack,
            teams=[dict(name=t.name, pos=_f(t.pos), state=t.state,
                        victim=(t.victim or {}).get("id"), rescued=list(t.rescued)) for t in s.dispatch.teams],
            victims=V, routes=routes, sos=s.sos,
            heat=[dict(id=k, pos=z["pos"], temp=z["temp"], t=z["t"], drone=z["drone"],
                       checked=k in s.command.victims) for k, z in s.heat_zones.items()],
            links=[[_f(a), _f(b)] for a, b in s.links],
            events=[dict(t=round(t, 1), msg=m, lvl=l) for t, m, l in s.events[-60:]],
        )

    def tick(self, force=False, final=False):
        if not force and self.sim.t < self.next_t:
            return
        self.next_t = self.sim.t + self.every
        if self.static is None:
            self.static = self._static()
        st = self._state(final)
        self.frames.append({k: v for k, v in st.items() if k not in ("trails", "cams", "snapshots", "ack")})
        tmp = os.path.join(self.sim.out, "live.json.tmp")
        with open(tmp, "w") as f:
            json.dump(dict(static=self.static, state=st), f, default=_j)
        os.replace(tmp, os.path.join(self.sim.out, "live.json"))
        if final:
            # replay keeps the full event list on the last frame only, to stay small
            for fr in self.frames[:-1]:
                fr["events"] = fr["events"][-8:]
            with open(os.path.join(self.sim.out, "replay.json"), "w") as f:
                json.dump(dict(static=self.static, frames=self.frames), f, separators=(",", ":"), default=_j)
