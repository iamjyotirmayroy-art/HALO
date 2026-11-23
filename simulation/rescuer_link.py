"""
Rescuer -> swarm link. The dashboard server (dashboard/serve.py) writes results/commands.json;
the running mission polls it and acts on it:

    queue    one-off commands: verify, send_team, recall, takeover, release, snapshot
    watch    drones whose cameras are open on the dashboard (only those are rendered)
    control  latest stick input of a rescuer-flown drone {drone, vx, vy, vz, yaw_rate}

Drone commands travel over the mesh: a drone without a link gets them when it reconnects.
"""
import json, os


class RescuerLink:
    def __init__(self, sim):
        self.path = os.path.join(sim.out, "commands.json")
        self.watch, self.pending, self.control = set(), [], None
        self.ctrl_t = 0.0                    # sim time the last stick input arrived
        self.ack = 0                         # last command carried out (shown on the dashboard)
        j = self._read()
        self.seq = max((c["seq"] for c in j.get("queue", [])), default=0)   # ignore a previous session's commands
        self.ctrl_seq = (j.get("control") or {}).get("seq", 0)

    def _read(self):
        try:
            with open(self.path) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def poll(self, sim):
        j = self._read()
        self.watch = {int(i) for i in j.get("watch", []) if 0 <= int(i) < len(sim.drones)}
        for c in j.get("queue", []):
            if c["seq"] > self.seq:
                self.seq = c["seq"]
                self.pending.append(c)
        ctrl = j.get("control")
        if ctrl and ctrl["seq"] > self.ctrl_seq:
            self.ctrl_seq, self.control, self.ctrl_t = ctrl["seq"], ctrl, sim.t
        self.pending = [c for c in self.pending if not self._apply(sim, c)]

    def stick(self, drone_id, t):
        """Latest fresh stick input for this drone (vx forward, vy left, vz up, yaw_rate), or None."""
        c = self.control
        if c and c["drone"] == drone_id and t - self.ctrl_t < 0.3:
            return c
        return None

    def _apply(self, sim, c):
        """Carry out one command. False = keep it queued (its drone has no radio link yet)."""
        typ = c["type"]
        if typ in ("verify", "send_team"):
            v = sim.command.victims.get(c.get("victim"))
            if v:
                if typ == "verify":
                    sim.dispatch.decide(sim, v, bool(c.get("person")))
                else:
                    sim.dispatch.send(sim, v)
        elif 0 <= c.get("drone", -1) < len(sim.drones):
            d = sim.drones[c["drone"]]
            if not d.online:
                if not c.get("queued"):
                    c["queued"] = True
                    sim.event(f"Drone {d.id} has no signal - '{typ}' queued until it reconnects", "net")
                return False
            {"recall": sim.recall, "takeover": sim.takeover, "release": sim.release,
             "snapshot": sim.snapshot}[typ](d)
        self.ack = c["seq"]
        return True
