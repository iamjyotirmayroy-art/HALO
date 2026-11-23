"""
HALO dashboard server: local only, Python standard library only.

    python dashboard/serve.py                 # http://localhost:8000/dashboard/  (+ phones on the same Wi-Fi)
    python dashboard/serve.py --replay        # open straight into the last mission's replay
    python dashboard/serve.py --cert c.pem --key k.pem   # HTTPS, so phones can install the app (PWA)

API (everything is stored in simulation/results/):
    POST /api/mission  {area: [[lat, lon], ...], base: [lat, lon] | null, drones, teams}  start a mission there
    POST /api/verify   {victim, person}                     rescuer's verdict on a victim photo
    POST /api/command  {type, drone | victim}               recall | takeover | release | snapshot | send_team
    POST /api/control  {drone, vx, vy, vz, yaw_rate}        manual flight stick, sent at 10 Hz
    POST /api/watch    {drones: [...]}                      cameras open on the dashboard (only those render)
    GET  /api/cam/<N>[?mode=thermal]                        live MJPEG stream of drone N's camera
Commands go to results/commands.json, which the running simulation polls.
"""
import argparse, http.server, json, math, os, socket, ssl, subprocess, sys, threading, time, webbrowser

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM = os.path.join(ROOT, "simulation")
RES = os.path.join(SIM, "results")
CMD = os.path.join(RES, "commands.json")
SIM_AREA = 12.0                     # side of the simulated search square (metres, see simulation/config.py)
COMMANDS = {"recall", "takeover", "release", "snapshot", "send_team"}

lock = threading.Lock()
state = {"seq": 0, "proc": None}


def update_commands(fn):
    """Read-modify-write commands.json atomically (the simulation reads it concurrently)."""
    with lock:
        try:
            with open(CMD) as f:
                j = json.load(f)
        except (OSError, ValueError):
            j = {}
        fn(j)
        j["queue"] = j.get("queue", [])[-100:]
        with open(CMD + ".tmp", "w") as f:
            json.dump(j, f)
        os.replace(CMD + ".tmp", CMD)


def next_seq(j):
    """Command numbers only ever grow, even across server restarts (the simulation skips numbers it has seen)."""
    state["seq"] = max([state["seq"], (j.get("control") or {}).get("seq", 0)] + [c["seq"] for c in j.get("queue", [])]) + 1
    return state["seq"]


def queue(cmd):
    update_commands(lambda j: j.setdefault("queue", []).append(dict(cmd, seq=next_seq(j))))


def control(c):
    def put(j):
        j["control"] = dict(seq=next_seq(j), drone=int(c["drone"]),
                            **{k: max(-1.0, min(1.0, float(c.get(k, 0)))) for k in ("vx", "vy", "vz", "yaw_rate")})
    update_commands(put)


def fit_area(area, base=None):
    """Fit the simulated search square onto a drawn lat/lon area: origin, scale and the area + base in sim metres."""
    pts = [(float(a), float(b)) for a, b in area]
    if len(pts) < 3:
        raise ValueError("draw an area with at least 3 corners")
    lat0, lon0 = min(p[0] for p in pts), min(p[1] for p in pts)
    kx = 111320 * math.cos(math.radians(lat0))
    to_m = lambda lat, lon: ((lon - lon0) * kx, (lat - lat0) * 111320)
    poly = [to_m(*p) for p in pts]
    scale = max(max(x for x, _ in poly), max(y for _, y in poly), 1.0) / SIM_AREA   # real metres per sim metre
    return dict(origin=[lat0, lon0], scale=scale, poly=[[x / scale, y / scale] for x, y in poly],
                base=[c / scale for c in to_m(*base)] if base else None)


def start_mission(req):
    """Start run_mission.py on the drawn area (stops any mission still running)."""
    spec = fit_area(req["area"], req.get("base"))
    if state["proc"] and state["proc"].poll() is None:
        state["proc"].terminate()
        state["proc"].wait()
    os.makedirs(os.path.join(RES, "cams"), exist_ok=True)
    for f in ["live.json"] + [os.path.join("cams", c) for c in os.listdir(os.path.join(RES, "cams"))]:
        if os.path.exists(os.path.join(RES, f)):
            os.remove(os.path.join(RES, f))
    with open(os.path.join(RES, "area.json"), "w") as f:
        json.dump(spec, f)
    update_commands(lambda j: j.clear())
    n_drones, n_teams = (str(max(1, min(10, int(req.get(k, d))))) for k, d in (("drones", 6), ("teams", 3)))
    state["proc"] = subprocess.Popen(
        [sys.executable, "run_mission.py", "--area", os.path.join(RES, "area.json"), "--live", "--no-video",
         "--drones", n_drones, "--teams", n_teams], cwd=SIM,
        stdout=open(os.path.join(RES, "mission.log"), "w"), stderr=subprocess.STDOUT)
    return dict(ok=True, scale=round(spec["scale"], 2), origin=spec["origin"])


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **k):
        super().__init__(*a, directory=ROOT, **k)

    def log_message(self, *args):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", ""):
            self.send_response(302)
            self.send_header("Location", "/dashboard/")
            self.end_headers()
        elif self.path.startswith("/api/cam/"):
            self.stream_cam()
        else:
            super().do_GET()

    def stream_cam(self):
        """MJPEG: push a new frame whenever the simulation rewrites the drone's camera image."""
        n, _, q = self.path[len("/api/cam/"):].partition("?")
        path = os.path.join(RES, "cams", f"drone{int(n)}{'_thermal' if 'thermal' in q else ''}.jpg")
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last, sent = 0, 0.0
        try:
            while True:
                m = os.path.getmtime(path) if os.path.exists(path) else 0
                if m and (m != last or time.time() - sent > 2):   # re-send every 2 s to notice a closed tab
                    last, sent = m, time.time()
                    with open(path, "rb") as f:
                        jpg = f.read()
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def do_POST(self):
        try:
            req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
            route = self.path.rstrip("/")
            if route == "/api/mission":
                return self._json(start_mission(req))
            if route == "/api/verify":
                queue(dict(type="verify", victim=str(req["victim"]), person=bool(req["person"])))
            elif route == "/api/command" and req.get("type") in COMMANDS:
                queue({k: req[k] for k in ("type", "drone", "victim") if k in req})
            elif route == "/api/control":
                control(req)
            elif route == "/api/watch":
                update_commands(lambda j: j.__setitem__("watch", [int(i) for i in req.get("drones", [])]))
            else:
                return self._json(dict(error="unknown request"), 404)
            self._json(dict(ok=True))
        except (KeyError, TypeError, ValueError) as e:
            self._json(dict(error=str(e)), 400)


def lan_ip():
    """This laptop's address on the local network (no packet is sent)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect(("10.255.255.255", 1))
            return s.getsockname()[0]
        except OSError:
            return "127.0.0.1"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--replay", action="store_true")
    ap.add_argument("--cert", help="TLS certificate (PEM) to serve HTTPS")
    ap.add_argument("--key", help="TLS private key (PEM)")
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    os.makedirs(RES, exist_ok=True)
    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", a.port), Handler)
    scheme = "http"
    if a.cert:
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(a.cert, a.key)
        httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)
        scheme = "https"
    page = "/dashboard/" + ("?mode=replay" if a.replay else "")
    print(f"HALO dashboard running (Ctrl+C to stop)\n  this laptop:  {scheme}://localhost:{a.port}{page}\n"
          f"  phones on the same Wi-Fi:  {scheme}://{lan_ip()}:{a.port}{page}")
    if not a.no_browser:
        threading.Thread(target=webbrowser.open, args=(f"{scheme}://localhost:{a.port}{page}",), daemon=True).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if state["proc"] and state["proc"].poll() is None:
            state["proc"].terminate()
