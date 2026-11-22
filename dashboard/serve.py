"""
Serve the HALO dashboard locally (everything stays on this computer).

    python dashboard/serve.py            # then open http://localhost:8000/dashboard/
    python dashboard/serve.py --replay   # opens straight into the mission replay

Run the simulation in another terminal (python simulation/run_mission.py --gui)
and the dashboard updates live from simulation/results/live.json.
"""
import argparse, functools, http.server, os, socketserver, webbrowser

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ap = argparse.ArgumentParser()
ap.add_argument("--port", type=int, default=8000)
ap.add_argument("--replay", action="store_true")
a = ap.parse_args()

Handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=ROOT)
Handler.log_message = lambda *args: None
url = f"http://localhost:{a.port}/dashboard/" + ("?mode=replay" if a.replay else "")
with socketserver.TCPServer(("", a.port), Handler) as httpd:
    print(f"HALO dashboard running at {url}  (Ctrl+C to stop)")
    try:
        webbrowser.open(url)
    except Exception:
        pass
    httpd.serve_forever()
