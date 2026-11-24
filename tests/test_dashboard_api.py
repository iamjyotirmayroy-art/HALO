import json
import pytest
import serve


def test_fit_area_maps_drawn_area_onto_sim_square():
    # ~96 m x 48 m rectangle in Mumbai: the long side becomes the 12 m sim square
    lat, lon, dlat = 19.07, 72.87, 48 / 111320
    dlon = 96 / (111320 * 0.94541)
    spec = serve.fit_area([[lat, lon], [lat, lon + dlon], [lat + dlat, lon + dlon], [lat + dlat, lon]], [lat + dlat / 2, lon - dlon / 4])
    assert spec["origin"] == [lat, lon]
    assert spec["scale"] == pytest.approx(8.0, rel=0.01)
    xs, ys = zip(*spec["poly"])
    assert max(xs) == pytest.approx(12.0) and max(ys) == pytest.approx(6.0, rel=0.01) and min(xs + ys) == 0
    assert spec["base"][0] == pytest.approx(-3.0, rel=0.01)


def test_fit_area_rejects_degenerate_area():
    with pytest.raises(ValueError):
        serve.fit_area([[19.0, 72.0], [19.1, 72.1]])


def test_commands_keep_increasing_and_controls_are_clamped(tmp_path, monkeypatch):
    monkeypatch.setattr(serve, "CMD", str(tmp_path / "commands.json"))
    monkeypatch.setitem(serve.state, "seq", 0)
    serve.queue(dict(type="recall", drone=2))
    serve.control(dict(drone=1, vx=5, vy=-3, vz=0.2, yaw_rate=0))
    serve.queue(dict(type="verify", victim="V3", person=True))
    j = json.load(open(serve.CMD))
    assert [c["seq"] for c in j["queue"]] == [1, 3] and j["control"]["seq"] == 2
    assert (j["control"]["vx"], j["control"]["vy"], j["control"]["vz"]) == (1.0, -1.0, 0.2)
    monkeypatch.setitem(serve.state, "seq", 0)            # server restart: numbering continues from the file
    serve.queue(dict(type="snapshot", drone=0))
    assert json.load(open(serve.CMD))["queue"][-1]["seq"] == 4


def test_fleet_preflight_ready_fail_retry_remove(monkeypatch):
    monkeypatch.setattr(serve, "fleet", {"next": 0, "drones": []})
    clock = [1000.0]
    monkeypatch.setattr(serve.time, "time", lambda: clock[0])
    for _ in range(4):
        serve.fleet_cmd("add", {})
    assert serve.fleet_view()["drones"][0]["status"] == "CHECKING"
    clock[0] += serve.STEP_S * (len(serve.CHECKS) + 1)
    v = serve.fleet_view()
    assert (v["total"], v["ready"]) == (4, 3)                      # every 4th drone's thermal camera fails first
    bad = v["drones"][3]
    assert bad["status"] == "FAILED" and bad["checks"][-1]["name"] == "Thermal camera"
    serve.fleet_cmd("retry", {"id": 3})
    clock[0] += serve.STEP_S * (len(serve.CHECKS) + 1)
    assert serve.fleet_view()["ready"] == 4
    assert serve.fleet_cmd("remove", {"id": 0})["total"] == 3
