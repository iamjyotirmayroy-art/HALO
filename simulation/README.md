# HALO Simulation (PyBullet)

A full rescue mission in an earthquake-hit city block: **6 autonomous drones**, **20 hidden victims** (some only partly visible, 3 fully buried) and **3 ground rescue teams**. The scene is a 12 m × 12 m scale model for Crazyflie-size drones. At 1:8 scale it stands for a ~96 m × 96 m (about 100 m) block, and that's how the dashboard shows it on the map. The quadcopter physics and flight controller come from [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones) (Crazyflie 2.x model, MIT License). Everything else in this folder is HALO's own code: the disaster scene, swarm AI, perception, path planning, comms and dashboard.

## Run it

```bash
cd simulation
pip install -r requirements.txt      # PyBullet compiles on first install (a few minutes)

python run_mission.py --gui          # watch live in the 3D window (good for screen recording)
python run_mission.py                # headless: writes results/halo_demo.mp4 + report
python run_mission.py --no-video     # fastest, metrics only
python run_mission.py --drones 1 --no-aftershock --max-time 900 --no-video   # single-drone baseline
```

Outputs go to `results/`:

| File | What it is |
|------|-----------|
| `halo_demo.mp4` | Side-by-side video: 3D PyBullet view + live rescuer map with event feed |
| `final_map.png` | Final rescuer map: searched area, victims with priority, SOS, safe routes |
| `photos/` | Photo taken by the drone for every heat signal (sent to the rescuer to verify) |
| `mission_report.json` | Metrics, every victim with AI result, priority and route, full timeline |
| `live.json` / `replay.json` | Local data feed for the rescuer dashboard (live + full replay) |

## What happens in the mission

1. **Rescuer marks the area** → the Command AI splits it into 6 equal zones (equal-area strips, matched to drones with the Hungarian algorithm).
2. **Each drone flies a lawn-mower search** of its zone at its own altitude layer, avoiding buildings (A* path planning) and other drones (vertical deconfliction).
3. **Heat signal → photo → AI check.** On any unusual heat signal the drone dives, takes a photo and runs the victim check. **No signal is ever discarded**: whole people, partial body parts (hand, foot, head) and unclear signals all go to the rescuer with a confidence score and a "please verify" tag.
4. **Low battery → return to charge.** Drone 1 starts with a worn battery. It works out on its own when it needs to head home, the swarm re-splits its unsearched cells, and once recharged it rejoins.
5. **Aftershock → drone trapped → SOS.** A drone inspecting a victim inside a damaged building is trapped when the building collapses. It sends an SOS beacon with its last position, and the swarm takes over its zone.
6. **Signal loss → edge autonomy + mesh relay.** In the debris shadow (top right) drones have no radio. They keep searching on their own, buffer their reports and sync when they reconnect. Out of base range, drones relay through each other.
7. **Phone signals** heard by every drone are pooled to locate phones by least-squares fitting. A phone with no matching heat signal is flagged as a **buried victim**.
8. **Rescue teams**: 3 teams on foot are dispatched by priority, each walking the safe route. For "please verify" detections, a rescuer at base first looks at the drone's photo: real people get a team, non-human heat sources are dismissed.
9. **Safe routes**: for every victim, a hazard-aware A* route for rescuers (avoids radiation, unstable structures and rubble). If the direct route is unsafe, it reroutes and shows both.

## Rescuer dashboard

```bash
python run_mission.py --gui          # terminal 1: run the mission
python ../dashboard/serve.py         # terminal 2: opens http://localhost:8000/dashboard/ (live)
python ../dashboard/serve.py --replay   # replay the last mission any time
```

Everything is stored locally in `results/`. No cloud or database is needed for the demo.

## Code map

| File | Role |
|------|------|
| `run_mission.py` | Mission loop: physics, control, comms, events, video, report |
| `agents.py` | `DroneAgent` (per-drone autonomous AI) and `CommandAI` (central coordinator) |
| `planning.py` | Zone splitting, coverage paths, A* flight paths, rescuer safe routes |
| `perception.py` | Thermal scan, camera photo, AI victim check, phone RSSI locating, priority score |
| `rescue.py` | Ground rescue teams + dispatcher (priority queue, photo verification) |
| `dashboard_export.py` | Writes live state + replay for the dashboard (local JSON files) |
| `world.py` | Disaster scene in PyBullet + grid maps (buildings, rubble, radiation, unstable structure) |
| `render.py` | 3D frames + live 2D rescuer map |
| `config.py` | All mission parameters |

## Honest notes on what's simulated

- **Photos are real rendered camera images.** The person/body-part classifier is a calibrated stand-in that returns realistic confidence scores. On hardware it becomes a detector such as YOLOv8, trained on whole people and partial body parts in rubble. The never-discard decision policy stays the same.
- **Phone signals** use a log-distance path-loss model. Each victim's rubble attenuates differently, which the AI doesn't know, plus 3 dB noise.
- **Battery** uses a drain model (hover plus speed). Charging is sped up 6% per second so the demo stays short.
- **The aftershock** is a scripted event that triggers while a drone is inside the damaged building, so the SOS behaviour always shows in the demo.
- **Scale**: Crazyflie-size drones in a 12 m block. The same logic scales to a real city block with bigger drones.
