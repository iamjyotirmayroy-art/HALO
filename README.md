# 🛰️ HALO
### AI-Powered Autonomous Rescue Drone Swarm
**H**azard-**A**ware **L**ifesaving **O**perations

> One swarm for earthquakes, floods and collapsed buildings. HALO finds trapped people, even when only a hand is visible under debris. It maps safe routes for rescuers and keeps searching when a drone goes down, runs out of battery or loses signal.

**Track:** Disaster Management (Robotics + AI) · Physical AI Hackathon: From Simulation to Reality

🎥 **Demo video:** _link coming soon_  ·  📄 **Concept note:** _coming soon_

---

## 🚨 The problem

In the first **72 hours** after a disaster, every minute counts for people trapped under rubble or cut off by floodwater. Rescue teams today:

- search large areas **manually**, zone by zone, which is slow and puts rescuers at risk
- enter structures **without knowing** if they are about to collapse or if there is a radiation hazard
- lose **communication** when mobile networks go down
- miss victims who are **mostly buried**, with only a hand or foot visible
- have no single live map of **where victims are and how to reach them safely**

## 💡 Our solution

HALO is a **swarm of autonomous AI drones**. A rescuer marks an area on the map. The swarm splits it into zones, searches every zone in parallel, finds victims, checks hazards and plots the **safest route** for the rescue team. All of it shows up on a live dashboard.

```
Rescuer marks area ──► Swarm splits into zones ──► Each drone searches its zone
        ▲                                                   │
        │                                                   ▼
  Live dashboard  ◄── Safe route planned ◄── Heat signal → photo → AI check → rescuer verifies
```

---

## ✨ Key features

| # | Feature | What it does |
|---|---------|--------------|
| 1 | 🧩 **Smart zone splitting** | Rescuer marks an area, and the swarm divides it evenly so each drone owns one zone. |
| 2 | 🔁 **Self-healing swarm** | If a drone fails, runs low on battery or gets trapped, the remaining drones **re-split its zone automatically**. Nothing is left unsearched. |
| 3 | 🔋 **Auto return-to-charge & handover** | Low battery → drone flies back to the charging base, recharges and rejoins, then the swarm rebalances. |
| 4 | 🌡️ **Thermal victim detection** | Drones scan for body-heat signatures. |
| 5 | 📸 **AI photo verification (human in the loop)** | On any unusual heat signal, the drone **takes a photo** and AI checks for a **whole person or a partial body part** (hand, arm, foot, face) sticking out of debris. **No heat signal is ever thrown away**: uncertain ones are sent as *"possible victim, please verify"*. A missed victim is treated as far worse than a false alarm. |
| 6 | 📶 **Phone-signal victim locating** | Detects Wi-Fi / Bluetooth signals from trapped people's phones to estimate their position, even with **no line of sight**. |
| 7 | 🚦 **Victim priority scoring** | Ranks victims by urgency (body heat, AI confidence, nearby hazards) so rescuers reach the most critical first. |
| 8 | ☢️ **Hazard sensing** | Radiation and structural-integrity scanning flag dangerous zones **before** rescuers enter. |
| 9 | 🗺️ **Safe-route planning** | Hazard-weighted path planning for rescuers. If the direct route is unsafe, HALO **reroutes** and still marks the victim's location. |
| 10 | 🆘 **Drone distress beacon** | If a drone is trapped (e.g. inside a building that collapses), it broadcasts an **SOS with its last known position** so the team can recover it, and the swarm takes over its zone. |
| 11 | 🧠 **Edge autonomy (multi-agent AI)** | A central command AI talks to rescuers, and **each drone runs its own AI agent**. If debris cuts the signal, drones keep working on their own and sync when they reconnect. |
| 12 | 🚧 **Obstacle & collision avoidance** | Drones avoid buildings, debris and each other. |
| 13 | 📦 **Relief drops** | Light payloads (medicine, water pouches, a radio) delivered to victims waiting for rescue. Especially useful in **floods**. |
| 14 | 🖥️ **Live rescuer dashboard** | Map with thermal red zones, live AI verdicts, victims with thermal + photo, hazards, drones, rescue teams and safe routes. |
| 15 | 🚑 **Rescue team dispatch** | Command AI sends ground teams to victims by priority along safe routes. Uncertain photos are verified by a human first. |

---

## 🏗️ Architecture

```
                    ┌─────────────────────────────┐
   Rescuer  ◄────►  │   HALO Command AI (central) │  ◄────►  Live Dashboard
                    │ zones · alerts · re-tasking │
                    └──────────────┬──────────────┘
                     mesh / radio  │  (may drop out)
          ┌────────────────┬───────┴────────┬────────────────┐
     ┌────▼────┐      ┌────▼────┐      ┌────▼────┐      ┌────▼────┐
     │ Drone 1 │      │ Drone 2 │      │ Drone 3 │      │ Drone N │
     │ AI agent│      │ AI agent│      │ AI agent│      │ AI agent│
     └─────────┘      └─────────┘      └─────────┘      └─────────┘
   Perceive: thermal · camera · phone signal · radiation · structure · obstacles
   Decide:   search path · victim check & scoring · battery · avoid collisions
   Act:      fly · photograph · return to charge · SOS beacon · drop relief · report
```

---

## 🧪 Simulation

Built on **PyBullet** with **gym-pybullet-drones** quadcopter physics. Details and run instructions are in [`simulation/`](simulation/).

**Demo scenario:** earthquake hits a city block → rescuer marks the area → 6 drones split into zones → heat signals photographed and AI-checked → one drone returns to recharge and the swarm self-heals → a drone gets trapped and sends SOS → signal drops and drones keep working on their own → 3 rescue teams follow safe routes to all 20 victims, most urgent first.

## 📊 Results

From the PyBullet simulation: **6 drones**, **20 hidden victims** (7 only partly visible, 3 fully buried), **3 heat decoys** and **3 ground rescue teams** in an earthquake-hit block. It's a 12 m scale model of a ~100 m block. Reproduce it with `python simulation/run_mission.py`.

| Metric | Result |
|--------|--------|
| Victims found | **20 / 20** |
| Partial-body victims found (hand, foot, head) | **7 / 7** |
| Buried victims found only by phone signal | **3 / 3** |
| Heat signals discarded by AI | **0**. All 3 decoys went to a rescuer, who dismissed them from the photo |
| Victims reached by rescue teams | **20 / 20** |
| Avg time from detection to a rescue team reaching the victim | **72 s** |
| Area searched | **100%** in 104 s, despite 1 drone trapped and 1 recharging |
| Unsafe direct routes caught and rerouted | **3** (radiation zone / collapsed building) |
| Phone location error | **0.0 – 0.64 m** |

**Swarm vs single drone** (same scene, no aftershock):

| | 6-drone swarm | 1 drone |
|---|---|---|
| Time to search the whole area | **98 s** | 298 s |
| Time to find all visible victims | **64 s** | 203 s |

→ The swarm finds every victim about **3.2× faster**.

![Rescuer map](simulation/results/final_map.png)

🎬 Simulation recording: [`simulation/results/halo_demo.mp4`](simulation/results/halo_demo.mp4)

## 🖥️ Rescuer dashboard

A live web dashboard for the rescue team. Everything is stored locally, with no cloud needed:

- **Map** (street / satellite / schematic) with the search area, buildings, radiation, unstable structures and the no-signal zone
- **🔥 Thermal red zones** wherever a drone senses heat, marked *"AI checking…"* until the photo is analysed, then the AI verdict (person / body part / not a person)
- **Victim list sorted by priority** with the drone's **thermal image + photo**, AI confidence, and which team is on the way
- **Alerts**: drone SOS, heat signals being checked, low-confidence photos for a human to verify
- **Drone cards** (battery, state, online / mesh relay / offline-autonomous) and **rescue team** status
- **Safe routes** (green) vs unsafe direct routes (red dashed), with a full event feed and **mission replay**

```bash
python simulation/run_mission.py --gui     # terminal 1
python dashboard/serve.py                  # terminal 2 → http://localhost:8000/dashboard/
python dashboard/serve.py --replay         # replay the last mission
```

## 🔧 From simulation to reality

| Simulated | Real hardware |
|-----------|---------------|
| Quadcopter model (Crazyflie in gym-pybullet-drones) | Quadcopter with flight controller (PX4 / ArduPilot) |
| Simulated thermal camera | Thermal camera module (e.g. FLIR Lepton) |
| Simulated RGB photo + AI check | Onboard camera + person / body-part detection model |
| Simulated phone-signal sensing | Wi-Fi / BLE scanner module (e.g. ESP32) |
| Simulated radiation field | Geiger counter module |
| Drone-to-drone comms | ESP-NOW / LoRa mesh radio |
| Onboard AI agent | Raspberry Pi / Jetson Nano companion computer |
| Charging base | Landing pad with charging contacts |

---

## 🔭 Future scope

CO₂ breath detection under rubble · acoustic sensing (tapping / voices) · aftershock & collapse prediction · two-way voice beacon for trapped victims · cloud dashboard for multiple rescue teams.

## 🙏 Credits

- [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones) by the Learning Systems & Robotics Lab (University of Toronto), used as the drone physics base, MIT License.
- [PyBullet](https://pybullet.org) physics engine.

## 👥 Team

_Team members coming soon._
