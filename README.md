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
| 14 | 🖥️ **Live rescuer dashboard** | Map with drone positions, zones, victims (with photos), hazards and safe routes. |

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

**Demo scenario:** earthquake hits a city block → rescuer marks the area → drones split into zones → heat signals photographed and AI-checked → one drone returns to recharge and the swarm self-heals → a drone gets trapped and sends SOS → signal drops and drones keep working on their own → safe routes appear on the dashboard.

## 📊 Results

_Filled in from simulation runs._

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
