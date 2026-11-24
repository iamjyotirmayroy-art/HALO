"""
HALO mission configuration.

The scene is a 12 m x 12 m disaster block sized for Crazyflie-class drones
(the quadcopter model shipped with gym-pybullet-drones). Every number here is
a tunable parameter for the demo scenario.
"""
import numpy as np

# ---------------- world grid ----------------
AREA = 12.0            # search area is [0, AREA] x [0, AREA] metres
CELL = 0.5             # grid resolution (m)
N = int(AREA / CELL)   # cells per side

# ---------------- map placement (dashboard) ----------------
ORIGIN = (22.5726, 88.3639)            # lat, lon of the search square's south-west corner
MAP_SCALE = 8.0                        # real metres per sim metre (12 m model ~ 96 m block)
AREA_POLY = None                       # rescuer-drawn area in sim metres; None = whole square

# ---------------- base / charging pads ----------------
PAD_SPACING = 0.9


def set_base(xy):
    """Place the base 1.5 m outside the edge of the search square nearest to xy, facing it."""
    global BASE, ENTRY, NORMAL, TANGENT
    d = np.asarray(xy, float) - AREA / 2
    k = int(np.argmax(np.abs(d)))
    NORMAL = np.zeros(2)
    NORMAL[k] = 1.0 if d[k] > 0 else -1.0                  # outward normal of that edge
    TANGENT = np.abs(NORMAL[::-1])
    edge = np.zeros(2)
    edge[k] = AREA if d[k] > 0 else 0.0
    edge[1 - k] = np.clip(xy[1 - k], 2.5, AREA - 2.5)
    BASE = edge + 1.5 * NORMAL                             # rescuer base + charging pads
    ENTRY = edge - 0.25 * NORMAL                           # where rescuers walk into the area


def pad(i, n):
    """Charging pad of drone i (of n), in a row along the base edge."""
    return BASE + TANGENT * (i - (n - 1) / 2) * PAD_SPACING


def stage(pad_xy):
    """Point just inside the area, in line with a pad: drones enter and leave the base through it."""
    return pad_xy - NORMAL * 1.9


set_base((-1.5, 6.0))

# ---------------- swarm ----------------
NUM_DRONES = 6
SEARCH_ALT = 1.0                       # metres; each drone gets its own layer (+ALT_LAYER*i)
ALT_LAYER = 0.12
INSPECT_ALT = 0.55                     # dives lower for a close thermal look at a heat signal
PHOTO_ALT = 1.15                       # AI photo is taken on the way down, below this height
SPEED = 0.9                            # m/s cruise
LANE = 1.5                             # lawn-mower lane spacing (m)
SEPARATION = 0.6                       # min horizontal gap before vertical deconfliction

# ---------------- sensors ----------------
THERMAL_RADIUS = 1.0                   # ground footprint of the thermal camera at search altitude
PHONE_RANGE_DBM = -85.0                # weakest phone signal the scanner can hear
RSSI_P0 = -40.0                        # signal strength at 1 m (dBm)
RSSI_N = 2.6                           # path-loss exponent through rubble
RSSI_NOISE = 3.0                       # dB
RSSI_N_TRUE = (2.3, 3.1)               # real attenuation varies per victim (unknown to the AI)
OFFLINE_GRACE = 1.5                    # s without a link before a drone counts as offline

# ---------------- battery ----------------
BATTERY_DRAIN_HOVER = 0.30             # % per second
BATTERY_DRAIN_MOVE = 0.12              # extra % per second per m/s
BATTERY_RESERVE = 12.0                 # % kept in reserve on top of the trip home
CHARGE_RATE = 6.0                      # % per second (sim-accelerated fast charge)
START_BATTERY = [100, 27, 100, 100, 100, 100]   # drone 1 starts with a worn battery

# ---------------- comms ----------------
BASE_RANGE = 9.0                       # base station radio range (m)
MESH_RANGE = 6.0                       # drone-to-drone relay range (m)
DEAD_ZONE_C = np.array([10.6, 10.8])   # debris shadow: no radio at all inside
DEAD_ZONE_R = 2.0

# ---------------- rescue teams ----------------
NUM_RESCUERS = 3
RESCUER_SPEED = 0.6                    # m/s on foot through debris (scene scale)
RESCUER_ON_SITE = 3.0                  # s spent stabilising a victim before heading back
VERIFY_DELAY = 3.0                     # s for a rescuer at base to look at a "please verify" photo

# ---------------- rescuer routing ----------------
RAD_DOSE_LIMIT = 2.0                   # max acceptable route dose (arbitrary units)

# ---------------- scripted events ----------------
AFTERSHOCK_DELAY = 1.2                 # s after a drone goes inside the damaged building

# ---------------- simulation ----------------
PYB_FREQ = 240
CTRL_FREQ = 48
MAX_TIME = 420.0                       # hard stop (s)
CAM_FPS = 4                            # live camera frames per second (watched drones only)
FRAME_DT = 0.2                         # video frame every 0.2 s of sim time
SEED = 7

DRONE_COLORS = [(66, 135, 245), (245, 166, 35), (155, 89, 182), (46, 204, 113), (231, 76, 60), (26, 188, 196)]  # RGB
