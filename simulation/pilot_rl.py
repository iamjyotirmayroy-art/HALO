"""
HALO learned pilot: a small neural-network policy trained with reinforcement learning (PPO) to fly a
drone to its next waypoint while dodging buildings, debris and other drones, using only what a real
drone has: 16 rangefinder rays, the waypoint direction and its own velocity.

    python pilot_rl.py                 # train (~10 min CPU) -> models/pilot_policy.npz
    python pilot_rl.py --eval          # success / collision rate on 1000 unseen random courses

Training runs in a fast vectorised 2-D world of random boxes and moving drones. The trained weights are
exported to plain numpy, so the simulation (and a real drone's companion computer) needs no PyTorch.
In the mission the rangefinder rays come from PyBullet, and the anti-crash filter stays on as a safety
shield around the learned policy.
"""
import argparse, os
import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(ROOT, "models", "pilot_policy.npz")
N_RAYS, RANGE, START = 16, 2.0, 0.08                      # same rangefinders as safety.py
ANG = np.linspace(0, 2 * np.pi, N_RAYS, endpoint=False)
DIRS = np.c_[np.cos(ANG), np.sin(ANG)]
V_MAX, ACCEL, DT = 0.9, 2.0, 0.1
R_DRONE, R_OTHER, ARENA, K_BOX, M_DRONE, T_MAX = 0.12, 0.10, 8.0, 8, 3, 160
OBS = 2 * N_RAYS + 5


def observe(rays, prev_rays, goal_vec, vel):
    """Policy input (same in training and mission): normalised rays now and how much each ray shrank since the
    last step (tells moving drones apart from walls), waypoint direction + distance, own velocity."""
    d = np.linalg.norm(goal_vec, axis=-1, keepdims=True)
    return np.concatenate([rays / RANGE, np.clip((prev_rays - rays) / (V_MAX * 2 * DT), -1, 1),
                           goal_vec / np.maximum(d, 1e-6), np.minimum(d, 4) / 4, vel / V_MAX], -1)


class Courses:
    """Vectorised training world: E random courses of boxes + moving drones, start, goal."""
    def __init__(self, E, rng):
        self.E, self.rng = E, rng
        self.reset(np.ones(E, bool))

    def _free(self, idx, p):
        lo, hi = self.box[idx, :, :2], self.box[idx, :, 2:]
        q = np.clip(p[:, None], lo, hi)
        return np.linalg.norm(q - p[:, None], axis=-1).min(1) > 0.45

    def reset(self, m):
        idx = np.nonzero(m)[0]
        if not len(idx):
            return
        n, r = len(idx), self.rng
        if not hasattr(self, "box"):
            E = self.E
            self.box, self.pos, self.vel = np.zeros((E, K_BOX, 4)), np.zeros((E, 2)), np.zeros((E, 2))
            self.goal, self.other, self.ov, self.t = np.zeros((E, 2)), np.zeros((E, M_DRONE, 2)), np.zeros((E, M_DRONE, 2)), np.zeros(E, int)
        c, s = r.uniform(0, ARENA, (n, K_BOX, 2)), r.uniform(0.15, 1.0, (n, K_BOX, 2))
        s *= (r.random((n, K_BOX, 1)) < 0.85)                  # some courses have fewer boxes
        self.box[idx] = np.concatenate([c - s, c + s], -1)
        for k in range(30):                                    # sample free start / goal 1-4 m apart
            p = r.uniform(0.5, ARENA - 0.5, (n, 2))
            g = p + (lambda a, d: np.c_[np.cos(a), np.sin(a)] * d[:, None])(r.uniform(0, 2 * np.pi, n), r.uniform(1, 4, n))
            ok = self._free(idx, p) & self._free(idx, g) & (g > 0.3).all(1) & (g < ARENA - 0.3).all(1)
            if k == 0:
                self.pos[idx], self.goal[idx] = p, g
            self.pos[idx[ok]], self.goal[idx[ok]] = p[ok], g[ok]
        self.vel[idx] = 0
        mid = (self.pos[idx] + self.goal[idx]) / 2             # other drones cross the route
        oth = mid[:, None] + r.uniform(-2, 2, (n, M_DRONE, 2))
        near = np.linalg.norm(oth - self.pos[idx][:, None], axis=-1) < 0.9           # never spawn on top of us
        oth[near] += 3.0
        self.other[idx] = oth
        self.ov[idx] = r.uniform(-0.4, 0.4, (n, M_DRONE, 2)) * (r.random((n, M_DRONE, 1)) < 0.7)
        self.t[idx] = 0

    def rays(self):
        o = self.pos[:, None, None, :] + DIRS[None, :, None, :] * START          # (E,R,1,2)
        d = DIRS[None, :, None, :]
        lo, hi = self.box[:, None, :, :2], self.box[:, None, :, 2:]
        with np.errstate(divide="ignore", invalid="ignore"):
            t1, t2 = (lo - o) / d, (hi - o) / d
        tn, tf = np.nanmax(np.minimum(t1, t2), -1), np.nanmin(np.maximum(t1, t2), -1)
        hit = (tf >= np.maximum(tn, 0)) & ((hi - lo).sum(-1) > 0)
        tb = np.where(hit, np.maximum(tn, 0), np.inf).min(-1)                    # (E,R)
        oc = self.other[:, None] - (self.pos[:, None, None] + DIRS[None, :, None] * START)   # (E,R,M,2)
        b = (oc * DIRS[None, :, None]).sum(-1)
        disc = b ** 2 - ((oc ** 2).sum(-1) - R_OTHER ** 2)
        tc = np.where((disc >= 0) & (b > 0), b - np.sqrt(np.maximum(disc, 0)), np.inf).min(-1)
        return np.minimum(START + np.minimum(tb, tc), RANGE)

    def obs(self):
        r = self.rays()
        prev = getattr(self, "prev", r)
        prev[self.t == 0] = r[self.t == 0]
        self.prev = r
        return observe(r, prev, self.goal - self.pos, self.vel)

    def step(self, a):
        """a: (E,2) policy output -> desired velocity. Returns obs, reward, done, info."""
        d0 = np.linalg.norm(self.goal - self.pos, axis=1)
        dv = to_velocity(a) - self.vel
        n = np.linalg.norm(dv, axis=1, keepdims=True)
        self.vel += dv * np.minimum(1, ACCEL * DT / np.maximum(n, 1e-9))
        sp = np.linalg.norm(self.vel, axis=1, keepdims=True)
        self.vel *= np.minimum(1, V_MAX / np.maximum(sp, 1e-9))
        self.pos += self.vel * DT
        self.other += self.ov * DT
        self.t += 1
        d1 = np.linalg.norm(self.goal - self.pos, axis=1)
        q = np.clip(self.pos[:, None], self.box[..., :2], self.box[..., 2:])
        gap_box = np.where((self.box[..., 2:] - self.box[..., :2]).sum(-1) > 0, np.linalg.norm(q - self.pos[:, None], axis=-1), 9).min(1)
        gap_drone = np.linalg.norm(self.other - self.pos[:, None], axis=-1).min(1) - R_OTHER
        gap = np.minimum(gap_box, gap_drone)
        out = (self.pos < -1).any(1) | (self.pos > ARENA + 1).any(1)
        crash, goal, timeout = gap < R_DRONE, d1 < 0.2, (self.t >= T_MAX) | out
        r = 2.0 * (d0 - d1) - 0.01 - 3.0 * np.maximum(0, 0.45 - gap) + 10 * goal - 25 * crash
        done = crash | goal | timeout
        info = dict(crash=crash, goal=goal, timeout=timeout & ~crash & ~goal)
        self.reset(done)
        return self.obs(), r, done, info


def to_velocity(a):
    """Policy output -> desired velocity, keeping its direction (scaled to at most V_MAX, not clipped per axis)."""
    n = np.linalg.norm(a, axis=-1, keepdims=True)
    return a * V_MAX / np.maximum(n, 1)


# ---------------- policy (numpy inference) ----------------
class LearnedPilot:
    """Runs the exported policy MLP in numpy: obs -> desired velocity (m/s, x/y)."""
    def __init__(self, path=MODEL):
        w = np.load(path)
        self.W = [(w[f"w{i}"], w[f"b{i}"]) for i in range(3)]

    def act(self, obs):
        h = obs
        for i, (W, b) in enumerate(self.W):
            h = h @ W + b
            if i < 2:
                h = np.tanh(h)
        return to_velocity(h)


def available():
    return os.path.exists(MODEL)


# ---------------- PPO training ----------------
def train(updates, E=64, T=128, seed=0):
    import torch, torch.nn as nn
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    pi = nn.Sequential(nn.Linear(OBS, 64), nn.Tanh(), nn.Linear(64, 64), nn.Tanh(), nn.Linear(64, 2))
    vf = nn.Sequential(nn.Linear(OBS, 64), nn.Tanh(), nn.Linear(64, 64), nn.Tanh(), nn.Linear(64, 1))
    logstd = nn.Parameter(torch.full((2,), -0.5))
    opt = torch.optim.Adam([*pi.parameters(), *vf.parameters(), logstd], lr=3e-4)
    env = Courses(E, rng)
    o = env.obs()
    stats = []
    for u in range(updates):
        buf = {k: [] for k in ("o", "a", "lp", "r", "d", "v")}
        ep = dict(goal=0, crash=0, timeout=0)
        for _ in range(T):
            with torch.no_grad():
                ot = torch.as_tensor(o, dtype=torch.float32)
                mu, v = pi(ot), vf(ot).squeeze(-1)
                dist = torch.distributions.Normal(mu, logstd.exp())
                a = dist.sample()
            o2, r, d, info = env.step(a.numpy())
            for k, x in zip(buf, (ot, a, dist.log_prob(a).sum(-1), r, d, v)):
                buf[k].append(torch.as_tensor(x, dtype=torch.float32))
            for k in ep:
                ep[k] += int(info[k].sum())
            o = o2
        with torch.no_grad():
            last = vf(torch.as_tensor(o, dtype=torch.float32)).squeeze(-1)
        R, D, V = (torch.stack(buf[k]) for k in ("r", "d", "v"))
        adv, gae = torch.zeros_like(R), 0
        for t in reversed(range(T)):
            nv = last if t == T - 1 else V[t + 1]
            delta = R[t] + 0.99 * nv * (1 - D[t]) - V[t]
            gae = delta + 0.99 * 0.95 * (1 - D[t]) * gae
            adv[t] = gae
        ret = (adv + V).flatten()
        O, A, LP, adv = torch.stack(buf["o"]).flatten(0, 1), torch.stack(buf["a"]).flatten(0, 1), torch.stack(buf["lp"]).flatten(), adv.flatten()
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)
        for _ in range(4):
            for mb in torch.randperm(len(O)).split(1024):
                dist = torch.distributions.Normal(pi(O[mb]), logstd.exp())
                ratio = (dist.log_prob(A[mb]).sum(-1) - LP[mb]).exp()
                loss = (-torch.min(ratio * adv[mb], ratio.clamp(0.8, 1.2) * adv[mb]).mean()
                        + 0.5 * (vf(O[mb]).squeeze(-1) - ret[mb]).pow(2).mean() - 0.001 * dist.entropy().sum(-1).mean())
                opt.zero_grad(); loss.backward(); nn.utils.clip_grad_norm_([*pi.parameters(), *vf.parameters()], 0.5); opt.step()
        n = max(1, sum(ep.values()))
        stats.append((ep["goal"] / n, ep["crash"] / n))
        if u % 10 == 0 or u == updates - 1:
            g, c = np.mean(stats[-10:], 0)
            print(f"update {u:4d}  episodes reach goal {g:.0%}  crash {c:.1%}", flush=True)
    os.makedirs(os.path.dirname(MODEL), exist_ok=True)
    lin = [m for m in pi if isinstance(m, nn.Linear)]
    np.savez(MODEL, **{f"w{i}": l.weight.detach().numpy().T for i, l in enumerate(lin)},
             **{f"b{i}": l.bias.detach().numpy() for i, l in enumerate(lin)})


def evaluate(n=1000, seed=123):
    """Deterministic policy on unseen random courses."""
    pilot, env = LearnedPilot(), Courses(n, np.random.default_rng(seed))
    done_any, res = np.zeros(n, bool), {"goal": 0, "crash": 0, "timeout": 0}
    o = env.obs()
    for _ in range(T_MAX):
        o, _, d, info = env.step(pilot.act(o) / V_MAX)
        for k in res:
            res[k] += int((info[k] & ~done_any).sum())
        done_any |= d
        if done_any.all():
            break
    return {k: v / n for k, v in res.items()}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--updates", type=int, default=400)
    ap.add_argument("--eval", action="store_true")
    a = ap.parse_args()
    if not a.eval:
        train(a.updates)
    print("unseen courses:", {k: f"{v:.1%}" for k, v in evaluate().items()})
