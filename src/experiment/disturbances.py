"""
Step 19: disturbances (D1-D7) and noise-level scaling (N1, N2).

A disturbance is a physical change of the world or a fault - NOT the sensor noise (that is the noise
profile, step 13a). The robot does not know about it: the estimator and the navigation keep the
parameters of steps 15 / 15b. Every disturbance hits one source of information; the question is which
configuration's fusion compensates it.

    ID  disturbance                                   type                        active          injected
    D1  slippery floor: turn loss x k                 nonlinear external effect   whole run       plant (offline only)
    D2  wheel radius error (tyre wear)                systematic model error      whole run       wheel angles -> encoders
    D3  effective wheel separation error              systematic model error      whole run       wheel angles -> encoders
    D4  slip bursts while driving straight            stochastic external effect  random bursts   plant (offline only)
    D5  gyro bias step (temperature, knock)           sudden systematic fault     from mid-explore gyro model
    D6  LiDAR degradation (drop-outs, false returns)  stochastic sensor fault     from mid-explore scan
    D7  side proximity sensor calibration offset      sensor fault                from mid-explore proximity reading
    N1  all sensor noise sigma x k                    stochastic measurement noise whole run      noise profile
    N2  one sensor's noise sigma x k                  stochastic measurement noise whole run      noise profile

"Mid-explore" = the first tick in FOLLOW (driving) after the robot's own map holds half of the maze's
junctions (ceil(nodes / 2) distinct junctions observed) - the robot already has a good map and estimate,
so the run shows whether the fusion notices the change and recovers.

D2 / D3 are applied to the wheel joint angles before the encoders (mean part x (1 + e2), differential part
x (1 + e3)); this is equivalent to a true wheel radius r / (1 + e2) and a true effective separation
B (1 + e3) (the wheel PIs see the same scaled speeds as with a real worn wheel), and works identically
live and offline. Sign convention: e > 0 -> the encoders overstate the distance (D2) / the rotation (D3).

Command line (run.py, experiment.batch, experiment.step19):  --dist D2=0.02   --dist D7=L   --noise-scale 2
--noise-scale-gyro 4 (N2: only the gyro; also -lidar, -prox).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

IDS = ("D1", "D2", "D3", "D4", "D5", "D6", "D7")
OFFLINE_ONLY = ("D1", "D4")
D7_OFFSET_M = 0.020                     # side sensor reads 20 mm too far
D6_FALSE_SHARE = 0.5                    # of the affected rays: half no return, half a false short return
D4_RATE_PER_M = 1.0                     # slip bursts per metre driven straight (Poisson)
D4_BURST_S = 0.2
D4_ASYM = 0.25                          # one wheel slips (1 + a), the other (1 - a): heading kick
SEED_OFFSET = 190_000                   # disturbance randomness: own stream, same for every configuration


@dataclass(frozen=True)
class DisturbanceSpec:
    id: str
    level: object                       # float, or "L" / "R" for D7

    @property
    def label(self) -> str:
        v = self.level
        if isinstance(v, str):
            return f"{self.id}{v}"
        return f"{self.id}{'m' if v < 0 else 'p'}{abs(v):g}".replace(".", "_")

    def describe(self) -> dict:
        return {"id": self.id, "level": self.level, "label": self.label, "type": TYPES[self.id]}


TYPES = {"D1": "slippery floor, turn loss x k", "D2": "wheel radius error (fraction)",
         "D3": "effective wheel separation error (fraction)", "D4": "straight slip bursts (path fraction lost)",
         "D5": "gyro bias step [deg/s]", "D6": "LiDAR degradation (fraction of rays)",
         "D7": f"side proximity sensor offset +{1000 * D7_OFFSET_M:.0f} mm (L / R)"}


def parse(text: str | None) -> DisturbanceSpec | None:
    """'D2=0.02', 'D7=L', 'D5=0.5' -> DisturbanceSpec."""
    if not text:
        return None
    k, _, v = text.partition("=")
    k = k.strip().upper()
    if k not in IDS:
        raise ValueError(f"unknown disturbance {k} (use {', '.join(IDS)})")
    if k == "D7":
        v = v.strip().upper()
        if v not in ("L", "R"):
            raise ValueError("D7 level: L or R")
        return DisturbanceSpec(k, v)
    return DisturbanceSpec(k, float(v))


def noise_label(scale: dict | None) -> str | None:
    """{'gyro': 2, 'lidar': 2, 'proximity': 2} -> 'N1x2';  {'gyro': 4} -> 'N2gyrox4'."""
    if not scale:
        return None
    vals = {k: v for k, v in scale.items() if v != 1.0}
    if not vals:
        return None
    if set(vals) == {"gyro", "lidar", "proximity"} and len(set(vals.values())) == 1:
        return f"N1x{list(vals.values())[0]:g}".replace(".", "_")
    return "N2" + "".join(f"{k[:4]}x{v:g}" for k, v in sorted(vals.items())).replace(".", "_")


class Disturbance:
    """The active disturbance of one run (None-safe helpers: every hook is a no-op without one)."""

    def __init__(self, spec: DisturbanceSpec | None, seed: int, n_nodes: int) -> None:
        self.spec = spec
        self.id = spec.id if spec else None
        self.rng = random.Random(seed + SEED_OFFSET)
        self.trigger_junctions = math.ceil(n_nodes / 2)
        self.active_from_t: float | None = None if self.id in ("D5", "D6", "D7") else 0.0
        self._q_raw = None                      # D2 / D3: unwrapped raw and disturbed joint angles
        self._q_out = None

    # ------------------------------------------------------------------ timing
    @property
    def active(self) -> bool:
        return self.active_from_t is not None

    def maybe_trigger(self, t: float, junctions_known: int, driving: bool, noise) -> str | None:
        """D5-D7 start in the middle of the exploration. Returns an event name when it starts."""
        if self.spec is None or self.active or junctions_known < self.trigger_junctions or not driving:
            return None
        self.active_from_t = t
        if self.id == "D5":
            noise.gyro.bias += math.radians(float(self.spec.level))
        return f"{self.spec.label} starts"

    # ------------------------------------------------------------------ D2 / D3: wheel angles -> encoders
    def joints(self, ql: float, qr: float) -> tuple[float, float]:
        if self.id not in ("D2", "D3"):
            return ql, qr
        if self._q_raw is None:
            self._q_raw = [ql, qr]
            self._q_out = [ql, qr]
            return ql, qr
        dl = math.remainder(ql - self._q_raw[0], 2 * math.pi)
        dr = math.remainder(qr - self._q_raw[1], 2 * math.pi)
        self._q_raw = [ql, qr]
        c, d = 0.5 * (dl + dr), 0.5 * (dr - dl)
        e = float(self.spec.level)
        if self.id == "D2":
            c *= 1.0 + e
        else:
            d *= 1.0 + e
        self._q_out[0] += c - d
        self._q_out[1] += c + d
        return math.remainder(self._q_out[0], 2 * math.pi), math.remainder(self._q_out[1], 2 * math.pi)

    # ------------------------------------------------------------------ D6: LiDAR
    def scan(self, ranges: list) -> list:
        if self.id != "D6" or not self.active:
            return ranges
        p = float(self.spec.level)
        out = []
        for r in ranges:
            if self.rng.random() < p:
                if self.rng.random() < D6_FALSE_SHARE and r is not None:
                    out.append(round(self.rng.uniform(0.05, r), 3))      # false short return (dust, edge)
                else:
                    out.append(None)                                      # drop-out
            else:
                out.append(r)
        return out

    # ------------------------------------------------------------------ D7: proximity
    def proximity(self, name: str, d):
        if self.id != "D7" or not self.active or d is None:
            return d
        side = "left" if self.spec.level == "L" else "right"
        return d + D7_OFFSET_M if name == side else d

    # ------------------------------------------------------------------ D4: plant slip bursts (offline)
    def plant_slip(self):
        """-> callable(v, w, h) -> (left factor, right factor) for the offline plant, or None."""
        if self.id != "D4":
            return None
        loss = float(self.spec.level)                       # fraction of the straight path lost
        state = {"left": 0.0, "s": 0.0, "sign": 1.0}
        rng = self.rng

        def slip(v, w, h):
            straight = abs(v) > 0.1 and abs(w) < 0.2
            if state["left"] <= 0.0 and straight and rng.random() < D4_RATE_PER_M * abs(v) * h:
                # each burst loses loss / rate metres of the path over D4_BURST_S
                state["left"] = D4_BURST_S
                state["s"] = min(0.9, loss / D4_RATE_PER_M / max(abs(v) * D4_BURST_S, 1e-6))
                state["sign"] = rng.choice((-1.0, 1.0))
            if state["left"] > 0.0:
                state["left"] -= h
                s, a = state["s"], D4_ASYM * state["sign"]
                return 1.0 - s * (1.0 + a), 1.0 - s * (1.0 - a)
            return 1.0, 1.0
        return slip

    def turn_loss(self, default: float) -> float:
        return float(self.spec.level) if self.id == "D1" else default

    def summary(self) -> dict:
        if self.spec is None:
            return {"disturbance": None}
        return {"disturbance": self.spec.describe(), "disturbance_active_from_s": self.active_from_t,
                "disturbance_trigger_junctions": self.trigger_junctions}
