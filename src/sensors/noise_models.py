"""
Sensor noise models (step 13a): the "ideal" and the "realistic" profile.

The simulator gives IDEAL measurements (gyro = true body rate, LiDAR = rendered
ranges whose systematic error is calibrated out in step 12d, proximity = exact
ray distance). A "realistic" run adds, in Python, the errors of the real parts,
from their datasheets. All randomness comes from one seed per run, so a run is
reproducible and Monte Carlo varies only the seed.

                    ideal                      realistic
  encoders          1024 CPR quantization      same (quantization is the encoder's own error)
  gyro z            true body yaw rate         InvenSense MPU-6050, +-250 deg/s:
                                               white noise 0.05 deg/s rms (datasheet, DLPF 100 Hz),
                                               LSB 1/131 deg/s (datasheet), initial zero-rate
                                               offset uniform +-20 deg/s (datasheet ZRO tolerance,
                                               removed by the start-up calibration), bias random walk
                                               and residual scale-factor error after calibration
                                               (ASSUMPTIONS, marked below)
  LiDAR             calibrated sim ranges      Hokuyo URG-04LX-UG01: accuracy +-30 mm (60-1000 mm),
                                               +-3 % (1000-4095 mm) taken as 2 sigma, 1 mm resolution
  proximity         exact distance             ASSUMPTION until the real part is fixed:
                                               sigma = 2 mm + 1 % of the distance, 1 mm resolution
Values marked ASSUMPTION are not in a datasheet and are reported as such in the thesis.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field, replace

PROFILES = ("ideal", "realistic")


def _quantize(x: float, step: float) -> float:
    return round(x / step) * step if step > 0 else x


# --------------------------------------------------------------------------- gyro
@dataclass(frozen=True)
class GyroSpec:
    part: str = "InvenSense MPU-6050 (FS_SEL=0, +-250 deg/s)"
    range_rad_s: float = math.radians(250.0)                 # datasheet full scale
    lsb_rad_s: float = math.radians(1.0 / 131.0)             # datasheet sensitivity 131 LSB/(deg/s)
    noise_rms_rad_s: float = math.radians(0.05)              # datasheet total RMS noise (DLPF 100 Hz)
    initial_bias_max_rad_s: float = math.radians(20.0)       # datasheet initial ZRO tolerance
    bias_rw_rad_s_sqrt_s: float = math.radians(0.002)        # ASSUMPTION: bias random walk
    scale_residual_sd: float = 0.003                         # ASSUMPTION: after calibration (datasheet +-3 % raw)


class GyroModel:
    """z-axis rate gyro. measure(true_rate, dt) -> measured rate [rad/s]."""

    def __init__(self, rng: random.Random, ideal: bool, spec: GyroSpec = GyroSpec()) -> None:
        self.spec, self.ideal, self.rng = spec, ideal, rng
        self.bias = 0.0 if ideal else rng.uniform(-spec.initial_bias_max_rad_s, spec.initial_bias_max_rad_s)
        self.scale_error = 0.0 if ideal else rng.gauss(0.0, spec.scale_residual_sd)
        self.initial_bias = self.bias                  # the bias changes by its random walk; this is the drawn one

    def measure(self, true_rate: float, dt: float) -> float:
        if self.ideal:
            return true_rate
        s = self.spec
        self.bias += s.bias_rw_rad_s_sqrt_s * math.sqrt(dt) * self.rng.gauss(0.0, 1.0)
        m = (1.0 + self.scale_error) * true_rate + self.bias + self.rng.gauss(0.0, s.noise_rms_rad_s)
        m = max(-s.range_rad_s, min(s.range_rad_s, m))
        return _quantize(m, s.lsb_rad_s)


# --------------------------------------------------------------------------- LiDAR
@dataclass(frozen=True)
class LidarSpec:
    part: str = "Hokuyo URG-04LX-UG01"
    near_limit_m: float = 1.0
    sd_near_m: float = 0.015          # +-30 mm (60-1000 mm) as 2 sigma
    sd_rel_far: float = 0.015         # +-3 % (1000-4095 mm) as 2 sigma
    resolution_m: float = 0.001
    min_range_m: float = 0.02
    max_range_m: float = 5.0          # simulator far clipping (real sensor: 5.6 m)
    scan_period_s: float = 0.1        # 10 Hz

    def sigma(self, r: float) -> float:
        return self.sd_near_m if r < self.near_limit_m else self.sd_rel_far * r


class LidarModel:
    """measure(ranges) -> noisy ranges; None (no return) stays None, out-of-range becomes None."""

    def __init__(self, rng: random.Random, ideal: bool, spec: LidarSpec = LidarSpec()) -> None:
        self.spec, self.ideal, self.rng = spec, ideal, rng

    def measure(self, ranges):
        if self.ideal:
            return list(ranges)
        s, out = self.spec, []
        for r in ranges:
            if r is None:
                out.append(None)
                continue
            m = _quantize(r + self.rng.gauss(0.0, s.sigma(r)), s.resolution_m)
            out.append(m if s.min_range_m <= m <= s.max_range_m else None)
        return out


# --------------------------------------------------------------------------- proximity
@dataclass(frozen=True)
class ProximitySpec:
    part: str = "ASSUMPTION (real part not fixed yet)"
    sd_abs_m: float = 0.002
    sd_rel: float = 0.01
    resolution_m: float = 0.001


class ProximityModel:
    """measure(distance or None) -> noisy distance or None."""

    def __init__(self, rng: random.Random, ideal: bool, spec: ProximitySpec = ProximitySpec()) -> None:
        self.spec, self.ideal, self.rng = spec, ideal, rng

    def measure(self, d):
        if self.ideal or d is None:
            return d
        s = self.spec
        return max(0.0, _quantize(d + self.rng.gauss(0.0, s.sd_abs_m + s.sd_rel * d), s.resolution_m))


# --------------------------------------------------------------------------- profile
@dataclass
class NoiseProfile:
    """One run's sensor models. Separate random streams per sensor, all from one seed."""
    name: str = "ideal"
    seed: int = 0
    # step 19 (N1 / N2): sigma multipliers per sensor {"gyro": k, "lidar": k, "proximity": k}. Scales the random
    # parts (gyro white noise, bias random walk, scale residual; LiDAR and proximity sigma); quantisation and
    # the gyro start offset (removed by the start calibration) stay. Same seed -> same random draws.
    scale: dict | None = None
    gyro: GyroModel = field(init=False)
    lidar: LidarModel = field(init=False)
    proximity: ProximityModel = field(init=False)

    def __post_init__(self) -> None:
        if self.name not in PROFILES:
            raise ValueError(f"unknown noise profile {self.name!r} (use one of {PROFILES})")
        ideal = self.name == "ideal"
        master = random.Random(self.seed)
        streams = [random.Random(master.getrandbits(64)) for _ in range(3)]
        k = {"gyro": 1.0, "lidar": 1.0, "proximity": 1.0, **(self.scale or {})}
        g, l, q = GyroSpec(), LidarSpec(), ProximitySpec()
        g = replace(g, noise_rms_rad_s=k["gyro"] * g.noise_rms_rad_s, bias_rw_rad_s_sqrt_s=k["gyro"] * g.bias_rw_rad_s_sqrt_s,
                    scale_residual_sd=k["gyro"] * g.scale_residual_sd)
        l = replace(l, sd_near_m=k["lidar"] * l.sd_near_m, sd_rel_far=k["lidar"] * l.sd_rel_far)
        q = replace(q, sd_abs_m=k["proximity"] * q.sd_abs_m, sd_rel=k["proximity"] * q.sd_rel)
        self.gyro = GyroModel(streams[0], ideal, g)
        self.lidar = LidarModel(streams[1], ideal, l)
        self.proximity = ProximityModel(streams[2], ideal, q)

    def describe(self) -> dict:
        """Everything needed to reproduce / report the run."""
        return {
            "profile": self.name, "seed": self.seed, "scale": self.scale,
            "gyro": {**self.gyro.spec.__dict__, "drawn_initial_bias_rad_s": self.gyro.bias,
                     "drawn_scale_error": self.gyro.scale_error},
            "lidar": self.lidar.spec.__dict__,
            "proximity": self.proximity.spec.__dict__,
            "encoders": "1024 CPR quantization (both profiles)",
        }


class GyroStartCalibration:
    """Start-up bias calibration: average of the gyro while the robot stands (first HOLD seconds).
    Removes the +-20 deg/s initial zero-rate offset of a real MEMS gyro; used by C and E."""

    def __init__(self) -> None:
        self.n, self.s = 0, 0.0

    def add(self, rate: float) -> None:
        self.n += 1
        self.s += rate

    @property
    def bias(self) -> float:
        return self.s / self.n if self.n else 0.0
