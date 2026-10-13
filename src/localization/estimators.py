"""
Pose estimators per configuration (step 13b-2: interface; the EKF follows in step 15).

The navigation (FSM, motion control) always runs on the encoders. What the
configurations change is the POSE ESTIMATE that places the junctions of the
topological map, finds the goal and draws the metric map:

    A          odometry (encoders only)                 -> EkfEstimator, prediction only (step 15:
               the pose is the step-4 odometry, the filter adds its covariance -> NEES for A too)
    C          EKF, heading from the gyro, ZUPT bias updates                     (step 15)
    B          EKF, encoders + PROX-LOC (corridor walls, junctions on the lattice) (step 16)
    D          EKF, encoders + LiDAR scan-to-map matching (own occupancy grid)     (step 17)
    E          everything: gyro heading + ZUPT, PROX-LOC, LiDAR                  (step 18)

Every estimator is fed every tick with the same inputs (whatever its
configuration does not use it ignores), so the runner does not depend on the
configuration.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from core.robot_config import SIMULATION_TIMESTEP_S as DT_S

from core.configurations import Configuration


@dataclass
class TickInputs:
    t: float
    odo_increment: object          # localization.odometry increment of this tick (encoders)
    gyro_z_rad_s: float | None     # measured (profile applied), None if no IMU
    gyro_bias_start: float | None  # start-up calibration result (C / E)
    proximity: object              # ProximitySnapshot (profile applied)
    scan: dict | None              # {"angles", "ranges"} on a LiDAR tick (profile applied), else None


class OdometryEstimator:
    """Configuration A: the pose is the encoder odometry, nothing else."""
    name = "odometry"

    def __init__(self, odometry) -> None:
        self.odo = odometry

    def update(self, inputs: TickInputs) -> None:      # odometry is already updated by the drive stack
        pass

    def pose(self) -> tuple[float, float, float]:
        p = self.odo.pose
        return p.x_m, p.y_m, p.theta_rad

    def covariance(self):
        return None


class EkfEstimator:
    """localization.ekf.PoseEKF driven by the TickInputs; what it uses follows the configuration."""

    def __init__(self, config: Configuration, odometry, *, gating: bool = True) -> None:
        from localization.ekf import PoseEKF
        self.config = config
        self.odo = odometry
        self.name = "odometry+cov" if config.estimator == "odometry" else "ekf"
        self.ekf = PoseEKF(use_gyro=config.uses_gyro, zupt=config.gyro_zupt,
                           gate=None if gating else {1: math.inf, 2: math.inf, 3: math.inf})
        self.gating = gating
        self.zupt_ticks = 0
        self.prox_loc = None
        self.matcher = None
        self.lidar_skipped = 0

    def start(self, x: float, y: float, th: float, *, gyro_bias: float | None = None, gyro_samples: int = 0,
              lattice: tuple[float, float, float] | None = None) -> None:
        """lattice = (x0, y0, pitch) of the maze construction grid (known START cell, micromouse rules)."""
        from localization.ekf import GYRO_RATE_SD
        sd = GYRO_RATE_SD / math.sqrt(gyro_samples) if gyro_samples else math.radians(1.0)
        self.ekf.start(x, y, th, bias=gyro_bias or 0.0, bias_sd=sd if self.config.uses_gyro else 0.0)
        if self.config.prox_loc_update:
            from localization.prox_loc import Lattice, ProxLoc
            if lattice is None:
                raise ValueError("PROX-LOC needs the maze lattice")
            self.prox_loc = ProxLoc(self.ekf, Lattice(*lattice))

    def update(self, inputs: TickInputs) -> None:
        if not self.ekf.started:
            return
        inc = inputs.odo_increment
        omega = inputs.gyro_z_rad_s if self.config.uses_gyro else None
        self.ekf.predict(inc.ds_m, inc.dtheta_rad, omega, DT_S)
        if self.config.gyro_zupt and omega is not None:
            if self.ekf.at_rest(inc.left_delta_counts, inc.right_delta_counts):
                self.zupt_ticks += 1
                self.ekf.zupt_update(omega)
        if self.prox_loc is not None and inputs.proximity is not None:
            self.prox_loc.corridor(inputs.proximity)
        if self.matcher is not None and inputs.scan is not None:
            pose, ang, rng = self.ekf.pose(), inputs.scan["angles"], inputs.scan["ranges"]
            res = (self.matcher.match_split(pose, ang, rng, inputs.t) if SCAN_MODE == "split" else
                   self.matcher.match_ci(pose, ang, rng) if SCAN_MODE == "ci" else self.matcher.match(pose, ang, rng))
            if res is not None:
                z, R, kind = res if len(res) == 3 else (*res, "plain")
                H = np.hstack([np.eye(3), np.zeros((3, 1))])
                self.ekf.update("lidar_" + kind, list(z), list(self.ekf.x[:3]), H, R, angle_rows=(2,),
                                keep_cov=(kind == "relative"), ci=(kind == "ci"))
            else:
                self.lidar_skipped += 1

    def attach_lidar(self, grid, mount) -> None:
        """D / E: scan-to-map matching against the robot's own occupancy grid (step 17)."""
        if self.config.lidar_update:
            from localization.scan_match import ScanMatcher
            self.matcher = ScanMatcher(grid, mount)

    def on_junction(self, kind) -> None:
        """FSM junction event (reference tick): PROX-LOC along-corridor update."""
        if self.prox_loc is not None and self.ekf.started:
            self.prox_loc.junction(kind)

    def pose(self) -> tuple[float, float, float]:
        if not self.ekf.started:
            p = self.odo.pose
            return p.x_m, p.y_m, p.theta_rad
        return self.ekf.pose()

    def covariance(self):
        return self.ekf.covariance() if self.ekf.started else None

    @property
    def rejected_total(self) -> int:
        return sum(r for _, r in self.ekf.stats.values())

    @property
    def gyro_bias(self) -> float:
        return float(self.ekf.x[3])

    def stats(self) -> dict:
        return {"updates": {k: {"accepted": a, "rejected": r} for k, (a, r) in self.ekf.stats.items()},
                "zupt_ticks": self.zupt_ticks, "bias_resets": len(self.ekf.bias_resets), "lidar_no_match": self.lidar_skipped, "gyro_bias_final_deg_s": math.degrees(self.gyro_bias)}


IMPLEMENTED = ("A", "B", "C", "D", "E")
# step 17: "plain" = every scan match is an ordinary EKF update (inflated covariance, default);
# "split" = absolute update only against walls mapped > 30 s earlier, relative (mean-only) otherwise.
# Offline T1 realistic: plain L1 48 mm, split 309 mm -> plain.
# "ci" (step 15b): R = matching covariance + map variance, fused by covariance intersection.
SCAN_MODE = "plain"


def make_estimator(config: Configuration, odometry, *, gating: bool = True):
    """gating=False: every measurement accepted (ablation for the thesis, step 16)."""
    if config.name in IMPLEMENTED:
        return EkfEstimator(config, odometry, gating=gating)
    raise NotImplementedError(
        f"configuration {config.name} ({config.description}) needs the EKF - planned in step 15 "
        f"(sensors: {', '.join(config.sensors())}); implemented: {', '.join(IMPLEMENTED)}")
