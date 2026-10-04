"""
FusionBot differential-drive odometry (step 4).

Pose estimate from encoder increments only (configuration A, and the
prediction input of every EKF configuration later).

Per 100 Hz sample, from left/right delta counts dN_L, dN_R:

    ds_L = r * q_c * dN_L          ds_R = r * q_c * dN_R
    ds   = (ds_R + ds_L) / 2       dtheta = (ds_R - ds_L) / B_eff

Midpoint (second-order) integration:

    theta_m = theta + dtheta/2
    x += ds * cos(theta_m)
    y += ds * sin(theta_m)
    theta = wrap(theta + dtheta)

Midpoint integration is exact for straight lines and in-place rotation, and
for an arc its error per step is O(ds * dtheta^2): with 10 ms steps it is
below 1e-6 m per metre even in turns (checked in the test).

Conventions: world frame, x/y in metres, theta = 0 along +X, CCW positive.
Besides the wrapped pose the class also keeps:
    signed_distance_m   integral of ds (reverse subtracts) - used by the FSM
    path_length_m       integral of |ds|
    heading_unwrapped   integral of dtheta without wrapping - used for
                        relative turns (+180 and -180 stay distinct)
GT is never an input.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from core.robot_config import EFFECTIVE_WHEEL_SEPARATION_M, METERS_PER_COUNT


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Pose2D:
    x_m: float
    y_m: float
    theta_rad: float


@dataclass(frozen=True)
class OdometryIncrement:
    left_delta_counts: int
    right_delta_counts: int
    ds_m: float
    dtheta_rad: float
    pose: Pose2D


class DifferentialDriveOdometry:
    def __init__(
        self,
        x0_m: float = 0.0,
        y0_m: float = 0.0,
        theta0_rad: float = 0.0,
        meters_per_count: float = METERS_PER_COUNT,
        separation_m: float = EFFECTIVE_WHEEL_SEPARATION_M,
    ) -> None:
        if meters_per_count <= 0.0 or separation_m <= 0.0:
            raise ValueError("meters_per_count and separation_m must be > 0")
        self.meters_per_count = float(meters_per_count)
        self.separation_m = float(separation_m)
        self.reset(x0_m, y0_m, theta0_rad)

    def reset(self, x_m: float = 0.0, y_m: float = 0.0, theta_rad: float = 0.0) -> None:
        self._x, self._y = float(x_m), float(y_m)
        self._theta = wrap_to_pi(float(theta_rad))
        self.heading_unwrapped = float(theta_rad)
        self.signed_distance_m = 0.0
        self.path_length_m = 0.0

    @property
    def pose(self) -> Pose2D:
        return Pose2D(self._x, self._y, self._theta)

    def update(self, left_delta_counts: int, right_delta_counts: int) -> OdometryIncrement:
        ds_l = int(left_delta_counts) * self.meters_per_count
        ds_r = int(right_delta_counts) * self.meters_per_count
        ds = 0.5 * (ds_l + ds_r)
        dth = (ds_r - ds_l) / self.separation_m

        th_m = self._theta + 0.5 * dth
        self._x += ds * math.cos(th_m)
        self._y += ds * math.sin(th_m)
        self._theta = wrap_to_pi(self._theta + dth)

        self.heading_unwrapped += dth
        self.signed_distance_m += ds
        self.path_length_m += abs(ds)
        return OdometryIncrement(int(left_delta_counts), int(right_delta_counts), ds, dth, self.pose)
