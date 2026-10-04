"""
FusionBot incremental wheel encoder (step 2).

Model of an ideal incremental (quadrature) encoder with 1024 counts per wheel
revolution, read at 100 Hz.

    joint position q (rad, wraps at +-pi)
      -> unwrap:     dq = wrap(q - q_prev)          (valid while |dq| < pi)
      -> accumulate: theta += dq                    (continuous wheel angle)
      -> quantize:   N = floor(theta / q_c)         (count edges at fixed angles)
      -> output:     dN = N - N_prev, speed = dN * q_c / T

Why floor of the ACCUMULATED angle: a real encoder emits an edge every time
the shaft crosses a fixed angular mark, so the count is a function of the
absolute angle. This keeps the count exact across direction reversals and
never loses sub-count motion (the fraction stays inside theta).

Only joint POSITION is used. The true joint velocity belongs to the motor
model; every speed used for control comes from counts (dN), as on a real robot.

Unwrap limit: at the 22.65 rad/s command limit the wheel turns 0.23 rad per
10 ms sample, far below pi.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from core.robot_config import ENCODER_CPR, ENCODER_RATE_HZ

TWO_PI = 2.0 * math.pi


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % TWO_PI - math.pi


@dataclass(frozen=True)
class EncoderSample:
    delta_counts: int
    total_counts: int
    delta_angle_rad: float      # dN * q_c (quantized)
    speed_rad_s: float          # encoder-derived wheel speed
    period_s: float


class IncrementalEncoder:
    def __init__(self, counts_per_rev: int = ENCODER_CPR, rate_hz: float = ENCODER_RATE_HZ) -> None:
        if counts_per_rev <= 0 or rate_hz <= 0.0:
            raise ValueError("counts_per_rev and rate_hz must be > 0")
        self.counts_per_rev = int(counts_per_rev)
        self.rad_per_count = TWO_PI / self.counts_per_rev
        self.period_s = 1.0 / float(rate_hz)
        self.reset()

    def reset(self, joint_position_rad: float | None = None) -> None:
        """Zero the counter. With a position, initialize immediately;
        otherwise the next update() establishes the reference."""
        self._initialized = joint_position_rad is not None
        self._q_prev = float(joint_position_rad) if self._initialized else 0.0
        self._theta = 0.0               # accumulated continuous angle since reset
        self._total = 0
        self._total_at_last_sample = 0
        self._time_since_sample = 0.0

    @property
    def total_counts(self) -> int:
        return self._total

    def update(self, joint_position_rad: float, dt_s: float) -> EncoderSample | None:
        """Feed one position sample. Returns a sample once per encoder period."""
        if dt_s < 0.0:
            raise ValueError("dt_s must be >= 0")
        q = float(joint_position_rad)
        if not self._initialized:
            self._q_prev = q
            self._initialized = True
            return None

        self._theta += wrap_to_pi(q - self._q_prev)
        self._q_prev = q
        self._total = math.floor(self._theta / self.rad_per_count + 1e-12)
        self._time_since_sample += dt_s

        if self._time_since_sample + 1e-12 < self.period_s:
            return None

        period = self._time_since_sample
        dn = self._total - self._total_at_last_sample
        self._total_at_last_sample = self._total
        self._time_since_sample = 0.0
        dang = dn * self.rad_per_count
        return EncoderSample(
            delta_counts=dn,
            total_counts=self._total,
            delta_angle_rad=dang,
            speed_rad_s=dang / period if period > 0.0 else 0.0,
            period_s=period,
        )
