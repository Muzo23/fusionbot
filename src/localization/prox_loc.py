"""
PROX-LOC (step 16): pose corrections from the proximity sensors (configurations B and E).

The maze itself is unknown, but its construction is not (micromouse rules, here mazes.maze_spec):
corridors are axis-aligned and run along the lines of a square lattice of known pitch, and the
START cell is known. With that prior the proximity sensors give absolute information:

1. Corridor update (10 Hz, while driving in a plain corridor): the corridor model of step 8a
   (navigation.corridor_guidance.measure_corridor, all four lateral sensors; or one wall,
   measure_single_wall) gives the robot's lateral offset y from the corridor centre line and its
   angle alpha to the corridor axis.
       axis phi    = the multiple of 90 deg nearest to (theta - alpha)
       centre line = the lattice line nearest to the robot (perpendicular coordinate)
       z = [y, alpha],  h(x) = [n . p - c_line,  wrap(theta - phi)],  n = left normal of the axis
2. Junction update (on every junction event of the FSM): the junction centre lies D_canonical
   ahead of the robot (step 10) and on a lattice node, so the coordinate ALONG the corridor is
       z = node - D_canonical,  h(x) = u . p
   (skipped when the nearest lattice line is more than JUNCTION_SNAP_MAX_M away - e.g. the START
   dead end, which is not a lattice node).

Both go through the EKF chi-square gate: an obstacle at a side wall (DYN3) or a wrong lattice line
gives a large innovation and is rejected.

Noise: proximity sensors sigma = 2 mm + 1 % (noise profile) propagated through the linear corridor
model, plus a model floor (corridor model residual, wall placement); the 10 Hz rate keeps the
correlation of consecutive readings of the same wall down.
"""

from __future__ import annotations

import math

import numpy as np

from navigation import navigation_config as ncfg
from navigation.corridor_guidance import (
    ALPHA_FROM_E, ALPHA_FROM_S, Y_FROM_E, Y_FROM_S, measure_corridor, measure_single_wall,
)
from navigation.junction_types import ReferenceKind

PROX_SD_ABS, PROX_SD_REL = 0.002, 0.01        # assumed sensor (same as the noise profile)
Y_MODEL_SD = 0.004                             # corridor model / wall placement floor [m]
ALPHA_MODEL_SD = math.radians(0.5)
SINGLE_INFLATE = 2.0                           # one wall only: weaker
CORRIDOR_EVERY = 10                            # ticks (10 Hz)
JUNCTION_SD = {ReferenceKind.SIDE_CONFIRMED: 0.015, ReferenceKind.DEAD_END: 0.010}
JUNCTION_SNAP_MAX_M = 0.20


def _sd(d: float) -> float:
    return PROX_SD_ABS + PROX_SD_REL * d


def corridor_measurement(prox):
    """(y, alpha, R 2x2, kind) or None."""
    m = measure_corridor(prox)
    if m is not None:
        dl, dr = prox.left.distance_m, prox.right.distance_m
        fl, fr = prox.front_left.distance_m, prox.front_right.distance_m
        vs = 0.25 * (_sd(dl) ** 2 + _sd(dr) ** 2)
        ve = _sd(fl) ** 2 + _sd(fr) ** 2
        J = np.array([[Y_FROM_S, Y_FROM_E], [ALPHA_FROM_S, ALPHA_FROM_E]])
        R = J @ np.diag([vs, ve]) @ J.T + np.diag([Y_MODEL_SD ** 2, ALPHA_MODEL_SD ** 2])
        return m.y_m, m.alpha_rad, R, "full"
    for side in ("left", "right"):
        m = measure_single_wall(prox, side)
        if m is not None:
            R = SINGLE_INFLATE ** 2 * np.diag([Y_MODEL_SD ** 2 + 0.005 ** 2, ALPHA_MODEL_SD ** 2 + math.radians(1.0) ** 2])
            return m.y_m, m.alpha_rad, R, side
    return None


class Lattice:
    """Lines x = x0 + k p and y = y0 + k p (corridor centre lines, junction centres on the nodes)."""

    def __init__(self, x0: float, y0: float, pitch: float) -> None:
        self.x0, self.y0, self.p = x0, y0, pitch

    def snap(self, v: float, axis: str) -> float:
        o = self.x0 if axis == "x" else self.y0
        return o + round((v - o) / self.p) * self.p


class ProxLoc:
    def __init__(self, ekf, lattice: Lattice) -> None:
        self.ekf, self.lat = ekf, lattice
        self.k = 0

    def corridor(self, prox) -> bool | None:
        self.k += 1
        if self.k % CORRIDOR_EVERY:
            return None
        meas = corridor_measurement(prox)
        if meas is None:
            return None
        y, a, R, kind = meas
        x = self.ekf.x
        q = round((x[2] - a) / (0.5 * math.pi))
        phi = q * 0.5 * math.pi
        nx, ny = -math.sin(phi), math.cos(phi)            # left normal of the corridor axis
        if abs(nx) > 0.5:                                  # axis along y: the centre line is x = const
            c_line = self.lat.snap(x[0], "x") * nx
        else:
            c_line = self.lat.snap(x[1], "y") * ny
        h = [nx * x[0] + ny * x[1] - c_line, math.remainder(x[2] - phi, 2 * math.pi)]
        H = np.array([[nx, ny, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]])
        return self.ekf.update("prox_" + ("corridor" if kind == "full" else "wall"), [y, a], h, H, R,
                               angle_rows=(1,))

    def junction(self, kind: ReferenceKind) -> bool | None:
        x = self.ekf.x
        q = round(x[2] / (0.5 * math.pi))
        phi = q * 0.5 * math.pi
        ux, uy = round(math.cos(phi)), round(math.sin(phi))
        d = ncfg.D_SIDE_CANONICAL_M if kind is ReferenceKind.SIDE_CONFIRMED else ncfg.D_DEAD_END_CANONICAL_M
        axis = "x" if ux else "y"
        along = x[0] * ux + x[1] * uy                       # signed coordinate along the heading
        centre = (x[0] if ux else x[1]) + d * (ux or uy)
        node = self.lat.snap(centre, axis)
        if abs(node - centre) > JUNCTION_SNAP_MAX_M:
            return None
        z = node * (ux or uy) - d
        H = np.array([[ux, uy, 0.0, 0.0]], dtype=float)
        return self.ekf.update("junction", [z], [along], H, [[JUNCTION_SD.get(kind, 0.015) ** 2]])
