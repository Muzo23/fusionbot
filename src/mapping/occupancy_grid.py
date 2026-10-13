"""
Occupancy grid (step 14): the metric map of the unknown maze, built online from the LiDAR at the
ESTIMATED pose (every configuration A-E builds it the same way; only the pose differs).

Standard log-odds grid (Thrun, Burgard, Fox: Probabilistic Robotics, ch. 9, inverse sensor model):
    l(cell) += L_FREE  for every cell a beam passes through (before the return)
    l(cell) += L_OCC   for the cell of the return
    l clamped to [L_MIN, L_MAX]
The clamp keeps the map able to change: a cell that was free and becomes occupied (a dynamic
obstacle, step 14b) or occupied and becomes free again needs only a bounded number of scans.
Beams without a return (out of range) clear free space up to FREE_MAX_M.

The robot does not know the maze size: the grid starts around the start pose and grows in
GROW_M blocks when a beam leaves it. Grid frame = the start frame of the odometry (world frame of
the known START pose), so the GT comparison (K2, evaluation.metrics) is a direct overlay.

Cell states: occupied  l >  OCC_THRESHOLD  (p > 0.62)
             free      l < -FREE_THRESHOLD (p < 0.43)
             unknown   otherwise
"""

from __future__ import annotations

import math

import numpy as np

RESOLUTION_M = 0.02          # 2 cm cells (corridor 0.729 m = 36 cells, LiDAR noise +-30 mm 2 sigma)
L_OCC = 0.85                 # p = 0.70 per hit
L_FREE = -0.40               # p = 0.40 per pass
L_MIN, L_MAX = -4.0, 4.0     # p 0.018 .. 0.982 (the clamp keeps the map able to change):
                             #   saturated free -> occupied: 6 hits (0.6 s at 10 Hz),
                             #   saturated occupied -> not occupied: 9 passing beams (0.9 s)
OCC_THRESHOLD = 0.5          # one hit marks a cell occupied, one pass marks it free
FREE_THRESHOLD = 0.3
FREE_MAX_M = 4.0             # URG-04LX: specified range 4 m; free space only up to here
GROW_M = 2.0


class OccupancyGrid:
    def __init__(self, x0: float, y0: float, *, resolution: float = RESOLUTION_M, half_size_m: float = 3.0) -> None:
        self.res = float(resolution)
        n = int(math.ceil(2 * half_size_m / self.res))
        self.ox = x0 - half_size_m                      # world coordinates of cell (0, 0) corner
        self.oy = y0 - half_size_m
        self.L = np.zeros((n, n), dtype=np.float32)    # [iy, ix]
        self.seen = np.zeros((n, n), dtype=bool)
        # step 17: when (and with which pose uncertainty) a cell was first seen occupied
        self.t_occ = np.full((n, n), np.inf, dtype=np.float32)
        self.s2_xy = np.zeros((n, n), dtype=np.float32)
        self.s2_th = np.zeros((n, n), dtype=np.float32)
        self.scans = 0

    # ------------------------------------------------------------------ geometry
    @property
    def shape(self):
        return self.L.shape

    def extent(self):
        """(x_min, x_max, y_min, y_max) in metres."""
        ny, nx = self.L.shape
        return self.ox, self.ox + nx * self.res, self.oy, self.oy + ny * self.res

    def _ensure(self, xmin, xmax, ymin, ymax) -> None:
        x0, x1, y0, y1 = self.extent()
        g = GROW_M
        add_l = int(math.ceil((x0 - xmin) / g)) if xmin < x0 else 0
        add_r = int(math.ceil((xmax - x1) / g)) if xmax >= x1 else 0
        add_b = int(math.ceil((y0 - ymin) / g)) if ymin < y0 else 0
        add_t = int(math.ceil((ymax - y1) / g)) if ymax >= y1 else 0
        if not (add_l or add_r or add_b or add_t):
            return
        c = int(round(g / self.res))
        pad = ((add_b * c, add_t * c), (add_l * c, add_r * c))
        self.L = np.pad(self.L, pad)
        self.seen = np.pad(self.seen, pad)
        self.t_occ = np.pad(self.t_occ, pad, constant_values=np.inf)
        self.s2_xy = np.pad(self.s2_xy, pad)
        self.s2_th = np.pad(self.s2_th, pad)
        self.ox -= add_l * c * self.res
        self.oy -= add_b * c * self.res

    def cell_of(self, x, y):
        return (np.floor((np.asarray(y) - self.oy) / self.res).astype(np.int64),
                np.floor((np.asarray(x) - self.ox) / self.res).astype(np.int64))

    def cell_centres(self):
        ny, nx = self.L.shape
        xs = self.ox + (np.arange(nx) + 0.5) * self.res
        ys = self.oy + (np.arange(ny) + 0.5) * self.res
        return np.meshgrid(xs, ys)

    # ------------------------------------------------------------------ update
    def update(self, cx: float, cy: float, heading: float, angles, ranges, *, t: float | None = None,
               pose_var: tuple[float, float] = (0.0, 0.0)) -> None:
        """One scan. (cx, cy) scan centre and heading of the scan frame (estimated pose), angles in
        the scan frame, ranges in metres (None / nan = no return)."""
        a = np.asarray(angles, dtype=float) + heading
        r = np.array([np.nan if v is None else v for v in ranges], dtype=float)
        hit = np.isfinite(r) & (r > 0.0)
        free_len = np.where(hit, r - 0.5 * self.res, FREE_MAX_M)
        free_len = np.clip(free_len, 0.0, FREE_MAX_M)
        ca, sa = np.cos(a), np.sin(a)
        hx, hy = cx + r[hit] * ca[hit], cy + r[hit] * sa[hit]
        ex, ey = cx + free_len * ca, cy + free_len * sa
        self._ensure(min(cx, ex.min(), hx.min(initial=cx)), max(cx, ex.max(), hx.max(initial=cx)),
                     min(cy, ey.min(), hy.min(initial=cy)), max(cy, ey.max(), hy.max(initial=cy)))
        ny, nx = self.L.shape
        # free cells: samples every half cell along each beam (each cell counted once per scan)
        step = 0.5 * self.res
        k = np.arange(int(FREE_MAX_M / step) + 1) * step
        m = k[None, :] < free_len[:, None]
        px = (cx + ca[:, None] * k[None, :])[m]
        py = (cy + sa[:, None] * k[None, :])[m]
        iy, ix = self.cell_of(px, py)
        free = np.unique(iy * nx + ix)
        iy, ix = self.cell_of(hx, hy)
        occ = np.unique(iy * nx + ix)
        free = np.setdiff1d(free, occ, assume_unique=True)
        Lf = self.L.reshape(-1)
        Sf = self.seen.reshape(-1)
        Lf[free] += L_FREE
        was = Lf[occ] > OCC_THRESHOLD
        Lf[occ] += L_OCC
        Lf[free] = np.maximum(Lf[free], L_MIN)
        Lf[occ] = np.minimum(Lf[occ], L_MAX)
        Sf[free] = True
        Sf[occ] = True
        if t is not None:
            new = occ[(~was) & (Lf[occ] > OCC_THRESHOLD)]
            new = new[np.isinf(self.t_occ.reshape(-1)[new])]
            self.t_occ.reshape(-1)[new] = t
            self.s2_xy.reshape(-1)[new] = pose_var[0]
            self.s2_th.reshape(-1)[new] = pose_var[1]
        self.scans += 1

    # ------------------------------------------------------------------ queries
    def occupied(self):
        return self.L > OCC_THRESHOLD

    def free(self):
        return self.L < -FREE_THRESHOLD

    def probability(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.L))

    def save(self, path) -> None:
        np.savez_compressed(path, L=self.L, seen=self.seen, origin=np.array([self.ox, self.oy]),
                            resolution=np.array(self.res), scans=np.array(self.scans))

    @classmethod
    def load(cls, path) -> "OccupancyGrid":
        z = np.load(path)
        g = cls.__new__(cls)
        g.res = float(z["resolution"])
        g.ox, g.oy = (float(v) for v in z["origin"])
        g.L, g.seen, g.scans = z["L"].astype(np.float32), z["seen"].astype(bool), int(z["scans"])
        g.t_occ = np.full(g.L.shape, np.inf, dtype=np.float32)
        g.s2_xy = np.zeros(g.L.shape, dtype=np.float32)
        g.s2_th = np.zeros(g.L.shape, dtype=np.float32)
        return g
