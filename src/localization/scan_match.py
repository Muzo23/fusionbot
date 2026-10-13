"""
LiDAR scan-to-map matching (step 17): configurations D and E.

The LiDAR scan is aligned with the robot's OWN occupancy grid (step 14), built from the earlier scans
at the estimated poses - no prior knowledge of the maze, no lattice (unlike PROX-LOC, step 16).
The result is a measurement of the full pose (x, y, theta) with its covariance for the EKF.

Method (likelihood field + Gauss-Newton, as in Hector SLAM - Kohlbrecher et al. 2011 - and the
likelihood-field model of Thrun, Burgard, Fox 2005, ch. 6.4):
  * distance field: distance from every cell of a local window (WINDOW_M around the robot) to the
    nearest occupied cell, clipped at D_MAX_M; recomputed when the robot leaves the window centre or
    every FIELD_EVERY scans (the map grows) - numpy chamfer relaxation, no SciPy needed;
  * scan points (every RAY_STEP-th ray, 0.05 < r < RANGE_MAX_M) are moved to the world with the pose
    and the mount; the residual of a point is the field value at its position, the Jacobian the field
    gradient times d(point)/d(x, y, theta);
  * Gauss-Newton with Huber weights (HUBER_M); points where the field is saturated (no mapped wall near:
    a new corridor, or an obstacle that is not in the map) do not take part;
  * covariance = residual variance x (J^T W J)^-1, inflated by COV_INFLATE (neighbouring rays and the
    map itself are correlated with the estimate); in a straight corridor the direction along the axis
    is (almost) unobservable and gets a huge variance, so the EKF takes nothing from it.
The EKF then applies z = (x, y, theta) with its gate (step 15): a bad match is rejected.

The map is built from the same estimate, so matching against walls mapped a moment ago carries no
ABSOLUTE information (map and pose share the error). Every occupied cell therefore remembers when it was
first mapped and the pose variance at that time (OccupancyGrid.t_occ, s2_xy, s2_th):
  * "old" walls (mapped more than OLD_S ago, i.e. the robot comes back to a known place): absolute update,
    R = matching covariance + the pose variance the map had there (the map is a landmark with its own
    uncertainty);
  * otherwise "relative" update: the mean is corrected (drift of the estimate relative to the walls just
    seen - like a better odometry), the covariance is NOT reduced (keep_cov), so the filter stays honest.
"""

from __future__ import annotations

import math

import numpy as np

WINDOW_M = 6.0
D_MAX_M = 0.30
FIELD_EVERY = 10              # scans (1 s): the map grows
FIELD_RECENTRE_M = 1.5
RAY_STEP = 4                  # 684 rays -> 171
RANGE_MAX_M = 4.0
HUBER_M = 0.04
ITERATIONS = 8
MIN_POINTS = 40
MIN_OCCUPIED = 150
SIGMA_MIN_M = 0.015
# Matching covariance inflation. The ~170 rays of one scan and the scans at 10 Hz against the same walls
# are strongly correlated (and the map comes from the same estimate); tuned offline on T1 realistic
# (step 17: 9 -> L1 179 mm, NEES 11 574; 100 -> 80 mm / 313; 1000 -> 48 mm / 22; 10 000 -> 132 mm / 32).
# Step 15b (with the tuned process noise, 21 Monte Carlo replays): 10 -> L1 132 mm, 30 -> 84, 100 -> 62,
# 300 -> 76, 1000 -> 113, 3000 -> 180 mm; ANEES stays 240-11 000 for every value (the map is built from the
# same estimate - see docs/korak15b.md) -> 100, the most accurate.
COV_INFLATE = 100.0
OLD_S = 30.0
MAX_STEP_M, MAX_STEP_RAD = 0.25, math.radians(8.0)    # a larger correction is not a match


class LikelihoodField:
    def __init__(self, grid, cx: float, cy: float, with_attributes: bool = False) -> None:
        g = grid
        n = int(round(WINDOW_M / g.res))
        iy0, ix0 = g.cell_of(cx - WINDOW_M / 2, cy - WINDOW_M / 2)
        iy0, ix0 = int(iy0), int(ix0)
        ny, nx = g.shape
        occ = np.zeros((n, n), dtype=bool)
        t_occ = np.full((n, n), np.inf)
        s2xy = np.zeros((n, n))
        s2th = np.zeros((n, n))
        ya, yb = max(iy0, 0), min(iy0 + n, ny)
        xa, xb = max(ix0, 0), min(ix0 + n, nx)
        if ya < yb and xa < xb:
            sl, gl = np.s_[ya - iy0:yb - iy0, xa - ix0:xb - ix0], np.s_[ya:yb, xa:xb]
            occ[sl] = g.occupied()[gl]
            if hasattr(g, "t_occ"):
                t_occ[sl], s2xy[sl], s2th[sl] = g.t_occ[gl], g.s2_xy[gl], g.s2_th[gl]
        self.n_occupied = int(occ.sum())
        self.res = g.res
        self.ox = g.ox + ix0 * g.res
        self.oy = g.oy + iy0 * g.res
        self.cx, self.cy = cx, cy
        if with_attributes:
            self.d, (self.t_occ, self.s2xy, self.s2th) = _distance(occ, g.res, (t_occ, s2xy, s2th))
        else:
            self.d, _ = _distance(occ, g.res)
        self.gy, self.gx = np.gradient(self.d, g.res)

    def attributes(self, px, py):
        """(mapping time, pose variance xy, theta) of the nearest occupied cell at world points."""
        j = np.clip(((px - self.ox) / self.res).astype(int), 0, self.d.shape[1] - 1)
        i = np.clip(((py - self.oy) / self.res).astype(int), 0, self.d.shape[0] - 1)
        return self.t_occ[i, j], self.s2xy[i, j], self.s2th[i, j]

    def sample(self, px, py):
        """Field value and gradient at world points (bilinear); outside the window: D_MAX, 0."""
        fx = (px - self.ox) / self.res - 0.5
        fy = (py - self.oy) / self.res - 0.5
        n = self.d.shape[0]
        ok = (fx >= 0) & (fy >= 0) & (fx < n - 1) & (fy < n - 1)
        val = np.full(px.shape, D_MAX_M)
        gx = np.zeros(px.shape)
        gy = np.zeros(px.shape)
        if ok.any():
            x, y = fx[ok], fy[ok]
            i, j = y.astype(int), x.astype(int)
            a, b = y - i, x - j
            for arr, out in ((self.d, val), (self.gx, gx), (self.gy, gy)):
                v = (arr[i, j] * (1 - a) * (1 - b) + arr[i, j + 1] * (1 - a) * b
                     + arr[i + 1, j] * a * (1 - b) + arr[i + 1, j + 1] * a * b)
                out[ok] = v
        return val, gx, gy


_SHIFTS = ((1, 0), (1, 2), (0, 1), (2, 1), (0, 0), (0, 2), (2, 0), (2, 2))   # (row, col) slices of the padded array


def _distance(occ: np.ndarray, res: float, attrs=()):
    """Chamfer distance to the nearest True cell [m], clipped at D_MAX_M, and the attributes of that
    nearest cell (arrays of the same shape, carried along the relaxation)."""
    big = D_MAX_M
    d = np.where(occ, 0.0, big)
    att = [np.where(occ, a, np.nan) for a in attrs]
    steps = int(math.ceil(D_MAX_M / res)) + 1
    diag = math.sqrt(2.0) * res
    cost = (res, res, res, res, diag, diag, diag, diag)
    n0, n1 = d.shape
    if not attrs:
        # fast path: octagonal distance by alternating 4- and 8-neighbour boolean dilations (exact for
        # axis-aligned walls, <= 8 % off diagonally); ~20x faster than the relaxation
        reached = occ.copy()
        d = np.where(occ, 0.0, big)
        for k in range(1, steps):
            p = np.pad(reached, 1)
            nb = p[1:-1, :-2] | p[1:-1, 2:] | p[:-2, 1:-1] | p[2:, 1:-1]
            if k % 2 == 0:
                nb |= p[:-2, :-2] | p[:-2, 2:] | p[2:, :-2] | p[2:, 2:]
            new = nb & ~reached
            if not new.any():
                break
            d[new] = k * res
            reached |= new
        return np.minimum(d, D_MAX_M), []
    for _ in range(steps):
        p = np.pad(d, 1, constant_values=big)
        cands = np.stack([p[r:r + n0, c:c + n1] + w for (r, c), w in zip(_SHIFTS, cost)])
        k = np.argmin(cands, axis=0)
        best = np.take_along_axis(cands, k[None], axis=0)[0]
        upd = best < d - 1e-12
        if not upd.any():
            break
        d = np.where(upd, best, d)
        for i, a in enumerate(att):
            pa = np.pad(a, 1, constant_values=np.nan)
            sh = np.stack([pa[r:r + n0, c:c + n1] for (r, c) in _SHIFTS])
            att[i] = np.where(upd, np.take_along_axis(sh, k[None], axis=0)[0], a)
    return np.minimum(d, D_MAX_M), att


class ScanMatcher:
    def __init__(self, grid, mount: tuple[float, float, float]) -> None:
        self.grid = grid
        self.fwd, self.left, self.yaw = mount
        self.field: LikelihoodField | None = None
        self.scans = 0
        self.last: dict = {}

    def _points(self, angles, ranges):
        a = np.asarray(angles, float)[::RAY_STEP]
        r = np.array([np.nan if v is None else v for v in ranges], float)[::RAY_STEP]
        ok = np.isfinite(r) & (r > 0.05) & (r < RANGE_MAX_M)
        a, r = a[ok] + self.yaw, r[ok]
        return self.fwd + r * np.cos(a), self.left + r * np.sin(a)      # robot frame

    def _gauss_newton(self, f, p, bx, by):
        for _ in range(ITERATIONS):
            c, s = math.cos(p[2]), math.sin(p[2])
            wx, wy = p[0] + c * bx - s * by, p[1] + s * bx + c * by
            val, gx, gy = f.sample(wx, wy)
            use = val < 0.9 * D_MAX_M
            if use.sum() < MIN_POINTS:
                return None
            r = val[use]
            dthx, dthy = -s * bx[use] - c * by[use], c * bx[use] - s * by[use]
            J = np.stack([gx[use], gy[use], gx[use] * dthx + gy[use] * dthy], axis=1)
            w = np.where(np.abs(r) <= HUBER_M, 1.0, HUBER_M / np.maximum(np.abs(r), 1e-9))
            Hm = (J * w[:, None]).T @ J + np.diag([1e-6, 1e-6, 1e-7])
            dp = -np.linalg.solve(Hm, (J * w[:, None]).T @ r)
            p = p + dp
            if np.abs(dp[:2]).max() < 1e-4 and abs(dp[2]) < 1e-5:
                break
        sig2 = max(float(np.sum(w * r * r) / max(np.sum(w), 1.0)), SIGMA_MIN_M ** 2)
        return p, COV_INFLATE * sig2 * np.linalg.inv(Hm), wx[use], wy[use], int(use.sum()), math.sqrt(float(np.mean(r * r)))

    def match_split(self, pose, angles, ranges, t: float):
        """-> (pose, covariance, kind) with kind "absolute" (old walls, map variance added) or
        "relative", or None."""
        self.scans += 1
        x, y, th = pose
        f = self.field
        if f is None or self.scans % FIELD_EVERY == 0 or math.hypot(x - f.cx, y - f.cy) > FIELD_RECENTRE_M:
            f = self.field = LikelihoodField(self.grid, x, y, with_attributes=True)
        if f.n_occupied < MIN_OCCUPIED:
            self.last = {"reason": "map too small"}
            return None
        bx, by = self._points(angles, ranges)
        if bx.size < MIN_POINTS:
            self.last = {"reason": "few points"}
            return None
        res = self._gauss_newton(f, np.array([x, y, th], float), bx, by)
        if res is None:
            self.last = {"reason": "few matched points"}
            return None
        p, cov, wx, wy, n, rms = res
        t_occ, s2xy, s2th = f.attributes(wx, wy)
        old = (t - t_occ) > OLD_S
        kind = "relative"
        if old.sum() >= MIN_POINTS:
            # re-match on the old walls only and add the uncertainty the map had there
            c, s_ = math.cos(p[2]), math.sin(p[2])
            ox_, oy_ = wx[old] - p[0], wy[old] - p[1]
            rbx, rby = c * ox_ + s_ * oy_, -s_ * ox_ + c * oy_          # back to the robot frame
            r2 = self._gauss_newton(f, p.copy(), rbx, rby)
            if r2 is not None:
                p, cov = r2[0], r2[1] + np.diag([float(np.median(s2xy[old]))] * 2 + [float(np.median(s2th[old]))])
                kind = "absolute"
        if math.hypot(p[0] - x, p[1] - y) > MAX_STEP_M or abs(math.remainder(p[2] - th, 2 * math.pi)) > MAX_STEP_RAD:
            self.last = {"reason": "correction too large"}
            return None
        self.last = {"matched": n, "rms_m": rms, "kind": kind, "old": int(old.sum())}
        return (float(p[0]), float(p[1]), math.remainder(float(p[2]), 2 * math.pi)), cov, kind

    def match(self, pose, angles, ranges):
        """-> (pose (x, y, theta), covariance 3x3) or None (no split: every wall treated alike)."""
        self.scans += 1
        x, y, th = pose
        f = self.field
        if f is None or self.scans % FIELD_EVERY == 0 or math.hypot(x - f.cx, y - f.cy) > FIELD_RECENTRE_M:
            f = self.field = LikelihoodField(self.grid, x, y)
        if f.n_occupied < MIN_OCCUPIED:
            self.last = {"reason": "map too small"}
            return None
        bx, by = self._points(angles, ranges)
        if bx.size < MIN_POINTS:
            self.last = {"reason": "few points"}
            return None
        p = np.array([x, y, th], float)
        for _ in range(ITERATIONS):
            c, s = math.cos(p[2]), math.sin(p[2])
            wx, wy = p[0] + c * bx - s * by, p[1] + s * bx + c * by
            val, gx, gy = f.sample(wx, wy)
            use = val < 0.9 * D_MAX_M
            if use.sum() < MIN_POINTS:
                self.last = {"reason": "few matched points", "matched": int(use.sum())}
                return None
            r = val[use]
            dthx, dthy = -s * bx[use] - c * by[use], c * bx[use] - s * by[use]
            J = np.stack([gx[use], gy[use], gx[use] * dthx + gy[use] * dthy], axis=1)
            w = np.where(np.abs(r) <= HUBER_M, 1.0, HUBER_M / np.maximum(np.abs(r), 1e-9))
            Hm = (J * w[:, None]).T @ J + np.diag([1e-6, 1e-6, 1e-7])
            dp = -np.linalg.solve(Hm, (J * w[:, None]).T @ r)
            p = p + dp
            if np.abs(dp[:2]).max() < 1e-4 and abs(dp[2]) < 1e-5:
                break
        if math.hypot(p[0] - x, p[1] - y) > MAX_STEP_M or abs(math.remainder(p[2] - th, 2 * math.pi)) > MAX_STEP_RAD:
            self.last = {"reason": "correction too large"}
            return None
        sig2 = max(float(np.sum(w * r * r) / max(np.sum(w), 1.0)), SIGMA_MIN_M ** 2)
        cov = COV_INFLATE * sig2 * np.linalg.inv(Hm)
        self.last = {"matched": int(use.sum()), "rms_m": math.sqrt(float(np.mean(r * r)))}
        return (float(p[0]), float(p[1]), math.remainder(float(p[2]), 2 * math.pi)), cov

    # ------------------------------------------------------------------ step 15b: covariance intersection
    def _map_variance(self, wx, wy):
        """Median pose variance (xy, theta) the map had when the walls under the matched points were drawn
        (OccupancyGrid.s2_xy / s2_th of the occupied cells in the 3x3 neighbourhood of each point)."""
        g = self.grid
        occ = g.occupied()
        iy, ix = g.cell_of(wx, wy)
        ny, nx = occ.shape
        vxy, vth = [], []
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                i, j = np.clip(iy + di, 0, ny - 1), np.clip(ix + dj, 0, nx - 1)
                o = occ[i, j]
                vxy.append(np.where(o, g.s2_xy[i, j], np.nan))
                vth.append(np.where(o, g.s2_th[i, j], np.nan))
        vxy, vth = np.nanmax(np.stack(vxy), axis=0), np.nanmax(np.stack(vth), axis=0)
        ok = np.isfinite(vxy)
        if not ok.any():
            return None
        return float(np.median(vxy[ok])), float(np.median(vth[ok]))

    def match_ci(self, pose, angles, ranges):
        """-> (pose, R = matching covariance + map variance, "ci") or None. The EKF fuses it by covariance
        intersection (ekf.update(ci=True)): the map is built from the same estimate, so the correlation
        between the pose and the "measurement" is unknown - CI is consistent for any correlation
        (Julier & Uhlmann 1997); the pose can never become more certain than the walls it is matched to."""
        self.scans += 1
        x, y, th = pose
        f = self.field
        if f is None or self.scans % FIELD_EVERY == 0 or math.hypot(x - f.cx, y - f.cy) > FIELD_RECENTRE_M:
            f = self.field = LikelihoodField(self.grid, x, y)
        if f.n_occupied < MIN_OCCUPIED:
            self.last = {"reason": "map too small"}
            return None
        bx, by = self._points(angles, ranges)
        if bx.size < MIN_POINTS:
            self.last = {"reason": "few points"}
            return None
        res = self._gauss_newton(f, np.array([x, y, th], float), bx, by)
        if res is None:
            self.last = {"reason": "few matched points"}
            return None
        p, cov, wx, wy, n, rms = res
        if math.hypot(p[0] - x, p[1] - y) > MAX_STEP_M or abs(math.remainder(p[2] - th, 2 * math.pi)) > MAX_STEP_RAD:
            self.last = {"reason": "correction too large"}
            return None
        mv = self._map_variance(wx, wy)
        if mv is None:
            self.last = {"reason": "no map variance"}
            return None
        R = cov + np.diag([mv[0], mv[0], mv[1]])
        self.last = {"matched": n, "rms_m": rms, "map_sd_mm": 1000 * math.sqrt(mv[0])}
        return (float(p[0]), float(p[1]), math.remainder(float(p[2]), 2 * math.pi)), R, "ci"
