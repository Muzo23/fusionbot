"""
LiDAR scan geometry and validation against the known maze (step 12c).

Pure functions (no CoppeliaSim), used by the live static check and by the
offline tests:

  scan_from_xyzd(xyzd, hokuyo_matrix, centre)      raw property -> world rays
  evaluate_scan(rays, wallmap, centre, heading)    residuals vs the true walls
  fit_scan_pose(points_scan, wallmap, pose0)       2-D point-to-wall alignment
                                                   (x, y, yaw of the scan frame)
  RangeModel / fit_range_model / apply_range_model systematic range error of the
                                                   simulated sensor, e(r) = a r + b r^2
                                                   (step 12d calibration)

Grazing rays: a ray that meets a wall at a shallow angle turns a tiny error of
the point normal to the wall into a large error along the ray (1 / sin of the
incidence angle). Along-ray statistics and the range calibration therefore use
only rays with sin(incidence) >= GRAZING_SIN; the error normal to the wall is
reported for all rays.

Scan frame: origin = scan centre (mean of the two vision-sensor origins of the
Hokuyo model), x = centre of the 240 deg field of view (= robot heading after
sim.lidar_setup), y = left. Bearings are measured in this frame.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

NO_HIT_FRACTION = 0.9999          # distance >= 0.9999 * max range = no return
EDGE_JUMP_M = 0.05                # neighbouring expected ranges differ by more -> wall edge ray
GRAZING_SIN = 0.5                 # rays meeting the wall at < 30 deg are "grazing"
EDGE_WINDOW = 3                   # rays within +-3 steps (~1 deg) of a range discontinuity are edge rays:
                                  # the sensor resamples depth pixels (0.2-0.5 deg) to 0.351 deg rays,
                                  # so a ray next to a wall corner can mix the near and the far surface
ROBUST_K = 3.5                    # outlier rejection: |residual| > max(ROBUST_MIN_M, K * 1.4826 * MAD)
ROBUST_MIN_M = 0.003


@dataclass(frozen=True)
class Ray:
    bearing_rad: float            # in the scan frame
    range_m: float | None         # in-plane range from the scan centre, None = no return
    x_m: float | None = None      # world hit point
    y_m: float | None = None
    z_m: float | None = None


def _wrap(a: float) -> float:
    return math.remainder(a, 2 * math.pi)


def scan_from_xyzd(xyzd, hokuyo_matrix, centre, heading_rad: float, max_range_m: float) -> list[Ray]:
    """xyzd: 4 floats per ray (x, y, z in the Hokuyo frame, sensor distance);
    hokuyo_matrix: 12-float world matrix of the Hokuyo object (row-major 3x4);
    centre: world scan centre; heading_rad: world yaw of the scan x axis."""
    m = hokuyo_matrix
    cx, cy = centre[0], centre[1]
    out = []
    for i in range(0, len(xyzd) - 3, 4):
        px, py, pz, d = xyzd[i:i + 4]
        wx = m[0] * px + m[1] * py + m[2] * pz + m[3]
        wy = m[4] * px + m[5] * py + m[6] * pz + m[7]
        wz = m[8] * px + m[9] * py + m[10] * pz + m[11]
        b = _wrap(math.atan2(wy - cy, wx - cx) - heading_rad)
        if d >= NO_HIT_FRACTION * max_range_m:
            out.append(Ray(b, None))
        else:
            out.append(Ray(b, math.hypot(wx - cx, wy - cy), wx, wy, wz))
    return out


def expected_range(wallmap, x, y, angle, max_range_m):
    r = wallmap.raycast(x, y, angle, max_range_m)
    return None if not math.isfinite(r) else r


def incidence_sin(wallmap, x, y, angle, r) -> float:
    """sin of the angle between the ray and the wall it hits at range r."""
    hx, hy = x + r * math.cos(angle), y + r * math.sin(angle)
    dv = min((abs(hx - X) for X, y0, y1 in wallmap.v if y0 - 1e-6 <= hy <= y1 + 1e-6), default=math.inf)
    dh = min((abs(hy - Y) for Y, x0, x1 in wallmap.h if x0 - 1e-6 <= hx <= x1 + 1e-6), default=math.inf)
    return abs(math.cos(angle)) if dv <= dh else abs(math.sin(angle))


def _pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))] if v else math.nan


def evaluate_scan(rays, wallmap, centre, heading_rad: float, max_range_m: float,
                  plane_z_m: float | None = None) -> dict:
    """Compare every ray with the ray cast through the true walls from the true scan pose."""
    cx, cy = centre[0], centre[1]
    samples, wall_dist, z_dev = [], [], []
    missed = phantom = edge = 0
    bearings = [r.bearing_rad for r in rays]
    step = (max(bearings) - min(bearings)) / max(1, len(bearings) - 1) if rays else 0.0
    min_range = math.inf
    for r in rays:
        a = heading_rad + r.bearing_rad
        e = expected_range(wallmap, cx, cy, a, max_range_m * NO_HIT_FRACTION)
        win = [expected_range(wallmap, cx, cy, a + j * step, max_range_m) for j in range(-EDGE_WINDOW, EDGE_WINDOW + 1)]
        win[EDGE_WINDOW] = expected_range(wallmap, cx, cy, a, max_range_m)
        near_edge = any((u is None) != (v is None) or (u is not None and abs(u - v) > EDGE_JUMP_M)
                        for u, v in zip(win, win[1:]))
        if r.range_m is None:
            if e is not None and not near_edge:
                missed += 1
            continue
        min_range = min(min_range, r.range_m)
        if e is None:
            if not near_edge:
                phantom += 1
            continue
        if near_edge:
            edge += 1
            continue
        sin_i = incidence_sin(wallmap, cx, cy, a, e)
        samples.append((e, sin_i, r.range_m - e, r.bearing_rad))
        wall_dist.append(wallmap.distance_to_walls(r.x_m, r.y_m))
        if plane_z_m is not None and r.z_m is not None:
            z_dev.append(r.z_m - plane_z_m)
    res = [s[2] for s in samples]
    good = [s[2] for s in samples if s[1] >= GRAZING_SIN]
    normal = [abs(s[2] * s[1]) for s in samples]
    a = [abs(v) for v in res]
    g = [abs(v) for v in good]
    return {
        "rays": len(rays), "returns": sum(r.range_m is not None for r in rays),
        "compared": len(res), "non_grazing": len(good), "edge_rays": edge, "missed": missed, "phantom": phantom,
        "bias_m": sum(res) / len(res) if res else math.nan,
        "median_abs_m": _pct(a, 0.5), "p95_abs_m": _pct(a, 0.95), "max_abs_m": max(a) if a else math.nan,
        "ng_bias_m": sum(good) / len(good) if good else math.nan,
        "ng_median_abs_m": _pct(g, 0.5), "ng_p95_abs_m": _pct(g, 0.95),
        "normal_p95_m": _pct(normal, 0.95), "normal_max_m": max(normal) if normal else math.nan,
        "max_point_to_wall_m": max(wall_dist) if wall_dist else math.nan,
        "max_plane_dev_m": max((abs(z) for z in z_dev), default=math.nan),
        "min_range_m": min_range,
        "bearing_min_deg": math.degrees(min(bearings)) if rays else math.nan,
        "bearing_max_deg": math.degrees(max(bearings)) if rays else math.nan,
        "bearing_step_deg": math.degrees(step),
        "residuals": res,
        "samples": samples,          # (expected range, sin incidence, residual, bearing)
    }


@dataclass(frozen=True)
class RangeModel:
    """Systematic range error of the sensor: measured = true + a r + b r^2."""
    a: float = 0.0
    b: float = 0.0

    def error(self, r: float) -> float:
        return self.a * r + self.b * r * r

    def correct(self, r: float) -> float:
        # invert r_m = t + e(t) by fixed-point iteration (|a|, |b r| << 1, converges fast)
        t = r
        for _ in range(4):
            t = r - self.error(t)
        return t


def robust_inliers(res):
    """Mask of values within max(ROBUST_MIN_M, ROBUST_K * 1.4826 * MAD) of the median, and the threshold."""
    if not res:
        return [], math.inf
    v = sorted(res)
    med = v[len(v) // 2]
    mad = sorted(abs(x - med) for x in v)[len(v) // 2]
    thr = max(ROBUST_MIN_M, ROBUST_K * 1.4826 * mad)
    return [abs(x - med) <= thr for x in res], thr


def fit_range_model(samples, min_sin: float = GRAZING_SIN, robust: bool = True):
    """Least squares (through the origin) of the along-ray residual on [r, r^2], non-grazing rays.
    Robust: refitted 3 times without outliers (|residual - model| > ROBUST_K * MAD), e.g. mixed
    pixels next to wall corners. Returns (RangeModel, rays used)."""
    data = [(e, res) for e, sin_i, res, _ in samples if sin_i >= min_sin]
    model = RangeModel()
    for _ in range(4 if robust else 1):
        s11 = s12 = s22 = b1 = b2 = 0.0
        for e, res in data:
            s11 += e * e
            s12 += e ** 3
            s22 += e ** 4
            b1 += e * res
            b2 += e * e * res
        det = s11 * s22 - s12 * s12
        if len(data) < 10 or abs(det) < 1e-12:
            raise ValueError("not enough non-grazing rays to fit the range model")
        model = RangeModel((s22 * b1 - s12 * b2) / det, (s11 * b2 - s12 * b1) / det)
        if robust:
            all_ = [(e, res) for e, sin_i, res, _ in samples if sin_i >= min_sin]
            mask, _ = robust_inliers([res - model.error(e) for e, res in all_])
            data = [d for d, keep in zip(all_, mask) if keep]
    return model, len(data)


def apply_range_model(rays, centre, heading_rad: float, model: RangeModel) -> list[Ray]:
    """Corrected rays: range corrected, hit point moved along the ray from the scan centre."""
    out = []
    for r in rays:
        if r.range_m is None:
            out.append(r)
            continue
        rc = model.correct(r.range_m)
        a = heading_rad + r.bearing_rad
        out.append(Ray(r.bearing_rad, rc, centre[0] + rc * math.cos(a), centre[1] + rc * math.sin(a), r.z_m))
    return out


def scan_points(rays) -> list[tuple[float, float]]:
    """Hit points in the scan frame."""
    return [(r.range_m * math.cos(r.bearing_rad), r.range_m * math.sin(r.bearing_rad))
            for r in rays if r.range_m is not None]


def _nearest_wall(wallmap, x, y):
    """(normal_x, normal_y, offset) of the nearest wall line n.p = offset, and its distance."""
    best = (math.inf, None)
    for X, y0, y1 in wallmap.v:
        dy = 0.0 if y0 <= y <= y1 else min(abs(y - y0), abs(y - y1))
        d = math.hypot(x - X, dy)
        if d < best[0]:
            best = (d, (1.0, 0.0, X) if dy == 0.0 else None)
    for Y, x0, x1 in wallmap.h:
        dx = 0.0 if x0 <= x <= x1 else min(abs(x - x0), abs(x - x1))
        d = math.hypot(dx, y - Y)
        if d < best[0]:
            best = (d, (0.0, 1.0, Y) if dx == 0.0 else None)
    return best[1], best[0]


def fit_scan_pose(points, wallmap, pose0, iterations: int = 15, gate_m: float = 0.10):
    """Gauss-Newton point-to-line alignment of scan-frame points with the true walls.
    Returns (x, y, yaw, rms_m, n_used). Points matched to a wall END (not its face) are skipped;
    after 3 iterations the gate shrinks to 3x the rms (min 5 mm), so mixed-pixel outliers drop out."""
    x, y, th = pose0
    rms, n = math.nan, 0
    for it in range(iterations):
        if it >= 3 and math.isfinite(rms):
            gate_m = min(gate_m, max(0.005, 3.0 * rms))
        c, s = math.cos(th), math.sin(th)
        H = [[0.0] * 3 for _ in range(3)]
        g = [0.0] * 3
        sq, n = 0.0, 0
        for px, py in points:
            wx, wy = x + c * px - s * py, y + s * px + c * py
            line, d = _nearest_wall(wallmap, wx, wy)
            if line is None or d > gate_m:
                continue
            nx, ny, off = line
            r = nx * wx + ny * wy - off
            # d r / d (x, y, th)
            J = (nx, ny, nx * (-s * px - c * py) + ny * (c * px - s * py))
            for i in range(3):
                g[i] += J[i] * r
                for k in range(3):
                    H[i][k] += J[i] * J[k]
            sq += r * r
            n += 1
        if n < 10:
            break
        rms = math.sqrt(sq / n)
        dx = _solve3(H, [-v for v in g])
        if dx is None:
            break
        x, y, th = x + dx[0], y + dx[1], th + dx[2]
        if max(abs(dx[0]), abs(dx[1])) < 1e-7 and abs(dx[2]) < 1e-8:
            break
    return x, y, th, rms, n


def _solve3(A, b):
    m = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(3):
        p = max(range(c, 3), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-12:
            return None
        m[c], m[p] = m[p], m[c]
        for r in range(3):
            if r != c:
                f = m[r][c] / m[c][c]
                for k in range(c, 4):
                    m[r][k] -= f * m[c][k]
    return [m[i][3] / m[i][i] for i in range(3)]
