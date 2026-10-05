"""
Step 8a LIVE experiment: measurement model of the corridor from the proximity sensors.

Goal: from the four lateral sensors (L, FL, FR, R) estimate the robot's lateral
offset y [m, left positive] and heading error alpha [rad, CCW positive] relative
to the corridor axis. Two combinations are used:

    s = (d_R - d_L) / 2        side difference     (mostly y)
    e =  d_FL - d_FR           diagonal difference (y AND alpha)

Expected first-order model (rays, small angles):
    s = a_y*y + a_a*alpha + a_0     a_y ~ 1,  a_a ~ forward offset of the side sensors
    e = b_y*y + b_a*alpha + b_0     b_y ~ -2/sin(diag angle)

Experiment (5 simulation runs, scene T1, each from START, START corridor only):
    turn delta in {-6, -3, 0, +3, +6} deg at START, settle, then drive straight
    0.45 m at 0.25 m/s WITHOUT any correction (the heading error makes y grow).
The in-place turn sweeps alpha at y ~ 0; the straight sweeps y at constant alpha.

GT (axle position and heading) is used ONLY here, offline, as the calibration
reference (like a lab calibration rig). The START pose defines the corridor
axis: START is on the corridor centreline, heading along the corridor.

Valid sample: all four lateral sensors detect, and the axle is < 0.42 m from
START (beyond that the diagonals start to look into the ROOT opening).

Output: least-squares coefficients, residuals, a quadratic-term check of the
linear model over the calibrated range, and the inverse model
(y, alpha) = f(e, s) to paste into navigation/corridor_guidance.py.

    python -m checks.step08a_corridor_model     (from src/, scene T1, robot at START)
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np

from checks.step07_live_turns import RESULTS as _R7, run_mission

RESULTS = _R7.parent / "step08"
DELTAS_DEG = (-6.0, -3.0, 0.0, 3.0, 6.0)
STRAIGHT_M = 0.45
MAX_AXLE_FROM_START_M = 0.42


def collect():
    samples = []          # (run, y, alpha, dL, dFL, dFR, dR)
    for delta in DELTAS_DEG:
        prims = ([("turn", delta)] if delta != 0.0 else []) + [("straight", STRAIGHT_M)]
        name = f"calib_{delta:+.0f}.csv"
        _, rows = run_mission((), name, primitives=prims)
        (RESULTS / name).parent.mkdir(parents=True, exist_ok=True)
        (_R7 / name).replace(RESULTS / name)          # keep step-8 logs together
        x0, y0, th0 = rows[0]["gt_x"], rows[0]["gt_y"], rows[0]["gt_th_unwrapped"]
        coll = False
        for r in rows:
            coll |= bool(r["collision"])
            dx, dy = r["gt_x"] - x0, r["gt_y"] - y0
            along = dx * math.cos(th0) + dy * math.sin(th0)
            lat = -dx * math.sin(th0) + dy * math.cos(th0)
            d = [r[f"d_{n}"] for n in ("left", "front_left", "front_right", "right")]
            if any(v is None for v in d) or along > MAX_AXLE_FROM_START_M:
                continue
            samples.append((delta, lat, r["gt_th_unwrapped"] - th0, *d))
        print(f"run delta {delta:+.0f} deg: {len(rows)} ticks, collision {coll}")
    return np.array(samples, dtype=float)


def fit(X, z):
    A = np.column_stack([X, np.ones(len(z))])
    c, *_ = np.linalg.lstsq(A, z, rcond=None)
    res = z - A @ c
    return c, res


def main() -> None:
    S = collect()
    run, y, a = S[:, 0], S[:, 1], S[:, 2]
    dL, dFL, dFR, dR = S[:, 3], S[:, 4], S[:, 5], S[:, 6]
    s = 0.5 * (dR - dL)
    e = dFL - dFR
    with (RESULTS / "calib_samples.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["delta_deg", "y_m", "alpha_rad", "d_left", "d_front_left", "d_front_right", "d_right", "s", "e"])
        w.writerows(np.column_stack([S, s, e]).tolist())

    print(f"\nStep 8a corridor measurement model  ({len(S)} samples)")
    print("------------------------------------------------------")
    print(f"range: y {1000 * y.min():+.1f} .. {1000 * y.max():+.1f} mm, "
          f"alpha {math.degrees(a.min()):+.2f} .. {math.degrees(a.max()):+.2f} deg")

    print("\nper-sensor linear fit  d = c_y*y + c_a*alpha + c_0")
    for name, d in (("left", dL), ("front_left", dFL), ("front_right", dFR), ("right", dR)):
        c, r = fit(np.column_stack([y, a]), d)
        print(f"  {name:11} c_y {c[0]:+.4f}  c_a {c[1]:+.4f} m/rad  c_0 {c[2]:.5f} m   rms {1000 * r.std():.3f} mm")

    Y = np.column_stack([y, a])
    cs, rs = fit(Y, s)
    ce, re = fit(Y, e)
    print("\nlinear model")
    print(f"  s = {cs[0]:+.4f}*y {cs[1]:+.4f}*alpha {cs[2]:+.5f}   rms {1000 * rs.std():.3f} mm, max {1000 * abs(rs).max():.3f} mm")
    print(f"  e = {ce[0]:+.4f}*y {ce[1]:+.4f}*alpha {ce[2]:+.5f}   rms {1000 * re.std():.3f} mm, max {1000 * abs(re).max():.3f} mm")

    Q = np.column_stack([y, a, y * y, y * a, a * a])
    _, rs2 = fit(Q, s)
    _, re2 = fit(Q, e)
    print(f"  quadratic terms would reduce max residual to s {1000 * abs(rs2).max():.3f} mm, "
          f"e {1000 * abs(re2).max():.3f} mm")

    M = np.array([[cs[0], cs[1]], [ce[0], ce[1]]])
    Mi = np.linalg.inv(M)
    off = -Mi @ np.array([cs[2], ce[2]])
    # inverse: [y, alpha] = Mi @ [s, e] + off
    print("\ninverse model (paste into navigation/corridor_guidance.py)")
    print(f"  Y_FROM_S, Y_FROM_E, Y_OFFSET = {Mi[0, 0]:.4f}, {Mi[0, 1]:.4f}, {off[0]:.5f}")
    print(f"  ALPHA_FROM_S, ALPHA_FROM_E, ALPHA_OFFSET = {Mi[1, 0]:.4f}, {Mi[1, 1]:.4f}, {off[1]:.5f}")
    yh = Mi[0, 0] * s + Mi[0, 1] * e + off[0]
    ah = Mi[1, 0] * s + Mi[1, 1] * e + off[1]
    print(f"  self-check: y error rms {1000 * (yh - y).std():.3f} mm (max {1000 * abs(yh - y).max():.3f}), "
          f"alpha error rms {math.degrees((ah - a).std()):.4f} deg (max {math.degrees(abs(ah - a).max()):.4f})")
    print(f"\nsamples: {RESULTS / 'calib_samples.csv'}")


if __name__ == "__main__":
    main()
