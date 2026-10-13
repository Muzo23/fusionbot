"""
Step 7b LIVE experiment: systematic odometry error of in-place turns.

Live run 1 of step 7 showed GT/odo = 0.989-0.996 with CW worse than CCW, but in
that sequence every turn reversed the previous one, so a "reversal" effect
(castor swivel, wheel-direction change) could not be separated from a pure
direction (CW/CCW) or scale (B_eff) effect. This experiment separates them:

    run A   +90 +90 +90 +90  -90 -90 -90 -90
    run B   +180 +180  -180 -180

Each turn is classed as FIRST (after the straight), REPEAT (same sign as the
previous turn) or REVERSAL (opposite sign). The scene restarts at START for each
run, so axle drift does not accumulate between them.

Output: per-turn table, mean error per (direction, class), and the effective
track B that would make REPEAT turns unbiased on average over CW and CCW
(a UMBmark-style scale calibration; it is only a RECOMMENDATION, nothing is
changed automatically).

    python -m checks.step07b_turn_calibration              (from src/, T1, robot at START)
    python -m checks.step07b_turn_calibration --alpha 2.4  (angular-acceleration ramp)
"""

from __future__ import annotations

import argparse
from statistics import mean

from checks.step07_live_turns import run_mission, turn_table
from core import robot_config as rc

RUNS = {
    "A": (90.0, 90.0, 90.0, 90.0, -90.0, -90.0, -90.0, -90.0),
    "B": (180.0, 180.0, -180.0, -180.0),
}


def classify(turns):
    out, prev = [], None
    for d in turns:
        out.append("FIRST" if prev is None else ("REPEAT" if d * prev > 0 else "REVERSAL"))
        prev = d
    return out


def scale_loss_fit(rows):
    """|GT| = k * |odo| - loss_dir  (shared scale k, constant loss per turn for CCW / CW), least squares.
    k != 1 -> scale error (B_eff); loss -> transient slip per turn (not removable by B_eff)."""
    pts = [(abs(r[3]), abs(r[4]), 0 if r[1] > 0 else 1) for r in rows]
    # normal equations for (k, c0, c1) with model g = k*o - c_d
    n = [sum(1 for p in pts if p[2] == d) for d in (0, 1)]
    so = [sum(p[0] for p in pts if p[2] == d) for d in (0, 1)]
    sg = [sum(p[1] for p in pts if p[2] == d) for d in (0, 1)]
    soo = sum(p[0] * p[0] for p in pts)
    sog = sum(p[0] * p[1] for p in pts)
    # eliminate c_d = (k*so_d - sg_d) / n_d
    num = sog - sum(so[d] * sg[d] / n[d] for d in (0, 1) if n[d])
    den = soo - sum(so[d] ** 2 / n[d] for d in (0, 1) if n[d])
    k = num / den
    c = [(k * so[d] - sg[d]) / n[d] if n[d] else 0.0 for d in (0, 1)]
    res = [p[1] - (k * p[0] - c[p[2]]) for p in pts]
    return k, c[0], c[1], max(abs(x) for x in res)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--alpha", type=float, default=None, help="TURN angular acceleration limit [rad/s^2]")
    args = ap.parse_args()
    tag = "" if args.alpha is None else f"_alpha{args.alpha:g}"
    print(f"TURN alpha limit: {args.alpha if args.alpha is not None else 'none (step)'}")
    rows = []          # (run, target, class, odo, gt, transl)
    for name, turns in RUNS.items():
        results, log = run_mission(turns, f"calib_{name}{tag}.csv", alpha=args.alpha)
        print(f"\nrun {name}")
        tab = turn_table(results)
        if any(r["collision"] for r in log):
            print("  !! collision in this run")
        for (tgt, odo, gt, _, tr), cls in zip(tab, classify(turns)):
            rows.append((name, tgt, cls, odo, gt, tr))

    print("\nodo - GT [deg] by direction / amplitude / class")
    print(f"{'dir':>4} {'amp':>4} {'class':>9} {'n':>2} {'mean':>8} {'GT/odo':>7} {'transl':>7}")
    for amp in (90.0, 180.0):
        for sgn, dname in ((1, "CCW"), (-1, "CW")):
            for cls in ("FIRST", "REPEAT", "REVERSAL"):
                sel = [r for r in rows if r[1] == sgn * amp and r[2] == cls]
                if not sel:
                    continue
                e = mean(r[3] - r[4] for r in sel)
                k = mean(r[4] / r[3] for r in sel)
                tr = mean(r[5] for r in sel)
                print(f"{dname:>4} {amp:4.0f} {cls:>9} {len(sel):2d} {e:+8.3f} {k:7.4f} {tr:5.1f}mm")

    rep = [r for r in rows if r[2] == "REPEAT"]
    k_ccw = mean(r[4] / r[3] for r in rep if r[1] > 0)
    k_cw = mean(r[4] / r[3] for r in rep if r[1] < 0)
    k = 0.5 * (k_ccw + k_cw)
    b_new = rc.EFFECTIVE_WHEEL_SEPARATION_M / k
    for amp in (90.0, 180.0):
        sel = [abs(r[3] - r[4]) for r in rows if abs(r[1]) == amp]
        print(f"|odo-GT| all {amp:.0f} deg turns: mean {mean(sel):.3f} deg, max {max(sel):.3f} deg")
    print(f"\nREPEAT turns: GT/odo CCW {k_ccw:.5f}, CW {k_cw:.5f}, mean {k:.5f}")
    print(f"current B_eff {rc.EFFECTIVE_WHEEL_SEPARATION_M:.5f} m -> scale-calibrated B_eff {b_new:.5f} m")
    print(f"remaining CW/CCW asymmetry (not removable by B): "
          f"{100 * (k_ccw - k_cw):+.3f} % of the turn angle")
    ks, c_ccw, c_cw, rmax = scale_loss_fit(rows)
    print(f"\nmodel |GT| = k*|odo| - loss : k {ks:.5f} (scale -> B_eff {rc.EFFECTIVE_WHEEL_SEPARATION_M / ks:.5f} m), "
          f"loss per turn CCW {c_ccw:+.3f} deg, CW {c_cw:+.3f} deg, max residual {rmax:.3f} deg")
    print("  (B_eff used since 8. 10. 2026 = scale part only; the constant loss is left to the fusion)")
    sum_gt = {n: sum(r[4] for r in rows if r[0] == n) for n in RUNS}
    print("net GT heading per run (ideal 0): " + ", ".join(f"{n} {v:+.3f} deg" for n, v in sum_gt.items()))


if __name__ == "__main__":
    main()
