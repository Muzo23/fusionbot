"""
Step 6 LIVE check: proximity perception at the T1 ROOT junction.

From START the robot drives straight north at 0.25 m/s for 1.9 m (odometry)
and stops. ROOT has openings to the WEST (left) and EAST (right) and continues
NORTH, so both side state machines must go WALL -> CANDIDATE -> CONFIRMED.

Geometry check: START is 1.422634 m south of the ROOT centre and the validated
event-to-centre offset is D_SIDE_CANONICAL = 0.530606 m, so CONFIRMED is
expected at an odometry distance of 1.422634 - 0.530606 = 0.892028 m.

    python -m checks.step06_live_perception      (from src/, scene T1, robot at START)
"""

from __future__ import annotations

import csv
import time
from pathlib import Path

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from checks.step05_live_drive import Stack
from core import robot_config as rc
from perception.proximity_perception import ProximityPerception
from sim.sim_utils import prepare_lockstep, wait_until_stopped
from sim.transport import LockstepSession

RESULTS = Path(__file__).resolve().parents[2] / "results" / "step06"
EXPECTED_CONFIRMED_S = 1.422634 - 0.530606
DRIVE_M, V = 1.9, 0.25


def main() -> None:
    sim = RemoteAPIClient().require("sim")
    original_rt = prepare_lockstep(sim)
    stack, perc = Stack(), ProximityPerception()
    rows, events = [], {"left": {}, "right": {}}
    prev = {"left": "WALL", "right": "WALL"}
    link = LockstepSession()
    try:
        time.sleep(0.2)
        sim.startSimulation()
        stop_t = None
        while True:
            s = link.next_state(timeout_s=10.0)
            t = s.simulation_time_s
            if not stack.started:
                if t >= 0.5:
                    stack.start(s.left_joint_position_rad, s.right_joint_position_rad)
                link.send(s.state_sequence, 0.0, 0.0)
                continue
            dist = stack.odo.signed_distance_m
            v = V if dist < DRIVE_M else 0.0
            if v == 0.0 and stop_t is None:
                stop_t = t
            wl, wr = rc.wheel_speeds_from_body(v, 0.0)
            cl, cr, *_ = stack.tick(s.left_joint_position_rad, s.right_joint_position_rad, wl, wr)
            link.send(s.state_sequence, cl, cr)
            out = perc.update(s.proximity)
            for side, st in (("left", out.left_state.value), ("right", out.right_state.value)):
                if st != prev[side]:
                    events[side].setdefault(st, dist)
                    if st == "WALL":
                        events[side]["false_reset"] = dist
                    prev[side] = st
            p = s.proximity
            rows.append({
                "s_odo": dist, "left": out.left_state.value, "right": out.right_state.value,
                "front_blocked": int(out.front_blocked),
                **{f"d_{n}": getattr(p, n).distance_m for n in ("left", "front_left", "front", "front_right", "right")},
                "collision": int(s.collision or s.visual_collision),
            })
            if stop_t is not None and t - stop_t > 0.5:
                break
    finally:
        link.release()
        sim.stopSimulation()
        wait_until_stopped(sim)
        link.close()
        sim.setBoolProperty(sim.handle_scene, "realtimeSimulation", original_rt)

    RESULTS.mkdir(parents=True, exist_ok=True)
    with (RESULTS / "root_pass.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    print("\nStep 6 live check (ROOT junction)")
    print("---------------------------------")
    ok = True
    for side in ("left", "right"):
        e = events[side]
        c = e.get("CONFIRMED")
        err = None if c is None else 1000 * (c - EXPECTED_CONFIRMED_S)
        print(f"{side:5}: CANDIDATE at {e.get('CANDIDATE', float('nan')):.4f} m, "
              f"CONFIRMED at {c if c is not None else float('nan'):.4f} m "
              f"(expected {EXPECTED_CONFIRMED_S:.4f}, error {err if err is not None else float('nan'):+.1f} mm), "
              f"PASSED at {e.get('PASSED', float('nan')):.4f} m, false reset: {'false_reset' in e}")
        ok &= c is not None and abs(err) < 15.0 and "false_reset" not in e
    coll = any(r["collision"] for r in rows)
    print(f"collision: {coll}")
    print(f"verdict  : {'PASS' if ok and not coll else 'CHECK (see values)'}")
    print(f"log      : {RESULTS / 'root_pass.csv'}")


if __name__ == "__main__":
    main()
