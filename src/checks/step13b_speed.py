"""
Step 13b-7: how close to real time does the simulation run, and where does the time go?

The robot stands at START (zero motor commands) for --seconds of simulated time in
lockstep. Measured per 10 ms step:
    sim     time spent waiting for CoppeliaSim (physics, sensors, scripts, rendering)
    python  time between receiving the state and sending the command (our side)
and the real-time factor  RTF = simulated time / wall time  (1.0 = real time).

    python -m checks.step13b_speed                 (from src/, any scene, simulation stopped)
    python -m checks.step13b_speed --seconds 30

Run it once with the 3D view on and once with visualization toggled off in CoppeliaSim
to see how much the GUI rendering costs.
"""

from __future__ import annotations

import argparse
import statistics as st
import time


def main() -> None:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient

    from sim.sim_utils import prepare_lockstep, wait_until_stopped
    from sim.transport import LockstepSession

    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    args = ap.parse_args()
    sim = RemoteAPIClient().require("sim")
    original_rt = prepare_lockstep(sim)
    link = LockstepSession()
    t_sim, t_py, scans, imu_ok = [], [], 0, 0
    try:
        time.sleep(0.2)
        sim.startSimulation()
        s = link.next_state(timeout_s=10.0)
        t0_sim, w0 = s.simulation_time_s, time.perf_counter()
        last = time.perf_counter()
        link.send(s.state_sequence, 0.0, 0.0)
        while True:
            a = time.perf_counter()
            s = link.next_state(timeout_s=10.0)
            b = time.perf_counter()
            scans += 1 if s.lidar_xyzd else 0
            imu_ok += 1 if s.imu_count >= 0 else 0
            link.send(s.state_sequence, 0.0, 0.0)
            c = time.perf_counter()
            t_sim.append(b - a)
            t_py.append(c - b)
            if s.simulation_time_s - t0_sim >= args.seconds:
                break
        wall = time.perf_counter() - w0
        sim_s = s.simulation_time_s - t0_sim
    finally:
        link.release()
        sim.stopSimulation()
        wait_until_stopped(sim)
        link.close()
        sim.setBoolProperty(sim.handle_scene, "realtimeSimulation", original_rt)
    n = len(t_sim)
    print(f"\nStep 13b-7 simulation speed ({n} steps, {sim_s:.1f} s simulated in {wall:.1f} s)")
    print("------------------------------------------------------------")
    print(f"real-time factor     : {sim_s / wall:.2f}  (1.00 = real time; >1 faster than real time)")
    print(f"per 10 ms step       : simulator {1000 * st.mean(t_sim):.2f} ms (median {1000 * st.median(t_sim):.2f}, "
          f"p95 {1000 * sorted(t_sim)[int(0.95 * n)]:.2f}), python {1000 * st.mean(t_py):.3f} ms")
    print(f"LiDAR scans          : {scans} ({scans / max(sim_s, 1e-9):.1f} Hz), IMU in {imu_ok}/{n} packets")
    print("note                 : the runner adds its own work per step (recording, GT evaluation): "
          "see wall_time_s in summary.json")


if __name__ == "__main__":
    main()
