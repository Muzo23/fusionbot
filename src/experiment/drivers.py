"""
Where the measurements come from (step 13b-2): the live CoppeliaSim bridge or the offline plant.

    live_loop(run, spec)        CoppeliaSim, lockstep bridge protocol 2.1 (IMU every step,
                                LiDAR every 10th step); the scene must contain the maze
    offline_loop(run, spec)     tests/maze_sim.py plant (drive model with the live turn loss,
                                single-ray proximity sensors, rough footprint collision);
                                gyro = true body rate, LiDAR optional (ray casting, slow)

Both feed RobotRun.step() with the same state fields (sim.protocol.SimState), so
the robot code is identical; offline runs are for development, regression and
Monte Carlo, the live runs are the thesis results.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
LIDAR_DIVIDER = 10
LIDAR_RAYS = 684


def live_loop(run, spec, *, check_scene: bool = True, dyn: str | None = None):
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient

    from checks.step07_live_turns import start_guard
    from sim.maze_scene import scene_maze_id
    from sim.sim_utils import prepare_lockstep, wait_until_stopped
    from sim.transport import LockstepSession

    sim = RemoteAPIClient().require("sim")
    if check_scene:
        sid = scene_maze_id(sim)
        if sid is not None and sid != spec.maze_id:
            raise SystemExit(f"the scene contains maze {sid}, not {spec.maze_id}: "
                             f"python -m sim.maze_scene {spec.maze_id}  (then rerun)")
    world = poses = None
    if dyn:
        from core import robot_config as rc
        from mazes.dynamic import DynamicWorld, build_scenario
        from sim import dynamic_scene
        lj, rj = sim.getObject(rc.LEFT_MOTOR_JOINT_PATH), sim.getObject(rc.RIGHT_MOTOR_JOINT_PATH)
        floor_z = 0.5 * (sim.getObjectPosition(lj, sim.handle_world)[2]
                         + sim.getObjectPosition(rj, sim.handle_world)[2]) - rc.WHEEL_RADIUS_M
        dynamic_scene.create(sim, floor_z)
        poses = dynamic_scene.LivePoses(floor_z)

        def placer():
            obstacles = build_scenario(spec, dyn, run.planned_gt_route())
            run.attach_dynamic(dyn, obstacles)
            print(f"  dynamic: {', '.join(f'{o.name} on {o.edge[0]}-{o.edge[1]}' for o in obstacles) or 'no edge fits'}")
            return obstacles
        world = DynamicWorld(spec, placer=placer)
        run.attach_dynamic(dyn, [])
    original_rt = prepare_lockstep(sim)
    link = LockstepSession()
    wall0 = time.perf_counter()
    guarded = warned = False
    try:
        time.sleep(0.2)
        sim.startSimulation()
        while True:
            s = link.next_state(timeout_s=10.0)
            if s.imu_count < 0 and not warned:
                warned = True
                print("  (no IMU block in the packet - old bridge? python -m sim.install_bridge, Ctrl+S)")
            if not run.started and s.simulation_time_s >= 0.5 and not guarded:
                start_guard(s.proximity)
                guarded = True
            cl, cr, done = run.step(s)
            upd = None
            if world is not None:
                for name in world.update(s.simulation_time_s, s.gt_axle_x_m, s.gt_axle_y_m, run.goal_running(),
                                         s.gt_heading_rad):
                    run.note_dynamic(name, s.simulation_time_s)
                upd = poses.updates(world.obstacles, s.simulation_time_s)
            link.send(s.state_sequence, cl, cr, upd)
            if done:
                break
    finally:
        link.release()
        sim.stopSimulation()
        wait_until_stopped(sim)
        link.close()
        if world is not None:
            from sim import dynamic_scene
            dynamic_scene.remove(sim)
        sim.setBoolProperty(sim.handle_scene, "realtimeSimulation", original_rt)
    return time.perf_counter() - wall0


def offline_loop(run, spec, *, lidar: bool = False, turn_loss: float = 1.0, dyn: str | None = None):
    tests = str(ROOT / "tests")
    if tests not in sys.path:
        sys.path.insert(0, tests)
    from maze_sim import LIVE_TURN_DEFICIT, MazeSim            # noqa: E402  (test plant)

    deficit = tuple(turn_loss * d for d in LIVE_TURN_DEFICIT) if turn_loss else None
    msim = MazeSim(spec, turn_deficit=deficit)
    world = None
    if dyn:
        from mazes.dynamic import DynamicWorld, build_scenario
        def placer():
            obstacles = build_scenario(spec, dyn, run.planned_gt_route())
            run.attach_dynamic(dyn, obstacles)
            print(f"  dynamic: {', '.join(f'{o.name} on {o.edge[0]}-{o.edge[1]}' for o in obstacles) or 'no edge fits'}")
            return obstacles
        world = DynamicWorld(spec, placer=placer)
        msim.walls = world                                    # sensors, LiDAR and collision see the obstacles
        run.attach_dynamic(dyn, [])
    p = msim.plant
    wall0 = time.perf_counter()
    t, k, th_prev, imu_count = 0.0, 0, p.th, 0
    span = math.radians(240.0)
    angles = [-span / 2 + span * i / (LIDAR_RAYS - 1) for i in range(LIDAR_RAYS)]
    c0 = run.mount.get("scan_centre_in_base") or (0.0, 0.0, 0.0)
    R = run.mount.get("scan_rotation_from_base") or ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    while True:
        p.advance()
        t += 0.01
        k += 1
        if world is not None:
            for name in world.update(t, p.x, p.y, run.goal_running(), p.th):
                run.note_dynamic(name, t)
        msim._check_body()
        w_true = (p.th - th_prev) / 0.01
        th_prev = p.th
        xyzd = None
        if lidar and k % LIDAR_DIVIDER == 0:
            cx, cy, ch = run._scan_centre(p.x, p.y, p.th)
            xyzd = []
            for a in angles:
                d = msim.walls.raycast(cx, cy, ch + a, 5.0)
                d = d if math.isfinite(d) else 5.0
                ps = (d * math.cos(a), d * math.sin(a), 0.0)          # scan frame -> Hokuyo base frame
                xyzd += [sum(R[r][j] * ps[r] for r in range(3)) + c0[j] for j in range(3)] + [d]
        s = SimpleNamespace(
            simulation_time_s=t, state_sequence=k,
            left_joint_position_rad=math.remainder(p.ql, 2 * math.pi),
            right_joint_position_rad=math.remainder(p.qr, 2 * math.pi),
            proximity=msim.snapshot(), collision=msim.collision, visual_collision=False,
            gt_axle_x_m=p.x, gt_axle_y_m=p.y, gt_heading_rad=math.remainder(p.th, 2 * math.pi),
            gt_yaw_rate_rad_s=w_true, imu_count=imu_count, imu_gyro_rad_s=(0.0, 0.0, w_true),
            imu_accel_m_s2=None, lidar_count=k, lidar_xyzd=xyzd)
        imu_count += 1
        cl, cr, done = run.step(s)
        p.ul, p.ur = cl, cr
        if done:
            break
    return time.perf_counter() - wall0
