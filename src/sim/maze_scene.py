"""
Maze scene generator (step 12): builds T1/T2/T3 in the open CoppeliaSim scene.

    python -m sim.maze_scene T2            (from src/, simulation STOPPED)
    python -m sim.maze_scene T3 --no-markers

What it does (Remote API only, simulation must be stopped):
  1. removes ONLY objects whose alias starts with FB_MAZE_ (walls / markers of a
     previously generated maze; floor, robot and its scripts stay untouched);
  2. builds one static, respondable cuboid per boundary wall segment from
     mazes.maze_geometry.boundary_walls (the same geometry the offline simulator
     and the evaluation use). Aliases FB_MAZE_wall_NNN: the lockstep bridge puts
     exactly those into its collision set when the simulation starts;
  3. optional dummies FB_MAZE_node_<name>, FB_MAZE_start (visual only);
  4. places the robot so that the wheel-axle midpoint is at the START pose,
     heading NORTH (wheel joints keep their angles);
  5. stores the maze id in the scene (customData.fusionbotMazeId) so the live
     checks can verify they evaluate against the right maze.

Wall convention: walls are WALL_THICKNESS_M thick and stand OUTSIDE the free
space, so the free corridor width is exactly FREE_WIDTH_M (identical to the
FusionBot2 generator that built the scene used in steps 5-11).

The scene is modified in memory; save it (Ctrl+S) only if you want to keep it.
Regenerating takes a second, so one .ttt file serves every maze.
"""

from __future__ import annotations

import argparse
import math

from core import robot_config as rc
from mazes.maze_geometry import boundary_walls
from mazes.maze_spec import MAZES, WALL_THICKNESS_M, MazeSpec

PREFIX = "FB_MAZE_"
WALL_HEIGHT_M = 0.30
MAZE_ID_PROPERTY = "customData.fusionbotMazeId"


def wall_boxes(spec: MazeSpec):
    """[(alias, size_x, size_y, centre_x, centre_y)] - pure geometry, testable offline."""
    t = WALL_THICKNESS_M
    out = []
    for i, (axis, s, e, fixed, side) in enumerate(boundary_walls(spec)):
        length, mid = e - s, 0.5 * (s + e)
        if axis == "V":
            cx = fixed - 0.5 * t if side == "L" else fixed + 0.5 * t
            out.append((f"{PREFIX}wall_{i:03d}", t, length, cx, mid))
        else:
            cy = fixed - 0.5 * t if side == "B" else fixed + 0.5 * t
            out.append((f"{PREFIX}wall_{i:03d}", length, t, mid, cy))
    return out


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def remove_generated(sim) -> int:
    n = 0
    for h in reversed(list(sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0))):
        try:
            alias = str(sim.getObjectAlias(h))
        except Exception:                  # noqa: BLE001 - already removed with its parent
            continue
        if alias.startswith(PREFIX) or alias.startswith("DFS9_"):
            try:
                sim.removeObject(h)
                n += 1
            except Exception:              # noqa: BLE001
                pass
    return n


def _axle(sim, lj, rj):
    a, b = sim.getObjectPosition(lj, sim.handle_world), sim.getObjectPosition(rj, sim.handle_world)
    return [0.5 * (a[i] + b[i]) for i in range(3)]


def place_robot(sim, x: float, y: float, heading: float) -> tuple[float, float, float]:
    """Axle midpoint to (x, y), physical heading to `heading`. Returns the achieved (x, y, heading)."""
    robot, base = sim.getObject(rc.ROBOT_PATH), sim.getObject(rc.BASE_PATH)
    lj, rj = sim.getObject(rc.LEFT_MOTOR_JOINT_PATH), sim.getObject(rc.RIGHT_MOTOR_JOINT_PATH)
    root_z = sim.getObjectPosition(robot, sim.handle_world)[2]
    # rotate the robot root about z so that the physical heading equals `heading`
    for _ in range(2):
        base_yaw = sim.getObjectOrientation(base, sim.handle_world)[2]
        err = _wrap(heading - (base_yaw + rc.BASE_YAW_TO_PHYSICAL_HEADING_RAD))
        o = sim.getObjectOrientation(robot, sim.handle_world)
        sim.setObjectOrientation(robot, [o[0], o[1], o[2] + err], sim.handle_world)
    # translate so the axle midpoint lands on (x, y)
    for _ in range(2):
        ax = _axle(sim, lj, rj)
        p = sim.getObjectPosition(robot, sim.handle_world)
        sim.setObjectPosition(robot, [p[0] + (x - ax[0]), p[1] + (y - ax[1]), root_z], sim.handle_world)
    ax = _axle(sim, lj, rj)
    h = _wrap(sim.getObjectOrientation(base, sim.handle_world)[2] + rc.BASE_YAW_TO_PHYSICAL_HEADING_RAD)
    if math.hypot(ax[0] - x, ax[1] - y) > 1e-5 or abs(_wrap(h - heading)) > 1e-5:
        raise RuntimeError(f"robot placement failed: axle ({ax[0]:.6f}, {ax[1]:.6f}), heading {math.degrees(h):.4f} deg")
    return ax[0], ax[1], h


def generate(sim, spec: MazeSpec, *, markers: bool = True, place: bool = True) -> dict:
    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError("stop the simulation before generating a maze")
    lj, rj = sim.getObject(rc.LEFT_MOTOR_JOINT_PATH), sim.getObject(rc.RIGHT_MOTOR_JOINT_PATH)
    wheel_z = _axle(sim, lj, rj)[2]
    floor_z = wheel_z - rc.WHEEL_RADIUS_M
    removed = remove_generated(sim)
    walls = []
    for alias, sx, sy, cx, cy in wall_boxes(spec):
        h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [sx, sy, WALL_HEIGHT_M])
        sim.setObjectAlias(h, alias)
        sim.setObjectPosition(h, [cx, cy, floor_z + 0.5 * WALL_HEIGHT_M], sim.handle_world)
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 1)
        walls.append(h)
    if markers:
        for name, (x, y) in list(spec.nodes_m.items()) + [("start", spec.start_pose[:2])]:
            d = sim.createDummy(0.025)
            sim.setObjectAlias(d, f"{PREFIX}node_{name}" if name != "start" else f"{PREFIX}start")
            sim.setObjectPosition(d, [x, y, wheel_z], sim.handle_world)
    placed = place_robot(sim, *spec.start_pose) if place else None
    try:
        sim.setStringProperty(sim.handle_scene, MAZE_ID_PROPERTY, spec.maze_id)
    except Exception:                       # noqa: BLE001 - older API: not fatal
        pass
    return {"maze": spec.maze_id, "removed": removed, "walls": len(walls), "robot": placed}


def scene_maze_id(sim) -> str | None:
    """Maze id stored by generate(), or None (scene built by the old generator / API missing)."""
    try:
        v = sim.getStringProperty(sim.handle_scene, MAZE_ID_PROPERTY)
        return str(v) if v else None
    except Exception:                       # noqa: BLE001
        return None


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate a FusionBot maze in the open CoppeliaSim scene.")
    ap.add_argument("maze", choices=sorted(MAZES))
    ap.add_argument("--no-markers", action="store_true")
    ap.add_argument("--no-robot", action="store_true", help="do not move the robot to START")
    args = ap.parse_args()
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    sim = RemoteAPIClient().require("sim")
    spec = MAZES[args.maze]
    r = generate(sim, spec, markers=not args.no_markers, place=not args.no_robot)
    print(f"\nmaze {spec.maze_id} ({spec.name})")
    print(f"removed previous FB_MAZE_ objects : {r['removed']}")
    print(f"walls built                       : {r['walls']}")
    print(f"nodes / edges / cycles            : {len(spec.nodes_grid)} / {len(spec.edges)} / {spec.expected_cycle_count}")
    if r["robot"]:
        x, y, h = r["robot"]
        print(f"robot axle at START               : ({x:+.6f}, {y:+.6f}) m, heading {math.degrees(h):.4f} deg")
    print("simulation NOT started. Ctrl+S in CoppeliaSim only if you want to keep this maze in the .ttt")


if __name__ == "__main__":
    main()
