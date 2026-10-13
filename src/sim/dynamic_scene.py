"""
Dynamic obstacles in the live CoppeliaSim scene (step 14b).

Before a live run with --dyn the runner creates a small pool of hidden cuboids (Remote API,
simulation stopped), aliases FB_MAZE_dyn_<index>; the lockstep bridge (protocol 2.2) finds them in
sysCall_init and moves them with the poses appended to every command packet. The obstacles are
static and NOT respondable (moved kinematically, they must not push the robot); proximity sensors,
the LiDAR and the bridge's collision check see them like walls. maze_scene removes them with the
maze (prefix FB_MAZE_), the runner removes them after the run.

Which obstacle uses which cuboid: kind and corridor axis (mazes.dynamic gives axis-aligned boxes).
"""

from __future__ import annotations

from mazes import dynamic as dy
from mazes.maze_spec import HALF_FREE_WIDTH_M

PREFIX = "FB_MAZE_dyn_"
HEIGHT_M = 0.30
HIDDEN_Z = -1.0
COLOUR = [1.0, 0.45, 0.0]

# index -> (kind, corridor axis, size x, size y)
_W = 2 * HALF_FREE_WIDTH_M + 0.02
POOL = {
    0: ("crossing", "x", dy.CROSS_SIZE_M, dy.CROSS_SIZE_M),
    1: ("crossing", "y", dy.CROSS_SIZE_M, dy.CROSS_SIZE_M),
    2: ("door", "x", dy.DOOR_THICK_M, _W),
    3: ("door", "y", _W, dy.DOOR_THICK_M),
    4: ("side", "x", dy.SIDE_LEN_M, dy.SIDE_DEPTH_M + 0.01),
    5: ("side", "y", dy.SIDE_DEPTH_M + 0.01, dy.SIDE_LEN_M),
    6: ("permanent", "x", dy.DOOR_THICK_M, _W),
    7: ("permanent", "y", _W, dy.DOOR_THICK_M),
}


def pool_index(o: dy.DynObstacle) -> int:
    axis = "x" if abs(o.ux) > 0.5 else "y"
    return next(i for i, (k, a, _, _) in POOL.items() if k == o.kind and a == axis)


def remove(sim) -> int:
    n = 0
    for i in POOL:
        h = sim.getObject("/" + PREFIX + str(i), {"noError": True})
        if h is not None and h != -1:
            sim.removeObject(h)
            n += 1
    return n


def create(sim, floor_z: float) -> dict[int, int]:
    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError("stop the simulation before creating the dynamic obstacles")
    remove(sim)
    out = {}
    for i, (_, _, sx, sy) in POOL.items():
        h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [sx, sy, HEIGHT_M])
        sim.setObjectAlias(h, PREFIX + str(i))
        sim.setObjectPosition(h, [0.0, 0.0, HIDDEN_Z], sim.handle_world)
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)
        try:
            sim.setShapeColor(h, None, sim.colorcomponent_ambient_diffuse, COLOUR)
        except Exception:                    # noqa: BLE001 - colour is cosmetic
            pass
        out[i] = h
    return out


class LivePoses:
    """Obstacle rectangles of mazes.dynamic -> cuboid poses for the command packet (only changes)."""

    def __init__(self, floor_z: float) -> None:
        self.z = floor_z + 0.5 * HEIGHT_M
        self.sent: dict[int, tuple] = {}

    def updates(self, obstacles, t: float) -> list[tuple[int, float, float, float]]:
        want = {}
        for o in obstacles:
            r = o.rect(t)
            i = pool_index(o)
            want[i] = (0.0, 0.0, HIDDEN_Z) if r is None else (0.5 * (r[0] + r[1]), 0.5 * (r[2] + r[3]), self.z)
        out = []
        for i, p in want.items():
            if self.sent.get(i) != p:
                self.sent[i] = p
                out.append((i, *p))
        return out
