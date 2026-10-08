"""Step 12 acceptance (offline): maze scene generator against a fake CoppeliaSim.

    python -m unittest discover -s tests -p "test_step12*" -v
"""

import math
import unittest

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from core import robot_config as rc
from mazes.maze_geometry import WallMap
from mazes.maze_spec import FREE_WIDTH_M, MAZES, WALL_THICKNESS_M
from sim.maze_scene import PREFIX, generate, scene_maze_id, wall_boxes


def rot(yaw, x, y):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


class FakeSim:
    """Minimal scene: a robot root with a base (yaw offset) and two wheel joints
    at fixed offsets in the root frame, a floor and an old maze."""
    handle_world, handle_scene, handle_all = -1, -12, -2
    simulation_stopped, primitiveshape_cuboid = 0, 3
    shapeintparam_static, shapeintparam_respondable = 3003, 3004

    def __init__(self):
        self.obj = {}            # h -> dict(alias, pos, ori, size, params)
        self.root = self._new("FusionBot", [2.0, 3.0, 0.08], [0, 0, 0.7])
        self.base_off = 0.3      # base yaw = root yaw + base_off
        self.wheel = {"L": (0.05, 0.146), "R": (0.05, -0.146)}
        self._new("Floor", [0, 0, 0], [0, 0, 0])
        for i in range(3):
            self._new(f"{PREFIX}wall_{i:03d}", [9, 9, 0.15], [0, 0, 0])
        self.props = {}

    def _new(self, alias, pos, ori, size=None):
        h = len(self.obj) + 1
        self.obj[h] = {"alias": alias, "pos": list(pos), "ori": list(ori), "size": size, "params": {}}
        return h

    # --- API subset
    def getSimulationState(self): return self.simulation_stopped
    def getObject(self, path): return {rc.ROBOT_PATH: self.root, rc.BASE_PATH: -100,
                                       rc.LEFT_MOTOR_JOINT_PATH: -101, rc.RIGHT_MOTOR_JOINT_PATH: -102}[path]
    def getObjectsInTree(self, *_): return list(self.obj)
    def getObjectAlias(self, h): return self.obj[h]["alias"]
    def removeObject(self, h): del self.obj[h]
    def createPrimitiveShape(self, kind, size): return self._new("Cuboid", [0, 0, 0], [0, 0, 0], list(size))
    def createDummy(self, size): return self._new("Dummy", [0, 0, 0], [0, 0, 0])
    def setObjectAlias(self, h, a): self.obj[h]["alias"] = a
    def setObjectInt32Param(self, h, p, v): self.obj[h]["params"][p] = v
    def setStringProperty(self, h, k, v): self.props[k] = v
    def getStringProperty(self, h, k): return self.props[k]

    def getObjectPosition(self, h, ref):
        if h in (-101, -102):
            r = self.obj[self.root]
            ox, oy = self.wheel["L" if h == -101 else "R"]
            dx, dy = rot(r["ori"][2], ox, oy)
            return [r["pos"][0] + dx, r["pos"][1] + dy, 0.053]
        return list(self.obj[h]["pos"])

    def setObjectPosition(self, h, p, ref): self.obj[h]["pos"] = list(p)

    def getObjectOrientation(self, h, ref):
        if h == -100:
            o = self.obj[self.root]["ori"]
            return [0, 0, o[2] + self.base_off]
        return list(self.obj[h]["ori"])

    def setObjectOrientation(self, h, o, ref): self.obj[h]["ori"] = list(o)


class TestGeometry(unittest.TestCase):
    def test_walls_leave_exact_free_width(self):
        """Walls stand outside the free space: a ray across the START corridor
        hits the inner faces exactly FREE_WIDTH_M apart."""
        spec = MAZES["T1"]
        x, y, _ = spec.start_pose
        boxes = wall_boxes(spec)
        inner = []
        for _, sx, sy, cx, cy in boxes:
            if sx == WALL_THICKNESS_M and cy - sy / 2 <= y <= cy + sy / 2:
                inner.append(cx - sx / 2 if cx > x else cx + sx / 2)
        inner = sorted(inner, key=lambda v: abs(v - x))[:2]
        self.assertAlmostEqual(abs(inner[0] - inner[1]), FREE_WIDTH_M, places=9)

    def test_box_count_matches_segments(self):
        for k, spec in MAZES.items():
            self.assertEqual(len(wall_boxes(spec)), len(WallMap(spec).segments))


class TestGenerate(unittest.TestCase):
    def test_generate_t3(self):
        sim = FakeSim()
        r = generate(sim, MAZES["T3"])
        aliases = [o["alias"] for o in sim.obj.values()]
        self.assertIn("Floor", aliases)                                   # untouched
        self.assertIn("FusionBot", aliases)
        walls = [o for o in sim.obj.values() if o["alias"].startswith(f"{PREFIX}wall_")]
        self.assertEqual(r["removed"], 3)                                 # only the old maze
        self.assertEqual(len(walls), len(wall_boxes(MAZES["T3"])))
        self.assertTrue(all(o["params"][sim.shapeintparam_static] == 1 and
                            o["params"][sim.shapeintparam_respondable] == 1 for o in walls))
        self.assertEqual(scene_maze_id(sim), "T3")
        x, y, h = r["robot"]
        sx, sy, sh = MAZES["T3"].start_pose
        self.assertAlmostEqual(x, sx, places=6)
        self.assertAlmostEqual(y, sy, places=6)
        self.assertAlmostEqual(h, sh, places=6)

    def test_regenerate_replaces_maze(self):
        sim = FakeSim()
        generate(sim, MAZES["T1"])
        n1 = sum(o["alias"].startswith(PREFIX) for o in sim.obj.values())
        r = generate(sim, MAZES["T2"], markers=False)
        self.assertEqual(r["removed"], n1)
        self.assertEqual(sum(o["alias"].startswith(PREFIX) for o in sim.obj.values()),
                         len(wall_boxes(MAZES["T2"])))
        self.assertEqual(scene_maze_id(sim), "T2")


if __name__ == "__main__":
    unittest.main()
