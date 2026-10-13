"""Step 12f (offline): finding the IMU / LiDAR housings when the robot parts are grouped.

    python -m unittest discover -s tests -p "test_step12f*" -v
"""

import unittest

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from sim.robot_parts import find_part, pick_component, size_error


class FakeSim:
    """Shapes with axis-aligned boxes; compounds ungroup into their components (on copies)."""
    sceneobject_shape, sceneobject_dummy, handle_all, handle_world, handle_scene = 0, 1, -2, -1, -12

    def __init__(self, shapes):
        # shapes: {handle: (alias, centre, size, [component (alias, centre, size)])}
        self.objs = {h: (a, c, s, comps, True) for h, (a, c, s, comps) in shapes.items()}
        self.next = 1000
        self.removed = []
        for i, ax in enumerate("xyz"):
            setattr(self, f"objfloatparam_objbbox_min_{ax}", ("min", i))
            setattr(self, f"objfloatparam_objbbox_max_{ax}", ("max", i))

    def getObjectsInTree(self, root, kind, opt):
        if root == self.handle_scene:
            return list(self.objs)
        return [h for h, o in self.objs.items() if o[4]]

    def getObjectType(self, h):
        return self.sceneobject_shape

    def getObjectAlias(self, h, *a):
        return self.objs[h][0]

    def getObjectMatrix(self, h, rel):
        c = self.objs[h][1]
        return [1, 0, 0, c[0], 0, 1, 0, c[1], 0, 0, 1, c[2]]

    def getObjectFloatParam(self, h, p):
        kind, i = p
        s = self.objs[h][2][i]
        return -s / 2 if kind == "min" else s / 2

    def copyPasteObjects(self, hs, opt):
        out = []
        for h in hs:
            self.next += 1
            a, c, s, comps, _ = self.objs[h]
            self.objs[self.next] = (a, c, s, comps, False)       # copies are not in the robot tree
            out.append(self.next)
        return out

    def ungroupShape(self, h):
        a, c, s, comps, _ = self.objs[h]
        if not comps:
            return [h]
        del self.objs[h]
        out = []
        for ca, cc, cs in comps:
            self.next += 1
            self.objs[self.next] = (ca, cc, cs, [], False)
            out.append(self.next)
        return out

    def removeObject(self, h):
        self.removed.append(h)
        self.objs.pop(h, None)


IMU = ("Shape", (0.07, 0.0, 0.18), (0.064, 0.062, 0.019))
LIDAR = ("Shape", (0.18, 0.0, 0.185), (0.075, 0.075, 0.045))
POST = ("Shape", (0.0, 0.08, 0.13), (0.03, 0.03, 0.025))
PLATE = ("Shape", (0.12, 0.0, 0.16), (0.30, 0.22, 0.016))


class TestRobotParts(unittest.TestCase):
    def test_size_error_is_orientation_independent(self):
        self.assertAlmostEqual(size_error((0.019, 0.064, 0.062), (0.062, 0.064, 0.019)), 0.0)

    def test_separate_shape_is_used_directly(self):
        sim = FakeSim({1: ("IMU_phy", (0.07, 0.0, 0.18), (0.062, 0.064, 0.019), [])})
        p = find_part(sim, 0, "imu", 0.0)
        self.assertEqual(p.method, "shape")

    def test_housings_found_inside_the_grouped_base(self):
        sim = FakeSim({1: ("mainBase_phy", (0.1, 0.0, 0.1), (0.49, 0.37, 0.25), [PLATE, POST, IMU, LIDAR]),
                       2: ("leftWheel_phy", (0.0, 0.15, 0.05), (0.106, 0.106, 0.034), [])})
        n_before = len(sim.objs)
        imu = find_part(sim, 0, "imu", 0.0)
        self.assertEqual(imu.method, "component of mainBase_phy")
        self.assertEqual(imu.centre, (0.07, 0.0, 0.18))
        lid = find_part(sim, 0, "lidar", 0.0)
        self.assertEqual(lid.centre, (0.18, 0.0, 0.185))
        self.assertEqual(len(sim.objs), n_before)               # all copies removed, robot untouched
        self.assertTrue(all(o[4] for o in sim.objs.values()))

    def test_low_parts_of_the_right_size_are_ignored(self):
        low = ("Shape", (0.1, 0.0, 0.05), (0.062, 0.064, 0.019))
        self.assertIsNone(pick_component([low], (0.062, 0.064, 0.019), 0.0, 0.12))

    def test_not_found_raises(self):
        sim = FakeSim({1: ("mainBase_phy", (0.1, 0.0, 0.1), (0.49, 0.37, 0.25), [PLATE, POST])})
        with self.assertRaises(RuntimeError):
            find_part(sim, 0, "imu", 0.0)


if __name__ == "__main__":
    unittest.main()
