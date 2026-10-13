"""Step 13c (offline): real micromouse mazes -> FusionBot MazeSpec.

    python -m unittest discover -s tests -p "test_step13c*" -v
"""

import unittest

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from mazes.maze_geometry import WallMap
from mazes.maze_spec import MAZES
from mazes.micromouse import REAL_MAZES, metrics, parse, to_spec

README_4X4 = """o---o---o---o---o
| G |           |
o   o   o   o---o
|       |       |
o---o---o---o   o
|               |
o   o---o---o   o
| S |           |
o---o---o---o---o
"""


class TestParse(unittest.TestCase):
    def test_readme_example(self):
        m = parse(README_4X4)
        self.assertEqual((m.width, m.height), (4, 4))
        self.assertEqual(m.start, (0, 0))
        self.assertEqual(m.goals, [(0, 3)])
        self.assertEqual(m.open[(0, 0)], {"N"})
        self.assertEqual(m.open[(0, 1)], {"S", "E"})
        self.assertEqual(m.open[(3, 2)], {"S", "W"})

    def test_spec_from_example(self):
        s = to_spec(parse(README_4X4), "X", "readme")
        self.assertEqual(s.root, "R")
        self.assertEqual(s.nodes_grid["R"], (0, 0))           # first node north of the start cell
        self.assertNotIn("c0_0", s.nodes_grid)                 # start cell = approach, not a node
        self.assertEqual(s.expected_cycle_count, len(s.edges) - len(s.nodes_grid) + 1)
        WallMap(s)                                             # geometry builds

    def test_rotation_when_start_opens_east(self):
        rot = README_4X4.replace("| S |", "| S  ").replace("o   o---o---o   o\n| S", "o---o---o---o   o\n| S")
        s = to_spec(parse(rot), "Y", "rotated")
        self.assertIn("R", s.nodes_grid)


class TestRegistry(unittest.TestCase):
    def test_real_mazes_registered(self):
        for mid in REAL_MAZES:
            self.assertIn(mid, MAZES)
            m = metrics(MAZES[mid])
            self.assertGreater(m["nodes"], 10)
        self.assertEqual(metrics(MAZES["M5"])["cycles"], 14)
        self.assertEqual(metrics(MAZES["M6"])["extent_cells"], (16, 16))


if __name__ == "__main__":
    unittest.main()
