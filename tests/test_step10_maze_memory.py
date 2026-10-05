"""Step 10 acceptance: topological maze memory.

    python -m unittest discover -s tests -p "test_step10*" -v
"""

import csv
import math
import unittest
from pathlib import Path

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from maze_sim import MazeSim
from mazes.maze_spec import MAZES
from mazes.topology import evaluate_memory
from navigation import navigation_config as cfg
from navigation.junction_types import JunctionObservation, ManeuverDecision, ReferenceKind
from navigation.maze_memory import (
    DECISION_TURNS, JUNCTION_MATCH_RADIUS_M, Dir, MazeMemory, MemoryRecorder, opposite, rotate, snap_heading,
)
from navigation.navigation_fsm import NavigationFSM
from navigation.simple_policies import LeftHandPolicy

D = cfg.D_SIDE_CANONICAL_M
DATA = Path(__file__).resolve().parent / "data"


def obs(kind=ReferenceKind.SIDE_CONFIRMED, l=False, s=False, r=False):
    return JunctionObservation(kind, l, s, r, None, 0.0)


def _rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


class TestDirections(unittest.TestCase):
    def test_snap_and_rotate(self):
        self.assertIs(snap_heading(0.0), Dir.E)
        self.assertIs(snap_heading(math.radians(44)), Dir.E)
        self.assertIs(snap_heading(math.radians(46)), Dir.N)
        self.assertIs(snap_heading(math.radians(-91)), Dir.S)
        self.assertIs(snap_heading(math.radians(181)), Dir.W)
        self.assertIs(rotate(Dir.N, 1), Dir.W)
        self.assertIs(rotate(Dir.N, -1), Dir.E)
        self.assertIs(opposite(Dir.E), Dir.W)


class TestMemory(unittest.TestCase):
    def setUp(self):
        self.m = MazeMemory()
        self.m.start(0.0, -1.4226, math.pi / 2)          # heading N

    def test_new_junction_centre_and_exits(self):
        a = self.m.observe(obs(l=True, s=True, r=True), 0.0, -D, math.pi / 2)
        self.assertTrue(a.is_new)
        self.assertEqual(a.exits, frozenset({Dir.W, Dir.N, Dir.E, Dir.S}))
        self.assertAlmostEqual(a.centre_xy[1], 0.0, places=6)
        self.assertEqual(a.arrival_exit, Dir.S)
        e = self.m.edges[a.edge]
        self.assertEqual((e.a, e.dir_a, e.b, e.dir_b, e.traversals), (0, Dir.N, a.junction, Dir.S, 1))

    def test_recognition_within_radius_and_same_exits(self):
        a = self.m.observe(obs(l=True), 0.0, -D, math.pi / 2)         # exits W, S
        self.m.depart(Dir.W)
        # come back from the west, heading E: exits must be W (back) and S (right)
        b = self.m.observe(obs(r=True), -D + 0.2, 0.05, 0.0)
        self.assertFalse(b.is_new)
        self.assertEqual(b.junction, a.junction)
        self.assertAlmostEqual(b.match_distance_m, math.hypot(0.2, 0.05), places=6)

    def test_outside_radius_is_new(self):
        a = self.m.observe(obs(l=True), 0.0, -D, math.pi / 2)
        self.m.depart(Dir.W)
        b = self.m.observe(obs(r=True), -D + JUNCTION_MATCH_RADIUS_M + 0.01, 0.0, 0.0)
        self.assertTrue(b.is_new)
        self.assertNotEqual(b.junction, a.junction)

    def test_structure_mismatch_rejected(self):
        a = self.m.observe(obs(l=True), 0.0, -D, math.pi / 2)         # exits W, S
        self.m.depart(Dir.W)
        b = self.m.observe(obs(l=True, r=True), -D, 0.0, 0.0)         # exits W, N, S
        self.assertTrue(b.is_new)
        self.assertEqual(b.rejected_structure, (a.junction,))

    def test_backtrack_reuses_edge(self):
        a = self.m.observe(obs(l=True), 0.0, -D, math.pi / 2)
        e1 = self.m.depart(Dir.W)
        d = self.m.observe(obs(kind=ReferenceKind.DEAD_END), -1.0, 0.0, math.pi)
        e2 = self.m.depart(Dir.E)                                          # U-turn
        self.assertEqual(e1, e2)
        b = self.m.observe(obs(r=True), -D, 0.0, 0.0)
        self.assertEqual(b.junction, a.junction)
        self.assertEqual(self.m.edges[e1].traversals, 2)
        self.assertFalse(b.loop_closure)
        self.assertEqual(self.m.neighbour(a.junction, Dir.W), d.junction)
        self.assertEqual(self.m.unexplored_exits(d.junction), [])

    def test_depart_unknown_exit_raises(self):
        self.m.observe(obs(l=True), 0.0, -D, math.pi / 2)
        with self.assertRaises(ValueError):
            self.m.depart(Dir.N)


class TestOfflineMazes(unittest.TestCase):
    """FSM + left-hand policy in the offline simulator, memory on the plant's odometry."""

    def _run(self, maze_id, n):
        spec = MAZES[maze_id]
        sim = MazeSim(spec)
        mem = MazeMemory()
        mem.start(*spec.start_pose)
        odo = sim.plant.odo

        def pose():
            p = odo.pose
            return p.x_m, p.y_m, p.theta_rad
        fsm = NavigationFSM(MemoryRecorder(LeftHandPolicy(max_events=n), mem, pose))
        events = sim.run(fsm, max_s=1500)
        gt = []
        for _, (x, y, th), st in events:
            ahead = cfg.D_SIDE_CANONICAL_M if st.observation.kind is ReferenceKind.SIDE_CONFIRMED \
                else cfg.D_DEAD_END_CANONICAL_M
            gt.append(sim.nearest_node(x, y, th, ahead))
        return mem, evaluate_memory(mem, spec, gt)

    def test_t1(self):
        mem, r = self._run("T1", 44)
        self.assertTrue(r["correct"], r)
        self.assertEqual(r["memory_junctions"], len(MAZES["T1"].nodes_grid) + 1)    # + START
        self.assertEqual(r["edges_correct"], len(MAZES["T1"].edges) + 1)

    def test_t2_loop_closure(self):
        mem, r = self._run("T2", 40)
        self.assertTrue(r["correct"], r)
        self.assertGreaterEqual(mem.summary()["loop_closures"], 1)

    def test_t3_loop_closure(self):
        mem, r = self._run("T3", 30)
        self.assertTrue(r["correct"], r)
        self.assertGreaterEqual(mem.summary()["loop_closures"], 1)


class TestLiveReplay(unittest.TestCase):
    """The 45 junction events of the live step-9 run (T1, left-hand) with the
    pose at each event: GT and odometry reconstructed from the live log."""

    def _replay(self, use_gt, edge_association=True):
        spec = MAZES["T1"]
        ev = _rows(DATA / "step09_live_events.csv")
        ps = _rows(DATA / "step09_live_event_poses.csv")
        mem = MazeMemory(edge_association=edge_association)
        mem.start(*spec.start_pose)
        gt = []
        for e, p in zip(ev, ps):
            o = JunctionObservation(ReferenceKind(e["kind"]), e["left"] == "True", e["straight"] == "True",
                                    e["right"] == "True", None, 0.0)
            pre = "gt" if use_gt else "odo"
            a = mem.observe(o, float(p[f"{pre}_x"]), float(p[f"{pre}_y"]), float(p[f"{pre}_th"]))
            gt.append(e["node"])
            if e["decision"] in DECISION_TURNS:
                mem.depart(rotate(a.heading, DECISION_TURNS[e["decision"]]))
        return mem, evaluate_memory(mem, spec, gt)

    def test_with_gt_pose_memory_is_exact(self):
        for ea in (False, True):
            _, r = self._replay(True, edge_association=ea)
            self.assertTrue(r["correct"], r)

    def test_metric_only_recognition_fails_at_the_end(self):
        """Step-10 variant (every arrival by position), configuration A on real
        data: recognition holds while the odometry error stays below the radius,
        the match distance grows with time, and the return to ROOT after 500 s
        (heading drift 11 deg) is not recognised -> ROOT and START duplicated."""
        mem, r = self._replay(False, edge_association=False)
        self.assertFalse(r["correct"])
        self.assertEqual(r["duplicates"], ["R", "__START__"])
        self.assertEqual(r["false_merges"], [])
        first_dup = next(i for i, a in enumerate(mem.log) if a.is_new and a.junction > 22)
        self.assertEqual(first_dup, 42)
        self.assertGreater(mem.log[41].match_distance_m, 0.30)           # S, just before

    def test_edge_association_keeps_tree_map_correct(self):
        """Step-11 refinement: returns along known corridors are associated
        topologically. The same live odometry gives a correct T1 map; the drift
        stays visible as inconsistent (distance > radius) associations."""
        mem, r = self._replay(False)
        self.assertTrue(r["correct"], r)
        bad = [i for i, a in enumerate(mem.log) if a.inconsistent]
        self.assertEqual(bad, [42, 43, 44])
        self.assertEqual(mem.summary()["edge_associations"], 23)


class TestGraphExport(unittest.TestCase):
    def test_graph_dict_fields(self):
        from mapping.graph_map import graph_to_dict
        m = MazeMemory()
        m.start(0.0, -1.4226, math.pi / 2)
        a = m.observe(obs(l=True), 0.0, -D, math.pi / 2)
        m.depart(Dir.W)
        g = graph_to_dict(m)
        self.assertEqual([j["id"] for j in g["junctions"]], [0, a.junction])
        self.assertEqual(g["junctions"][1]["exits"], ["S", "W"])
        self.assertEqual(len(g["edges"]), 2)                 # START-R closed, R-W open
        self.assertIsNone(g["edges"][1]["b"])
        self.assertEqual(len(g["events"]), 1)


if __name__ == "__main__":
    unittest.main()
