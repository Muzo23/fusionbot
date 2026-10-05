"""Step 11 acceptance: DFS exploration over MazeMemory.

Offline maze simulator; "live slip" = the turn slip measured live in step 7
(body turns ~0.6 deg less per 90 deg than odometry says), i.e. configuration A
odometry drift.

    python -m unittest discover -s tests -p "test_step11*" -v
"""

import unittest

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from maze_sim import LIVE_TURN_SLIP, MazeSim
from mazes.maze_spec import MAZES
from mazes.topology import evaluate_memory, expected_left_hand_events
from navigation import navigation_config as cfg
from navigation.junction_types import ManeuverDecision, ReferenceKind
from navigation.maze_decision import DfsExplorer, relative_decision
from navigation.maze_memory import Dir, MazeMemory
from navigation.navigation_fsm import NavigationFSM, NavState


def explore(maze_id, slip, budget=200):
    spec = MAZES[maze_id]
    sim = MazeSim(spec, turn_slip=slip)
    mem = MazeMemory()
    odo = sim.plant.odo
    dfs = DfsExplorer(mem, lambda: (odo.pose.x_m, odo.pose.y_m, odo.pose.theta_rad),
                      start_pose=spec.start_pose, max_events=budget)
    fsm = NavigationFSM(dfs)
    events = sim.run(fsm, max_s=4000)
    gt = []
    for _, (x, y, th), st in events:
        ahead = cfg.D_SIDE_CANONICAL_M if st.observation.kind is ReferenceKind.SIDE_CONFIRMED \
            else cfg.D_DEAD_END_CANONICAL_M
        gt.append(sim.nearest_node(x, y, th, ahead))
    return sim, fsm, dfs, mem, gt, evaluate_memory(mem, spec, gt)


class TestRelative(unittest.TestCase):
    def test_relative_decision(self):
        self.assertIs(relative_decision(Dir.N, Dir.W), ManeuverDecision.LEFT)
        self.assertIs(relative_decision(Dir.N, Dir.N), ManeuverDecision.STRAIGHT)
        self.assertIs(relative_decision(Dir.N, Dir.E), ManeuverDecision.RIGHT)
        self.assertIs(relative_decision(Dir.N, Dir.S), ManeuverDecision.UTURN)
        self.assertIs(relative_decision(Dir.W, Dir.S), ManeuverDecision.LEFT)


class TestExploration(unittest.TestCase):
    def _common(self, sim, fsm, dfs):
        self.assertIs(fsm.state, NavState.FINISHED)
        self.assertFalse(sim.collision)
        self.assertEqual(dfs.steps[-1].arrival.junction, 0)          # stops at START

    def test_t1_tree_with_live_slip(self):
        """Tree: DFS with left preference = left-hand traversal, then FINISH at
        START. Returns along known corridors are associated topologically, so
        the map stays correct despite the configuration-A drift."""
        sim, fsm, dfs, mem, gt, r = explore("T1", LIVE_TURN_SLIP)
        self._common(sim, fsm, dfs)
        self.assertEqual(dfs.finished_reason, "complete")
        self.assertTrue(r["correct"], r)
        self.assertEqual(r["memory_junctions"], 23)
        exp = expected_left_hand_events(MAZES["T1"], 43)
        got = [(g, s.decision.value) for g, s in zip(gt, dfs.steps)]
        self.assertEqual(got[:43], [(e[0], e[4]) for e in exp])
        self.assertEqual(got[43], ("__START__", "FINISH"))
        self.assertGreater(mem.summary()["inconsistent_edge_associations"], 0)   # drift is visible

    def test_t2_loops_closed_with_live_slip(self):
        sim, fsm, dfs, mem, gt, r = explore("T2", LIVE_TURN_SLIP)
        self._common(sim, fsm, dfs)
        self.assertTrue(r["correct"], r)
        self.assertEqual(r["memory_junctions"], len(MAZES["T2"].nodes_grid) + 1)
        self.assertEqual(r["edges_correct"], len(MAZES["T2"].edges) + 1)        # every corridor
        self.assertEqual(sum(s.reason == "loop_closure_return" for s in dfs.steps), 2)

    def test_t3_ideal_odometry_closes_the_ring(self):
        sim, fsm, dfs, mem, gt, r = explore("T3", 0.0)
        self._common(sim, fsm, dfs)
        self.assertTrue(r["correct"], r)
        self.assertEqual(sum(s.reason == "loop_closure_return" for s in dfs.steps), 1)

    def test_t3_live_slip_misses_the_ring_closure(self):
        """Configuration A: after the 4 x 5.4 m ring the odometry error exceeds
        the recognition radius, ROOT is not recognised, the ring is explored a
        second time and junctions are duplicated (the problem fusion must fix)."""
        sim, fsm, dfs, mem, gt, r = explore("T3", LIVE_TURN_SLIP)
        self._common(sim, fsm, dfs)
        self.assertFalse(r["correct"])
        self.assertIn("R", r["duplicates"])
        self.assertEqual(r["false_merges"], [])
        self.assertEqual(sum(s.reason == "loop_closure_return" for s in dfs.steps), 0)

    def test_budget_stops_exploration(self):
        sim, fsm, dfs, mem, gt, r = explore("T1", 0.0, budget=5)
        self.assertIs(fsm.state, NavState.FINISHED)
        self.assertEqual(dfs.finished_reason, "budget")
        self.assertEqual(len(dfs.steps), 6)


if __name__ == "__main__":
    unittest.main()
