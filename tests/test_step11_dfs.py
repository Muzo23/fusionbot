"""Step 11 acceptance: DFS exploration over MazeMemory.

Offline maze simulator; "live deficit" = fixed loss of body rotation per
in-place turn w.r.t. the encoders, calibrated on 68 live turns of the step-11
T1/T2/T3 runs (CCW 0.47 deg, CW 0.66 deg), i.e. configuration A odometry drift.
(The earlier proportional slip doubled the loss on U-turns and predicted a
missed T3 ring closure that the live robot did not show.)

    python -m unittest discover -s tests -p "test_step11*" -v
"""

import unittest

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from maze_sim import LIVE_TURN_DEFICIT, MazeSim
from mazes.maze_spec import MAZES
from mazes.topology import evaluate_memory, expected_left_hand_events
from navigation import navigation_config as cfg
from navigation.junction_types import ManeuverDecision, ReferenceKind
from navigation.maze_decision import DfsExplorer, relative_decision
from navigation.maze_memory import Dir, MazeMemory
from navigation.navigation_fsm import NavigationFSM, NavState


def explore(maze_id, deficit_scale, budget=200):
    spec = MAZES[maze_id]
    deficit = tuple(deficit_scale * d for d in LIVE_TURN_DEFICIT) if deficit_scale else None
    sim = MazeSim(spec, turn_deficit=deficit)
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

    def test_t1_tree_with_live_deficit(self):
        """Tree: DFS with left preference = left-hand traversal, then FINISH at
        START. Returns along known corridors are associated topologically, so
        the map stays correct despite the configuration-A drift."""
        sim, fsm, dfs, mem, gt, r = explore("T1", 1.0)
        self._common(sim, fsm, dfs)
        self.assertEqual(dfs.finished_reason, "complete")
        self.assertTrue(r["correct"], r)
        self.assertEqual(r["memory_junctions"], 23)
        exp = expected_left_hand_events(MAZES["T1"], 43)
        got = [(g, s.decision.value) for g, s in zip(gt, dfs.steps)]
        self.assertEqual(got[:43], [(e[0], e[4]) for e in exp])
        self.assertEqual(got[43], ("__START__", "FINISH"))
        self.assertGreater(mem.summary()["inconsistent_edge_associations"], 0)   # drift is visible

    def test_t2_loops_closed_with_live_deficit(self):
        sim, fsm, dfs, mem, gt, r = explore("T2", 1.0)
        self._common(sim, fsm, dfs)
        self.assertTrue(r["correct"], r)
        self.assertEqual(r["memory_junctions"], len(MAZES["T2"].nodes_grid) + 1)
        self.assertEqual(r["edges_correct"], len(MAZES["T2"].edges) + 1)        # every corridor
        self.assertEqual(sum(s.reason == "loop_closure_return" for s in dfs.steps), 2)

    def test_t3_ideal_odometry_closes_the_ring(self):
        sim, fsm, dfs, mem, gt, r = explore("T3", 0)
        self._common(sim, fsm, dfs)
        self.assertTrue(r["correct"], r)
        self.assertEqual(sum(s.reason == "loop_closure_return" for s in dfs.steps), 1)

    def test_t3_live_deficit_closes_the_ring_with_reduced_margin(self):
        """As live (step 12): configuration A closes the 4 x 5.4 m ring, but the
        match distance is already about half of the recognition radius
        (offline 0.18 m, live 0.236 m, radius 0.373 m)."""
        sim, fsm, dfs, mem, gt, r = explore("T3", 1.0)
        self._common(sim, fsm, dfs)
        self.assertTrue(r["correct"], r)
        d = [s.arrival.match_distance_m for s in dfs.steps if s.reason == "loop_closure_return"]
        self.assertEqual(len(d), 1)
        self.assertTrue(0.10 < d[0] < 0.30, d)

    def test_t3_tripled_turn_loss_misses_the_ring(self):
        """Disturbed configuration A (3x the live turn loss, e.g. a slippery
        floor): the odometry error at the end of the ring exceeds the radius, R
        is not recognised, the ring is explored again and junctions are
        duplicated; when the heading error later passes 45 deg the memory no
        longer matches the world and the explorer stops ("inconsistent"), or it
        keeps adding duplicated junctions until the event budget ends ("budget";
        with the turn loss of the grouped robot, 8. 10. 2026)."""
        sim, fsm, dfs, mem, gt, r = explore("T3", 3.0)
        self.assertIs(fsm.state, NavState.FINISHED)                 # graceful, no safety abort
        self.assertFalse(sim.collision)
        self.assertIn(dfs.finished_reason, ("complete", "inconsistent", "budget"))
        self.assertFalse(r["correct"])
        self.assertIn("R", r["duplicates"])           # (false merges may also appear)
        # the ring is never closed correctly: any later "closure" is a false merge
        closures = [s.arrival.junction for s in dfs.steps if s.reason == "loop_closure_return"]
        self.assertTrue(all(j in r["false_merges"] for j in closures), (closures, r["false_merges"]))

    def test_budget_stops_exploration(self):
        sim, fsm, dfs, mem, gt, r = explore("T1", 0, budget=5)
        self.assertIs(fsm.state, NavState.FINISHED)
        self.assertEqual(dfs.finished_reason, "budget")
        self.assertEqual(len(dfs.steps), 6)


if __name__ == "__main__":
    unittest.main()
