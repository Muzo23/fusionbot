"""Step 14b (offline): dynamic obstacles in the goal run.

    python -m unittest discover -s tests -p "test_step14b*" -v
"""

import json
import math
import unittest

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from mazes import dynamic as dy
from mazes.maze_spec import HALF_FREE_WIDTH_M, MAZES
from navigation import navigation_config as cfg
from navigation.junction_types import ManeuverDecision
from navigation.maze_memory import Dir, MazeMemory
from navigation.mission import shortest_path
from navigation.navigation_fsm import NavigationFSM, NavState, Phase
from test_step09_fsm import snap, tick


class TestObstacles(unittest.TestCase):
    def _o(self, kind):
        return dy.DynObstacle("X", kind, ("a", "b"), 0.0, 0.0, 1.0, 0.0, 1.2)

    def test_not_present_before_trigger(self):
        self.assertIsNone(self._o("door").rect(5.0))

    def test_door_closes_the_corridor_then_opens(self):
        o = self._o("door")
        o.triggered_t = 0.0
        x0, x1, y0, y1 = o.rect(1.0)
        self.assertAlmostEqual(x1 - x0, dy.DOOR_THICK_M)
        self.assertGreater(y1 - y0, 2 * HALF_FREE_WIDTH_M)
        self.assertIsNone(o.rect(dy.DOOR_S + 0.1))
        p = self._o("permanent")
        p.triggered_t = 0.0
        self.assertIsNotNone(p.rect(1e4))

    def test_crossing_moves_across_and_leaves(self):
        o = self._o("crossing")
        o.triggered_t = 0.0
        ys = [0.5 * (r[2] + r[3]) for r in (o.rect(t) for t in (0.0, 2.0, 4.0))]
        self.assertTrue(ys[0] < ys[1] < ys[2])
        self.assertIsNone(o.rect(20.0))

    def test_side_object_at_one_wall(self):
        o = self._o("side")
        o.triggered_t = 0.0
        x0, x1, y0, y1 = o.rect(1.0)
        self.assertAlmostEqual(y0, HALF_FREE_WIDTH_M - dy.SIDE_DEPTH_M, places=6)
        self.assertGreater(y1, HALF_FREE_WIDTH_M)

    def test_trigger_needs_corridor_and_heading(self):
        o = self._o("door")
        self.assertFalse(o.maybe_trigger(0.0, -1.0, 0.5, 0.0))            # parallel corridor behind the wall
        self.assertFalse(o.maybe_trigger(0.0, -1.0, 0.0, math.pi))        # driving away
        self.assertFalse(o.maybe_trigger(0.0, -1.0, 0.0, math.pi / 2))    # turning in the junction
        self.assertTrue(o.maybe_trigger(3.0, -1.0, 0.0, 0.05))
        self.assertEqual(o.triggered_t, 3.0)

    def test_world_raycast_and_free(self):
        spec = MAZES["T1"]
        o = dy.build_scenario(spec, "DYN2")[0]
        w = dy.DynamicWorld(spec, [o])
        ang = math.atan2(o.uy, o.ux)
        x, y = o.cx - 0.8 * o.ux, o.cy - 0.8 * o.uy
        free_ray = w.raycast(x, y, ang, 5.0)
        w.update(1.0, x, y, True, ang)
        self.assertEqual(o.triggered_t, 1.0)
        self.assertAlmostEqual(w.raycast(x, y, ang, 5.0), 0.8 - 0.5 * dy.DOOR_THICK_M, places=6)
        self.assertLess(w.raycast(x, y, ang, 5.0), free_ray)
        self.assertFalse(w.is_free(o.cx, o.cy))
        self.assertAlmostEqual(w.distance_to_walls(x, y), min(0.8 - 0.5 * dy.DOOR_THICK_M,
                                                               w.walls.distance_to_walls(x, y)), places=6)

    def test_placement(self):
        for m in ("M2", "M3", "M4"):
            spec = MAZES[m]
            obs = dy.build_scenario(spec, "ALL")
            self.assertEqual([o.name for o in obs], ["DYN3", "DYN1", "DYN2", "DYN4"])
            self.assertEqual(len({o.edge for o in obs}), 4)
            p = obs[-1]
            self.assertTrue(dy._connected_without(spec, *p.edge))
            self.assertGreaterEqual(dy._edge_len(spec, *p.edge), dy.PERM_MIN_EDGE_M)
            a, b = spec.nodes_m[p.edge[0]], spec.nodes_m[p.edge[1]]
            self.assertGreaterEqual(math.hypot(p.cx - a[0], p.cy - a[1]), dy.PERM_POS_FROM_START_M - 1e-9)
            self.assertGreaterEqual(math.hypot(p.cx - b[0], p.cy - b[1]), dy.PERM_FAR_CLEAR_M - 1e-9)
            o2 = dy.DynObstacle.from_dict(json.loads(json.dumps(p.to_dict())))
            self.assertEqual(o2, p)


class TestMemory(unittest.TestCase):
    def test_turn_back_blocks_the_edge_and_returns(self):
        from navigation.junction_types import JunctionObservation, ReferenceKind
        m = MazeMemory()
        m.start(0.0, 0.0, math.pi / 2)
        obs = JunctionObservation(ReferenceKind.SIDE_CONFIRMED, True, True, False, None, 1.0)
        a = m.observe(obs, 0.0, 0.5, math.pi / 2)                      # junction 1 at (0, 1.03)
        eid = m.depart(Dir.N)
        self.assertEqual(m.turn_back(), eid)
        self.assertTrue(m.edges[eid].blocked)
        back = JunctionObservation(ReferenceKind.SIDE_CONFIRMED, False, True, True, None, 3.0)
        r = m.observe(back, 0.0, 1.6, -math.pi / 2)
        self.assertEqual((r.junction, r.association, r.arrival_exit), (a.junction, "RETURN", Dir.N))
        self.assertFalse(r.inconsistent)

    def test_planning_avoids_blocked_edges(self):
        from navigation.maze_memory import Edge, Junction
        m = MazeMemory()
        m.junctions = {i: Junction(i, x, y, "x", frozenset()) for i, (x, y) in enumerate(((0, 0), (1, 0), (1, 1), (0, 1)))}
        for i, (a, da, b, db) in enumerate(((0, Dir.E, 1, Dir.W), (1, Dir.N, 2, Dir.S), (0, Dir.N, 3, Dir.S), (3, Dir.E, 2, Dir.W))):
            m.edges[i] = Edge(i, a, da, b, db)
            m.junctions[a].edge_at[da] = i
            m.junctions[b].edge_at[db] = i
        path, L = shortest_path(m, 0, 1)
        self.assertAlmostEqual(L, 1.0)
        m.edges[0].blocked = True
        path, L = shortest_path(m, 0, 1)
        self.assertAlmostEqual(L, 3.0)


class Provider:
    """Goal-run stand-in: the map allows the FRONT reading down to 1.0 m."""
    def __init__(self, expect=1.0, on_blocked=ManeuverDecision.UTURN):
        self.expect, self.ret, self.blocked_calls = expect, on_blocked, 0

    def decide(self, obs):
        raise AssertionError("no junction in this test")

    def front_expected_min(self, odo_s):
        return self.expect

    def on_blocked(self, odo_s):
        self.blocked_calls += 1
        return self.ret


class TestFsm(unittest.TestCase):
    def setUp(self):
        tick.__defaults__[-1][0] = 0.0

    def _stopped_for_obstacle(self, prov):
        fsm = NavigationFSM(prov)
        tick(fsm, snap())
        for i in range(cfg.OBSTACLE_CONFIRM_SAMPLES):
            st = tick(fsm, snap(front=0.20))
        self.assertIs(st.state, NavState.BLOCKED)
        self.assertEqual(st.v_ref_m_s, 0.0)
        for _ in range(25):
            st = tick(fsm, snap(front=0.20), ds=0, counts=(0, 0))
        self.assertIs(st.phase, Phase.WAIT)
        return fsm

    def test_expected_wall_is_not_an_obstacle(self):
        fsm = NavigationFSM(Provider(expect=0.30))
        tick(fsm, snap())
        for _ in range(10):
            st = tick(fsm, snap(front=0.20))
        self.assertIsNot(st.state, NavState.BLOCKED)

    def test_no_expectation_no_obstacle(self):
        fsm = NavigationFSM(Provider(expect=None))
        tick(fsm, snap())
        for _ in range(10):
            st = tick(fsm, snap(front=0.20))
        self.assertIsNot(st.state, NavState.BLOCKED)

    def test_single_noisy_reading_ignored(self):
        fsm = NavigationFSM(Provider())
        tick(fsm, snap())
        tick(fsm, snap(front=0.20))
        self.assertIsNot(tick(fsm, snap()).state, NavState.BLOCKED)

    def test_obstacle_goes_away(self):
        fsm = self._stopped_for_obstacle(Provider())
        n = int(round(cfg.OBSTACLE_CLEAR_S / 0.01))
        for _ in range(n - 1):
            st = tick(fsm, snap(), ds=0, counts=(0, 0))
        self.assertIs(st.state, NavState.BLOCKED)
        st = tick(fsm, snap(), ds=0, counts=(0, 0))
        self.assertIs(st.state, NavState.FOLLOW)
        self.assertGreater(st.v_ref_m_s, 0.0)
        self.assertEqual((fsm.obstacle_stops, fsm.obstacle_resumes), (1, 1))

    def test_obstacle_stays_turn_back(self):
        prov = Provider()
        fsm = self._stopped_for_obstacle(prov)
        for _ in range(int(cfg.OBSTACLE_WAIT_MAX_S / 0.01)):
            st = tick(fsm, snap(front=0.20), ds=0, counts=(0, 0))
            if st.state is not NavState.BLOCKED:
                break
        self.assertEqual(prov.blocked_calls, 1)
        self.assertEqual((st.state, st.phase), (NavState.POSITION, Phase.HOLD))
        for _ in range(int(cfg.PRE_TURN_HOLD_S / 0.01) + 1):
            st = tick(fsm, snap(front=0.20), ds=0, counts=(0, 0))
        self.assertIs(st.state, NavState.TURN)
        self.assertEqual(fsm.obstacle_turnbacks, 1)

    def test_no_way_around_finishes(self):
        fsm = self._stopped_for_obstacle(Provider(on_blocked=ManeuverDecision.FINISH))
        for _ in range(int(cfg.OBSTACLE_WAIT_MAX_S / 0.01) + 5):
            st = tick(fsm, snap(front=0.20), ds=0, counts=(0, 0))
        self.assertIs(st.state, NavState.FINISHED)


class TestRun(unittest.TestCase):
    def test_m3_all_obstacles_offline(self):
        """Robot code end to end: M3, all four obstacles, ideal profile, offline plant."""
        import run as runner
        s = runner.main(["--offline", "--config", "A", "--noise", "ideal", "--maze", "M3", "--seed", "1",
                         "--dyn", "ALL", "--tag", "test14b"])
        self.assertTrue(s["S4_mission_success"])
        self.assertEqual(sorted(s["dyn_triggered"]), ["DYN1", "DYN2", "DYN3", "DYN4"])
        self.assertEqual((s["dyn_resumes"], s["dyn_turnbacks"]), (2, 1))
        self.assertEqual(s["dyn_blocked"][0]["result"], "replanned")


if __name__ == "__main__":
    unittest.main()
