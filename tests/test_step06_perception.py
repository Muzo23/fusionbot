"""Step 6 acceptance: proximity perception on REAL recorded sensor data.

Fixtures in tests/data are trimmed copies of live CoppeliaSim recordings from
the validated project (robot driven past a left / right opening in 5 mm steps,
and two FRONT_BLOCKED end-to-end runs).

    python -m unittest tests.test_step06_perception -v
"""

import csv
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from perception.proximity_perception import (  # noqa: E402
    FRONT_BLOCKED_DISTANCE_M, OpeningState, ProximityPerception, advance_opening,
)
from perception.proximity_types import ProximityReading, ProximitySnapshot  # noqa: E402

DATA = Path(__file__).resolve().parent / "data"
NAMES = ("left", "front_left", "front", "front_right", "right")


def snap_from_row(row):
    def one(n):
        det = row[f"{n}_detected"] in ("1", "True")
        d = row.get(f"{n}_distance_m", "")
        return ProximityReading(n, det, float(d) if det and d != "" else None)
    return ProximitySnapshot(**{n: one(n) for n in NAMES})


def replay_opening(side):
    p = ProximityPerception()
    events, prev = {}, OpeningState.WALL
    states = []
    for row in csv.DictReader(open(DATA / f"step06_opening_{side}.csv")):
        out = p.update(snap_from_row(row))
        st = out.left_state if side == "left" else out.right_state
        if st is not prev and st.value not in events:
            events[st.value] = float(row["axle_x_m"])
        states.append(st)
        prev = st
    return events, states


class TestOpeningReplay(unittest.TestCase):
    """Validated positions (axle x relative to the opening): CANDIDATE -0.875,
    CONFIRMED -0.530, PASSED +0.200 m; the diagonal re-detects at -0.215 m and
    must NOT cancel the opening."""

    def _check(self, side):
        events, states = replay_opening(side)
        self.assertAlmostEqual(events["CANDIDATE"], -0.875, places=9)
        self.assertAlmostEqual(events["CONFIRMED"], -0.530, places=9)
        self.assertAlmostEqual(events["PASSED"], 0.200, places=9)
        first_passed = states.index(OpeningState.PASSED)
        self.assertTrue(all(s is OpeningState.PASSED for s in states[first_passed:]))   # latched
        self.assertNotIn("WALL", events)                                                 # never cancelled

    def test_left_opening(self):
        self._check("left")

    def test_right_opening(self):
        self._check("right")

    def test_other_side_stays_wall(self):
        p = ProximityPerception()
        for row in csv.DictReader(open(DATA / "step06_opening_left.csv")):
            out = p.update(snap_from_row(row))
            if float(row["axle_x_m"]) < 0.95:   # the test scene's opposite wall ends at ~+1.0 m
                self.assertIs(out.right_state, OpeningState.WALL)


class TestFrontReplay(unittest.TestCase):
    """First front_blocked must be the exact tick on which the validated run braked."""

    def _check(self, tag):
        p = ProximityPerception()
        first_blocked = first_brake = None
        for i, row in enumerate(csv.DictReader(open(DATA / f"step06_front_{tag}.csv"))):
            det = row["front_detected"] == "1"
            d = float(row["front_distance_m"]) if det and row["front_distance_m"] else None
            wall = ProximityReading("x", True, 0.18)
            snap = ProximitySnapshot(wall, wall, ProximityReading("front", det, d), wall, wall)
            if p.update(snap).front_blocked and first_blocked is None:
                first_blocked = i
            if row["brake_triggered_this_tick"] == "1" and first_brake is None:
                first_brake = i
        self.assertIsNotNone(first_brake)
        self.assertEqual(first_blocked, first_brake)

    def test_front_p180(self):
        self._check("p180")

    def test_front_m180(self):
        self._check("m180")


class TestStateMachine(unittest.TestCase):
    def test_false_candidate_returns_to_wall(self):
        s = advance_opening(OpeningState.WALL, diagonal_detected=False, side_detected=True)
        self.assertIs(s, OpeningState.CANDIDATE)
        s = advance_opening(s, diagonal_detected=True, side_detected=True)
        self.assertIs(s, OpeningState.WALL)

    def test_side_confirmation_wins_same_tick(self):
        s = advance_opening(OpeningState.CANDIDATE, diagonal_detected=True, side_detected=False)
        self.assertIs(s, OpeningState.CONFIRMED)

    def test_threshold_and_reset(self):
        p = ProximityPerception()
        wall = ProximityReading("x", True, 0.18)
        at = ProximitySnapshot(wall, wall, ProximityReading("front", True, FRONT_BLOCKED_DISTANCE_M), wall, wall)
        self.assertTrue(p.update(at).front_blocked)
        p.left_state = OpeningState.PASSED
        p.reset_openings()
        self.assertIs(p.left_state, OpeningState.WALL)

    def test_invalid_front_distance_rejected(self):
        p = ProximityPerception()
        wall = ProximityReading("x", True, 0.18)
        bad = ProximitySnapshot(wall, wall, ProximityReading("front", True, None), wall, wall)
        with self.assertRaises(ValueError):
            p.update(bad)


if __name__ == "__main__":
    unittest.main()
