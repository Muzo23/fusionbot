"""
FusionBot proximity perception (step 6).

Turns raw proximity readings into the semantics the navigation FSM needs:

1. FRONT
       front_detected, front_distance_m,
       front_blocked = detected and distance <= FRONT_BLOCKED_DISTANCE_M

2. Side openings, one state machine per side (validated sequence):

       WALL --(diagonal loses the wall)--> CANDIDATE
       CANDIDATE --(side loses the wall)--> CONFIRMED
       CANDIDATE --(diagonal sees a wall again, side still sees it)--> WALL
       CONFIRMED --(side sees a wall again)--> PASSED

   LEFT uses FRONT_LEFT as the diagonal and LEFT as the side sensor, RIGHT
   uses FRONT_RIGHT and RIGHT. Why two stages: the diagonal (+-35 deg) looks
   ahead and sees an opening first, but also flickers at wall ends; only when
   the perpendicular side sensor also loses the wall is the opening real.

   CONFIRMED and PASSED are latched: when the diagonal re-detects the far
   wall of the side corridor, the opening must NOT be cancelled.

3. reset_openings() is called by the FSM only after it has left a junction
   (reacquire). Perception cannot know by itself when a junction ends.

The front threshold 0.057581 m was derived from the stopping distance at
0.25 m/s and validated end-to-end; it is separate from the lower physical
safety envelope.

Pure logic: no simulator, no odometry, no GT, no debounce that was not
experimentally justified.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from perception.proximity_types import ProximitySnapshot

FRONT_BLOCKED_DISTANCE_M = 0.057581


class OpeningState(str, Enum):
    WALL = "WALL"
    CANDIDATE = "CANDIDATE"
    CONFIRMED = "CONFIRMED"
    PASSED = "PASSED"


@dataclass(frozen=True)
class PerceptionSnapshot:
    front_detected: bool
    front_distance_m: float | None
    front_blocked: bool
    left_state: OpeningState
    right_state: OpeningState

    @property
    def left_open(self) -> bool:
        return self.left_state in (OpeningState.CONFIRMED, OpeningState.PASSED)

    @property
    def right_open(self) -> bool:
        return self.right_state in (OpeningState.CONFIRMED, OpeningState.PASSED)

    @property
    def any_candidate(self) -> bool:
        return OpeningState.CANDIDATE in (self.left_state, self.right_state)


def advance_opening(state: OpeningState, *, diagonal_detected: bool, side_detected: bool) -> OpeningState:
    if state is OpeningState.WALL:
        return OpeningState.CANDIDATE if not diagonal_detected else OpeningState.WALL
    if state is OpeningState.CANDIDATE:
        if not side_detected:
            return OpeningState.CONFIRMED      # side confirmation wins in the same tick
        if diagonal_detected:
            return OpeningState.WALL           # false candidate (wall end flicker)
        return OpeningState.CANDIDATE
    if state is OpeningState.CONFIRMED:
        return OpeningState.PASSED if side_detected else OpeningState.CONFIRMED
    return OpeningState.PASSED


class ProximityPerception:
    def __init__(self, front_blocked_distance_m: float = FRONT_BLOCKED_DISTANCE_M) -> None:
        d = float(front_blocked_distance_m)
        if not math.isfinite(d) or d <= 0.0:
            raise ValueError("front_blocked_distance_m must be finite and > 0")
        self.front_blocked_distance_m = d
        self.reset_openings()

    def reset_openings(self) -> None:
        self.left_state = OpeningState.WALL
        self.right_state = OpeningState.WALL

    def update(self, snap: ProximitySnapshot) -> PerceptionSnapshot:
        f = snap.front
        dist = None
        if f.detected:
            if f.distance_m is None or not math.isfinite(f.distance_m) or f.distance_m < 0.0:
                raise ValueError(f"front: invalid distance {f.distance_m!r} with detected=True")
            dist = float(f.distance_m)
        blocked = dist is not None and dist <= self.front_blocked_distance_m

        self.left_state = advance_opening(
            self.left_state, diagonal_detected=snap.front_left.detected, side_detected=snap.left.detected
        )
        self.right_state = advance_opening(
            self.right_state, diagonal_detected=snap.front_right.detected, side_detected=snap.right.detected
        )
        return PerceptionSnapshot(bool(f.detected), dist, blocked, self.left_state, self.right_state)
