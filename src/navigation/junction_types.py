"""
Junction observation / decision types shared by the navigation FSM (step 9),
the maze memory (step 10) and the maze decision layer (step 11).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol


class ReferenceKind(str, Enum):
    SIDE_CONFIRMED = "SIDE_CONFIRMED"     # a side opening was confirmed (junction with side exit)
    DEAD_END = "DEAD_END"                 # FRONT_BLOCKED with no side opening


class ManeuverDecision(str, Enum):
    STRAIGHT = "STRAIGHT"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    UTURN = "UTURN"
    FINISH = "FINISH"                     # exploration complete -> stop here


TURN_ANGLE_DEG = {
    ManeuverDecision.LEFT: +90.0,
    ManeuverDecision.RIGHT: -90.0,
}


@dataclass(frozen=True)
class JunctionObservation:
    """Local topology at one junction reference event (robot-relative).

    BACK is always available (the robot came from there).
    The reference event is the tick on which the observation was made; the
    junction centre lies D_SIDE_CANONICAL_M (side) or D_DEAD_END_CANONICAL_M
    (dead end) further ahead along the corridor.
    """

    kind: ReferenceKind
    left_available: bool
    straight_available: bool
    right_available: bool
    front_distance_m: float | None
    reference_odom_distance_m: float      # odometry signed distance at the event

    @property
    def back_available(self) -> bool:
        return True

    def available(self, decision: ManeuverDecision) -> bool:
        return {
            ManeuverDecision.LEFT: self.left_available,
            ManeuverDecision.STRAIGHT: self.straight_available,
            ManeuverDecision.RIGHT: self.right_available,
            ManeuverDecision.UTURN: True,
            ManeuverDecision.FINISH: True,
        }[decision]


class DecisionProvider(Protocol):
    def decide(self, observation: JunctionObservation) -> ManeuverDecision: ...
