"""
Ground-truth topological traversal (step 9, evaluation only).

expected_left_hand_events(spec, n) walks the maze graph with the left-hand
rule from START (arriving at the ROOT heading NORTH) and returns the first n
junction events the robot must produce: (node, left, straight, right, decision).
Nodes where only STRAIGHT continues produce no event (plain corridor); the
START approach corridor ends in a dead end behind the start.
"""

from __future__ import annotations

from mazes.maze_spec import MazeSpec

DIRS = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}
LEFT_OF = {"N": "W", "W": "S", "S": "E", "E": "N"}
RIGHT_OF = {v: k for k, v in LEFT_OF.items()}
BACK_OF = {"N": "S", "S": "N", "E": "W", "W": "E"}
START = "__START__"


def _exits(spec: MazeSpec, node: str) -> dict[str, str]:
    """global direction -> neighbour (incl. the START approach at the root)."""
    g = spec.nodes_grid
    out = {}
    for nb in spec.neighbours(node):
        dx, dy = g[nb][0] - g[node][0], g[nb][1] - g[node][1]
        d = next(k for k, (ux, uy) in DIRS.items() if (ux, uy) == ((dx > 0) - (dx < 0), (dy > 0) - (dy < 0)))
        out[d] = nb
    if node == spec.root:
        out["S"] = START
    return out


def expected_left_hand_events(spec: MazeSpec, n: int):
    events = []
    node, heading = spec.root, "N"
    while len(events) < n:
        if node == START:                      # dead end behind the start
            events.append((START, False, False, False, "UTURN"))
            node, heading = spec.root, "N"
            continue
        ex = _exits(spec, node)
        left, straight, right = LEFT_OF[heading] in ex, heading in ex, RIGHT_OF[heading] in ex
        if not (left or right) and straight:   # plain corridor through this node
            node = ex[heading]
            continue
        if left:
            dec, new = "LEFT", LEFT_OF[heading]
        elif straight:
            dec, new = "STRAIGHT", heading
        elif right:
            dec, new = "RIGHT", RIGHT_OF[heading]
        else:
            dec, new = "UTURN", BACK_OF[heading]
        events.append((node, left, straight, right, dec))
        node, heading = ex[new], new
    return events
