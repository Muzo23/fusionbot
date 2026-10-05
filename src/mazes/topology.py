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


def evaluate_memory(memory, spec: MazeSpec, gt_nodes: list[str]) -> dict:
    """Compare a MazeMemory with the maze graph.

    gt_nodes[i] is the true node of memory.log[i] (from GT, evaluation only).
    Returns association metrics: every true node should be exactly one memory
    junction and vice versa; every closed memory edge should be a true edge.
    """
    from collections import defaultdict
    mem_to_gt = defaultdict(set)
    gt_to_mem = defaultdict(set)
    mem_to_gt[0].add(START)                       # junction 0 is the START pose
    gt_to_mem[START].add(0)
    for arr, g in zip(memory.log, gt_nodes):
        mem_to_gt[arr.junction].add(g)
        gt_to_mem[g].add(arr.junction)
    false_merges = sorted(j for j, g in mem_to_gt.items() if len(g) > 1)
    duplicates = sorted(g for g, js in gt_to_mem.items() if len(js) > 1)
    label = {j: next(iter(g)) for j, g in mem_to_gt.items() if len(g) == 1}
    true_edges = {frozenset(e) for e in spec.edges} | {frozenset((spec.root, START))}
    edges_ok = edges_bad = 0
    for e in memory.edges.values():
        if e.b is None:
            continue
        a, b = label.get(e.a), label.get(e.b)
        if a is not None and b is not None and frozenset((a, b)) in true_edges:
            edges_ok += 1
        else:
            edges_bad += 1
    visited = set(gt_nodes) | {START}
    match_d = [a.match_distance_m for a in memory.log if a.match_distance_m is not None]
    return {
        "events": len(gt_nodes),
        "memory_junctions": len(memory.junctions),
        "true_nodes_visited": len(visited),
        "duplicates": duplicates,          # true node stored as >1 junction (missed recognition)
        "false_merges": false_merges,      # memory junction covering >1 true node
        "edges_correct": edges_ok,
        "edges_wrong": edges_bad,
        "max_match_distance_m": max(match_d, default=None),
        "correct": not duplicates and not false_merges and edges_bad == 0,
    }
