"""
Topological map of the maze (step 10b): export and drawing of MazeMemory.

MazeMemory IS the graph map: nodes = junctions at their ESTIMATED centres
(pose estimate of the configuration), edges = traversed corridors. This module
saves it (JSON, one file per run) and draws it, optionally over the true maze
walls and with true node labels (GT, evaluation only).
"""

from __future__ import annotations

import json
from pathlib import Path

from navigation.maze_memory import MazeMemory

DIR_VEC = {"E": (1, 0), "N": (0, 1), "W": (-1, 0), "S": (0, -1)}


def graph_to_dict(memory: MazeMemory) -> dict:
    return {
        "junctions": [
            {"id": j.id, "x": j.x, "y": j.y, "kind": getattr(j.kind, "value", j.kind),
             "exits": sorted(d.value for d in j.exits), "visits": j.visits,
             "edges": {d.value: e for d, e in j.edge_at.items()}}
            for j in memory.junctions.values()
        ],
        "edges": [
            {"id": e.id, "a": e.a, "dir_a": e.dir_a.value, "b": e.b,
             "dir_b": e.dir_b.value if e.dir_b else None, "traversals": e.traversals}
            for e in memory.edges.values()
        ],
        "events": [
            {"junction": a.junction, "new": a.is_new, "heading": a.heading.value,
             "centre": list(a.centre_xy), "match_distance_m": a.match_distance_m,
             "loop_closure": a.loop_closure, "rejected_structure": list(a.rejected_structure)}
            for a in memory.log
        ],
    }


def save_graph(memory: MazeMemory, path: str | Path) -> None:
    Path(path).write_text(json.dumps(graph_to_dict(memory), indent=1))


def draw_graph(memory: MazeMemory, ax, *, walls=None, gt_labels: dict[int, str] | None = None,
               duplicates: set[int] = frozenset(), title: str = "", stub_m: float = 0.25):
    """Draw the graph on a matplotlib Axes.
    walls: mazes.maze_geometry.WallMap (true walls, grey background, evaluation only)
    gt_labels: memory junction id -> true node name; duplicates: ids drawn in red."""
    if walls is not None:
        for axis, s, e, f in walls.segments:
            xs, ys = ((f, f), (s, e)) if axis == "V" else ((s, e), (f, f))
            ax.plot(xs, ys, color="0.75", lw=1.0, zorder=1)
    J = memory.junctions
    for e in memory.edges.values():
        a = J[e.a]
        if e.b is not None:
            b = J[e.b]
            ax.plot((a.x, b.x), (a.y, b.y), color="#1f6fb4", lw=1.0 + 0.8 * min(e.traversals, 3), zorder=2)
        else:
            dx, dy = DIR_VEC[e.dir_a.value]
            ax.plot((a.x, a.x + stub_m * dx), (a.y, a.y + stub_m * dy), color="#1f6fb4", ls="--", zorder=2)
    for j in J.values():
        for d in j.exits:                          # known but unexplored exits: short grey ticks
            if d not in j.edge_at:
                dx, dy = DIR_VEC[d.value]
                ax.plot((j.x, j.x + 0.15 * dx), (j.y, j.y + 0.15 * dy), color="0.4", lw=1, zorder=2)
        bad = j.id in duplicates
        ax.scatter([j.x], [j.y], s=60, zorder=3, color="#c0392b" if bad else ("#2e8b57" if j.id == 0 else "#1f6fb4"),
                   edgecolors="white", linewidths=0.8)
        lab = f"J{j.id}" + (f" {gt_labels[j.id]}" if gt_labels and j.id in gt_labels else "")
        ax.annotate(lab, (j.x, j.y), xytext=(4, 4), textcoords="offset points", fontsize=7,
                    color="#c0392b" if bad else "0.15", zorder=4)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=9)
    ax.tick_params(labelsize=7)
