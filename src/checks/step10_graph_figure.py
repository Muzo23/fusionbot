"""
Step 10b: draw the topological map built from the live step-9 run (T1,
left-hand policy, 45 junction events) twice: with the GT pose and with the
odometry pose (configuration A). Uses the recorded events and poses
(tests/data), no simulator needed.

    python -m checks.step10_graph_figure        (from src/)

Output: results/step10/graph_T1_live.png and graph_T1_live_{gt,odometry}.json
"""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from mapping.graph_map import draw_graph, save_graph  # noqa: E402
from mazes.maze_geometry import WallMap  # noqa: E402
from mazes.maze_spec import MAZES  # noqa: E402
from mazes.topology import evaluate_memory  # noqa: E402
from navigation.junction_types import JunctionObservation, ReferenceKind  # noqa: E402
from navigation.maze_memory import DECISION_TURNS, MazeMemory, rotate  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "tests" / "data"
RESULTS = ROOT / "results" / "step10"


def replay(pose_prefix: str):
    spec = MAZES["T1"]
    ev = list(csv.DictReader(open(DATA / "step09_live_events.csv")))
    ps = list(csv.DictReader(open(DATA / "step09_live_event_poses.csv")))
    mem = MazeMemory()
    mem.start(*spec.start_pose)
    gt = []
    for e, p in zip(ev, ps):
        o = JunctionObservation(ReferenceKind(e["kind"]), e["left"] == "True", e["straight"] == "True",
                                e["right"] == "True", None, 0.0)
        a = mem.observe(o, float(p[f"{pose_prefix}_x"]), float(p[f"{pose_prefix}_y"]), float(p[f"{pose_prefix}_th"]))
        gt.append(e["node"])
        if e["decision"] in DECISION_TURNS:
            mem.depart(rotate(a.heading, DECISION_TURNS[e["decision"]]))
    return mem, gt


def main() -> None:
    spec = MAZES["T1"]
    walls = WallMap(spec)
    RESULTS.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(13, 7.5))
    for ax, (pre, name) in zip(axes, (("gt", "GT pose"), ("odo", "odometry (configuration A)"))):
        mem, gt = replay(pre)
        r = evaluate_memory(mem, spec, gt)
        labels = {0: "START"}
        for a, g in zip(mem.log, gt):
            labels.setdefault(a.junction, g.replace("__START__", "START"))
        dup_nodes = set(r["duplicates"])
        dup_ids = {j for j, g in labels.items() if g in dup_nodes or (g == "START" and "__START__" in dup_nodes)}
        draw_graph(mem, ax, walls=walls, gt_labels=labels, duplicates=dup_ids,
                   title=f"{name}: {r['memory_junctions']} junctions, duplicates {r['duplicates'] or 'none'}, "
                         f"max match {r['max_match_distance_m']:.3f} m")
        save_graph(mem, RESULTS / f"graph_T1_live_{'gt' if pre == 'gt' else 'odometry'}.json")
        print(f"{name:28}: {r}")
    fig.suptitle("T1, live step-9 run (left-hand policy, 45 events): topological map from MazeMemory", fontsize=10)
    fig.tight_layout()
    out = RESULTS / "graph_T1_live.png"
    fig.savefig(out, dpi=150)
    print(f"figure: {out}")


if __name__ == "__main__":
    main()
