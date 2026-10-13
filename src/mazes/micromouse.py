"""
Real micromouse contest / training mazes -> FusionBot MazeSpec (step 13c).

Source: the public collection github.com/micromouseonline/mazefiles (classic 16x16
contest mazes, training mazes; text format with posts 'o', walls '---' / '|',
start 'S', goal 'G'). A copy of the used files is in data/mazes/ with the source
commit, so the thesis can cite exactly which mazes were used.

Conversion
----------
* every cell is a square of the FusionBot lattice (pitch PITCH_M, free width
  FREE_WIDTH_M): the micromouse 18 cm cell becomes a 1.085 m cell, the wall
  layout (topology) is kept exactly;
* only the part reachable from the start cell is used;
* graph nodes = cells that are NOT a plain straight corridor (junctions, corners,
  dead ends); edges = straight runs between them (MazeSpec convention);
* the start cell (walls on three sides, open to the north by the contest rules;
  the maze is rotated if needed) becomes the START approach: ROOT is the first
  node north of it, the robot starts START_OFFSET_FROM_ROOT_M south of ROOT;
* GOAL: the marked goal cells of a contest maze; a training maze without a
  goal gets the node farthest from ROOT along the corridors;
* open areas (e.g. the 2x2 goal) keep a post in the middle, as in the real maze
  (here a 0.36 m pillar, since FusionBot corridors are narrower than its cells).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from mazes.maze_spec import MazeSpec

DIRS = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}
OPP = {"N": "S", "S": "N", "E": "W", "W": "E"}


@dataclass
class CellMaze:
    width: int
    height: int
    open: dict                 # (c, r) -> set of open directions
    start: tuple[int, int]
    goals: list

    def rotated(self) -> "CellMaze":
        """Rotate 90 deg counter-clockwise: (c, r) -> (H-1-r, c), N->W, E->N, S->E, W->S."""
        rot = {"N": "W", "E": "N", "S": "E", "W": "S"}
        H = self.height
        def f(cell):
            return (H - 1 - cell[1], cell[0])
        return CellMaze(self.height, self.width, {f(k): {rot[d] for d in v} for k, v in self.open.items()},
                        f(self.start), [f(g) for g in self.goals])


def parse(text: str) -> CellMaze:
    lines = [ln.rstrip("\n") for ln in text.splitlines() if ln.strip()]
    n_rows = (len(lines) - 1) // 2
    n_cols = (len(lines[0]) - 1) // 4

    def at(line, i):
        return line[i] if i < len(line) else " "

    open_ = {}
    start, goals = None, []
    for i in range(n_rows):                       # i = row from the top
        top, mid, bot = lines[2 * i], lines[2 * i + 1], lines[2 * i + 2]
        r = n_rows - 1 - i
        for c in range(n_cols):
            o = set()
            if "-" not in top[4 * c + 1:4 * c + 4]:
                o.add("N")
            if "-" not in bot[4 * c + 1:4 * c + 4]:
                o.add("S")
            if at(mid, 4 * c) != "|":
                o.add("W")
            if at(mid, 4 * c + 4) != "|":
                o.add("E")
            # boundary of the grid is always closed
            if r == n_rows - 1:
                o.discard("N")
            if r == 0:
                o.discard("S")
            if c == 0:
                o.discard("W")
            if c == n_cols - 1:
                o.discard("E")
            open_[(c, r)] = o
            mark = at(mid, 4 * c + 2)
            if mark == "S":
                start = (c, r)
            elif mark == "G":
                goals.append((c, r))
    # make the openings symmetric (a wall seen from either side closes both)
    for (c, r), o in open_.items():
        for d in list(o):
            dc, dr = DIRS[d]
            nb = open_.get((c + dc, r + dr))
            if nb is None or OPP[d] not in nb:
                o.discard(d)
    if start is None:
        start = (0, 0)
    return CellMaze(n_cols, n_rows, open_, start, goals)


def reachable(m: CellMaze) -> set:
    seen, q = {m.start}, deque([m.start])
    while q:
        c, r = q.popleft()
        for d in m.open[(c, r)]:
            dc, dr = DIRS[d]
            nb = (c + dc, r + dr)
            if nb not in seen:
                seen.add(nb)
                q.append(nb)
    return seen


def _is_node(o: set) -> bool:
    return not (o == {"N", "S"} or o == {"E", "W"})


def to_spec(m: CellMaze, maze_id: str, name: str) -> MazeSpec:
    for _ in range(4):                            # start must open to the north
        if m.open[m.start] == {"N"}:
            break
        m = m.rotated()
    else:
        raise ValueError(f"{maze_id}: start cell {m.start} is not a dead end opening to one side")
    cells = reachable(m)
    nodes = {c for c in cells if _is_node(m.open[c])}
    # ROOT = first node north of the start cell
    c, r = m.start
    r += 1
    while (c, r) not in nodes:
        r += 1
    root = (c, r)
    nodes.discard(m.start)
    edges = set()
    for n in nodes:
        for d in m.open[n]:
            dc, dr = DIRS[d]
            cur = (n[0] + dc, n[1] + dr)
            while cur not in nodes and cur != m.start:
                cur = (cur[0] + dc, cur[1] + dr)
            if cur == m.start:                    # the start approach is not part of the graph
                continue
            edges.add(tuple(sorted((n, cur))))
    name_of = {cell: ("R" if cell == root else f"c{cell[0]}_{cell[1]}") for cell in nodes}
    grid = {name_of[cell]: (cell[0] - root[0], cell[1] - root[1]) for cell in nodes}
    edge_names = tuple(sorted((name_of[a], name_of[b]) for a, b in edges))
    goal_cells = [g for g in m.goals if g in cells]
    if goal_cells:                                # contest maze: the marked goal (centre) area
        goal = min(nodes, key=lambda x: min(abs(x[0] - g[0]) + abs(x[1] - g[1]) for g in goal_cells))
    else:                                         # training maze without a goal: the node farthest
        goal = _farthest(nodes, edges, root)      # from ROOT along the corridors (hardest goal)
    cycles = len(edge_names) - len(grid) + 1
    return MazeSpec(maze_id, name, grid, edge_names, "R", name_of[goal], cycles)


def _farthest(nodes, edges, root):
    import heapq
    adj = {n: [] for n in nodes}
    for a, b in edges:
        w = abs(a[0] - b[0]) + abs(a[1] - b[1])
        adj[a].append((b, w))
        adj[b].append((a, w))
    dist, pq = {root: 0}, [(0, root)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue
        for v, w in adj[u]:
            if d + w < dist.get(v, 10 ** 9):
                dist[v] = d + w
                heapq.heappush(pq, (d + w, v))
    return max(dist, key=lambda n: (dist[n], n[1], n[0]))


def metrics(spec: MazeSpec) -> dict:
    """Difficulty indicators of a maze graph."""
    deg = {n: 0 for n in spec.nodes_grid}
    lengths = []
    for a, b in spec.edges:
        deg[a] += 1
        deg[b] += 1
        (ax, ay), (bx, by) = spec.nodes_grid[a], spec.nodes_grid[b]
        lengths.append(abs(ax - bx) + abs(ay - by))
    deg[spec.root] += 1                           # the start approach
    xs = [p[0] for p in spec.nodes_grid.values()]
    ys = [p[1] for p in spec.nodes_grid.values()]
    return {
        "nodes": len(spec.nodes_grid), "edges": len(spec.edges), "cycles": spec.expected_cycle_count,
        "dead_ends": sum(1 for d in deg.values() if d == 1),
        "corners": sum(1 for n, d in deg.items() if d == 2),
        "junctions_3": sum(1 for d in deg.values() if d == 3),
        "junctions_4": sum(1 for d in deg.values() if d == 4),
        "corridor_cells": sum(lengths),
        "longest_straight_cells": max(lengths) if lengths else 0,
        "extent_cells": (max(xs) - min(xs) + 1, max(ys) - min(ys) + 1),
    }


# --------------------------------------------------------------------------- registry
from pathlib import Path as _Path

DATA_DIR = _Path(__file__).resolve().parents[2] / "data" / "mazes"
REAL_MAZES = {      # id: (file, name, difficulty)
    "M1": ("minimaze.txt", "Micromouse training 5x5 (minimaze)", "lako"),
    "M2": ("maze-train-10x5-a.txt", "Micromouse training 10x5-a", "srednje"),
    "M3": ("training-10x5-test1.txt", "Micromouse training 10x5 test1", "srednje-teško"),
    "M4": ("practice-8x8-a.txt", "Micromouse practice 8x8-a", "teško"),
    "M5": ("training-8x8-test2.txt", "Micromouse training 8x8 test2", "vrlo teško"),
    "M6": ("uk2016f.txt", "UK micromouse contest 2016, final (16x16)", "iznimno teško"),
    "M7": ("japan2017ef.txt", "All Japan micromouse 2017, expert final (16x16)", "iznimno teško"),
}


def real_maze(maze_id: str) -> MazeSpec:
    f, name, _ = REAL_MAZES[maze_id]
    return to_spec(parse((DATA_DIR / f).read_text()), maze_id, name)
