"""
astar_engine.py  --  Own implementation of the A* search algorithm
===================================================================
Project : Smart Campus Navigator (AISC)

This file contains ONLY the AI part: the graph model and A* itself.
It has no GUI code, so the algorithm can be read, tested and explained
on its own.

A* in one paragraph
-------------------
A* keeps an OPEN list (frontier: nodes discovered but not yet expanded) and a
CLOSED set (nodes already expanded).  For every node n it tracks

        g(n) = cheapest known cost from START to n          (real cost so far)
        h(n) = heuristic estimate of cost from n to a GOAL  (guess about future)
        f(n) = g(n) + h(n)                                  (estimated total)

Each iteration it removes the node with the LOWEST f from OPEN, and if that
node is a goal it stops.  Otherwise it expands it: for every usable neighbour v
it computes the tentative cost g(u) + cost(u, v); if that beats the best g
known for v, v's parent and g are updated and v is (re)inserted into OPEN.

Why the answer is optimal
-------------------------
* The heuristic h is ADMISSIBLE: it never over-estimates the true remaining
  cost.  Here h = straight-line (Euclidean) distance, and every walkway costs
  at least its straight-line length (cost = length * factor, factor >= 1).
* The heuristic is also CONSISTENT (monotone):  h(u) <= cost(u,v) + h(v).
  With a consistent heuristic the first time a node is expanded its g is already
  optimal, so a node never needs to be re-opened and CLOSED is safe to use.
* The goal test is done when a node is POPPED, not when it is generated.
  Testing at generation time could return a path that is not the cheapest.
"""

from __future__ import annotations

import heapq
import itertools
import math
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, Iterator, List, Optional, Set, Tuple


# ----------------------------------------------------------------------------
# 1.  GRAPH MODEL
# ----------------------------------------------------------------------------
@dataclass(frozen=True)
class Node:
    key: str      # short id, e.g. "LIB"
    name: str     # display name, e.g. "Central Library"
    x: float      # position in metres (also used by the heuristic)
    y: float
    kind: str     # gate | building | junction | medical | assembly | parking


@dataclass(frozen=True)
class Edge:
    a: str
    b: str
    kind: str       # road | footpath | covered | stairs | crowded
    length: float   # straight-line length in metres
    cost: float     # walking cost = length * factor  (factor >= 1)


class Graph:
    """Undirected weighted graph with support for blocked corridors."""

    def __init__(self) -> None:
        self.nodes: Dict[str, Node] = {}
        self.adj: Dict[str, Dict[str, Edge]] = {}
        self.edges: List[Edge] = []
        self.blocked: Set[frozenset] = set()   # set of {a, b} pairs

    # ---- construction -------------------------------------------------------
    def add_node(self, key: str, name: str, x: float, y: float, kind: str) -> None:
        self.nodes[key] = Node(key, name, x, y, kind)
        self.adj[key] = {}

    def add_edge(self, a: str, b: str, kind: str, factor: float) -> None:
        if factor < 1.0:
            # factor < 1 would make the straight-line heuristic over-estimate
            # and A* would lose its optimality guarantee.
            raise ValueError("cost factor must be >= 1 to keep h admissible")
        length = self.straight_line(a, b)
        edge = Edge(a, b, kind, length, length * factor)
        self.adj[a][b] = edge
        self.adj[b][a] = edge
        self.edges.append(edge)

    # ---- geometry -----------------------------------------------------------
    def straight_line(self, a: str, b: str) -> float:
        na, nb = self.nodes[a], self.nodes[b]
        return math.hypot(na.x - nb.x, na.y - nb.y)

    # ---- blockages ----------------------------------------------------------
    def is_blocked(self, a: str, b: str) -> bool:
        return frozenset((a, b)) in self.blocked

    def toggle_block(self, a: str, b: str) -> bool:
        """Block/unblock the corridor a-b.  Returns True if now blocked."""
        key = frozenset((a, b))
        if key in self.blocked:
            self.blocked.remove(key)
            return False
        self.blocked.add(key)
        return True

    def clear_blocks(self) -> None:
        self.blocked.clear()


# ----------------------------------------------------------------------------
# 2.  HEURISTICS  h(node) for a SET of goals
# ----------------------------------------------------------------------------
def euclidean_heuristic(graph: Graph, node: str, goals: Iterable[str]) -> float:
    """Straight-line distance to the NEAREST goal.

    With several goals (e.g. all fire-assembly points) taking the minimum keeps
    the heuristic admissible: it is never larger than the distance to the goal
    that is actually closest.
    """
    return min(graph.straight_line(node, g) for g in goals)


def zero_heuristic(graph: Graph, node: str, goals: Iterable[str]) -> float:
    """h = 0 turns A* into Dijkstra's algorithm (used for comparison)."""
    return 0.0


Heuristic = Callable[[Graph, str, Iterable[str]], float]


# ----------------------------------------------------------------------------
# 3.  A* SEARCH  (written as a generator so the GUI can step through it)
# ----------------------------------------------------------------------------
@dataclass
class Step:
    """Snapshot of the search after one iteration (used for visualisation)."""
    number: int
    kind: str                                   # init | expand | goal | fail
    current: Optional[str]                      # node expanded in this step
    g: float = 0.0
    h: float = 0.0
    f: float = 0.0
    open_list: List[Tuple[float, float, float, str]] = field(default_factory=list)
    closed: Set[str] = field(default_factory=set)
    g_score: Dict[str, float] = field(default_factory=dict)
    came_from: Dict[str, str] = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)   # human-readable explanation
    path: Optional[List[str]] = None
    cost: float = 0.0


def astar_steps(graph: Graph, start: str, goals: Iterable[str],
                heuristic: Heuristic = euclidean_heuristic) -> Iterator[Step]:
    """Run A* and yield a Step after every iteration of the main loop."""
    goals = set(goals)
    name = lambda k: graph.nodes[k].name

    g_score: Dict[str, float] = {start: 0.0}   # best known cost from start
    came_from: Dict[str, str] = {}             # parent pointers -> path
    h_cache: Dict[str, float] = {}             # h never changes: compute once

    def h(n: str) -> float:
        if n not in h_cache:
            h_cache[n] = heuristic(graph, n, goals)
        return h_cache[n]

    # OPEN list = binary min-heap of (f, h, tie_breaker, node).
    #  * smallest f is expanded first
    #  * ties are broken by smaller h (the node that looks closer to the goal)
    #  * the counter keeps ordering deterministic when f and h are equal
    # Python's heapq cannot decrease a key, so when a better path to a node is
    # found we push a NEW entry and ignore the old one later ("lazy deletion").
    counter = itertools.count()
    open_heap: List[Tuple[float, float, int, str]] = []
    in_open: Set[str] = {start}                # nodes with a valid OPEN entry
    closed: Set[str] = set()
    heapq.heappush(open_heap, (h(start), h(start), next(counter), start))

    step_no = 0

    def snapshot() -> List[Tuple[float, float, float, str]]:
        rows = [(g_score[n] + h(n), g_score[n], h(n), n) for n in in_open]
        rows.sort(key=lambda r: (r[0], r[2]))   # same order as the heap
        return rows

    yield Step(0, "init", None, open_list=snapshot(), closed=set(closed),
               g_score=dict(g_score), came_from=dict(came_from),
               notes=[f"Start at {name(start)}. g=0, h={h(start):.0f}, "
                      f"f={h(start):.0f}. OPEN = [{name(start)}], CLOSED = []."])

    while open_heap:
        f, hu, _, u = heapq.heappop(open_heap)
        if u in closed:
            continue                            # stale duplicate entry
        in_open.discard(u)
        gu = g_score[u]
        step_no += 1
        notes = [f"Step {step_no}: pop {name(u)} -> lowest f in OPEN "
                 f"(g={gu:.0f} + h={hu:.0f} = f={f:.0f})."]
        closed.add(u)

        # ---- GOAL TEST (on pop, not on generation) --------------------------
        if u in goals:
            path = [u]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            path.reverse()
            notes.append(f"GOAL TEST passed: {name(u)} is a destination. "
                         f"Because h is admissible, this route is optimal "
                         f"(cost {gu:.0f} m-equivalent).")
            yield Step(step_no, "goal", u, gu, hu, f, snapshot(), set(closed),
                       dict(g_score), dict(came_from), notes, path, gu)
            return

        # ---- EXPAND u -------------------------------------------------------
        for v, edge in graph.adj[u].items():
            if graph.is_blocked(u, v):
                notes.append(f"  [BLOCKED] {name(u)} -> {name(v)} is closed, "
                             f"not considered.")
                continue
            if v in closed:
                notes.append(f"  [SKIP]    {name(v)} already in CLOSED "
                             f"(its g is already optimal).")
                continue
            tentative = gu + edge.cost
            old = g_score.get(v)
            if old is None or tentative < old - 1e-9:
                g_score[v] = tentative
                came_from[v] = u
                heapq.heappush(open_heap,
                               (tentative + h(v), h(v), next(counter), v))
                in_open.add(v)
                if old is None:
                    notes.append(
                        f"  [ADD]     {name(v)}: g={gu:.0f}+{edge.cost:.0f}="
                        f"{tentative:.0f}, h={h(v):.0f}, f={tentative + h(v):.0f}"
                        f"  (newly discovered)")
                else:
                    notes.append(
                        f"  [UPDATE]  {name(v)}: cheaper route found via "
                        f"{name(u)}, g {old:.0f} -> {tentative:.0f}, "
                        f"f={tentative + h(v):.0f}")
            else:
                notes.append(
                    f"  [KEEP]    {name(v)}: via {name(u)} g={tentative:.0f} "
                    f"is not better than {old:.0f}.")

        yield Step(step_no, "expand", u, gu, hu, f, snapshot(), set(closed),
                   dict(g_score), dict(came_from), notes)

    # OPEN is empty and no goal was reached
    yield Step(step_no + 1, "fail", None, open_list=[], closed=set(closed),
               g_score=dict(g_score), came_from=dict(came_from),
               notes=["OPEN list is empty and no destination was reached: "
                      "every route is blocked."])


# ----------------------------------------------------------------------------
# 4.  Convenience wrapper: run to completion
# ----------------------------------------------------------------------------
@dataclass
class SearchResult:
    path: Optional[List[str]]
    cost: float
    expanded: int          # number of nodes moved to CLOSED
    last_step: Step


def astar(graph: Graph, start: str, goals: Iterable[str],
          heuristic: Heuristic = euclidean_heuristic) -> SearchResult:
    last = None
    for last in astar_steps(graph, start, goals, heuristic):
        pass
    return SearchResult(last.path, last.cost, len(last.closed), last)


def path_cost(graph: Graph, path: List[str]) -> float:
    return sum(graph.adj[a][b].cost for a, b in zip(path, path[1:]))
