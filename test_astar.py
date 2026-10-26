"""
test_astar.py -- Proves the A* implementation is correct.

Run:  python test_astar.py

1. Heuristic consistency:  h(u) <= cost(u,v) + h(v)  for every edge/goal.
2. Optimality: A* cost == cost from an INDEPENDENT Dijkstra implementation,
   for every start/goal pair, with and without random blocked corridors.
3. Emergency (multi-goal) search returns the cheapest assembly point.
4. Unreachable goals are reported correctly.
"""

import heapq
import random

from astar_engine import astar, euclidean_heuristic, path_cost, zero_heuristic
from campus_data import ASSEMBLY_POINTS, build_campus

EPS = 1e-6


def dijkstra_cost(graph, start, goals):
    """Reference implementation (does not share code with astar_engine)."""
    goals = set(goals)
    dist = {start: 0.0}
    pq = [(0.0, start)]
    done = set()
    while pq:
        d, u = heapq.heappop(pq)
        if u in done:
            continue
        done.add(u)
        if u in goals:
            return d
        for v, e in graph.adj[u].items():
            if graph.is_blocked(u, v):
                continue
            nd = d + e.cost
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                heapq.heappush(pq, (nd, v))
    return None


def test_consistency():
    g = build_campus()
    for goal in g.nodes:
        for e in g.edges:
            hu = euclidean_heuristic(g, e.a, [goal])
            hv = euclidean_heuristic(g, e.b, [goal])
            assert hu <= e.cost + hv + EPS, (e, goal)
            assert hv <= e.cost + hu + EPS, (e, goal)
    print("PASS consistency / admissibility of the heuristic")


def test_optimal_all_pairs(trials=60):
    g = build_campus()
    rng = random.Random(7)
    checked = 0
    for t in range(trials):
        g.clear_blocks()
        if t:                                    # trial 0 = no blockages
            for e in rng.sample(g.edges, rng.randint(1, 8)):
                g.toggle_block(e.a, e.b)
        for s in g.nodes:
            for d in g.nodes:
                a = astar(g, s, [d])
                ref = dijkstra_cost(g, s, [d])
                if ref is None:
                    assert a.path is None, (s, d)
                else:
                    assert a.path is not None, (s, d)
                    assert abs(a.cost - ref) < EPS, (s, d, a.cost, ref)
                    assert abs(path_cost(g, a.path) - a.cost) < EPS
                    assert all(not g.is_blocked(x, y)
                               for x, y in zip(a.path, a.path[1:]))
                checked += 1
    print(f"PASS A* == Dijkstra on {checked} queries (with random blockages)")


def test_multi_goal():
    g = build_campus()
    rng = random.Random(3)
    for t in range(40):
        g.clear_blocks()
        for e in rng.sample(g.edges, rng.randint(0, 6)):
            g.toggle_block(e.a, e.b)
        for s in g.nodes:
            a = astar(g, s, ASSEMBLY_POINTS)
            ref = dijkstra_cost(g, s, ASSEMBLY_POINTS)
            assert (a.path is None) == (ref is None)
            if ref is not None:
                assert abs(a.cost - ref) < EPS
                assert a.path[-1] in ASSEMBLY_POINTS
    print("PASS multi-goal evacuation returns the cheapest assembly point")


def test_never_worse_than_dijkstra_expansions():
    g = build_campus()
    total_a = total_d = 0
    for s in g.nodes:
        for d in g.nodes:
            total_a += astar(g, s, [d]).expanded
            total_d += astar(g, s, [d], zero_heuristic).expanded
    print(f"INFO total nodes expanded over all pairs: "
          f"A* = {total_a}, Dijkstra = {total_d}")
    assert total_a <= total_d


def test_unreachable():
    g = build_campus()
    for nb in list(g.adj["MG"]):                 # cut the main gate off
        g.toggle_block("MG", nb)
    r = astar(g, "MG", ["CS"])
    assert r.path is None and r.last_step.kind == "fail"
    print("PASS unreachable destination detected")


if __name__ == "__main__":
    test_consistency()
    test_optimal_all_pairs()
    test_multi_goal()
    test_never_worse_than_dijkstra_expansions()
    test_unreachable()
    print("\nAll tests passed.")
