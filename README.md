# Smart Campus Navigator
**An Interactive Campus Navigation and Emergency Rerouting System Using A\* Search** (AISC project)

## Files
| File | Purpose |
|---|---|
| `astar_engine.py` | **The algorithm.** Graph model, heuristics, A* written from scratch (`astar_steps` generator + `astar` wrapper). No GUI code. |
| `campus_data.py` | Campus map: 21 locations, 30 corridors, cost factors. Edit this to model your own campus. |
| `navigator_gui.py` | Tkinter visual front-end (step-through search, blocking, emergency modes, walker). |
| `test_astar.py` | Correctness proof: A* vs an independent Dijkstra on 26,000+ queries. |
| `VIVA_NOTES.md` | Concept explanations and likely viva questions with answers. |

## Run
Python 3.8+ only, **no pip installs** (Tkinter ships with Python on Windows/macOS; on Ubuntu: `sudo apt install python3-tk`).

```
python navigator_gui.py      # the application
python test_astar.py         # the correctness tests
```

## Controls
* **Left-click** a location: set *Start*. **Right-click** a location: set *Destination*.
* **Click a corridor** (either button): block / unblock it. Blocked corridors show a red X.
* **Step** (or Space): one A* iteration. **Auto-play**: animated, with speed slider. **Run to end** (or R).
* **Mode**: Normal navigation, Fire evacuation (goal = nearest of 3 assembly points), Medical emergency (goal = Medical Centre).
* **Walk the route**: a walker moves along the path. Block a corridor *ahead* of it and A* re-runs from its current position.
* **Compare with Dijkstra**: same query with h = 0; shows how many fewer nodes A* expands.

## What you see on the map
Orange = node being expanded, yellow = OPEN (shows `f`), blue = CLOSED (shows `g`), light-blue glow = parent pointers found so far,
green thick line = final route, grey dashed = old route before a reroute. Numbers on corridors are walking costs.
The right panel shows the sorted OPEN list (top row = next to expand) and a line-by-line explanation of every decision
(`[ADD]`, `[UPDATE]`, `[KEEP]`, `[SKIP]`, `[BLOCKED]`).

## Suggested 5-minute demo
1. **Normal route**: Main Gate -> CSE & IT Block. Step 3-4 times, point at the OPEN table and the f = g + h values. Run to end.
2. **Compare with Dijkstra**: show same cost, fewer expansions.
3. **Blocked corridor**: click a corridor on the green route; watch it reroute and see the grey old route.
4. **Fire mode**: start at CSE & IT Block, Run. Block the chosen corridor; A* may switch to a different assembly point.
5. **Walker**: Walk the route, block a corridor ahead, A* replans from the walker's position.
6. Show `python test_astar.py` passing.

## Design decisions to mention
* Edge cost = straight-line length x surface factor (>= 1), so the Euclidean heuristic is admissible and consistent.
* Goal test on **pop**, closed set, lazy-deletion heap, tie-break on smaller h.
* Multi-goal heuristic = distance to the nearest goal (still admissible), so evacuation needs one search, not one per exit.
* Rerouting is simply a *fresh A\* run* from the walker's current node on the updated graph.
