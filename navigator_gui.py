"""
navigator_gui.py  --  Smart Campus Navigator (Tkinter front-end)
=================================================================
Run:   python navigator_gui.py

Controls
  Left-click a building/junction ........ set START
  Right-click a building/junction ....... set DESTINATION (Normal mode)
  Click a corridor (left or right) ...... block / unblock it
  Space ................................. one A* step
  R ..................................... run A* to the end

All search logic lives in astar_engine.py; this file only draws and controls.
"""

import math
import random
import tkinter as tk
from tkinter import ttk

from astar_engine import (astar, astar_steps, euclidean_heuristic,
                          zero_heuristic)
from campus_data import (ASSEMBLY_POINTS, EDGE_TYPES, MEDICAL_CENTRE,
                         WALKING_SPEED_M_PER_MIN, build_campus)

CANVAS_W, CANVAS_H = 880, 650
BG = "#f4f1ea"
COL_OPEN, COL_CLOSED, COL_CURRENT = "#fff176", "#a9d0f5", "#ff9800"
COL_START, COL_GOAL, COL_PATH = "#0a8f3c", "#c2185b", "#1fa94b"
KIND_FILL = {"gate": "#e6d3a3", "building": "#ffffff", "junction": "#cfcfcf",
             "medical": "#ffd6d6", "assembly": "#d6f5d6", "parking": "#e3e3e3"}
KIND_SYMBOL = {"gate": "G", "medical": "+", "assembly": "A", "parking": "P"}


class NavigatorApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        root.title("Smart Campus Navigator - A* Search (AISC)")
        self.graph = build_campus()
        self.names = {k: n.name for k, n in self.graph.nodes.items()}
        self.keys_by_name = {v: k for k, v in self.names.items()}

        self.mode = tk.StringVar(value="nav")        # nav | fire | medical
        self.start, self.dest = "MG", "CS"

        # search state
        self.gen = None
        self.step = None
        self.finished = False
        self.path = None
        self.old_path = None
        self.autoplaying = False
        # walking state
        self.walking = False
        self.walker = None
        self.route, self.route_i = [], 0

        self._build_ui()
        self.new_search()

    # ------------------------------------------------------------------ UI --
    def _build_ui(self) -> None:
        left = tk.Frame(self.root)
        left.pack(side="left")
        self.canvas = tk.Canvas(left, width=CANVAS_W, height=CANVAS_H, bg=BG,
                                highlightthickness=0)
        self.canvas.pack()
        self.canvas.bind("<Button-1>", self.on_left_click)
        self.canvas.bind("<Button-2>", self.on_right_click)   # macOS
        self.canvas.bind("<Button-3>", self.on_right_click)
        self.root.bind("<space>", lambda e: self.step_once())
        self.root.bind("r", lambda e: self.run_all(animate=False))

        side = tk.Frame(self.root, padx=8, pady=6)
        side.pack(side="right", fill="both", expand=True)

        tk.Label(side, text="Smart Campus Navigator",
                 font=("Helvetica", 14, "bold")).pack(anchor="w")
        tk.Label(side, text="f(n) = g(n) + h(n)   g = walking cost so far, "
                 "h = straight-line distance\nto nearest goal",
                 justify="left", fg="#444", font=("Helvetica", 9)).pack(anchor="w")

        # mode
        mf = ttk.LabelFrame(side, text="Mode")
        mf.pack(fill="x", pady=4)
        for text, val in (("Normal navigation", "nav"),
                          ("Fire evacuation (nearest assembly point)", "fire"),
                          ("Medical emergency (to Medical Centre)", "medical")):
            ttk.Radiobutton(mf, text=text, value=val, variable=self.mode,
                            command=self.on_inputs_changed).pack(anchor="w")

        # start / destination
        sf = ttk.LabelFrame(side, text="Route")
        sf.pack(fill="x", pady=4)
        values = [n.name for n in self.graph.nodes.values()]
        ttk.Label(sf, text="Start").grid(row=0, column=0, sticky="w")
        self.start_box = ttk.Combobox(sf, values=values, state="readonly", width=30)
        self.start_box.grid(row=0, column=1, padx=4, pady=1)
        ttk.Label(sf, text="Destination").grid(row=1, column=0, sticky="w")
        self.dest_box = ttk.Combobox(sf, values=values, state="readonly", width=30)
        self.dest_box.grid(row=1, column=1, padx=4, pady=1)
        self.start_box.bind("<<ComboboxSelected>>", self.on_combo)
        self.dest_box.bind("<<ComboboxSelected>>", self.on_combo)
        self.sync_boxes()

        # search buttons
        bf = ttk.LabelFrame(side, text="A* search")
        bf.pack(fill="x", pady=4)
        row = tk.Frame(bf); row.pack(fill="x")
        ttk.Button(row, text="Step", command=self.step_once).pack(side="left", padx=2)
        self.auto_btn = ttk.Button(row, text="Auto-play", command=self.toggle_auto)
        self.auto_btn.pack(side="left", padx=2)
        ttk.Button(row, text="Run to end",
                   command=lambda: self.run_all(animate=False)).pack(side="left", padx=2)
        ttk.Button(row, text="Reset", command=lambda: self.new_search()).pack(side="left", padx=2)
        sp = tk.Frame(bf); sp.pack(fill="x")
        tk.Label(sp, text="Speed").pack(side="left")
        self.speed = tk.Scale(sp, from_=1, to=10, orient="horizontal", length=200)
        self.speed.set(5)
        self.speed.pack(side="left")

        # scenario buttons
        cf = ttk.LabelFrame(side, text="Scenario")
        cf.pack(fill="x", pady=4)
        r1 = tk.Frame(cf); r1.pack(fill="x")
        ttk.Button(r1, text="Walk the route", command=self.start_walk).pack(side="left", padx=2)
        ttk.Button(r1, text="Block a corridor on route",
                   command=self.block_random).pack(side="left", padx=2)
        r2 = tk.Frame(cf); r2.pack(fill="x")
        ttk.Button(r2, text="Clear blockages", command=self.clear_blocks).pack(side="left", padx=2)
        ttk.Button(r2, text="Compare with Dijkstra",
                   command=self.compare).pack(side="left", padx=2)

        # stats
        self.stats = tk.Label(side, text="", justify="left", anchor="w",
                              font=("Helvetica", 9, "bold"), fg="#222")
        self.stats.pack(fill="x", pady=2)

        # OPEN list table
        tk.Label(side, text="OPEN list (sorted by f - top row is expanded next)",
                 font=("Helvetica", 9, "bold")).pack(anchor="w")
        cols = ("node", "g", "h", "f")
        self.tree = ttk.Treeview(side, columns=cols, show="headings", height=7)
        for c, w in zip(cols, (190, 50, 50, 50)):
            self.tree.heading(c, text=c)
            self.tree.column(c, width=w, anchor="w" if c == "node" else "e")
        self.tree.tag_configure("next", background="#ffe0b2")
        self.tree.pack(fill="x")

        # explanation log
        tk.Label(side, text="Why did A* decide this?",
                 font=("Helvetica", 9, "bold")).pack(anchor="w", pady=(6, 0))
        lf = tk.Frame(side); lf.pack(fill="both", expand=True)
        self.log_box = tk.Text(lf, width=52, height=14, wrap="word",
                               font=("Courier", 9), bg="#fcfcfc")
        sb = ttk.Scrollbar(lf, command=self.log_box.yview)
        self.log_box.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.log_box.pack(side="left", fill="both", expand=True)
        for tag, colour in (("head", "#000"), ("add", "#1b5e20"), ("upd", "#0d47a1"),
                            ("skip", "#777"), ("block", "#b71c1c"),
                            ("goal", "#1b5e20"), ("keep", "#555")):
            self.log_box.tag_configure(tag, foreground=colour)
        self.log_box.tag_configure("head", font=("Courier", 9, "bold"))
        self.log_box.tag_configure("goal", font=("Courier", 9, "bold"))

    # ------------------------------------------------------------- helpers --
    def goals(self):
        m = self.mode.get()
        if m == "fire":
            return list(ASSEMBLY_POINTS)
        if m == "medical":
            return [MEDICAL_CENTRE]
        return [self.dest]

    def sync_boxes(self) -> None:
        self.start_box.set(self.names[self.start])
        m = self.mode.get()
        if m == "fire":
            self.dest_box.set("(nearest of 3 assembly points)")
            self.dest_box.state(["disabled"])
        elif m == "medical":
            self.dest_box.set(self.names[MEDICAL_CENTRE])
            self.dest_box.state(["disabled"])
        else:
            self.dest_box.state(["!disabled"])
            self.dest_box.state(["readonly"])
            self.dest_box.set(self.names[self.dest])

    def log(self, text: str, tag: str = "") -> None:
        self.log_box.insert("end", text + "\n", tag)
        self.log_box.see("end")

    def log_step_notes(self, notes) -> None:
        for line in notes:
            s = line.strip()
            tag = ("add" if s.startswith("[ADD]") else
                   "upd" if s.startswith("[UPDATE]") else
                   "skip" if s.startswith("[SKIP]") else
                   "keep" if s.startswith("[KEEP]") else
                   "block" if s.startswith("[BLOCKED]") else
                   "goal" if s.startswith("GOAL") else
                   "head" if s.startswith("Step") or s.startswith("Start") else "")
            self.log(line, tag)

    # ------------------------------------------------------- state changes --
    def on_combo(self, _event=None) -> None:
        self.start = self.keys_by_name[self.start_box.get()]
        if self.mode.get() == "nav":
            self.dest = self.keys_by_name[self.dest_box.get()]
        self.on_inputs_changed()

    def on_inputs_changed(self) -> None:
        self.stop_walk()
        self.old_path = None
        self.sync_boxes()
        self.new_search()

    def new_search(self, clear: bool = True) -> None:
        self.autoplaying = False
        self.auto_btn.config(text="Auto-play")
        self.gen = astar_steps(self.graph, self.start, self.goals())
        self.step, self.finished, self.path = None, False, None
        if clear:
            self.log_box.delete("1.0", "end")
        goal_txt = ", ".join(self.names[g] for g in self.goals())
        self.log(f"=== New search: {self.names[self.start]} -> {goal_txt} ===", "head")
        if self.graph.blocked:
            self.log(f"{len(self.graph.blocked)} corridor(s) currently blocked.", "block")
        self.tree.delete(*self.tree.get_children())
        self.update_stats()
        self.redraw()

    def step_once(self, draw: bool = True) -> None:
        if self.gen is None or self.finished:
            self.new_search(clear=False)
        try:
            s = next(self.gen)
        except StopIteration:
            self.finished = True
            return
        self.step = s
        self.log_step_notes(s.notes)
        if s.kind == "goal":
            self.finished, self.path = True, s.path
            self.log_route_summary(s)
        elif s.kind == "fail":
            self.finished = True
            self.log("No route exists with the current blockages.", "block")
        if draw:
            self.update_table()
            self.update_stats()
            self.redraw()

    def run_all(self, animate: bool = False) -> None:
        self.autoplaying = False
        guard = 0
        while not self.finished and guard < 1000:
            self.step_once(draw=False)
            guard += 1
        self.update_table()
        self.update_stats()
        self.redraw()

    def toggle_auto(self) -> None:
        if self.autoplaying:
            self.autoplaying = False
            self.auto_btn.config(text="Auto-play")
            return
        if self.finished:
            self.new_search(clear=False)
        self.autoplaying = True
        self.auto_btn.config(text="Pause")
        self.auto_tick()

    def auto_tick(self) -> None:
        if not self.autoplaying:
            return
        self.step_once()
        if self.finished:
            self.autoplaying = False
            self.auto_btn.config(text="Auto-play")
            return
        self.root.after(1650 - self.speed.get() * 150, self.auto_tick)

    def log_route_summary(self, s) -> None:
        legs = []
        for a, b in zip(s.path, s.path[1:]):
            legs.append(f"{self.names[a]} -({self.graph.adj[a][b].cost:.0f})-> ")
        legs.append(self.names[s.path[-1]])
        mins = s.cost / WALKING_SPEED_M_PER_MIN
        self.log("ROUTE: " + "".join(legs), "goal")
        self.log(f"Total cost {s.cost:.0f}  (~{mins:.1f} min walk), "
                 f"{len(s.closed)} nodes expanded.", "goal")

    def update_table(self) -> None:
        self.tree.delete(*self.tree.get_children())
        if not self.step:
            return
        for i, (f, g, h, n) in enumerate(self.step.open_list):
            self.tree.insert("", "end", values=(self.names[n], f"{g:.0f}",
                                                f"{h:.0f}", f"{f:.0f}"),
                             tags=("next",) if i == 0 else ())

    def update_stats(self) -> None:
        if not self.step:
            self.stats.config(text="Press Step / Auto-play / Run to end.")
            return
        s = self.step
        txt = f"Step {s.number}   CLOSED: {len(s.closed)}   OPEN: {len(s.open_list)}"
        if self.path:
            txt += (f"\nRoute cost {s.cost:.0f}  (~{s.cost / WALKING_SPEED_M_PER_MIN:.1f}"
                    f" min walk)")
        elif self.finished:
            txt += "\nNO ROUTE AVAILABLE"
        self.stats.config(text=txt)

    # -------------------------------------------------------- user actions --
    def clear_blocks(self) -> None:
        self.graph.clear_blocks()
        self.log("All corridors reopened.", "head")
        self.after_block_change(None)

    def block_random(self) -> None:
        """Close one corridor that lies on the current route (demo helper)."""
        if self.path and len(self.path) > 1:
            if self.walking:
                pairs = list(zip(self.route[self.route_i:],
                                 self.route[self.route_i + 1:]))
            else:
                pairs = list(zip(self.path, self.path[1:]))
            pairs = [p for p in pairs if not self.graph.is_blocked(*p)]
            if pairs:
                a, b = random.choice(pairs)
                self.toggle_edge(a, b)
                return
        free = [e for e in self.graph.edges if not self.graph.is_blocked(e.a, e.b)]
        if free:
            e = random.choice(free)
            self.toggle_edge(e.a, e.b)

    def compare(self) -> None:
        goals = self.goals()
        a = astar(self.graph, self.start, goals, euclidean_heuristic)
        d = astar(self.graph, self.start, goals, zero_heuristic)
        self.log("=== A* vs Dijkstra (h = 0) on this query ===", "head")
        if a.path is None:
            self.log("No route exists, nothing to compare.", "block")
            return
        self.log(f"A*       : cost {a.cost:.0f}, nodes expanded {a.expanded}", "add")
        self.log(f"Dijkstra : cost {d.cost:.0f}, nodes expanded {d.expanded}", "upd")
        saved = d.expanded - a.expanded
        self.log(f"Same optimal cost, A* expanded {saved} fewer node(s) "
                 f"because h steers it toward the goal.", "goal")

    # --- hit testing --------------------------------------------------------
    def hit_node(self, x: float, y: float):
        for n in self.graph.nodes.values():
            r = 11 if n.kind == "junction" else 17
            if math.hypot(n.x - x, n.y - y) <= r:
                return n.key
        return None

    def hit_edge(self, x: float, y: float):
        for e in self.graph.edges:
            a, b = self.graph.nodes[e.a], self.graph.nodes[e.b]
            dx, dy = b.x - a.x, b.y - a.y
            t = max(0, min(1, ((x - a.x) * dx + (y - a.y) * dy) / (dx * dx + dy * dy)))
            if math.hypot(a.x + t * dx - x, a.y + t * dy - y) <= 7:
                return e
        return None

    def on_left_click(self, ev) -> None:
        k = self.hit_node(ev.x, ev.y)
        if k:
            self.start = k
            self.on_inputs_changed()
            return
        e = self.hit_edge(ev.x, ev.y)
        if e:
            self.toggle_edge(e.a, e.b)

    def on_right_click(self, ev) -> None:
        k = self.hit_node(ev.x, ev.y)
        if k:
            if self.mode.get() == "nav":
                self.dest = k
                self.on_inputs_changed()
            else:
                self.log("Destination is fixed in emergency modes.", "block")
            return
        e = self.hit_edge(ev.x, ev.y)
        if e:
            self.toggle_edge(e.a, e.b)

    # --- blockage handling & rerouting -------------------------------------
    def toggle_edge(self, a: str, b: str) -> None:
        now_blocked = self.graph.toggle_block(a, b)
        word = "BLOCKED" if now_blocked else "reopened"
        self.log(f"Corridor {self.names[a]} - {self.names[b]} {word}.",
                 "block" if now_blocked else "head")
        if self.walking:
            remaining = list(zip(self.route[self.route_i:],
                                 self.route[self.route_i + 1:]))
            if now_blocked and any(frozenset(p) == frozenset((a, b)) for p in remaining):
                self.reroute_from(self.route[self.route_i],
                                  "ALERT: blockage ahead on the current route "
                                  "-> re-running A* from the current position.")
            else:
                self.redraw()
            return
        self.after_block_change(frozenset((a, b)))

    def after_block_change(self, _edge) -> None:
        had_route = self.path is not None
        self.old_path = self.path if had_route else None
        self.new_search(clear=False)
        if had_route:
            self.run_all()

    def reroute_from(self, cur: str, message: str) -> None:
        remaining = self.route[self.route_i:]
        self.log(message, "block")
        self.start = cur
        self.sync_boxes()
        self.new_search(clear=False)
        self.run_all()
        if self.path:
            self.old_path = remaining
            self.route, self.route_i = list(self.path), 0
            self.log("Walker continues on the NEW route.", "goal")
        else:
            self.walking = False
            self.log("Walker is stranded: no alternative route.", "block")
        self.redraw()

    # --- walker animation ---------------------------------------------------
    def start_walk(self) -> None:
        if not self.finished:
            self.run_all()
        if not self.path:
            self.log("Cannot walk: no route available.", "block")
            return
        self.old_path = None
        self.route, self.route_i = list(self.path), 0
        self.walker, self.walking = self.route[0], True
        self.log("Walker started. Click a corridor ahead to block it "
                 "and watch A* reroute.", "head")
        self.redraw()
        self.root.after(700, self.walk_tick)

    def stop_walk(self) -> None:
        self.walking, self.walker = False, None

    def walk_tick(self) -> None:
        if not self.walking:
            return
        if self.route_i >= len(self.route) - 1:
            self.walking = False
            self.log(f"Arrived at {self.names[self.route[-1]]}.", "goal")
            self.redraw()
            return
        cur, nxt = self.route[self.route_i], self.route[self.route_i + 1]
        if self.graph.is_blocked(cur, nxt):          # safety net
            self.reroute_from(cur, "Blocked ahead -> rerouting.")
            if not self.walking:
                return
        else:
            self.route_i += 1
            self.walker = self.route[self.route_i]
            self.redraw()
        self.root.after(700, self.walk_tick)

    # -------------------------------------------------------------- drawing --
    def redraw(self) -> None:
        c = self.canvas
        c.delete("all")
        g = self.graph
        s = self.step
        N = g.nodes

        # decorative zones
        c.create_rectangle(370, 240, 600, 300, fill="#e3efd8", outline="")
        c.create_text(485, 255, text="Central Lawn", fill="#7a9a63", font=("Helvetica", 9, "italic"))
        c.create_rectangle(700, 380, 860, 520, fill="#e3efd8", outline="")
        c.create_text(780, 395, text="Sports Ground", fill="#7a9a63", font=("Helvetica", 9, "italic"))

        # glow for the search tree (parent pointers)
        if s:
            for v, u in s.came_from.items():
                c.create_line(N[u].x, N[u].y, N[v].x, N[v].y, fill="#9ec3ff", width=8)

        path_edges = set()
        if self.path:
            path_edges = {frozenset(p) for p in zip(self.path, self.path[1:])}
        old_edges = set()
        if self.old_path:
            old_edges = {frozenset(p) for p in zip(self.old_path, self.old_path[1:])}

        # corridors
        for e in g.edges:
            a, b = N[e.a], N[e.b]
            key = frozenset((e.a, e.b))
            colour = EDGE_TYPES[e.kind][1]
            if key in old_edges:
                c.create_line(a.x, a.y, b.x, b.y, fill="#b0b0b0", width=6, dash=(3, 5))
            if g.is_blocked(e.a, e.b):
                c.create_line(a.x, a.y, b.x, b.y, fill="#d32f2f", width=4, dash=(8, 5))
                mx, my = (a.x + b.x) / 2, (a.y + b.y) / 2
                c.create_text(mx, my, text="X", fill="#d32f2f", font=("Helvetica", 16, "bold"))
                continue
            if key in path_edges:
                c.create_line(a.x, a.y, b.x, b.y, fill=COL_PATH, width=8)
            c.create_line(a.x, a.y, b.x, b.y, fill=colour,
                          width=4 if e.kind == "road" else 2)
            mx, my = (a.x + b.x) / 2, (a.y + b.y) / 2
            c.create_rectangle(mx - 11, my - 7, mx + 11, my + 7, fill=BG, outline="")
            c.create_text(mx, my, text=f"{e.cost:.0f}", fill="#333", font=("Helvetica", 8))

        # nodes
        goals = set(self.goals())
        open_nodes = {n: f for f, _, _, n in s.open_list} if s else {}
        closed = s.closed if s else set()
        path_nodes = set(self.path or [])
        for n in N.values():
            r = 7 if n.kind == "junction" else 14
            fill = KIND_FILL[n.kind]
            if n.key in closed:
                fill = COL_CLOSED
            if n.key in open_nodes:
                fill = COL_OPEN
            if s and s.current == n.key:
                fill = COL_CURRENT
            outline, width = "#444", 2
            if n.key in path_nodes:
                outline, width = COL_PATH, 4
            if n.key in goals:
                outline, width = COL_GOAL, 4
            if n.key == self.start:
                outline, width = COL_START, 4
            c.create_oval(n.x - r, n.y - r, n.x + r, n.y + r, fill=fill,
                          outline=outline, width=width)
            sym = KIND_SYMBOL.get(n.kind)
            if sym:
                c.create_text(n.x, n.y, text=sym, font=("Helvetica", 10, "bold"))
            if n.kind == "junction":
                c.create_text(n.x + 14, n.y - 10, text=n.key, fill="#666", font=("Helvetica", 8))
            else:
                c.create_text(n.x, n.y + r + 9, text=n.name, font=("Helvetica", 9, "bold"))
            if n.key in open_nodes:
                gv = s.g_score[n.key]
                c.create_text(n.x, n.y - r - 8, text=f"f={open_nodes[n.key]:.0f}",
                              fill="#8a6d00", font=("Helvetica", 8, "bold"))
            elif n.key in closed:
                c.create_text(n.x, n.y - r - 8, text=f"g={s.g_score[n.key]:.0f}",
                              fill="#1a4f8a", font=("Helvetica", 8))

        # walker
        if self.walker:
            w = N[self.walker]
            c.create_oval(w.x - 9, w.y - 9, w.x + 9, w.y + 9, fill="#2962ff", outline="white", width=2)
            c.create_text(w.x, w.y - 22, text="YOU", fill="#2962ff", font=("Helvetica", 9, "bold"))

        self.draw_legend()

    def draw_legend(self) -> None:
        c = self.canvas
        items = [(COL_CURRENT, "expanding"), (COL_OPEN, "OPEN (f shown)"),
                 (COL_CLOSED, "CLOSED (g shown)"), (COL_PATH, "final route")]
        x = 14
        for colour, label in items:
            c.create_rectangle(x, 628, x + 14, 642, fill=colour, outline="#444")
            c.create_text(x + 20, 635, text=label, anchor="w", font=("Helvetica", 8))
            x += 40 + 7 * len(label)
        c.create_text(x, 635, anchor="w", fill="#444", font=("Helvetica", 8),
                      text="green ring = start, pink ring = goal, red X = blocked, "
                           "numbers on paths = walking cost")


def main() -> None:
    root = tk.Tk()
    NavigatorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
