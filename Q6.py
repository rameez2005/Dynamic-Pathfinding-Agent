"""
Dynamic Pathfinding Agent
=========================
Implements Greedy Best-First Search (GBFS) and A* Search with
Manhattan / Euclidean heuristics on a dynamic grid with real-time
obstacle spawning and automatic re-planning.

GUI built with Tkinter.
"""

import tkinter as tk
from tkinter import ttk, messagebox
import heapq
import math
import random
import time

# ─── Colour Palette 
COL_EMPTY      = "#FFFFFF"
COL_WALL       = "#2C3E50"
COL_START      = "#E74C3C"
COL_GOAL       = "#27AE60"
COL_FRONTIER   = "#F1C40F"
COL_VISITED    = "#3498DB"
COL_PATH       = "#2ECC71"
COL_AGENT      = "#E67E22"
COL_GRID_LINE  = "#BDC3C7"
COL_DYNAMIC    = "#8E44AD"   # dynamically spawned obstacles

# ─── Heuristic helpers ─────────────────────────────────────────────────────────
def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def euclidean(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

HEURISTICS = {"Manhattan": manhattan, "Euclidean": euclidean}

# ─── Search algorithms ─────────────────────────────────────────────────────────
def greedy_bfs(grid, start, goal, heuristic, rows, cols):
    """Greedy Best-First Search: f(n) = h(n)"""
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), start))
    came_from = {start: None}
    visited = set()
    frontier_set = {start}
    order_visited = []
    order_frontier = [start]

    while open_set:
        _, current = heapq.heappop(open_set)
        frontier_set.discard(current)

        if current in visited:
            continue
        visited.add(current)
        order_visited.append(current)

        if current == goal:
            path = _reconstruct(came_from, goal)
            return path, order_visited, order_frontier, len(visited)

        for nb in _neighbours(current, grid, rows, cols):
            if nb not in visited and nb not in frontier_set:
                came_from[nb] = current
                heapq.heappush(open_set, (heuristic(nb, goal), nb))
                frontier_set.add(nb)
                order_frontier.append(nb)

    return None, order_visited, order_frontier, len(visited)


def a_star(grid, start, goal, heuristic, rows, cols):
    """A* Search: f(n) = g(n) + h(n)"""
    open_set = []
    g_score = {start: 0}
    f_start = heuristic(start, goal)
    heapq.heappush(open_set, (f_start, start))
    came_from = {start: None}
    visited = set()
    frontier_set = {start}
    order_visited = []
    order_frontier = [start]

    while open_set:
        _, current = heapq.heappop(open_set)
        frontier_set.discard(current)

        if current in visited:
            continue
        visited.add(current)
        order_visited.append(current)

        if current == goal:
            path = _reconstruct(came_from, goal)
            return path, order_visited, order_frontier, len(visited)

        for nb in _neighbours(current, grid, rows, cols):
            tentative_g = g_score[current] + 1
            if nb not in visited and (nb not in g_score or tentative_g < g_score[nb]):
                g_score[nb] = tentative_g
                f = tentative_g + heuristic(nb, goal)
                came_from[nb] = current
                heapq.heappush(open_set, (f, nb))
                frontier_set.add(nb)
                order_frontier.append(nb)

    return None, order_visited, order_frontier, len(visited)


ALGORITHMS = {"Greedy BFS": greedy_bfs, "A*": a_star}


def _neighbours(pos, grid, rows, cols):
    r, c = pos
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
            yield (nr, nc)


def _reconstruct(came_from, current):
    path = []
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


# ─── Main Application ─────────────────────────────────────────────────────────
class PathfindingApp:
    """Tkinter application for the Dynamic Pathfinding Agent."""

    # ── Initialisation ────────────────────────────────────────────────────
    def __init__(self, root):
        self.root = root
        self.root.title("Dynamic Pathfinding Agent")
        self.root.configure(bg="#ECF0F1")

        # defaults
        self.rows = 20
        self.cols = 20
        self.cell_size = 28
        self.obstacle_density = 0.30
        self.dynamic_spawn_prob = 0.05

        # grid: 0 = empty, 1 = wall
        self.grid = [[0] * self.cols for _ in range(self.rows)]
        self.start = (0, 0)
        self.goal = (self.rows - 1, self.cols - 1)

        # visualisation state
        self.cell_rects = {}
        self.running = False
        self.dynamic_mode = False
        self.animation_speed = 30          # ms per step
        self.agent_pos = None
        self.current_path = None

        self._build_gui()
        self._draw_grid()

    # ── GUI Layout ────────────────────────────────────────────────────────
    def _build_gui(self):
        # top control bar
        ctrl = tk.Frame(self.root, bg="#ECF0F1")
        ctrl.pack(side=tk.TOP, fill=tk.X, padx=6, pady=4)

        # --- Row 1: Grid dimensions & density ---
        row1 = tk.Frame(ctrl, bg="#ECF0F1")
        row1.pack(fill=tk.X, pady=2)

        tk.Label(row1, text="Rows:", bg="#ECF0F1").pack(side=tk.LEFT, padx=2)
        self.rows_var = tk.IntVar(value=self.rows)
        tk.Spinbox(row1, from_=5, to=80, textvariable=self.rows_var,
                   width=5).pack(side=tk.LEFT, padx=2)

        tk.Label(row1, text="Cols:", bg="#ECF0F1").pack(side=tk.LEFT, padx=2)
        self.cols_var = tk.IntVar(value=self.cols)
        tk.Spinbox(row1, from_=5, to=80, textvariable=self.cols_var,
                   width=5).pack(side=tk.LEFT, padx=2)

        tk.Label(row1, text="Obstacle %:", bg="#ECF0F1").pack(
            side=tk.LEFT, padx=(12, 2))
        self.density_var = tk.DoubleVar(value=self.obstacle_density * 100)
        tk.Spinbox(row1, from_=0, to=60, textvariable=self.density_var,
                   width=5, increment=5).pack(side=tk.LEFT, padx=2)

        tk.Button(row1, text="Apply Size", command=self._apply_size,
                  bg="#3498DB", fg="white").pack(side=tk.LEFT, padx=6)
        tk.Button(row1, text="Generate Maze", command=self._generate_maze,
                  bg="#9B59B6", fg="white").pack(side=tk.LEFT, padx=4)
        tk.Button(row1, text="Clear Grid", command=self._clear_grid,
                  bg="#E74C3C", fg="white").pack(side=tk.LEFT, padx=4)

        # --- Row 2: Algorithm, Heuristic, Dynamic mode, Start & Goal ---
        row2 = tk.Frame(ctrl, bg="#ECF0F1")
        row2.pack(fill=tk.X, pady=2)

        tk.Label(row2, text="Algorithm:", bg="#ECF0F1").pack(
            side=tk.LEFT, padx=2)
        self.algo_var = tk.StringVar(value="A*")
        ttk.Combobox(row2, textvariable=self.algo_var,
                     values=list(ALGORITHMS.keys()), state="readonly",
                     width=12).pack(side=tk.LEFT, padx=2)

        tk.Label(row2, text="Heuristic:", bg="#ECF0F1").pack(
            side=tk.LEFT, padx=(12, 2))
        self.heur_var = tk.StringVar(value="Manhattan")
        ttk.Combobox(row2, textvariable=self.heur_var,
                     values=list(HEURISTICS.keys()), state="readonly",
                     width=12).pack(side=tk.LEFT, padx=2)

        tk.Label(row2, text="Speed (ms):", bg="#ECF0F1").pack(
            side=tk.LEFT, padx=(12, 2))
        self.speed_var = tk.IntVar(value=self.animation_speed)
        tk.Spinbox(row2, from_=1, to=500, textvariable=self.speed_var,
                   width=5, increment=5).pack(side=tk.LEFT, padx=2)

        self.dynamic_var = tk.BooleanVar(value=False)
        tk.Checkbutton(row2, text="Dynamic Mode", variable=self.dynamic_var,
                       bg="#ECF0F1", command=self._toggle_dynamic
                       ).pack(side=tk.LEFT, padx=12)

        tk.Label(row2, text="Spawn %:", bg="#ECF0F1").pack(
            side=tk.LEFT, padx=2)
        self.spawn_var = tk.DoubleVar(value=self.dynamic_spawn_prob * 100)
        tk.Spinbox(row2, from_=0, to=30, textvariable=self.spawn_var,
                   width=5, increment=1).pack(side=tk.LEFT, padx=2)

        # --- Row 3: Start/Goal entry & action buttons ---
        row3 = tk.Frame(ctrl, bg="#ECF0F1")
        row3.pack(fill=tk.X, pady=2)

        tk.Label(row3, text="Start (r,c):", bg="#ECF0F1").pack(
            side=tk.LEFT, padx=2)
        self.start_var = tk.StringVar(value="0,0")
        tk.Entry(row3, textvariable=self.start_var, width=7).pack(
            side=tk.LEFT, padx=2)

        tk.Label(row3, text="Goal (r,c):", bg="#ECF0F1").pack(
            side=tk.LEFT, padx=(12, 2))
        self.goal_var = tk.StringVar(
            value=f"{self.rows - 1},{self.cols - 1}")
        tk.Entry(row3, textvariable=self.goal_var, width=7).pack(
            side=tk.LEFT, padx=2)

        tk.Button(row3, text="Set Start/Goal", command=self._set_endpoints,
                  bg="#2ECC71", fg="white").pack(side=tk.LEFT, padx=6)
        tk.Button(row3, text="▶  Run Search", command=self._run_search,
                  bg="#27AE60", fg="white",
                  font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT, padx=6)
        tk.Button(row3, text="■  Stop", command=self._stop_search,
                  bg="#C0392B", fg="white").pack(side=tk.LEFT, padx=4)
        tk.Button(row3, text="Reset View", command=self._reset_colours,
                  bg="#7F8C8D", fg="white").pack(side=tk.LEFT, padx=4)

        # --- Canvas ---
        canvas_frame = tk.Frame(self.root, bg="#ECF0F1")
        canvas_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True,
                          padx=6, pady=6)
        self.canvas = tk.Canvas(canvas_frame, bg="white",
                                highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self._canvas_click)
        self.canvas.bind("<B1-Motion>", self._canvas_drag)
        self.canvas.bind("<Button-3>", self._canvas_right_click)
        self.canvas.bind("<B3-Motion>", self._canvas_right_drag)

        # --- Metrics panel (right side) ---
        metrics = tk.Frame(self.root, bg="#ECF0F1", width=220)
        metrics.pack(side=tk.RIGHT, fill=tk.Y, padx=6, pady=6)
        metrics.pack_propagate(False)

        tk.Label(metrics, text="Metrics", font=("Segoe UI", 14, "bold"),
                 bg="#ECF0F1").pack(pady=(8, 12))

        self.metric_labels = {}
        for label in ["Nodes Visited", "Path Cost", "Execution Time (ms)",
                      "Re-plans"]:
            f = tk.Frame(metrics, bg="#ECF0F1")
            f.pack(fill=tk.X, padx=8, pady=4)
            tk.Label(f, text=label + ":", bg="#ECF0F1",
                     font=("Segoe UI", 10)).pack(anchor=tk.W)
            val = tk.Label(f, text="—", bg="#ECF0F1",
                           font=("Segoe UI", 12, "bold"), fg="#2C3E50")
            val.pack(anchor=tk.W)
            self.metric_labels[label] = val

        # legend
        tk.Label(metrics, text="Legend", font=("Segoe UI", 12, "bold"),
                 bg="#ECF0F1").pack(pady=(20, 6))
        legend_items = [
            (COL_START, "Start"), (COL_GOAL, "Goal"),
            (COL_WALL, "Wall"), (COL_FRONTIER, "Frontier"),
            (COL_VISITED, "Visited"), (COL_PATH, "Path"),
            (COL_AGENT, "Agent"), (COL_DYNAMIC, "Dynamic Wall"),
        ]
        for colour, name in legend_items:
            f = tk.Frame(metrics, bg="#ECF0F1")
            f.pack(fill=tk.X, padx=8, pady=1)
            swatch = tk.Canvas(f, width=16, height=16, bg=colour,
                               highlightthickness=1,
                               highlightbackground="#7F8C8D")
            swatch.pack(side=tk.LEFT, padx=(0, 6))
            tk.Label(f, text=name, bg="#ECF0F1",
                     font=("Segoe UI", 9)).pack(side=tk.LEFT)

    # ── Grid Drawing ──────────────────────────────────────────────────────
    def _draw_grid(self):
        self.canvas.delete("all")
        self.cell_rects.clear()
        w = self.cols * self.cell_size
        h = self.rows * self.cell_size
        self.canvas.config(width=w, height=h)

        for r in range(self.rows):
            for c in range(self.cols):
                x1 = c * self.cell_size
                y1 = r * self.cell_size
                x2 = x1 + self.cell_size
                y2 = y1 + self.cell_size
                colour = COL_EMPTY
                if self.grid[r][c] == 1:
                    colour = COL_WALL
                elif self.grid[r][c] == 2:
                    colour = COL_DYNAMIC
                rect = self.canvas.create_rectangle(
                    x1, y1, x2, y2, fill=colour, outline=COL_GRID_LINE,
                    width=1)
                self.cell_rects[(r, c)] = rect

        # highlight start & goal
        self._colour_cell(self.start, COL_START)
        self._colour_cell(self.goal, COL_GOAL)

    def _colour_cell(self, pos, colour):
        if pos in self.cell_rects:
            self.canvas.itemconfig(self.cell_rects[pos], fill=colour)

    # ── Canvas Interaction (toggle walls) ─────────────────────────────────
    def _cell_from_event(self, event):
        c = event.x // self.cell_size
        r = event.y // self.cell_size
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return (r, c)
        return None

    def _canvas_click(self, event):
        cell = self._cell_from_event(event)
        if cell and cell != self.start and cell != self.goal:
            self.grid[cell[0]][cell[1]] = 1
            self._colour_cell(cell, COL_WALL)

    def _canvas_drag(self, event):
        self._canvas_click(event)

    def _canvas_right_click(self, event):
        cell = self._cell_from_event(event)
        if cell and cell != self.start and cell != self.goal:
            self.grid[cell[0]][cell[1]] = 0
            self._colour_cell(cell, COL_EMPTY)

    def _canvas_right_drag(self, event):
        self._canvas_right_click(event)

    # ── Control Callbacks ─────────────────────────────────────────────────
    def _apply_size(self):
        self._stop_search()
        self.rows = self.rows_var.get()
        self.cols = self.cols_var.get()
        # auto-scale cell size so grid fits ~600px
        self.cell_size = max(8, min(36, 600 // max(self.rows, self.cols)))
        self.grid = [[0] * self.cols for _ in range(self.rows)]
        self.start = (0, 0)
        self.goal = (self.rows - 1, self.cols - 1)
        self.start_var.set("0,0")
        self.goal_var.set(f"{self.rows - 1},{self.cols - 1}")
        self._draw_grid()

    def _generate_maze(self):
        self._stop_search()
        density = self.density_var.get() / 100.0
        for r in range(self.rows):
            for c in range(self.cols):
                if (r, c) == self.start or (r, c) == self.goal:
                    self.grid[r][c] = 0
                else:
                    self.grid[r][c] = 1 if random.random() < density else 0
        self._draw_grid()

    def _clear_grid(self):
        self._stop_search()
        self.grid = [[0] * self.cols for _ in range(self.rows)]
        self._draw_grid()

    def _set_endpoints(self):
        try:
            sr, sc = map(int, self.start_var.get().split(","))
            gr, gc = map(int, self.goal_var.get().split(","))
            if not (0 <= sr < self.rows and 0 <= sc < self.cols):
                raise ValueError
            if not (0 <= gr < self.rows and 0 <= gc < self.cols):
                raise ValueError
            # clear old positions
            old_start, old_goal = self.start, self.goal
            self.start = (sr, sc)
            self.goal = (gr, gc)
            self.grid[sr][sc] = 0
            self.grid[gr][gc] = 0
            # repaint old cells
            self._colour_cell(old_start,
                              COL_WALL if self.grid[old_start[0]][old_start[1]] == 1 else COL_EMPTY)
            self._colour_cell(old_goal,
                              COL_WALL if self.grid[old_goal[0]][old_goal[1]] == 1 else COL_EMPTY)
            self._colour_cell(self.start, COL_START)
            self._colour_cell(self.goal, COL_GOAL)
        except Exception:
            messagebox.showerror("Invalid input",
                                 "Enter start and goal as  row,col  (0-indexed).")

    def _toggle_dynamic(self):
        self.dynamic_mode = self.dynamic_var.get()

    def _stop_search(self):
        self.running = False

    def _reset_colours(self):
        """Reset visualisation colours without changing walls."""
        self._stop_search()
        for r in range(self.rows):
            for c in range(self.cols):
                if (r, c) == self.start:
                    self._colour_cell((r, c), COL_START)
                elif (r, c) == self.goal:
                    self._colour_cell((r, c), COL_GOAL)
                elif self.grid[r][c] == 1:
                    self._colour_cell((r, c), COL_WALL)
                elif self.grid[r][c] == 2:
                    self._colour_cell((r, c), COL_DYNAMIC)
                else:
                    self._colour_cell((r, c), COL_EMPTY)
        self._update_metrics(0, 0, 0.0, 0)

    # ── Metrics helpers ───────────────────────────────────────────────────
    def _update_metrics(self, visited, cost, time_ms, replans):
        self.metric_labels["Nodes Visited"].config(text=str(visited))
        self.metric_labels["Path Cost"].config(
            text=str(cost) if cost else "—")
        self.metric_labels["Execution Time (ms)"].config(
            text=f"{time_ms:.2f}")
        self.metric_labels["Re-plans"].config(text=str(replans))

    # ── Run Search (animated) ─────────────────────────────────────────────
    def _run_search(self):
        if self.running:
            return
        self._reset_colours()
        self.running = True
        self.animation_speed = self.speed_var.get()
        self.dynamic_mode = self.dynamic_var.get()
        self.dynamic_spawn_prob = self.spawn_var.get() / 100.0

        algo_fn = ALGORITHMS[self.algo_var.get()]
        heur_fn = HEURISTICS[self.heur_var.get()]

        # run search
        t0 = time.perf_counter()
        path, visited, frontier, n_visited = algo_fn(
            self.grid, self.start, self.goal, heur_fn, self.rows, self.cols)
        t1 = time.perf_counter()
        elapsed_ms = (t1 - t0) * 1000

        if path is None:
            self.running = False
            self._update_metrics(n_visited, 0, elapsed_ms, 0)
            messagebox.showinfo("No Path", "No path found to the goal!")
            return

        # begin animated visualisation
        self._animate_search(visited, frontier, path, n_visited, elapsed_ms,
                             algo_fn, heur_fn)

    def _animate_search(self, visited, frontier, path, n_visited, elapsed_ms,
                        algo_fn, heur_fn):
        """Step-by-step animation: frontier → visited → path → agent walk."""
        step = {"idx": 0, "phase": "frontier", "replans": 0,
                "total_visited": n_visited, "total_time": elapsed_ms}

        def tick():
            if not self.running:
                return
            phase = step["phase"]
            idx = step["idx"]

            # Phase 1: show frontier expansion
            if phase == "frontier":
                if idx < len(frontier):
                    cell = frontier[idx]
                    if cell != self.start and cell != self.goal:
                        self._colour_cell(cell, COL_FRONTIER)
                    step["idx"] += 1
                    self.root.after(max(1, self.animation_speed // 3), tick)
                else:
                    step["phase"] = "visited"
                    step["idx"] = 0
                    self.root.after(1, tick)

            # Phase 2: overlay visited nodes
            elif phase == "visited":
                if idx < len(visited):
                    cell = visited[idx]
                    if cell != self.start and cell != self.goal:
                        self._colour_cell(cell, COL_VISITED)
                    step["idx"] += 1
                    self.root.after(max(1, self.animation_speed // 2), tick)
                else:
                    step["phase"] = "path"
                    step["idx"] = 0
                    self.root.after(1, tick)

            # Phase 3: draw final path
            elif phase == "path":
                if idx < len(path):
                    cell = path[idx]
                    if cell != self.start and cell != self.goal:
                        self._colour_cell(cell, COL_PATH)
                    step["idx"] += 1
                    self.root.after(self.animation_speed, tick)
                else:
                    # update metrics with path cost
                    self._update_metrics(
                        step["total_visited"], len(path) - 1,
                        step["total_time"], step["replans"])
                    step["phase"] = "walk"
                    step["idx"] = 0
                    self.current_path = list(path)
                    self.root.after(1, tick)

            # Phase 4: agent walks the path + dynamic obstacles
            elif phase == "walk":
                if idx < len(self.current_path):
                    pos = self.current_path[idx]
                    # erase previous agent marker
                    if idx > 0:
                        prev = self.current_path[idx - 1]
                        if prev == self.start:
                            self._colour_cell(prev, COL_START)
                        elif prev == self.goal:
                            self._colour_cell(prev, COL_GOAL)
                        else:
                            self._colour_cell(prev, COL_PATH)
                    # draw agent
                    self._colour_cell(pos, COL_AGENT)
                    self.agent_pos = pos

                    # --- Dynamic obstacle spawning ---
                    if self.dynamic_mode and idx < len(self.current_path) - 1:
                        blocked = self._spawn_dynamic_obstacles()
                        # check if any future path cell is now blocked
                        remaining = self.current_path[idx + 1:]
                        collision = any(
                            self.grid[r][c] != 0 for r, c in remaining)
                        if collision or blocked:
                            # re-plan from current position
                            t0 = time.perf_counter()
                            new_path, new_vis, new_fr, new_nv = algo_fn(
                                self.grid, pos, self.goal, heur_fn,
                                self.rows, self.cols)
                            t1 = time.perf_counter()
                            step["total_time"] += (t1 - t0) * 1000
                            step["total_visited"] += new_nv
                            step["replans"] += 1

                            if new_path is None:
                                self.running = False
                                self._update_metrics(
                                    step["total_visited"], 0,
                                    step["total_time"], step["replans"])
                                messagebox.showinfo(
                                    "Blocked",
                                    "Dynamic obstacle blocked all paths!")
                                return

                            # quickly visualise new path
                            for cell in new_path:
                                if (cell != self.start and cell != self.goal
                                        and cell != pos):
                                    self._colour_cell(cell, COL_PATH)
                            self.current_path = new_path
                            step["idx"] = 0
                            self._update_metrics(
                                step["total_visited"],
                                len(self.current_path) - 1,
                                step["total_time"], step["replans"])
                            self.root.after(self.animation_speed, tick)
                            return

                    step["idx"] += 1
                    self.root.after(self.animation_speed, tick)
                else:
                    # reached goal
                    self._colour_cell(self.goal, COL_GOAL)
                    self._update_metrics(
                        step["total_visited"],
                        len(self.current_path) - 1,
                        step["total_time"], step["replans"])
                    self.running = False
        tick()

    # ── Dynamic obstacle spawning ─────────────────────────────────────────
    def _spawn_dynamic_obstacles(self):
        """Spawn a few random obstacles. Returns True if any were placed."""
        placed = False
        prob = self.dynamic_spawn_prob
        # attempt to place a small number of obstacles each step
        attempts = max(1, int(self.rows * self.cols * prob * 0.05))
        for _ in range(attempts):
            if random.random() < prob:
                r = random.randint(0, self.rows - 1)
                c = random.randint(0, self.cols - 1)
                if ((r, c) != self.start and (r, c) != self.goal
                        and self.grid[r][c] == 0
                        and (r, c) != self.agent_pos):
                    self.grid[r][c] = 2
                    self._colour_cell((r, c), COL_DYNAMIC)
                    placed = True
        return placed


# ─── Entry Point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1050x700")
    app = PathfindingApp(root)
    root.mainloop()
