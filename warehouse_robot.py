# warehouse delivery robot
# intelligent systems task 1
#
# how to run
#   python warehouse_robot.py
#   needs python 3 and nothing else since everything is from the standard library
#
# how to change the map
#   go to the GRID line right under the maps and set it to MAP_A or MAP_B or MAP_C
#   or paste your own map as a list of strings and point GRID at it
#   if you set RUN_ALL_MAPS to False it only runs the map in GRID
#   if you leave it True it runs every map in ALL_MAPS and prints the results table at the end

import heapq
import time
from collections import deque

MAP_A = [
    "S..#....",
    "##.#.##.",
    "....~~..",
    ".####.#.",
    "...~..#G"
]

MAP_B = [
    "S~~~~~~~G",
    ".#######.",
    ".........",
    ".#.#.#.#.",
    "........."
]

MAP_C = [
    "S..#....",
    ".#.#.##.",
    "...#..#.",
    ".###.##.",
    "....#..G"
]

# change this one to test a different map
GRID = MAP_A

RUN_ALL_MAPS = True
ALL_MAPS = {"MAP_A": MAP_A, "MAP_B": MAP_B, "MAP_C": MAP_C}

# how much it costs to step into each kind of cell
# S is never entered so it doesnt really matter but G counts as normal floor
CELL_COST = {".": 1, "S": 1, "G": 1, "~": 3}

# up down left right
MOVES = [(-1, 0), (1, 0), (0, -1), (0, 1)]


def find_start_and_goal(grid):
    # just scan every cell until we see S and G
    start = goal = None
    for r, row in enumerate(grid):
        for c, ch in enumerate(row):
            if ch == "S":
                start = (r, c)
            elif ch == "G":
                goal = (r, c)
    return start, goal


def get_neighbors(grid, cell):
    # gives back the cells we can actually walk into along with the cost of entering them
    r, c = cell
    result = []
    for dr, dc in MOVES:
        nr, nc = r + dr, c + dc
        # skip anything off the map
        if nr < 0 or nr >= len(grid) or nc < 0 or nc >= len(grid[nr]):
            continue
        ch = grid[nr][nc]
        # walls are a no go
        if ch == "#":
            continue
        result.append(((nr, nc), CELL_COST[ch]))
    return result


def manhattan(a, b):
    # the robot can only go in 4 directions and every step costs at least 1
    # so this never overestimates which keeps a star optimal
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def rebuild_path(came_from, goal):
    # walk backwards from the goal using the parent links then flip it
    path = [goal]
    while came_from[path[-1]] is not None:
        path.append(came_from[path[-1]])
    path.reverse()
    return path


def path_cost(grid, path):
    # add up the cost of every cell we stepped into and leave out the start
    return sum(CELL_COST[grid[r][c]] for r, c in path[1:])


def bfs(grid, start, goal):
    # plain breadth first search
    # it finds the path with the fewest steps but it has no idea about rough floor
    frontier = deque([start])
    came_from = {start: None}
    expanded = 0
    max_frontier = 1

    while frontier:
        current = frontier.popleft()
        expanded += 1
        if current == goal:
            return rebuild_path(came_from, goal), expanded, max_frontier
        for nxt, _ in get_neighbors(grid, current):
            # only add cells we havent seen yet
            if nxt not in came_from:
                came_from[nxt] = current
                frontier.append(nxt)
        max_frontier = max(max_frontier, len(frontier))

    return None, expanded, max_frontier


def ucs(grid, start, goal):
    # uniform cost search
    # always pops the cheapest cell so far so the first time we pop the goal its the cheapest path
    counter = 0  # tie breaker so heapq never has to compare tuples of cells
    frontier = [(0, counter, start)]
    came_from = {start: None}
    best_g = {start: 0}
    closed = set()
    expanded = 0
    max_frontier = 1

    while frontier:
        g, _, current = heapq.heappop(frontier)
        # an old worse copy of this cell can still be sitting in the heap so skip it
        if current in closed:
            continue
        closed.add(current)
        expanded += 1
        if current == goal:
            return rebuild_path(came_from, goal), expanded, max_frontier
        for nxt, step in get_neighbors(grid, current):
            new_g = g + step
            if nxt not in best_g or new_g < best_g[nxt]:
                best_g[nxt] = new_g
                came_from[nxt] = current
                counter += 1
                heapq.heappush(frontier, (new_g, counter, nxt))
        max_frontier = max(max_frontier, len(frontier))

    return None, expanded, max_frontier


def greedy(grid, start, goal):
    # greedy best first search
    # only looks at how close a cell is to the goal and ignores what it cost to get there
    # fast but not guaranteed to be cheapest
    counter = 0
    frontier = [(manhattan(start, goal), counter, start)]
    came_from = {start: None}
    closed = set()
    expanded = 0
    max_frontier = 1

    while frontier:
        _, _, current = heapq.heappop(frontier)
        if current in closed:
            continue
        closed.add(current)
        expanded += 1
        if current == goal:
            return rebuild_path(came_from, goal), expanded, max_frontier
        for nxt, _ in get_neighbors(grid, current):
            if nxt not in closed and nxt not in came_from:
                came_from[nxt] = current
                counter += 1
                heapq.heappush(frontier, (manhattan(nxt, goal), counter, nxt))
        max_frontier = max(max_frontier, len(frontier))

    return None, expanded, max_frontier


def astar(grid, start, goal):
    # a star search
    # same as ucs but the priority is cost so far plus the manhattan guess to the goal
    # so it heads toward the goal instead of spreading out in every direction
    counter = 0
    frontier = [(manhattan(start, goal), 0, counter, start)]
    came_from = {start: None}
    best_g = {start: 0}
    closed = set()
    expanded = 0
    max_frontier = 1

    while frontier:
        _, g, _, current = heapq.heappop(frontier)
        if current in closed:
            continue
        closed.add(current)
        expanded += 1
        if current == goal:
            return rebuild_path(came_from, goal), expanded, max_frontier
        for nxt, step in get_neighbors(grid, current):
            new_g = g + step
            if nxt not in best_g or new_g < best_g[nxt]:
                best_g[nxt] = new_g
                came_from[nxt] = current
                counter += 1
                f = new_g + manhattan(nxt, goal)
                heapq.heappush(frontier, (f, new_g, counter, nxt))
        max_frontier = max(max_frontier, len(frontier))

    return None, expanded, max_frontier


ALGORITHMS = [
    ("BFS", bfs),
    ("UCS", ucs),
    ("Greedy Best-First", greedy),
    ("A*", astar),
]


def draw_path(grid, path):
    # copy the map and put a star on every path cell except S and G so you can still see them
    rows = [list(row) for row in grid]
    for r, c in path:
        if rows[r][c] not in ("S", "G"):
            rows[r][c] = "*"
    return ["".join(row) for row in rows]


def run_one(name, func, grid, start, goal):
    # time the search and print everything the task asks for
    t0 = time.perf_counter()
    path, expanded, max_frontier = func(grid, start, goal)
    ms = (time.perf_counter() - t0) * 1000

    print(f"Algorithm: {name}")
    if path is None:
        print("No path found")
        print(f"Nodes expanded: {expanded}")
        print(f"Max frontier size: {max_frontier}")
        print(f"Run time: {ms:.3f} ms")
        print()
        return {"algo": name, "found": False, "length": None, "cost": None,
                "expanded": expanded, "frontier": max_frontier, "ms": ms}

    cost = path_cost(grid, path)
    print(f"Path: {path}")
    # length here means number of moves not number of cells
    print(f"Path length: {len(path) - 1} moves")
    print(f"Total cost: {cost}")
    print(f"Nodes expanded: {expanded}")
    print(f"Max frontier size: {max_frontier}")
    print(f"Run time: {ms:.3f} ms")
    for line in draw_path(grid, path):
        print("   " + line)
    print()
    return {"algo": name, "found": True, "length": len(path) - 1, "cost": cost,
            "expanded": expanded, "frontier": max_frontier, "ms": ms}


def run_map(map_name, grid):
    print("=" * 50)
    print(map_name)
    print("=" * 50)
    for line in grid:
        print("   " + line)
    print()
    start, goal = find_start_and_goal(grid)
    if start is None or goal is None:
        print("map is missing S or G")
        return []
    print(f"Start: {start}   Goal: {goal}\n")
    rows = []
    for name, func in ALGORITHMS:
        res = run_one(name, func, grid, start, goal)
        res["map"] = map_name
        rows.append(res)
    return rows


def print_table(rows):
    # one row per map and algorithm combo
    header = f"{'Map':<7}{'Algorithm':<20}{'Length':>8}{'Cost':>7}{'Expanded':>10}{'MaxFrontier':>13}{'Time(ms)':>10}"
    print(header)
    print("-" * len(header))
    for r in rows:
        length = r["length"] if r["found"] else "-"
        cost = r["cost"] if r["found"] else "-"
        print(f"{r['map']:<7}{r['algo']:<20}{str(length):>8}{str(cost):>7}"
              f"{r['expanded']:>10}{r['frontier']:>13}{r['ms']:>10.3f}")


if __name__ == "__main__":
    if RUN_ALL_MAPS:
        all_rows = []
        for map_name, grid in ALL_MAPS.items():
            all_rows += run_map(map_name, grid)
        print("RESULTS TABLE")
        print_table(all_rows)
    else:
        print_table(run_map("GRID", GRID))
