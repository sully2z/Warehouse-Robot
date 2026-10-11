# Warehouse-Robot
FCDS Intelligent Programming Task
A robot in a warehouse has to get from the start cell S to the goal cell G on a grid map. It can move up, down, left or right one cell at a time and can't go through walls. This project finds a path using four search algorithms written from scratch, with no path-finding libraries:

BFS (Breadth-First Search), uninformed
UCS (Uniform Cost Search), uninformed
Greedy Best-First Search, informed (Manhattan distance)
A* Search, informed (Manhattan distance)
Team
Name	ID
Muhammad Ahmed Sulaiman	2403249505
Eman Gamal Ahmed	2403247543
Map symbols
Symbol	Meaning	Cost to enter
S	Start cell	not counted
G	Goal cell	1
.	Normal floor	1
~	Rough floor	3
#	Wall	can't enter
How to run

You only need Python 3. Everything comes from the standard library, so there's nothing to install.

bash
python warehouse_robot.py

By default it runs all four algorithms on MAP_A, MAP_B and MAP_C. For each one it prints:

the path as a list of (row, column)
the path length and total cost
the number of nodes expanded
the max frontier size and the run time
the map with the path marked as *

At the end it prints a results table with everything together. If there's no path it prints No path found.

Changing the map

The maps are defined at the top of warehouse_robot.py. To run a single map:

Set RUN_ALL_MAPS = False
Change GRID = MAP_A to MAP_B, MAP_C, or your own map

To use your own map, add it as a list of strings (one string per row) with exactly one S and one G:

python
MY_MAP = [
    "S..~.",
    ".#.#.",
    "....G"
]

GRID = MY_MAP
Results
Map	Algorithm	Length	Cost	Nodes expanded	Max frontier
MAP_A	BFS	11	15	28	5
MAP_A	UCS	11	15	28	4
MAP_A	Greedy	11	15	14	5
MAP_A	A*	11	15	21	5
MAP_B	BFS	8	22	25	4
MAP_B	UCS	12	12	31	4
MAP_B	Greedy	8	22	9	2
MAP_B	A*	12	12	15	7
MAP_C	all four	No path found	–	13	3

MAP_B is the trap map. BFS and Greedy take the shortest route straight across the rough floor, which costs 22. UCS and A* go around and pay only 12. A* finds that same cheapest path while expanding about half as many nodes as UCS, so it's the one we'd choose for the robot.

Files
File	What it is
warehouse_robot.py	Source code
Search_Algorithms_Report.pdf	Full report: formulation, algorithms, heuristic, results, screenshots and comparison
README.md	This file
