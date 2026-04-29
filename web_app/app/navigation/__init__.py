"""Navigation package: grid maps + path planners.

Algorithms planned for comparison in the diploma:

* ``astar``       — classic A* with octile heuristic (baseline).
* ``dijkstra``    — uniform cost, ignores goal direction.
* ``theta_star``  — any-angle A* using line-of-sight parent updates.
* ``jps``         — Jump Point Search, symmetry reduction on uniform grids.
* ``rrt``         — Rapidly-exploring Random Tree / RRT*.
* ``dstar_lite``  — incremental replanning for dynamic obstacles.

Only ``astar`` and ``dijkstra`` are fully implemented right now; the rest
have working stubs so experiments/ can already register them.
"""
