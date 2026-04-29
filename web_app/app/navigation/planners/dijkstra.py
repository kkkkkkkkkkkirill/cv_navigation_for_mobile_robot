"""Dijkstra — A* with a zero heuristic.

Kept as a separate class (instead of `AStarPlanner(heuristic_weight=0)`)
to make benchmark plots self-explanatory.
"""
from __future__ import annotations

import heapq
import math
import time
from itertools import count

from ..maps import Cell, GridMap
from .astar import step_cost
from .base import PlanResult, Planner


class DijkstraPlanner(Planner):
    name = "dijkstra"

    def plan(self, grid: GridMap, start: Cell, goal: Cell) -> PlanResult:
        t0 = time.perf_counter()
        if not grid.is_free(start) or not grid.is_free(goal):
            return PlanResult(path=[], success=False, elapsed_s=time.perf_counter() - t0)

        tiebreak = count()
        open_heap: list[tuple[float, int, Cell]] = []
        dist: dict[Cell, float] = {start: 0.0}
        came_from: dict[Cell, Cell] = {}
        heapq.heappush(open_heap, (0.0, next(tiebreak), start))
        visited: set[Cell] = set()
        expansions = 0

        while open_heap:
            d, _, current = heapq.heappop(open_heap)
            if current in visited:
                continue
            visited.add(current)
            expansions += 1

            if current == goal:
                return PlanResult(
                    path=_reconstruct(came_from, current),
                    success=True,
                    cost=d,
                    expansions=expansions,
                    elapsed_s=time.perf_counter() - t0,
                )

            for nxt in grid.neighbours(current, allow_diagonal=True):
                nd = d + step_cost(current, nxt)
                if nd < dist.get(nxt, math.inf):
                    dist[nxt] = nd
                    came_from[nxt] = current
                    heapq.heappush(open_heap, (nd, next(tiebreak), nxt))

        return PlanResult(
            path=[], success=False, expansions=expansions,
            elapsed_s=time.perf_counter() - t0,
        )


def _reconstruct(came_from: dict[Cell, Cell], end: Cell) -> list[Cell]:
    path = [end]
    while end in came_from:
        end = came_from[end]
        path.append(end)
    path.reverse()
    return path
