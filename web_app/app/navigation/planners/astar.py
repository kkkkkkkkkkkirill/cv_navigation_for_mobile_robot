"""Classic A* with the octile heuristic.

This is the baseline planner in the diploma comparison. The octile
heuristic is admissible on 8-connected grids with uniform cost and gives
stronger guidance than Manhattan, so A* here should dominate Dijkstra on
the same grid in expansions.
"""
from __future__ import annotations

import heapq
import math
import time
from itertools import count

from ..maps import Cell, GridMap
from .base import PlanResult, Planner

SQRT2 = math.sqrt(2.0)


def octile(a: Cell, b: Cell) -> float:
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return (dx + dy) + (SQRT2 - 2) * min(dx, dy)


def step_cost(a: Cell, b: Cell) -> float:
    return SQRT2 if (a[0] != b[0] and a[1] != b[1]) else 1.0


class AStarPlanner(Planner):
    name = "astar"

    def __init__(self, heuristic_weight: float = 1.0) -> None:
        # w > 1 → weighted A* (inadmissible but faster); useful for the
        # hyperparameter sensitivity analysis section of the diploma.
        self.heuristic_weight = heuristic_weight

    def plan(self, grid: GridMap, start: Cell, goal: Cell) -> PlanResult:
        t0 = time.perf_counter()
        if not grid.is_free(start) or not grid.is_free(goal):
            return PlanResult(path=[], success=False, elapsed_s=time.perf_counter() - t0)

        tiebreak = count()  # strict ordering when f-scores tie
        open_heap: list[tuple[float, int, Cell]] = []
        g_score: dict[Cell, float] = {start: 0.0}
        came_from: dict[Cell, Cell] = {}
        heapq.heappush(open_heap, (0.0, next(tiebreak), start))
        closed: set[Cell] = set()
        expansions = 0

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)
            expansions += 1

            if current == goal:
                return PlanResult(
                    path=_reconstruct(came_from, current),
                    success=True,
                    cost=g_score[current],
                    expansions=expansions,
                    elapsed_s=time.perf_counter() - t0,
                )

            for nxt in grid.neighbours(current, allow_diagonal=True):
                tentative = g_score[current] + step_cost(current, nxt)
                if tentative < g_score.get(nxt, math.inf):
                    g_score[nxt] = tentative
                    came_from[nxt] = current
                    f = tentative + self.heuristic_weight * octile(nxt, goal)
                    heapq.heappush(open_heap, (f, next(tiebreak), nxt))

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
