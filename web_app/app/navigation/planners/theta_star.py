"""Theta* — any-angle planner built on A*.

Differs from A* only in the ``update_vertex`` step: when expanding a node
whose parent has line-of-sight to the current node's neighbour, the
neighbour's parent is set to the grandparent instead. The resulting path
is no longer grid-locked and produces fewer sharp turns — a property that
matters for a real robot chassis with limited steering agility.

Reference: Daniel, Nash, Koenig, Felner — "Theta*: Any-Angle Path
Planning on Grids", JAIR 2010.
"""
from __future__ import annotations

import heapq
import math
import time
from itertools import count

from ..maps import Cell, GridMap
from .astar import octile
from .base import PlanResult, Planner


def _euclidean(a: Cell, b: Cell) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class ThetaStarPlanner(Planner):
    name = "theta_star"

    def plan(self, grid: GridMap, start: Cell, goal: Cell) -> PlanResult:
        t0 = time.perf_counter()
        if not grid.is_free(start) or not grid.is_free(goal):
            return PlanResult(path=[], success=False, elapsed_s=time.perf_counter() - t0)

        tiebreak = count()
        open_heap: list[tuple[float, int, Cell]] = []
        g_score: dict[Cell, float] = {start: 0.0}
        parent: dict[Cell, Cell] = {start: start}
        heapq.heappush(open_heap, (octile(start, goal), next(tiebreak), start))
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
                    path=_reconstruct(parent, current),
                    success=True,
                    cost=g_score[current],
                    expansions=expansions,
                    elapsed_s=time.perf_counter() - t0,
                )

            for nxt in grid.neighbours(current, allow_diagonal=True):
                if nxt in closed:
                    continue
                # --- Theta*'s any-angle update -----------------------------
                par = parent[current]
                if grid.line_of_sight(par, nxt):
                    tentative = g_score[par] + _euclidean(par, nxt)
                    new_parent = par
                else:
                    tentative = g_score[current] + _euclidean(current, nxt)
                    new_parent = current

                if tentative < g_score.get(nxt, math.inf):
                    g_score[nxt] = tentative
                    parent[nxt] = new_parent
                    f = tentative + octile(nxt, goal)
                    heapq.heappush(open_heap, (f, next(tiebreak), nxt))

        return PlanResult(
            path=[], success=False, expansions=expansions,
            elapsed_s=time.perf_counter() - t0,
        )


def _reconstruct(parent: dict[Cell, Cell], end: Cell) -> list[Cell]:
    path = [end]
    while parent[end] != end:
        end = parent[end]
        path.append(end)
    path.reverse()
    return path
