"""Rapidly-exploring Random Tree — minimal working implementation.

Sampling-based planners are usually studied in continuous configuration
spaces, but on a grid they are still useful for comparison: their time
and path length statistics are very different from A*-family planners.

This implementation uses grid cells as configurations and a fixed
step size. It is intentionally small — a full RRT* rewire pass is left
for the benchmark-phase extension.
"""
from __future__ import annotations

import math
import random
import time

from ..maps import Cell, GridMap
from .base import PlanResult, Planner


class RRTPlanner(Planner):
    name = "rrt"

    def __init__(
        self,
        max_iterations: int = 5000,
        step_size: int = 3,
        goal_bias: float = 0.1,
        seed: int | None = 42,
    ) -> None:
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.rng = random.Random(seed)

    def plan(self, grid: GridMap, start: Cell, goal: Cell) -> PlanResult:
        t0 = time.perf_counter()
        if not grid.is_free(start) or not grid.is_free(goal):
            return PlanResult(path=[], success=False, elapsed_s=time.perf_counter() - t0)

        tree: dict[Cell, Cell | None] = {start: None}
        nodes: list[Cell] = [start]

        for it in range(self.max_iterations):
            if self.rng.random() < self.goal_bias:
                sample = goal
            else:
                sample = (
                    self.rng.randrange(grid.width),
                    self.rng.randrange(grid.height),
                )

            nearest = min(nodes, key=lambda c: _sq_dist(c, sample))
            new_cell = _steer(nearest, sample, self.step_size)
            if not grid.is_free(new_cell):
                continue
            if not grid.line_of_sight(nearest, new_cell):
                continue
            if new_cell in tree:
                continue

            tree[new_cell] = nearest
            nodes.append(new_cell)

            if _sq_dist(new_cell, goal) <= self.step_size ** 2 and grid.line_of_sight(new_cell, goal):
                tree[goal] = new_cell
                path = _reconstruct(tree, goal)
                return PlanResult(
                    path=path,
                    success=True,
                    cost=_path_length(path),
                    expansions=it + 1,
                    elapsed_s=time.perf_counter() - t0,
                )

        return PlanResult(
            path=[], success=False, expansions=self.max_iterations,
            elapsed_s=time.perf_counter() - t0,
        )


def _sq_dist(a: Cell, b: Cell) -> int:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def _steer(a: Cell, b: Cell, step: int) -> Cell:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dist = math.hypot(dx, dy)
    if dist == 0:
        return a
    ratio = min(1.0, step / dist)
    return (int(round(a[0] + dx * ratio)), int(round(a[1] + dy * ratio)))


def _reconstruct(tree: dict[Cell, Cell | None], end: Cell) -> list[Cell]:
    path: list[Cell] = [end]
    parent = tree.get(end)
    while parent is not None:
        path.append(parent)
        parent = tree.get(parent)
    path.reverse()
    return path


def _path_length(path: list[Cell]) -> float:
    total = 0.0
    for i in range(1, len(path)):
        total += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
    return total
