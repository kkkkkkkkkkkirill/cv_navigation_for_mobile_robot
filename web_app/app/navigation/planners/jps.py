"""Jump Point Search — stub.

JPS exploits symmetries on uniform-cost grids: instead of expanding every
neighbour, it "jumps" along straight lines until it either hits a wall, a
forced neighbour, or the goal. This dramatically reduces the open list on
long corridors typical of warehouse aisles — a property worth quantifying
in the benchmark chapter.

Implementation is deferred; a correct JPS requires careful handling of
forced neighbours and prune rules (Harabor & Grastien, 2011). For now the
stub falls back to A* so the benchmark harness can still run end-to-end.
"""
from __future__ import annotations

from ..maps import Cell, GridMap
from .astar import AStarPlanner
from .base import PlanResult, Planner


class JPSPlanner(Planner):
    name = "jps"

    def __init__(self) -> None:
        self._fallback = AStarPlanner()

    def plan(self, grid: GridMap, start: Cell, goal: Cell) -> PlanResult:
        # TODO: implement proper jump-point identification. For now defer
        # to A* so experiments that register JPS don't crash.
        result = self._fallback.plan(grid, start, goal)
        result.metadata["fallback"] = "astar"
        return result
