"""D* Lite — stub.

D* Lite (Koenig & Likhachev, 2002) is the canonical incremental planner
for environments where obstacles appear/disappear after the initial plan.
In the manufacturing facility setting forklifts and workers are exactly
that — dynamic obstacles — which makes D* Lite the natural pick for the
"dynamic replanning" chapter.

Full implementation is deferred; this stub falls back to re-running A*
from scratch each time the grid changes, which is functionally correct
but loses D* Lite's incremental-update speedup.
"""
from __future__ import annotations

from ..maps import Cell, GridMap
from .astar import AStarPlanner
from .base import PlanResult, Planner


class DStarLitePlanner(Planner):
    name = "dstar_lite"

    def __init__(self) -> None:
        self._fallback = AStarPlanner()

    def plan(self, grid: GridMap, start: Cell, goal: Cell) -> PlanResult:
        result = self._fallback.plan(grid, start, goal)
        result.metadata["fallback"] = "astar"
        return result

    def replan(self, grid: GridMap, start: Cell, goal: Cell) -> PlanResult:
        """Placeholder for incremental replanning — identical to plan() for now."""
        return self.plan(grid, start, goal)
