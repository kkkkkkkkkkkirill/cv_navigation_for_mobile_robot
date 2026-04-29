"""Common interface shared by every planner.

The diploma benchmark harness will call ``plan(map, start, goal)`` and
collect the returned ``PlanResult`` to compute path length, computation
time, expansion count and smoothness for comparison.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Protocol

from ..maps import Cell, GridMap


@dataclass
class PlanResult:
    """Outcome of a single planning call.

    Keeping ``expansions`` and ``elapsed_s`` here means every planner can be
    plugged into the benchmark without a separate instrumentation layer.
    """

    path: list[Cell]
    success: bool
    cost: float = 0.0
    expansions: int = 0
    elapsed_s: float = 0.0
    metadata: dict = field(default_factory=dict)


class Planner(ABC):
    """Base class for all planners."""

    name: str = "base"

    @abstractmethod
    def plan(self, grid: GridMap, start: Cell, goal: Cell) -> PlanResult:
        """Compute a path from ``start`` to ``goal`` on ``grid``."""
