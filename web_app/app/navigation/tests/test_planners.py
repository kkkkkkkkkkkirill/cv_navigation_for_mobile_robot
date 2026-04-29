"""Smoke tests for the navigation planners."""
from __future__ import annotations

import math

import pytest

from navigation.maps import GridMap
from navigation.planners import (
    AStarPlanner,
    DijkstraPlanner,
    DStarLitePlanner,
    JPSPlanner,
    RRTPlanner,
    ThetaStarPlanner,
)


def _simple_grid() -> GridMap:
    # 10x10 grid with a vertical wall at x=5 and a gap at y=9
    rows = []
    for y in range(10):
        row = [0] * 10
        if y != 9:
            row[5] = 1
        rows.append(row)
    return GridMap.from_rows(rows)


@pytest.mark.parametrize(
    "planner_cls", [AStarPlanner, DijkstraPlanner, ThetaStarPlanner, JPSPlanner, DStarLitePlanner]
)
def test_planner_finds_path_through_gap(planner_cls):
    grid = _simple_grid()
    result = planner_cls().plan(grid, start=(0, 0), goal=(9, 0))
    assert result.success, f"{planner_cls.__name__} failed on solvable map"
    assert result.path[0] == (0, 0)
    assert result.path[-1] == (9, 0)
    # Cost must be >= Euclidean since the wall forces a detour.
    assert result.cost >= math.hypot(9, 0)


def test_rrt_finds_path_on_open_map():
    grid = GridMap.empty(20, 20)
    result = RRTPlanner(max_iterations=2000).plan(grid, (0, 0), (19, 19))
    assert result.success
    assert result.path[0] == (0, 0)
    assert result.path[-1] == (19, 19)


def test_astar_dominates_dijkstra_expansions():
    grid = GridMap.empty(30, 30)
    a = AStarPlanner().plan(grid, (0, 0), (29, 29))
    d = DijkstraPlanner().plan(grid, (0, 0), (29, 29))
    assert a.success and d.success
    # A* should explore no more nodes than Dijkstra on an empty map.
    assert a.expansions <= d.expansions


def test_blocked_goal_returns_failure():
    grid = GridMap.from_rows([[0, 1, 0]])
    result = AStarPlanner().plan(grid, (0, 0), (1, 0))
    assert not result.success
