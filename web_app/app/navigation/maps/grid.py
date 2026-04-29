"""Occupancy grid map.

A minimal, dependency-free grid container. Cells are either ``FREE`` or
``OCCUPIED``. Diagonal moves are allowed but not through tight corners
(we forbid "squeezing" between two occupied orthogonal neighbours) —
standard convention for factory-floor path planning.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

Cell = tuple[int, int]

FREE = 0
OCCUPIED = 1


@dataclass(frozen=True)
class GridMap:
    """A 2D occupancy grid.

    ``cells[y][x]`` is 0 for free and 1 for occupied. The origin is the
    bottom-left corner; ``y`` increases upward. Distances are in cells.
    """

    width: int
    height: int
    cells: tuple[tuple[int, ...], ...]

    @classmethod
    def empty(cls, width: int, height: int) -> "GridMap":
        return cls(
            width=width,
            height=height,
            cells=tuple(tuple(FREE for _ in range(width)) for _ in range(height)),
        )

    @classmethod
    def from_rows(cls, rows: Iterable[Iterable[int]]) -> "GridMap":
        cells = tuple(tuple(int(c) for c in row) for row in rows)
        height = len(cells)
        width = len(cells[0]) if height else 0
        for row in cells:
            if len(row) != width:
                raise ValueError("All rows must have the same length")
        return cls(width=width, height=height, cells=cells)

    def in_bounds(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, cell: Cell) -> bool:
        x, y = cell
        return self.in_bounds(cell) and self.cells[y][x] == FREE

    def neighbours(self, cell: Cell, allow_diagonal: bool = True) -> list[Cell]:
        """Return walkable 4- or 8-connected neighbours.

        Diagonals through two adjacent obstacles are disallowed — a robot
        cannot physically slip through a concrete corner.
        """
        x, y = cell
        result: list[Cell] = []
        orthogonal = ((1, 0), (-1, 0), (0, 1), (0, -1))
        for dx, dy in orthogonal:
            nxt = (x + dx, y + dy)
            if self.is_free(nxt):
                result.append(nxt)

        if not allow_diagonal:
            return result

        diagonals = ((1, 1), (1, -1), (-1, 1), (-1, -1))
        for dx, dy in diagonals:
            nxt = (x + dx, y + dy)
            if not self.is_free(nxt):
                continue
            # Block corner-cutting through obstacles.
            if not self.is_free((x + dx, y)) or not self.is_free((x, y + dy)):
                continue
            result.append(nxt)
        return result

    def line_of_sight(self, a: Cell, b: Cell) -> bool:
        """Bresenham-based visibility check. Used by Theta*.

        Returns ``True`` if every cell between ``a`` and ``b`` (exclusive)
        is free.
        """
        x0, y0 = a
        x1, y1 = b
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        if dx >= dy:
            err = dx / 2.0
            while x != x1:
                if not self.is_free((x, y)):
                    return False
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                if not self.is_free((x, y)):
                    return False
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        return self.is_free((x1, y1))
