from __future__ import annotations

import heapq
import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import numpy.typing as npt
from shapely import intersects_xy
from shapely.geometry.base import BaseGeometry

from .utils_function_library import parse_polygons

DIAGONAL_COST = math.sqrt(2)

def obstacles_to_mask(
    obstacles: BaseGeometry | Sequence[BaseGeometry],
    rows: int,
    cols: int,
    origin: tuple[float, float] = (0.0, 0.0),
    cell_size: float = 1.0,
    buffer: float = 0.0
) -> npt.NDArray[np.bool_]:
    """
    Rasterizes obstacle polygons onto a grid of cells.

    Parameters
    ----------
    obstacles
        A single Shapely geometry or a sequence of them. ``Polygon`` and
        ``MultiPolygon`` geometries are the intended inputs.
    rows
        The number of rows in the grid.
    cols
        The number of columns in the grid.
    origin
        The world coordinate of grid cell ``(0, 0)``. The first element is the
        coordinate along the row axis and the second along the column axis, so
        cell ``(row, col)`` is tested at
        ``(origin[0] + row * cell_size, origin[1] + col * cell_size)``.
    cell_size
        The size, in world units, of a single cell. The cell indices are scaled
        by this before the obstacle geometry is tested, so the mask lines up with
        the world coordinates of the grid no matter how coarse the grid is.
    buffer
        An optional clearance, in world units, applied to every obstacle before
        testing. A positive value also blocks cells within ``buffer`` of an
        obstacle boundary, keeping the path that far away from it.

    Returns
    -------
    ndarray
        A boolean array of shape ``(rows, cols)`` which is ``True`` at every
        cell whose center falls inside or on an obstacle. Touching an edge or a
        vertex counts as blocked, which keeps the path clear of the boundaries.
    """

    if isinstance(obstacles, BaseGeometry):
        obstacles = (obstacles,)

    row_coordinates, col_coordinates = np.meshgrid(np.arange(rows), np.arange(cols), indexing="ij")
    x = origin[0] + row_coordinates * cell_size
    y = origin[1] + col_coordinates * cell_size

    mask = np.zeros((rows, cols), dtype=np.bool_)
    for obstacle in obstacles:
        geometry = obstacle.buffer(buffer) if buffer > 0.0 else obstacle
        mask |= intersects_xy(geometry, x, y)

    return mask


@dataclass(eq=False)
class GridCell:
    """
    A class representing a cell in the A* grid.
    
    Attributes
    ----------
    row
        The row index of the cell in the grid.
    col
        The column index of the cell in the grid.
    neighbors
        A mapping of direction name to the neighboring cell reachable in that
        direction, or ``None`` if the neighbor is out of bounds or the
        direction is disabled. Diagonal keys are only populated when the grid
        was created with ``diagonal=True``.
    blocked
        Whether the cell is covered by an obstacle and can never be entered.
    """
    
    row: int
    col: int
    blocked: bool = False
    neighbors: dict[str, GridCell] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """This method is called after the dataclass is initialized."""

        self.neighbors = {
            "up": None,
            "down": None,
            "left": None,
            "right": None,
            "up_left": None,
            "up_right": None,
            "down_left": None,
            "down_right": None,
        }

class Grid:
    """
    A class representing a grid for the A* pathfinding algorithm.
    
    Attributes
    ----------
    rows
        The number of rows in the grid.
    cols
        The number of columns in the grid.
    diagonal
        Whether diagonal (8-way) movement is allowed. When ``False`` the grid
        is restricted to orthogonal (4-way) movement.
    cells
        A 2D list of GridCell objects representing the cells in the grid.
    """

    def __init__(
        self,
        rows: int,
        cols: int,
        diagonal: bool = True,
        obstacles: BaseGeometry | Sequence[BaseGeometry] | None = None,
        origin: tuple[float, float] = (0.0, 0.0),
        cell_size: float = 1.0,
        buffer: float = 0.0,
    ) -> None:
        """
        Parameters
        ----------
        rows
            The number of rows in the grid.
        cols
            The number of columns in the grid.
        diagonal
            Whether diagonal (8-way) movement is allowed.
        obstacles
            Optional obstacle polygons to rasterize onto the grid. Any cell
            covered by an obstacle is marked as blocked and can never be
            entered.
        origin
            The world coordinate of cell ``(0, 0)``, used when rasterizing the
            obstacles.
        cell_size
            The size, in world units, of a single grid cell, used when
            rasterizing the obstacles so the mask lines up with the world
            coordinates.
        buffer
            An optional clearance applied to every obstacle before rasterizing.
        """

        self.rows = rows
        self.cols = cols
        self.diagonal = diagonal
        
        self.cells = [[GridCell(row=i, col=j) for j in range(cols)] for i in range(rows)]

        if obstacles is not None:
            mask = obstacles_to_mask(obstacles, rows=rows, cols=cols, origin=origin, cell_size=cell_size, buffer=buffer)
            for i in range(self.rows):
                for j in range(self.cols):
                    if mask[i, j]:
                        self.cells[i][j].blocked = True

        for i in range(self.rows):
            for j in range(self.cols):
                cell = self.cells[i][j]

                if i > 0:
                    cell.neighbors["up"] = self.cells[i - 1][j]
                if i < self.rows - 1:
                    cell.neighbors["down"] = self.cells[i + 1][j]
                if j > 0:
                    cell.neighbors["left"] = self.cells[i][j - 1]
                if j < self.cols - 1:
                    cell.neighbors["right"] = self.cells[i][j + 1]

                if not self.diagonal:
                    continue

                if i > 0 and j > 0:
                    cell.neighbors["up_left"] = self.cells[i - 1][j - 1]
                if i > 0 and j < self.cols - 1:
                    cell.neighbors["up_right"] = self.cells[i - 1][j + 1]
                if i < self.rows - 1 and j > 0:
                    cell.neighbors["down_left"] = self.cells[i + 1][j - 1]
                if i < self.rows - 1 and j < self.cols - 1:
                    cell.neighbors["down_right"] = self.cells[i + 1][j + 1]

    def get_neighbors(self, cell: GridCell) -> list[GridCell]:
        """
        Returns the neighboring cells that can actually be entered.

        Blocked cells are excluded. A diagonal move is also rejected when both
        of the orthogonal cells it passes between are blocked, so the path can
        never squeeze through the corner of an obstacle.
        """

        neighbors: list[GridCell] = []
        for direction, neighbor in cell.neighbors.items():
            if neighbor is None or neighbor.blocked:
                continue

            if "_" in direction:
                first, second = direction.split("_")
                orthogonal_first = cell.neighbors[first]
                orthogonal_second = cell.neighbors[second]
                if (
                    orthogonal_first is not None and orthogonal_first.blocked
                    and orthogonal_second is not None and orthogonal_second.blocked
                ):
                    continue

            neighbors.append(neighbor)

        return neighbors

    def is_valid(self, row: int, col: int) -> bool:
        """Checks if the specified row and column are within the grid bounds."""

        return 0 <= row < self.rows and 0 <= col < self.cols

    def get_cell(self, row: int, col: int) -> GridCell | None:
        """Returns the cell at the specified row and column, or None if out of bounds."""

        if self.is_valid(row, col):
            return self.cells[row][col]
        
        return None

    def is_blocked(self, row: int, col: int) -> bool:
        """Returns True if the given cell is out of bounds or covered by an obstacle."""

        cell = self.get_cell(row, col)

        return cell is None or cell.blocked

    def manhattan_distance(self, cell1: GridCell, cell2: GridCell) -> int:
        """Calculates the Manhattan distance between two cells."""

        return abs(cell1.row - cell2.row) + abs(cell1.col - cell2.col)

    def octile_distance(self, cell1: GridCell, cell2: GridCell) -> float:
        """
        Calculates the octile distance between two cells.

        This is the true minimum cost of travelling between the cells when
        diagonal moves cost ``sqrt(2)`` and orthogonal moves cost 1, making it
        an admissible and consistent heuristic for an 8-connected grid.
        """

        delta_row = abs(cell1.row - cell2.row)
        delta_col = abs(cell1.col - cell2.col)
        diagonal_steps = min(delta_row, delta_col)

        return (delta_row + delta_col) + (DIAGONAL_COST - 2) * diagonal_steps

    def distance(self, cell1: GridCell, cell2: GridCell) -> float:
        """
        Calculates the heuristic distance between two cells.

        Returns the octile distance when diagonal movement is enabled, and the
        Manhattan distance otherwise. Both are admissible for the grid's
        connectivity.
        """

        if self.diagonal:
            return self.octile_distance(cell1, cell2)

        return float(self.manhattan_distance(cell1, cell2))

    def move_cost(self, cell1: GridCell, cell2: GridCell) -> float:
        """
        Calculates the cost of moving between two adjacent cells.

        Orthogonal moves cost 1, diagonal moves cost ``sqrt(2)``.
        """

        if cell1.row != cell2.row and cell1.col != cell2.col:
            return DIAGONAL_COST

        return 1.0

class Astar:
    """Class implementing the A* pathfinding algorithm."""

    def __init__(
        self,
        grid_dimensions: tuple[int, int],
        diagonal: bool = True,
        obstacles: dict[str, Any] | str | BaseGeometry | Sequence[BaseGeometry] | None = None,
        origin: tuple[float, float] = (0.0, 0.0),
        cell_size: float = 1.0,
        buffer: float = 0.0,
    ) -> None:
        """
        Parameters
        ----------
        grid_dimensions
            The ``(rows, cols)`` dimensions of the grid to search.
        diagonal
            Whether diagonal (8-way) movement is allowed. When ``False`` the
            search is restricted to orthogonal (4-way) movement.
        obstacles
            Optional obstacle polygons, provided either as Shapely geometries,
            or as a GeoJSON document (an already-parsed ``dict`` or a JSON
            string). Any cell covered by an obstacle is treated as
            impassable.
        origin
            The world coordinate of cell ``(0, 0)``, used when rasterizing the
            obstacles onto the grid.
        cell_size
            The size, in world units, of a single grid cell, used when
            rasterizing the obstacles.
        buffer
            An optional clearance, in world units, applied to every obstacle
            before rasterizing it.
        """

        if obstacles is not None and not isinstance(obstacles, (list, tuple, BaseGeometry)):
            obstacles = parse_polygons(obstacles)

        if isinstance(obstacles, BaseGeometry):
            obstacles = [obstacles]

        self.diagonal = diagonal
        self.grid = Grid(
            rows=grid_dimensions[0],
            cols=grid_dimensions[1],
            diagonal=diagonal,
            obstacles=obstacles,
            origin=origin,
            cell_size=cell_size,
            buffer=buffer,
        )
    
    def find_path(self, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]] | None:
        """
        Finds the shortest path from start to goal.
        
        Parameters
        ----------
        start
            The starting cell coordinates (row, col).
        goal
            The goal cell coordinates (row, col).
        
        Returns
        -------
        `list[tuple[int, int]] | None`
            A list of cell coordinates representing the path from start to goal,
            or `None` if no path is found. When diagonal movement is enabled,
            consecutive cells may differ by one row *and* one column.
        """

        if not self.grid.is_valid(start[0], start[1]) or not self.grid.is_valid(goal[0], goal[1]):
            return None

        start_cell = self.grid.get_cell(start[0], start[1])
        goal_cell = self.grid.get_cell(goal[0], goal[1])

        if start_cell is None or goal_cell is None:
            return None

        if start_cell.blocked or goal_cell.blocked:
            return None

        counter = 0
        open_set: list[tuple[float, int, GridCell]] = []
        heapq.heappush(open_set, (0.0, counter, start_cell))

        came_from: dict[GridCell, GridCell | None] = {start_cell: None}
        g_score: dict[GridCell, float] = {start_cell: 0.0}

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current is goal_cell:
                return self._reconstruct_path(came_from, current)

            for neighbor in self.grid.get_neighbors(current):
                tentative_g = g_score[current] + self.grid.move_cost(current, neighbor)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g

                    f_score = tentative_g + self.grid.distance(neighbor, goal_cell)
                    counter += 1
                    heapq.heappush(open_set, (f_score, counter, neighbor))

        return None

    def _reconstruct_path(self, came_from: dict[GridCell, GridCell | None], current: GridCell) -> list[tuple[int, int]]:
        """
        Walks the ``came_from`` chain backwards from the goal to the start.

        Parameters
        ----------
        came_from
            A mapping of each visited cell to the cell it was reached from.
        current
            The goal cell.

        Returns
        -------
        `list[tuple[int, int]]`
            The path as a list of `(row, col)` coordinates, ordered from the
            start cell to the goal cell.
        """

        path: list[tuple[int, int]] = []
        while current is not None:
            path.append((current.row, current.col))
            current = came_from[current]

        path.reverse()
        return path
