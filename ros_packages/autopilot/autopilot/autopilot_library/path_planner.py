from __future__ import annotations

import math
from collections.abc import Sequence

from shapely.geometry import Polygon

from .utils.astar import Astar
from .utils.position import Position

Coordinate = Position | tuple[float, float]
PolygonCoordinates = Sequence[Coordinate]


class PathPlanner:
    """
    Plans obstacle-avoiding paths between GPS positions using A*.

    The planner rasterizes the obstacles and the boat onto a small local grid
    whose cell ``(span, span)`` is the boat, then converts the resulting grid
    path back into GPS ``Position`` objects. The local frame is the same one
    used by :class:`Position` (``x`` is north, ``y`` is east) and the grid's row
    axis maps to ``x`` and its column axis to ``y``.

    Because the planner can be asked to plan very frequently, it is cheap to
    construct and holds no mutable state between calls.

    Parameters
    ----------
    cell_scale
        The size, in local units (metres), of a single grid cell. Larger cells
        make planning faster at the cost of positional accuracy.
    buffer
        The clearance, in local units, applied to every obstacle before
        rasterizing it. Defaults to one cell so the path never hugs an
        obstacle edge.
    diagonal
        Whether diagonal movement is allowed.
    """

    def __init__(self, cell_scale: float = 1.0, buffer: float | None = None, diagonal: bool = True) -> None:
        self.cell_scale = cell_scale
        self.buffer = cell_scale if buffer is None else buffer
        self.diagonal = diagonal

    @staticmethod
    def _as_lon_lat(coordinate: Coordinate) -> tuple[float, float]:
        """Normalizes a :class:`Position` or ``(lon, lat)`` pair to ``(lon, lat)``."""

        if isinstance(coordinate, Position):
            return float(coordinate.longitude), float(coordinate.latitude)

        return float(coordinate[0]), float(coordinate[1])

    @staticmethod
    def _to_local(coordinate: Coordinate, reference: Position) -> tuple[float, float]:
        """Converts a GPS coordinate into the local north/east frame of ``reference``."""

        if isinstance(coordinate, Position):
            position = coordinate
        else:
            position = Position(longitude=coordinate[0], latitude=coordinate[1])

        local_x, local_y = position.get_local_coordinates(reference.get_longitude_latitude())
        return float(local_x), float(local_y)

    @staticmethod
    def _to_gps(local_point: tuple[float, float], reference: Position) -> Position:
        """Converts a local north/east point back into a GPS :class:`Position`."""

        return Position(
            local_x=local_point[0],
            local_y=local_point[1],
            reference_longitude=float(reference.longitude),
            reference_latitude=float(reference.latitude),
        )

    def _local_polygons(self, obstacles: Sequence[PolygonCoordinates] | None, reference: Position) -> list[Polygon]:
        """Converts obstacle vertex lists into local-coordinate Shapely polygons."""

        polygons: list[Polygon] = []
        for obstacle in obstacles or []:
            coordinates = [self._to_local(vertex, reference) for vertex in obstacle]
            if len(coordinates) >= 3:
                polygons.append(Polygon(coordinates))

        return polygons

    @staticmethod
    def _simplify(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
        """Drops intermediate points that lie on a straight line, keeping only the turns."""

        if len(points) < 3:
            return points

        simplified = [points[0]]
        for previous, current, following in zip(points, points[1:], points[2:], strict=False):
            cross = (current[0] - previous[0]) * (following[1] - current[1]) - (current[1] - previous[1]) * (
                following[0] - current[0]
            )
            if cross != 0:
                simplified.append(current)
        
        simplified.append(points[-1])
        return simplified

    def plan(
        self,
        source: Coordinate,
        destination: Coordinate,
        obstacles: Sequence[PolygonCoordinates] | None = None,
    ) -> list[Position]:
        """
        Plans an obstacle-avoiding path from ``source`` to ``destination``.

        Parameters
        ----------
        source
            The current position of the boat.
        destination
            The destination to reach.
        obstacles
            The obstacle polygons to avoid, each given as a sequence of GPS
            vertices (``Position`` objects or ``(lon, lat)`` pairs). Polygons
            with fewer than three vertices are ignored.

        Returns
        -------
        `list[Position]`
            The path as a list of :class:`Position` objects ordered from the
            source to the destination, or an empty list if no path could be
            found.
        """

        reference = source if isinstance(source, Position) else Position(longitude=source[0], latitude=source[1])

        source_local = (0.0, 0.0)
        destination_local = self._to_local(destination, reference)

        distance = math.hypot(destination_local[0] - source_local[0], destination_local[1] - source_local[1])

        # size the grid to comfortably hold the straight-line route, and center
        # the boat in the middle so the goal can never fall off the grid
        span = math.ceil(distance / self.cell_scale) + 1
        rows = span * 2
        cols = span * 2
        offset = span

        start = (span, span)
        goal = (
            offset + round(destination_local[0] / self.cell_scale),
            offset + round(destination_local[1] / self.cell_scale),
        )
        goal = (min(max(goal[0], 0), rows - 1), min(max(goal[1], 0), cols - 1))

        astar = Astar(
            (rows, cols),
            diagonal=self.diagonal,
            obstacles=self._local_polygons(obstacles, reference),
            origin=(-offset * self.cell_scale, -offset * self.cell_scale),
            cell_size=self.cell_scale,
            buffer=self.buffer,
        )
        matrix_path = astar.find_path(start, goal)

        if matrix_path is None:
            return []

        local = [
            ((row - offset) * self.cell_scale, (col - offset) * self.cell_scale) for row, col in matrix_path
        ]
        local[0] = source_local
        local[-1] = destination_local

        return [self._to_gps(point, reference) for point in self._simplify(local)]

    def plan_route(
        self,
        source: Coordinate,
        waypoints: Sequence[Coordinate],
        obstacles: Sequence[PolygonCoordinates] | None = None,
    ) -> list[Position]:
        """
        Plans an obstacle-avoiding path through an ordered list of waypoints.

        Each leg is planned independently from the previous leg's endpoint, and
        the shared joint between legs is only included once, so the result can
        be handed straight to an autopilot as a single dense path.

        Parameters
        ----------
        source
            The current position of the boat.
        waypoints
            The ordered waypoints to visit.
        obstacles
            The obstacle polygons to avoid.

        Returns
        -------
        `list[Position]`
            The full path from the source through every waypoint, or an empty
            list if any leg could not be planned.
        """

        if not waypoints:
            return []

        full_path: list[Position] = []
        current = source

        for waypoint in waypoints:
            leg = self.plan(current, waypoint, obstacles)
            if not leg:
                return []

            # the first point of every leg after the first duplicates the joint
            full_path.extend(leg if not full_path else leg[1:])
            current = waypoint

        return full_path
