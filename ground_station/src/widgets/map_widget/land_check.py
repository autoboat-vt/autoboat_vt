from __future__ import annotations

import math
import threading
from pathlib import Path

import shapefile
import shapely
from shapely import wkb
from shapely.geometry import shape
from shapely.geometry.base import BaseGeometry

from utils.console_logger import get_logger

__all__ = ["LandChecker"]

logger = get_logger(__name__)

# meters per degree of latitude (nearly constant)
_M_PER_DEG_LAT = 110_574.0

# meters per degree of longitude at the equator
_M_PER_DEG_LON_EQ = 111_320.0


class LandChecker:
    """
    Point-in-ocean lookup backed by the Natural Earth 10m ocean shapefile.

    The ocean polygons are loaded once from ``app_data/git_keep/assets/ocean_layer``,
    repaired with :meth:`shapely.make_valid`, and cached as WKB so subsequent
    launches skip the expensive repair step. The prepared geometry is then used
    for vectorized ``contains_xy`` queries, where "not contained in the ocean"
    means the point is on land.

    Boundary points are handled flexibly: a point within ``boundary_tolerance_m``
    meters of the ocean polygon counts as ocean, so clicks near a coastline
    (where the 10m-resolution shapefile is coarse or slightly offset from
    reality) are still accepted.

    Parameters
    ----------
    shapefile_path
        Path to the `.shp` file (without extension is also accepted by pyshp).
    cache_path
        Path to the WKB cache file, e.g. ``app_data/git_ignore/ocean_geometry.wkb``.
        Pass `None` to disable caching (rebuild the geometry on every init).
    boundary_tolerance_m
        Distance in meters within which a point near the ocean boundary counts
        as ocean. Use `0` for the strict, exact-polygon behavior.

    Attributes
    ----------
    ready
        `True` once the ocean geometry has been loaded and prepared. All checks
        fail open (return `False`, i.e. "not on land") while this is `False`.
    """

    def __init__(
        self,
        shapefile_path: Path,
        cache_path: Path | None = None,
        boundary_tolerance_m: float = 100.0,
    ) -> None:
        self._lock = threading.Lock()
        self._ocean: BaseGeometry | None = None
        self.ready = False
        self._load_error: str | None = None

        self._shapefile_path = shapefile_path
        self._cache_path = cache_path
        self.boundary_tolerance_m = boundary_tolerance_m

        self._loader = threading.Thread(target=self._load, name="LandCheckerLoader", daemon=True)
        self._loader.start()

    def _load(self) -> None:
        """Load, repair, and prepare the ocean geometry in a background thread."""

        try:
            geometry = self._load_from_cache()

            if geometry is None:
                geometry = self._build_from_shapefile()
                self._write_cache(geometry)

            shapely.prepare(geometry)

            with self._lock:
                self._ocean = geometry
                self.ready = True

            logger.info("Ocean geometry loaded; waypoints on land will be rejected.")

        except Exception as exc:
            self._load_error = str(exc)
            logger.error(f"Failed to load ocean geometry, land checks disabled: {exc}")

    def _load_from_cache(self) -> BaseGeometry | None:
        """
        Load the repaired ocean geometry from the WKB cache, if present.

        Returns
        -------
        `BaseGeometry | None`
            The cached geometry, or None if there is no usable cache.
        """

        if self._cache_path is None or not self._cache_path.is_file():
            return None

        try:
            cached = wkb.loads(self._cache_path.read_bytes())

        except Exception as exc:
            logger.warning(f"Ignoring corrupt ocean geometry cache: {exc}")
            return None

        logger.info(f"Loaded ocean geometry from cache at {self._cache_path}.")
        return cached

    def _write_cache(self, geometry: BaseGeometry) -> None:
        """Write the repaired geometry to the WKB cache, if configured."""

        if self._cache_path is None:
            return

        try:
            self._cache_path.parent.mkdir(parents=True, exist_ok=True)
            self._cache_path.write_bytes(geometry.wkb)

        except OSError as exc:
            logger.warning(f"Could not write ocean geometry cache: {exc}")

    def _build_from_shapefile(self) -> BaseGeometry:
        """
        Build a valid ocean geometry from the Natural Earth shapefile.

        Returns
        -------
        :class:`BaseGeometry`
            The repaired ocean geometry, ready for vectorized ``contains_xy`` queries.

        Raises
        ------
        :class:`FileNotFoundError`
            If the shapefile is missing.
        :class:`RuntimeError`
            If the shapefile contains no polygon shapes.
        """

        if not self._shapefile_path.is_file():
            raise FileNotFoundError(f"Ocean shapefile not found at {self._shapefile_path}.")

        logger.info(f"Loading ocean geometry from {self._shapefile_path} (first run only)...")

        reader = shapefile.Reader(self._shapefile_path.as_posix())

        polygons: list[BaseGeometry] = []
        for shp in reader.shapes():
            if shp.shapeType == shapefile.NULL:
                continue

            polygons.append(shape(shp.__geo_interface__))

        if not polygons:
            raise RuntimeError(f"No ocean polygons found in {self._shapefile_path}.")

        merged = shapely.union_all(polygons) if len(polygons) > 1 else polygons[0]
        return shapely.make_valid(merged)

    def is_on_land(self, lat: float, lon: float) -> bool:
        """
        Check whether a coordinate is on land.

        Points that fall outside the ocean polygons are considered land, except
        within ``boundary_tolerance_m`` meters of the ocean boundary where the
        check is relaxed in favor of the ocean (the shapefile is only 10m
        resolution, so coastline clicks are frequently a few hundred meters off
        in either direction).

        If the ocean geometry is missing or still loading, this fails open and
        returns False so that waypoint placement is never blocked by a data
        problem.

        Parameters
        ----------
        lat
            Latitude in decimal degrees.
        lon
            Longitude in decimal degrees.

        Returns
        -------
        `bool`
            True if the point is on land (or the check is unavailable).
        """

        with self._lock:
            ocean = self._ocean

        if ocean is None:
            return False

        if shapely.contains_xy(ocean, lon, lat):
            return False

        if self.boundary_tolerance_m <= 0:
            return True

        # the point is outside the ocean polygon; accept it anyway if it is
        # within the tolerance of the coastline (i.e. near the boundary)
        tolerance_box = self._tolerance_box(lat, lon)
        return not shapely.intersects(ocean, tolerance_box)

    def _tolerance_box(self, lat: float, lon: float) -> BaseGeometry:
        """
        Build a lon/lat-axis-aligned box around a point spanning ``boundary_tolerance_m`` meters.

        The longitude span compensates for the convergence of meridians at
        higher latitudes so the box is roughly square in meters everywhere.

        Parameters
        ----------
        lat
            Latitude in decimal degrees.
        lon
            Longitude in decimal degrees.

        Returns
        -------
        :class:`BaseGeometry`
            A small :class:`shapely.geometry.polygon.Polygon` (a box) centered
            on the point.
        """

        dx = self.boundary_tolerance_m / (_M_PER_DEG_LON_EQ * max(math.cos(math.radians(lat)), 1e-6))
        dy = self.boundary_tolerance_m / _M_PER_DEG_LAT
        return shapely.box(lon - dx, lat - dy, lon + dx, lat + dy)
