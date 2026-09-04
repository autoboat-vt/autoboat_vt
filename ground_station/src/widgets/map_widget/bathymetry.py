from __future__ import annotations

import json
import re
import threading
from pathlib import Path
from typing import Any

import shapefile

from utils.console_logger import get_logger

__all__ = ["BathymetryProvider"]

logger = get_logger(__name__)


def _depth_from_filename(shp_path: Path) -> int:
    """
    Extract the depth value encoded in a Natural Earth bathymetry file name.

    Natural Earth names its bathymetry files ``ne_10m_bathymetry_<L>_<DEPTH>.shp``
    where ``<DEPTH>`` is the depth in metres (e.g. ``K_200`` → 200,
    ``A_10000`` → 10000). Failing to parse a depth, returns 0.

    Parameters
    ----------
    shp_path
        Path to the ``.shp`` file.

    Returns
    -------
    `int`
        The depth in metres for this band.
    """

    match = re.search(r"_(\d+)\.shp$", shp_path.name)
    if match:
        return int(match.group(1))

    logger.warning(f"Could not parse depth from bathymetry file name: {shp_path.name}")
    return 0


class BathymetryProvider:
    """
    Serves Natural Earth 10 m bathymetry depth-band polygons as GeoJSON.

    The ``ne_10m_bathymetry_*.shp`` files in ``app_data/git_keep/assets/ocean_depth_layer``
    are parsed once in a background thread and converted to a GeoJSON
    ``FeatureCollection`` where every feature carries a ``depth`` property (in
    metres, taken from the file name). The result is cached as a JSON file under
    ``app_data/git_ignore`` so subsequent launches skip the reparse.

    The ``depth`` property is negative following the bathymetric convention that
    greater depth is more negative; the frontend uses it to pick a colour ramp.

    Parameters
    ----------
    depth_dir
        Directory containing the ``ne_10m_bathymetry_*.shp`` layers.
    cache_path
        Path to the cached GeoJSON document, e.g.
        ``app_data/git_ignore/bathymetry.json``. Pass `None` to disable caching.

    Attributes
    ----------
    ready
        `True` once the GeoJSON has been built (or loaded from cache). The server
        returns 503 for bathymetry requests while this is `False`.
    """

    def __init__(self, depth_dir: Path, cache_path: Path | None = None) -> None:
        self._lock = threading.Lock()
        self._geojson: dict[str, Any] = {"type": "FeatureCollection", "features": []}
        self.ready = False
        self._load_error: str | None = None

        self._depth_dir = depth_dir
        self._cache_path = cache_path

        self._loader = threading.Thread(target=self._load, name="BathymetryLoader", daemon=True)
        self._loader.start()

    def _load(self) -> None:
        """Load the bathymetry GeoJSON from cache, else build it from the shapefiles."""

        try:
            geojson = self._load_from_cache()

            if geojson is None:
                geojson = self._build_from_shapefiles()
                self._write_cache(geojson)

            count = len(geojson.get("features", []))
            with self._lock:
                self._geojson = geojson
                self.ready = True

            logger.info(f"Bathymetry layer ready with {count} depth-band polygons.")

        except Exception as exc:
            self._load_error = str(exc)
            logger.error(f"Failed to load bathymetry layer: {exc}")

    def _load_from_cache(self) -> dict[str, Any] | None:
        """
        Load the cached GeoJSON document, if present and valid.

        Returns
        -------
        `dict[str, Any] | None`
            The cached document, or None if there is no usable cache.
        """

        if self._cache_path is None or not self._cache_path.is_file():
            return None

        try:
            with open(self._cache_path, encoding="utf-8") as cache_file:
                cached = json.load(cache_file)

            if isinstance(cached, dict) and isinstance(cached.get("features"), list):
                logger.info(f"Loaded bathymetry GeoJSON from cache at {self._cache_path}.")
                return cached

            logger.warning("Ignoring invalid bathymetry cache.")

        except Exception as exc:
            logger.warning(f"Ignoring corrupt bathymetry cache: {exc}")

        return None

    def _write_cache(self, geojson: dict[str, Any]) -> None:
        """Write the built GeoJSON document to cache, if configured."""

        if self._cache_path is None:
            return

        try:
            self._cache_path.parent.mkdir(parents=True, exist_ok=True)
            self._cache_path.write_text(json.dumps(geojson, separators=(",", ":")), encoding="utf-8")

        except Exception as exc:
            logger.warning(f"Could not write bathymetry cache: {exc}")

    def _build_from_shapefiles(self) -> dict[str, Any]:
        """
        Build the GeoJSON FeatureCollection from all bathymetry shapefiles.

        Each ``.shp`` file contributes a `depth` to every polygon it contains,
        taken from the file name so no DBF field name is assumed.

        Returns
        -------
        `dict[str, Any]`
            The bathymetry GeoJSON FeatureCollection.
        """

        features: list[dict[str, Any]] = []
        shp_files = sorted(self._depth_dir.glob("ne_10m_bathymetry_*.shp"))
        if not shp_files:
            logger.warning(f"No bathymetry shapefiles found in {self._depth_dir}.")

        for shp_path in shp_files:
            depth = -_depth_from_filename(shp_path)
            reader = shapefile.Reader(shp_path.as_posix())

            for shp in reader.shapes():
                geometry = shp.__geo_interface__
                if not geometry or geometry.get("type") not in ("Polygon", "MultiPolygon"):
                    continue

                features.append(
                    {
                        "type": "Feature",
                        "geometry": geometry,
                        "properties": {"depth": depth},
                    }
                )

        return {"type": "FeatureCollection", "features": features}

    def geojson(self) -> dict[str, Any]:
        """
        Get the bathymetry GeoJSON FeatureCollection.

        Returns
        -------
        `dict[str, Any]`
            The bathymetry FeatureCollection. Check :attr:`ready` before serving;
            this may be empty until loading completes.
        """

        with self._lock:
            return self._geojson
