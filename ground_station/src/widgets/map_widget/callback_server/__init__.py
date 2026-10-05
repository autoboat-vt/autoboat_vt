"""
This module contains the callback server for the map widget, allowing the map widget to communicate with the Python backend.

The server listens on a specified port and handles requests related to waypoints, land checks, and bathymetry
data. It uses a :class:`WaypointsHandler` to process incoming requests, which in turn relies on a :class:`LandChecker` and a
:class:`BathymetryProvider` to provide the necessary data.

Exposes:
- :func:`assemble_map_callback_server()`: A function that assembles and returns a pre-configured :class:`ThreadingHTTPServer`.
- :class:`WaypointsHandler`: A class that handles HTTP requests related to waypoints, land checks, and bathymetry data.
- :class:`LandChecker`: A class that checks if a given coordinate is on land using the Natural Earth shapefile.
- :class:`BathymetryProvider`: A class that provides bathymetry data for given coordinates.

Contains:
- `server.py`: Contains the `assemble_map_callback_server()` function that sets up the HTTP server and its request handler.
- `waypoints_handler.py`: Contains the `WaypointsHandler` class that processes HTTP requests for waypoints, land
  checks, and bathymetry data.
- `land_check.py`: Contains the `LandChecker` class that checks if a coordinate is on land using the Natural Earth shapefile.
- `bathymetry.py`: Contains the `BathymetryProvider` class that provides bathymetry data for given coordinates.
"""

__all__ = ["BathymetryProvider", "LandChecker", "WaypointsHandler", "assemble_map_callback_server"]

from .bathymetry import BathymetryProvider
from .land_check import LandChecker
from .server import assemble_map_callback_server
from .waypoints_handler import WaypointsHandler
