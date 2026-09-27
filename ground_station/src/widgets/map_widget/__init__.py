"""
Package for the map widget in the Groundstation application.

Exposes:
- MapBridge: A class which provides a typed Python interface to the Typescript `MapInterface`.
- MapOptionsHandler: A class that manages the map options and features.
- `run()`: A function that initializes and starts the HTTP server to handle waypoint requests.

Contains:
- `bridge.py`: Contains the `MapBridge` class - the Python->TS bridge with typed methods matching `MapInterface`.
- `land_check.py`: Contains the `LandChecker` class for point-in-ocean lookups against the Natural Earth shapefile.
- `map_options_handler.py`: Contains the `MapOptionsHandler` class for managing map options and features.
- `server.py`: Contains the `run()` function that starts the HTTP server for handling waypoint requests.
- `waypoints_handler.py`: Contains the `WaypointsHandler` class for handling HTTP requests related to waypoints.
- `frontend`: A directory containing the HTML and Typescript code for the map widget's frontend interface.
"""

__all__ = ["MapBridge", "MapOptionsHandler", "run"]

from .bridge import MapBridge
from .map_options_handler import MapOptionsHandler
from .server import run
