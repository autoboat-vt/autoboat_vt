from http.server import ThreadingHTTPServer

from utils import constants
from utils.console_logger import get_logger

from .land_check import LandChecker
from .waypoints_handler import WaypointsHandler, set_land_checker

logger = get_logger(__name__)


def run() -> None:
    """Run the map widget server."""

    logger.info(f"Running map backend server on port {constants.MAP_SERVER_PORT}...")

    land_checker = LandChecker(
        shapefile_path=constants.OCEAN_SHAPEFILE_PATH,
        cache_path=constants.OCEAN_GEOMETRY_CACHE_PATH,
    )
    set_land_checker(land_checker)

    server = ThreadingHTTPServer(("127.0.0.1", constants.MAP_SERVER_PORT), WaypointsHandler)
    server.serve_forever()
