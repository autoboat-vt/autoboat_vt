from http.server import ThreadingHTTPServer

from utils import constants
from utils.logger import get_logger

from .waypoints_handler import WaypointsHandler

logger = get_logger(__name__)


def run() -> None:
    """Run the map widget server."""

    logger.info(f"Running map backend server on port {constants.MAP_SERVER_PORT}...")
    server = ThreadingHTTPServer(("127.0.0.1", constants.MAP_SERVER_PORT), WaypointsHandler)
    server.serve_forever()
