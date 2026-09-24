import time
from functools import partial
from http.server import ThreadingHTTPServer

from utils import constants, misc
from utils.console_logger import get_logger

from .bathymetry import BathymetryProvider
from .land_check import LandChecker
from .waypoints_handler import WaypointsHandler

logger = get_logger(__name__)

def assemble_map_callback_server() -> ThreadingHTTPServer:
    """
    Assemble and return a :class:`ThreadingHTTPServer` that handles map callbacks.

    The server listens on ``constants.MAP_CALLBACK_PORT`` and handles requests for
    waypoints, land checks, and bathymetry data. It uses a :class:`WaypointsHandler`
    to process incoming requests, which in turn relies on a :class:`LandChecker`
    and a :class:`BathymetryProvider` to provide the necessary data. The server is
    intended to run in a separate thread, independent of the main PyQt event loop.

    Since :class:`http.server.BaseServer` instantiates the request handler with
    ``(request, client_address, server)`` only, the shared providers are bound to
    the handler with :func:`functools.partial`.

    Returns
    -------
    :class:`ThreadingHTTPServer`
        A configured HTTP server ready to handle map-related requests.
    """

    land_checker = LandChecker(
        shapefile_path=constants.OCEAN_SHAPEFILE_PATH,
        cache_path=constants.OCEAN_GEOMETRY_CACHE_PATH,
    )
    bathymetry_provider = BathymetryProvider(
        constants.OCEAN_DEPTH_LAYER_DIR,
        constants.BATHYMETRY_GEOJSON_CACHE_PATH,
    )

    waypoints_handler = partial(
        WaypointsHandler,
        land_checker=land_checker,
        bathymetry_provider=bathymetry_provider,
    )

    while not misc.check_port_available(constants.MAP_CALLBACK_PORT):
        logger.warning(
            f"Port {constants.MAP_CALLBACK_PORT} is already in use. Retrying in 1 second..."
        )
        time.sleep(1)

    return ThreadingHTTPServer(("127.0.0.1", constants.MAP_CALLBACK_PORT), waypoints_handler)
