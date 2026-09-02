import json
from http.server import BaseHTTPRequestHandler
from threading import Lock
from urllib.parse import parse_qs, urlparse

from utils.console_logger import get_logger

from .land_check import LandChecker

logger = get_logger(__name__)

_WAYPOINTS_LOCK = Lock()
_WAYPOINTS: list[tuple[float, float]] = []

# mutable holder so the land checker can be swapped in without a `global` statement
_LAND_CHECKER_HOLDER: list[LandChecker | None] = [None]


def set_land_checker(land_checker: LandChecker | None) -> None:
    """
    Provide the land checker used by the ``/check_land`` endpoint.

    Parameters
    ----------
    land_checker
        The shared :class:`LandChecker` instance, or None to disable checks.
    """

    _LAND_CHECKER_HOLDER[0] = land_checker


class WaypointsHandler(BaseHTTPRequestHandler):
    """
    HTTP server for receiving waypoints created by clicking on the map.

    The server runs in a separate thread with its own lifecycle, independent of the main PyQt event loop.
    It listens on ``constants.MAP_SERVER_PORT``. It handles CORS GET/POST requests to
    ``/waypoints``, storing the waypoints in a global list protected by a :class:`threading.Lock`.

    Also serves ``GET /check_land?lat=...&lon=...`` which reports whether a coordinate is on land,
    according to the Natural Earth ocean layer. Used by the frontend to reject clicks on land.

    Inherits
    --------
    :class:`BaseHTTPRequestHandler`
    """

    server_version = "WaypointsHTTP/1.0"

    def _set_headers(self, status_code: int) -> None:
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def _not_found(self) -> None:
        self._set_headers(404)
        self.wfile.write(b'{"message": "Not found"}')

    def do_OPTIONS(self) -> None:
        """Handle CORS preflight requests."""

        self._set_headers(204)

    def do_GET(self) -> None:
        """Handle GET requests to retrieve waypoints or check if a point is on land."""

        if self.path == "/waypoints":
            with _WAYPOINTS_LOCK:
                payload = json.dumps(_WAYPOINTS).encode("utf-8")

            self._set_headers(200)
            self.wfile.write(payload)
            return

        if self.path.startswith("/check_land"):
            self._handle_check_land()
            return

        self._not_found()

    def _handle_check_land(self) -> None:
        """
        Handle ``GET /check_land?lat=...&lon=...`` requests.

        Responds with `{"on_land": bool}`. Unknown or malformed coordinates
        result in a 400 response; a missing land checker results in
        `{"on_land": false}` so waypoint placement is never blocked.
        """

        query = parse_qs(urlparse(self.path).query)

        try:
            lat = float(query["lat"][0])
            lon = float(query["lon"][0])

        except (KeyError, IndexError, ValueError, TypeError):
            self._set_headers(400)
            self.wfile.write(b'{"message": "Invalid request parameters"}')
            return

        if not (-90.0 <= lat <= 90.0) or not (-180.0 <= lon <= 180.0):
            self._set_headers(400)
            self.wfile.write(b'{"message": "Coordinates out of range"}')
            return

        land_checker = _LAND_CHECKER_HOLDER[0]
        on_land = land_checker.is_on_land(lat, lon) if land_checker is not None else False

        self._set_headers(200)
        self.wfile.write(json.dumps({"on_land": on_land}).encode("utf-8"))

    def do_POST(self) -> None:
        """Handle POST requests to update waypoints."""

        if self.path != "/waypoints":
            self._not_found()
            return

        content_length = int(self.headers.get("Content-Length", "0"))
        raw_body = self.rfile.read(content_length)

        try:
            body = json.loads(raw_body.decode("utf-8"))
            if not isinstance(body, dict):
                raise TypeError("request body must be a JSON object")

            waypoints = body.get("waypoints")
            if not isinstance(waypoints, list):
                raise TypeError("waypoints must be a list")

            normalized_waypoints: list[tuple[float, float]] = []
            for waypoint in waypoints:
                if not isinstance(waypoint, (list, tuple)) or len(waypoint) != 2:
                    raise TypeError("each waypoint must be a list of two numbers")

                latitude, longitude = waypoint
                if not isinstance(latitude, (int, float)) or not isinstance(longitude, (int, float)):
                    raise TypeError("each waypoint must be two numbers")

                normalized_waypoints.append((float(latitude), float(longitude)))

        except (TypeError, UnicodeDecodeError, json.JSONDecodeError):
            self._set_headers(400)
            self.wfile.write(b'{"message": "Invalid request body"}')
            return

        with _WAYPOINTS_LOCK:
            _WAYPOINTS.clear()
            _WAYPOINTS.extend(normalized_waypoints)

        self._set_headers(200)

    def log_message(self, format_string: str, *args: object) -> None:
        """Suppress default per-request logging."""

        pass
