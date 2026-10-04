import json
from http.server import BaseHTTPRequestHandler
from threading import Lock
from urllib.parse import parse_qs, urlparse

from utils.console_logger import get_logger
from widgets.map_widget.land_click_prompt import LAND_CLICK_PROMPT

from .bathymetry import BathymetryProvider
from .land_check import LandChecker

logger = get_logger(__name__)

_WAYPOINTS_LOCK = Lock()
_WAYPOINTS: list[tuple[float, float]] = []


class WaypointsHandler(BaseHTTPRequestHandler):
    """
    HTTP server for receiving waypoints created by clicking on the map.

    The server runs in a separate thread with its own lifecycle, independent of the main PyQt event loop.
    It listens on ``constants.MAP_CALLBACK_PORT``. It handles CORS GET/POST requests to
    ``/waypoints``, storing the waypoints in a global list protected by a :class:`threading.Lock`.

    Also serves ``GET /check_land?lat=...&lon=...`` which reports whether a coordinate is on land,
    according to the Natural Earth ocean layer. When a point is on land, the frontend asks for
    user confirmation; this endpoint blocks on the Qt-side dialog via the shared
    :class:`land_click_prompt.LandClickPrompt` and reports the user's decision to the frontend.

    Inherits
    --------
    :class:`BaseHTTPRequestHandler`
    """

    server_version = "WaypointsHTTP/1.0"

    def __init__(
        self,
        *args: object,
        land_checker: LandChecker,
        bathymetry_provider: BathymetryProvider,
        **kwargs: object,
    ) -> None:
        """
        Create the request handler with its shared data providers.

        The providers are keyword-only so that the positional ``request``,
        ``client_address``, and ``server`` arguments passed by
        :class:`http.server.BaseServer` are forwarded untouched to
        :class:`http.server.BaseHTTPRequestHandler`. Use :func:`functools.partial`
        (or a factory closure) to bind the providers when constructing the server.

        Parameters
        ----------
        land_checker
            The shared :class:`LandChecker` used by ``/check_land`` and ``/land_boundary``.
        bathymetry_provider
            The shared :class:`BathymetryProvider` used by ``/bathymetry``.
        """

        self.land_checker = land_checker
        self.bathymetry_provider = bathymetry_provider
        super().__init__(*args, **kwargs)

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

        if self.path == "/bathymetry":
            self._handle_bathymetry()
            return

        if self.path == "/land_boundary":
            self._handle_land_boundary()
            return

        if self.path.startswith("/check_land"):
            self._handle_check_land()
            return

        self._not_found()

    def _handle_bathymetry(self) -> None:
        """
        Handle ``GET /bathymetry`` requests.

        Serves the bathymetry depth bands as a GeoJSON ``FeatureCollection``
        from the shared :class:`BathymetryProvider`. Responds 503 while the
        layer is still loading, and 404 if no provider is configured, so the
        frontend silently skips the layer in both cases.
        """

        if not self.bathymetry_provider.ready:
            self._set_headers(503)
            self.wfile.write(b'{"message": "Bathymetry not ready"}')
            return

        self._set_headers(200)
        self.wfile.write(json.dumps(self.bathymetry_provider.geojson(), separators=(",", ":")).encode("utf-8"))

    def _handle_land_boundary(self) -> None:
        """
        Handle ``GET /land_boundary`` requests.

        Serves the ocean geometry used by the land checker as a GeoJSON
        ``FeatureCollection`` for the faint boundary overlay. Responds 503 while
        the geometry is still loading, and 404 if no land checker is configured,
        so the frontend silently skips the layer in both cases.
        """

        if not self.land_checker.ready:
            self._set_headers(503)
            self.wfile.write(b'{"message": "Land boundary not ready"}')
            return

        self._set_headers(200)
        self.wfile.write(json.dumps(self.land_checker.geojson(), separators=(",", ":")).encode("utf-8"))

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

        on_land = self.land_checker.is_on_land(lat, lon)
        add_waypoint = True if not on_land else LAND_CLICK_PROMPT.ask(lat, lon)

        self._set_headers(200)
        self.wfile.write(json.dumps({"on_land": on_land, "add_waypoint": add_waypoint}).encode("utf-8"))

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
