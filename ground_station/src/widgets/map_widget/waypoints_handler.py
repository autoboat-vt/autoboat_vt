import json
from http.server import BaseHTTPRequestHandler
from threading import Lock

_WAYPOINTS_LOCK = Lock()
_WAYPOINTS: list[tuple[float, float]] = []

class WaypointsHandler(BaseHTTPRequestHandler):
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
        """Handle GET requests to retrieve waypoints."""

        if self.path != "/waypoints":
            self._not_found()
            return

        with _WAYPOINTS_LOCK:
            payload = json.dumps(_WAYPOINTS).encode("utf-8")

        self._set_headers(200)
        self.wfile.write(payload)

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
        """Suppress default request logging."""

        pass
