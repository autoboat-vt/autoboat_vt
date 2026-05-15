from http.server import ThreadingHTTPServer

from utils import constants

from widgets.map_widget.waypoints_handler import WaypointsHandler


def run() -> None:
    print(f"[Info] Running map backend server on port {constants.MAP_SERVER_PORT}...")
    server = ThreadingHTTPServer(("127.0.0.1", constants.MAP_SERVER_PORT), WaypointsHandler)
    server.serve_forever()
