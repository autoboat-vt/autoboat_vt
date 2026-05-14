"""
Widget for displaying a map with waypoints and the boat's current location.

This module implements a simple HTTP server that listens for GET and POST requests on the `/waypoints` endpoint. The server
maintains an in-memory list of waypoints, which can be retrieved or updated by sending appropriate requests to the endpoint.

Contains:
- `WaypointsHandler`: A request handler class that processes incoming HTTP requests to manage waypoints data.
- `run()`: A function that initializes and starts the HTTP server to handle waypoint requests.
- `frontend`: A directory containing the HTML and Typescript code for the map widget's frontend interface.
"""

__all__ = ["WaypointsHandler", "run"]
