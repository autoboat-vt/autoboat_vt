import requests
import constants
from typing import Union
from qtpy.QtCore import QThread, Signal


class TelemetryUpdater(QThread):
    """
    Thread to fetch telemetry data from the telemetry server.

    Inherits
    --------
    `QThread`

    Attributes
    ----------
    boat_data_fetched: `Signal`
        Signal to send boat data to the main thread. Emits a dictionary containing telemetry data.
    """

    boat_data_fetched = Signal(dict)

    def __init__(self) -> None:
        super().__init__()

    def get_boat_data(self) -> None:
        """Fetch boat data from the telemetry server and emit it."""

        try:
            boat_status: dict[str, Union[str, float, list[float], list[list[float]]]]
            boat_status = requests.get(
                constants.TELEMETRY_SERVER_ENDPOINTS["boat_status"]
            ).json()

        except requests.exceptions.RequestException:
            boat_status = {
                "position": [36.983731367697374, -76.29555376681454],
                "state": "failed_to_fetch",
                "full_autonomy_maneuver": "N/A",
                "speed": 0.0,
                "bearing": 0.0,
                "heading": 0.0,
                "true_wind_speed": 0.0,
                "true_wind_angle": 0.0,
                "apparent_wind_speed": 0.0,
                "apparent_wind_angle": 0.0,
                "sail_angle": 0.0,
                "rudder_angle": 0.0,
                "current_waypoint_index": 0,
                "current_route": [[0.0, 0.0]],
                "vesc_data_rpm": 0.0,
                "vesc_data_duty_cycle": 0.0,
                "vesc_data_amp_hours": 0.0,
                "vesc_data_amp_hours_charged": 0.0,
                "vesc_data_current_to_vesc": 0.0,
                "vesc_data_voltage_to_motor": 0.0,
                "vesc_data_voltage_to_vesc": 0.0,
                "vesc_data_wattage_to_motor": 0.0,
                "vesc_data_time_since_vesc_startup_in_ms": 0.0,
                "vesc_data_motor_temperature": 0.0,
            }
            print("Warning: Failed to fetch boat data. Using default values.")

        self.boat_data_fetched.emit(boat_status)

    def run(self) -> None:
        """Run the thread to fetch boat data from the telemetry server."""

        self.get_boat_data()


class LocalWaypointFetcher(QThread):
    """
    Thread to fetch waypoints from the local server.

    Inherits
    -------
    `QThread`

    Attributes
    ----------
    waypoints_fetched: `Signal`
        Signal to send waypoints to the main thread. Emits a list of lists containing
        waypoints, where each waypoint is a list of `[latitude, longitude]`.
    """

    waypoints_fetched = Signal(list)

    def __init__(self) -> None:
        super().__init__()

    def run(self) -> None:
        """Run the thread to fetch waypoints from the local server."""

        self.get_waypoints()

    def get_waypoints(self) -> None:
        """Fetch waypoints from the local server and emit them."""

        try:
            waypoints: list[list[float]] = requests.get(
                constants.WAYPOINTS_SERVER_URL
            ).json()
            if not isinstance(waypoints, list):
                raise ValueError("Waypoints data is not a list")

        except requests.exceptions.RequestException:
            waypoints = []
            print("Warning: Failed to fetch waypoints. Using empty list.")

        except ValueError:
            print(
                f"Warning: Waypoints data is not in expected format. Using empty list.\nExpected: {list[list[float]]}, Received: {waypoints}",
            )
            waypoints = []

        self.waypoints_fetched.emit(waypoints)


class RemoteWaypointFetcher(QThread):
    """
    Thread to fetch waypoints from the telemetry server.

    Inherits
    -------
    `QThread`

    Attributes
    ----------
    waypoints_fetched: `Signal`
        Signal to send waypoints to the main thread. Emits a list of lists containing
        waypoints, where each waypoint is a list of `[latitude, longitude]`.
    """

    waypoints_fetched = Signal(list)
    request_url_change = Signal(bool)

    def __init__(self) -> None:
        super().__init__()

    def run(self) -> None:
        """Run the thread to fetch waypoints from the telemetry server."""

        self.get_waypoints()

    def get_waypoints(self) -> None:
        """Fetch waypoints from the telemetry server and emit them."""

        try:
            waypoints = requests.get(
                constants.TELEMETRY_SERVER_ENDPOINTS["get_waypoints"]
            ).json()
            if not isinstance(waypoints, list):
                raise ValueError("Waypoints data is not a list")

        except requests.exceptions.RequestException:
            waypoints = []
            print("Warning: Failed to fetch waypoints. Using empty list.")
            self.request_url_change.emit(True)

        except ValueError:
            print(
                f"Warning: Waypoints data is not in expected format. Using empty list.\nExpected: {list[list[float]]}, Received: {waypoints}",
            )
            waypoints = []
            self.request_url_change.emit(True)

        self.request_url_change.emit(False)
        self.waypoints_fetched.emit(waypoints)


class ImageFetcher(QThread):
    """
    Thread to fetch images from the telemetry server.

    Inherits
    -------
    `QThread`

    Attributes
    ----------
    image_fetched: `Signal`
        Signal to send image to the main thread. Emits a base64 encoded string of the image.
    """

    image_fetched = Signal(str)

    def __init__(self) -> None:
        super().__init__()

    def run(self) -> None:
        """Run the thread to fetch images from the telemetry server."""

        self.get_image()

    def get_image(self) -> None:
        """Fetch an image from the telemetry server and emit it as a base64 encoded string."""

        try:
            image_data = requests.get(
                constants.TELEMETRY_SERVER_ENDPOINTS["get_autopilot_parameters"],
                timeout=5,
            ).json()
            base64_encoded_image = image_data.get("current_camera_image")
            if base64_encoded_image is None:
                raise ValueError("Image data is None")

        except requests.exceptions.RequestException:
            base64_encoded_image = open(
                constants.ASSETS_DIR / "cool-guy-base64.txt"
            ).read()
            print("Warning: Failed to fetch image. Using cool guy image.")

        except ValueError as e:
            print(f"Warning: {e}")
            base64_encoded_image = open(
                constants.ASSETS_DIR / "cool-guy-base64.txt"
            ).read()

        self.image_fetched.emit(base64_encoded_image)
