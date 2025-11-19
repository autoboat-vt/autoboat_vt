"""
Module containing classes for handling background tasks in the ground station application.

Contains:
- TelemetryUpdater: Fetches telemetry data from the telemetry server.
- InstanceFetcher: Fetches the currently available instances from the telemetry server.
- LocalWaypointFetcher: Fetches waypoints from the local server.
- RemoteWaypointFetcher: Fetches waypoints from the telemetry server.
- TelemetryUpdater: Fetches telemetry data from the telemetry server.
"""

__all__ = ["ImageFetcher", "LocalWaypointFetcher", "RemoteWaypointFetcher", "TelemetryUpdater"]

import requests
from utils import constants
from urllib.parse import urljoin
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

    request_url_change: `Signal`
        Signal to request a change in the telemetry server URL.
        <ul>
        <li> <code>SUCCESS</code> indicates that the telemetry server is reachable and waypoints were fetched successfully.</li>
        <li> <code>FAILURE</code> indicates that the telemetry server is not reachable and waypoints could not be fetched.</li>
        </ul>
    """

    boat_data_fetched = Signal(dict)
    request_url_change = Signal(constants.TelemetryStatus)

    def __init__(self) -> None:
        super().__init__()

    def get_boat_data(self) -> None:
        """Fetch boat data from the telemetry server and emit it."""

        try:
            boat_status = constants.REQ_SESSION.get(
                urljoin(
                    constants.TELEMETRY_SERVER_ENDPOINTS["get_boat_status"], str(constants.TELEMETRY_SERVER_INSTANCE_ID)
                ),
                timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
            ).json()

            if not isinstance(boat_status, dict):
                raise TypeError

            self.request_url_change.emit(constants.TelemetryStatus.SUCCESS)

        except requests.exceptions.RequestException:
            boat_status = {}
            self.request_url_change.emit(constants.TelemetryStatus.FAILURE)

        except TypeError:
            print(
                f"[Warning] Telemetry data is not in expected format. Using empty dict. \nExpected: {dict}, Received: {boat_status}"
            )
            boat_status = {}
            self.request_url_change.emit(constants.TelemetryStatus.FAILURE)

        self.boat_data_fetched.emit(boat_status)

    def run(self) -> None:
        """Run the thread to fetch boat data from the telemetry server."""

        self.get_boat_data()


class InstanceFetcher(QThread):
    """
    Thread to fetch the currently available instances from the telemetry server.

    Inherits
    -------
    `QThread`

    Attributes
    ----------
    instances_fetched: `Signal`
        Signal to send instances to the main thread. Emits a list of dictionaries containing instance data.

    request_url_change: `Signal`
        Signal to request a change in the telemetry server URL.
        <ul>
        <li> <code>SUCCESS</code> indicates that the telemetry server is reachable and waypoints were fetched successfully.</li>
        <li> <code>FAILURE</code> indicates that the telemetry server is not reachable and waypoints could not be fetched.</li>
        </ul>
    """

    instances_fetched: list[dict] = Signal(list)
    request_url_change: constants.TelemetryStatus = Signal(constants.TelemetryStatus)

    def __init__(self) -> None:
        super().__init__()

    def run(self) -> None:
        """Run the thread to fetch instances from the telemetry server."""

        self.get_instances()

    def get_instances(self) -> None:
        """Fetch instances from the telemetry server and emit them."""

        try:
            instances = constants.REQ_SESSION.get(
                constants.TELEMETRY_SERVER_ENDPOINTS["get_all_instance_info"],
                timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
            ).json()

            if not isinstance(instances, list):
                raise TypeError

            if not all(isinstance(instance, dict) for instance in instances):
                raise TypeError

            self.request_url_change.emit(constants.TelemetryStatus.SUCCESS)

        except requests.exceptions.RequestException:
            instances = []
            self.request_url_change.emit(constants.TelemetryStatus.FAILURE)

        except TypeError:
            print(
                f"[Warning] Instances data is not in expected format. Using empty list.\nExpected: {list[dict]}, Received: {instances}",
            )
            instances = []
            self.request_url_change.emit(constants.TelemetryStatus.FAILURE)

        self.instances_fetched.emit(instances)


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
            waypoints = constants.REQ_SESSION.get(constants.WAYPOINTS_SERVER_URL).json()

            if not isinstance(waypoints, list):
                raise TypeError

            for waypoint in waypoints:
                if not isinstance(waypoint, (tuple, list)):
                    raise TypeError
                if not all(isinstance(cord, (int, float)) for cord in waypoint):
                    raise TypeError

        except requests.exceptions.RequestException as e:
            print(f"[Warning] Failed to fetch waypoints. Using empty list. Exception: {e}")
            waypoints = []

        except TypeError:
            print(
                f"[Warning] Waypoints data is not in expected format. Using empty list.\nExpected: {list[list[float]]}, Received: {waypoints}",
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

    request_url_change: `Signal`
        Signal to request a change in the telemetry server URL. Emits a value from the `constants.TelemetryStatus`. \\
        `SUCCESS` indicates that the telemetry server is reachable and waypoints were fetched successfully. \\
        `FAILURE` indicates that the telemetry server is not reacha le and waypoints could not be fetched.
    """

    waypoints_fetched = Signal(list)
    request_url_change = Signal(constants.TelemetryStatus)

    def __init__(self) -> None:
        super().__init__()

    def run(self) -> None:
        """Run the thread to fetch waypoints from the telemetry server."""

        self.get_waypoints()

    def get_waypoints(self) -> None:
        """Fetch waypoints from the telemetry server and emit them."""

        try:
            waypoints = constants.REQ_SESSION.get(
                urljoin(
                    constants.TELEMETRY_SERVER_ENDPOINTS["get_waypoints"], str(constants.TELEMETRY_SERVER_INSTANCE_ID)
                ),
                timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
            ).json()

            if not isinstance(waypoints, list):
                raise TypeError

            for waypoint in waypoints:
                if not isinstance(waypoint, (tuple, list)):
                    raise TypeError

                if not all(isinstance(cord, (int, float)) for cord in waypoint):
                    raise TypeError

            self.request_url_change.emit(constants.TelemetryStatus.SUCCESS)

        except requests.exceptions.RequestException:
            waypoints = []
            self.request_url_change.emit(constants.TelemetryStatus.FAILURE)

        except TypeError:
            print(
                f"[Warning] Waypoints data is not in expected format. Using empty list.\nExpected: {list[list[float]]}, Received: {waypoints}",
            )
            waypoints = []
            self.request_url_change.emit(constants.TelemetryStatus.FAILURE)

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
            image_data = constants.REQ_SESSION.get(
                urljoin(
                    constants.TELEMETRY_SERVER_ENDPOINTS["get_autopilot_parameters"],
                    str(constants.TELEMETRY_SERVER_INSTANCE_ID),
                ),
                timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
            ).json()

            base64_encoded_image = image_data.get("current_camera_image")
            if base64_encoded_image is None:
                raise ValueError("Image data is None")

        except requests.exceptions.RequestException:
            print("[Warning] Failed to fetch image. Using cool guy image.")
            with open(constants.ASSETS_DIR / "cool-guy-base64.txt") as f:
                base64_encoded_image = f.read()

        except ValueError as e:
            print(f"[Warning] {e}")
            with open(constants.ASSETS_DIR / "cool-guy-base64.txt") as f:
                base64_encoded_image = f.read()

        self.image_fetched.emit(base64_encoded_image)
