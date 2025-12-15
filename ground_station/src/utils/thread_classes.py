"""
Module containing classes for handling background tasks in the ground station application.

Contains:
- AutopilotThreadRouter: Class containing `QThread` classes dealing with the `autopilot_parameters` endpoint.
- BoatStatusThreadRouter: Class containing `QThread` classes dealing with the `boat_status` endpoint.
- InstanceManagerThreadRouter: Class containing `QThread` classes dealing with the `instance_manager` endpoint.
- WaypointThreadRouter: Class containing `QThread` classes dealing with waypoints, both from the `waypoints` endpoint and the local server.
- ImageFetcher: `QThread` class for fetching images from the telemetry server.
"""

__all__ = [
    "AutopilotThreadRouter",
    "BoatStatusThreadRouter",
    "InstanceManagerThreadRouter",
    "WaypointThreadRouter",
    "ImageFetcher",
]

from urllib.parse import urljoin

from httpx import RequestError
from qtpy.QtCore import QThread, Signal

from utils import constants


class AutopilotThreadRouter:
    """
    Class containing `QThread` classes dealing with the `autopilot_parameters` endpoint.

    Subclasses
    ----------
    - `ParamFetcherThread` -> Fetches autopilot parameters.
    - `DefaultsFetcherThread` -> Fetches default autopilot parameters.
    """

    class ParamFetcherThread(QThread):
        """
        Thread to fetch autopilot parameters from the telemetry server.

        Inherits
        -------
        `QThread`

        Attributes
        ----------
        response
            Signal to send autopilot parameters to the main thread. Emits a tuple containing:
                - a dictionary of autopilot parameters,
                - a `TelemetryStatus` enum value indicating the status of the request.
        """

        response = Signal(tuple)

        def __init__(self) -> None:
            super().__init__()

        def run(self) -> None:
            """Run the thread to fetch autopilot parameters from the telemetry server."""

            self.get_params()

        def get_params(self) -> None:
            """Fetch autopilot parameters from the telemetry server and emit them."""

            try:
                data = constants.REQ_SESSION.get(
                    urljoin(
                        constants.TELEMETRY_SERVER_ENDPOINTS["get_autopilot_parameters"],
                        str(constants.TELEMETRY_SERVER_INSTANCE_ID),
                    )
                ).json()

                if not isinstance(data, dict):
                    raise TypeError

            except RequestError:
                self.response.emit(({}, constants.TelemetryStatus.FAILURE))

            except TypeError:
                self.response.emit(({}, constants.TelemetryStatus.WRONG_FORMAT))

            else:
                self.response.emit((data, constants.TelemetryStatus.SUCCESS))

    class DefaultsFetcherThread(QThread):
        """
        Thread to fetch default autopilot parameters from the telemetry server.

        Inherits
        -------
        `QThread`

        Attributes
        ----------
        response
            Signal to send default autopilot parameters to the main thread. Emits a tuple containing:
                - a dictionary of default autopilot parameters,
                - a `TelemetryStatus` enum value indicating the status of the request.
        """

        response = Signal(tuple)

        def __init__(self) -> None:
            super().__init__()

        def run(self) -> None:
            """Run the thread to fetch default autopilot parameters from the telemetry server."""

            self.get_defaults()

        def get_defaults(self) -> None:
            """Fetch default autopilot parameters from the telemetry server and emit them."""

            try:
                data = constants.REQ_SESSION.get(
                    urljoin(
                        constants.TELEMETRY_SERVER_ENDPOINTS["get_default_autopilot_parameters"],
                        str(constants.TELEMETRY_SERVER_INSTANCE_ID),
                    )
                ).json()

                if not isinstance(data, dict):
                    raise TypeError

            except RequestError:
                self.response.emit(({}, constants.TelemetryStatus.FAILURE))

            except TypeError:
                self.response.emit(({}, constants.TelemetryStatus.WRONG_FORMAT))

            else:
                self.response.emit((data, constants.TelemetryStatus.SUCCESS))


class BoatStatusThreadRouter:
    """
    Class containing `QThread` classes dealing with the `boat_status` endpoint.

    Subclasses
    ----------
    - `BoatStatusFetcherThread` -> Fetches boat status via WebSocket.
    """

    class BoatStatusFetcherThread(QThread):
        """
        Thread to fetch boat status from the telemetry server via HTTP polling.

        Inherits
        -------
        `QThread`

        Attributes
        ----------
        response
            Signal to send boat status to the main thread. Emits a tuple containing:
                - a dictionary of boat status,
                - a `TelemetryStatus` enum value indicating the status of the request.
        """

        response = Signal(tuple)

        def __init__(self) -> None:
            super().__init__()

        def run(self) -> None:
            """Run the thread to fetch boat status from the telemetry server."""

            self.get_boat_status()

        def get_boat_status(self) -> None:
            """Fetch boat status from the telemetry server and emit it continuously."""

            while True:
                try:
                    data = constants.REQ_SESSION.get(
                        urljoin(
                            constants.TELEMETRY_SERVER_ENDPOINTS["get_boat_status"],
                            str(constants.TELEMETRY_SERVER_INSTANCE_ID),
                        )
                    ).json()

                    if not isinstance(data, dict):
                        raise TypeError

                except RequestError:
                    self.response.emit(({}, constants.TelemetryStatus.FAILURE))

                except TypeError:
                    self.response.emit(({}, constants.TelemetryStatus.WRONG_FORMAT))

                else:
                    self.response.emit((data, constants.TelemetryStatus.SUCCESS))


class InstanceManagerThreadRouter:
    """
    Class containing `QThread` classes dealing with the `instance_manager` endpoint.

    Subclasses
    ----------
    - `InstanceFetcherThread` -> Fetches instances.
    """

    class InstanceFetcherThread(QThread):
        """
        Thread to fetch instances from the telemetry server.

        Inherits
        -------
        `QThread`

        Attributes
        ----------
        response
            Signal to send instances to the main thread. Emits a tuple containing:
                - a list of dictionaries representing instances,
                - a `TelemetryStatus` enum value indicating the status of the request.
        """

        response = Signal(tuple)

        def __init__(self) -> None:
            super().__init__()

        def run(self) -> None:
            """Run the thread to fetch instances from the telemetry server."""

            self.get_instances()

        def get_instances(self) -> None:
            """Fetch instances from the telemetry server and emit them."""

            try:
                data = constants.REQ_SESSION.get(constants.TELEMETRY_SERVER_ENDPOINTS["get_all_instance_info"]).json()

                if not isinstance(data, list):
                    raise TypeError

                if not all(isinstance(instance, dict) for instance in data):
                    raise TypeError

            except RequestError:
                self.response.emit(([], constants.TelemetryStatus.FAILURE))

            except TypeError:
                self.response.emit(([], constants.TelemetryStatus.WRONG_FORMAT))

            else:
                self.response.emit((data, constants.TelemetryStatus.SUCCESS))


class WaypointThreadRouter:
    """
    Class containing `QThread` classes dealing with waypoints.

    Subclasses
    ----------
    - `RemoteFetcherThread` -> Fetches waypoints from the telemetry server.
    - `LocalFetcherThread` -> Fetches waypoints from the local server.
    """

    class RemoteFetcherThread(QThread):
        """
        Thread to fetch waypoints from the telemetry server.

        Inherits
        -------
        `QThread`

        Attributes
        ----------
        response
            Signal to send waypoints to the main thread. Emits a tuple containing:
                - a list of waypoints, where each waypoint is a list of `[latitude, longitude]`,
                - a `TelemetryStatus` enum value indicating the status of the request.
        """

        response = Signal(tuple)

        def __init__(self) -> None:
            super().__init__()

        def run(self) -> None:
            """Run the thread to fetch waypoints from the telemetry server."""

            self.get_waypoints()

        def get_waypoints(self) -> None:
            """Fetch waypoints from the telemetry server and emit them."""

            try:
                data = constants.REQ_SESSION.get(
                    urljoin(
                        constants.TELEMETRY_SERVER_ENDPOINTS["get_waypoints"],
                        str(constants.TELEMETRY_SERVER_INSTANCE_ID),
                    )
                ).json()

                if not isinstance(data, list):
                    raise TypeError

                for waypoint in data:
                    if not isinstance(waypoint, (tuple, list)):
                        raise TypeError

                    if not all(isinstance(cord, (int, float)) for cord in waypoint):
                        raise TypeError

            except RequestError:
                self.response.emit(([], constants.TelemetryStatus.FAILURE))

            except TypeError:
                self.response.emit(([], constants.TelemetryStatus.WRONG_FORMAT))

            else:
                self.response.emit((data, constants.TelemetryStatus.SUCCESS))

    class LocalFetcherThread(QThread):
        """
        Thread to fetch waypoints from the local server.

        Inherits
        -------
        `QThread`

        Attributes
        ----------
        response
            Signal to send waypoints to the main thread. Emits a tuple containing:
                - a list of waypoints, where each waypoint is a list of `[latitude, longitude]`,
                - a `TelemetryStatus` enum value indicating the status of the request.
        """

        response = Signal(tuple)

        def __init__(self) -> None:
            super().__init__()

        def run(self) -> None:
            """Run the thread to fetch waypoints from the local server."""

            self.get_waypoints()

        def get_waypoints(self) -> None:
            """Fetch waypoints from the local server and emit them."""

            while True:
                try:
                    data = constants.REQ_SESSION.get(constants.WAYPOINTS_SERVER_URL).json()

                    if not isinstance(data, list):
                        raise TypeError

                    for waypoint in data:
                        if not isinstance(waypoint, (tuple, list)):
                            raise TypeError
                        if not all(isinstance(cord, (int, float)) for cord in waypoint):
                            raise TypeError

                except RequestError:
                    self.response.emit(([], constants.TelemetryStatus.FAILURE))

                except TypeError:
                    self.response.emit(([], constants.TelemetryStatus.WRONG_FORMAT))

                else:
                    self.response.emit((data, constants.TelemetryStatus.SUCCESS))


class ImageFetcher(QThread):
    """
    Thread to fetch images from the telemetry server.

    Inherits
    -------
    `QThread`

    Attributes
    ----------
    data_fetched
        Signal to send image to the main thread. Emits a base64 encoded string of the image.
    """

    data_fetched = Signal(str)

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
                )
            ).json()

            base64_encoded_image = image_data.get("current_camera_image")
            if base64_encoded_image is None:
                raise ValueError("Image data is None")

        except RequestError:
            print("[Warning] Failed to fetch image. Using cool guy image.")
            with open(constants.ASSETS_DIR / "cool-guy-base64.txt") as f:
                base64_encoded_image = f.read()

        except ValueError as e:
            print(f"[Warning] {e}")
            with open(constants.ASSETS_DIR / "cool-guy-base64.txt") as f:
                base64_encoded_image = f.read()

        self.data_fetched.emit(base64_encoded_image)
