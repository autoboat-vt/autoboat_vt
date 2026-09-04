import pathlib
from typing import cast
from urllib.parse import urljoin

from requests import RequestException

from qtpy.QtCore import QThread, Signal

from utils import constants, misc
from utils.console_logger import get_logger

__all__ = [
    "AutopilotThreadRouter",
    "BoatStatusThreadRouter",
    "ImageThreadRouter",
    "InstanceManagerThreadRouter",
    "WaypointThreadRouter",
]

logger = get_logger(__name__)


class AutopilotThreadRouter:
    """
    Class containing :class:`QThread` classes dealing with the ``autopilot_parameters`` endpoint.

    Attributes
    ----------
    - :class:`ActiveHashFetcherThread` -> Fetches the currently active autopilot parameter configuration hash.
    - :class:`AvailableHashesFetcherThread` -> Fetches available autopilot parameter configuration hashes.
    """

    class ActiveHashFetcherThread(QThread):
        """
        Fetch the currently active autopilot parameter configuration hash from the telemetry server.

        Attributes
        ----------
        response
            Signal to send the active hash to the main thread. Emits a tuple containing:
                - a dictionary containing the active hash information,
                - a :class:`TelemetryStatus` enum value indicating the status of the request.

        Inherits
        --------
        :class:`QThread`
        """

        response = Signal(tuple)

        def __init__(self) -> None:
            super().__init__()

        def run(self) -> None:
            """Run the thread to fetch currently active autopilot parameter configuration hash from the telemetry server."""

            self.get_params()

        def get_params(self) -> None:
            """Fetch currently active autopilot parameter configuration hash and emit it."""

            try:
                data = constants.REQ_SESSION.get(
                    urljoin(
                        misc.get_route("get_current_hash"),
                        str(constants.SM.read_int("telemetry_server_instance_id")),
                    )
                ).text

                if not isinstance(data, str):
                    raise TypeError

            except RequestException:
                self.response.emit(("", constants.TelemetryStatus.FAILURE))

            except TypeError:
                self.response.emit(("", constants.TelemetryStatus.WRONG_FORMAT))

            else:
                self.response.emit((data, constants.TelemetryStatus.SUCCESS))

    class AvailableHashesFetcherThread(QThread):
        """
        Thread to fetch available autopilot parameter configuration hashes from the telemetry server.

        Attributes
        ----------
        response
            Signal to send available hashes to the main thread. Emits a tuple containing:
                - a list of dictionaries, where each dictionary contains information about an available hash,
                - a :class:`TelemetryStatus` enum value indicating the status of the request.

        Inherits
        --------
        :class:`QThread`
        """

        response = Signal(tuple)

        def __init__(self) -> None:
            super().__init__()

        def run(self) -> None:
            """Run the thread to fetch available default autopilot parameter hashes."""

            self.get_available_hashes()

        def get_available_hashes(self) -> None:
            """Fetch available default autopilot parameter hashes and emit them."""

            try:
                data = constants.REQ_SESSION.get(misc.get_route("get_all_hashes")).json()

                if not isinstance(data, list):
                    raise TypeError

                if not all(isinstance(hash_info, dict) for hash_info in data):
                    raise TypeError

            except RequestException:
                self.response.emit(([], constants.TelemetryStatus.FAILURE))

            except TypeError:
                self.response.emit(([], constants.TelemetryStatus.WRONG_FORMAT))

            else:
                self.response.emit((data, constants.TelemetryStatus.SUCCESS))


class BoatStatusThreadRouter:
    """
    Class containing :class:`QThread` classes dealing with the ``boat_status`` endpoint.

    Attributes
    ----------
    - :class:`BoatStatusFetcherThread` -> Fetches boat status via HTTP polling.
    """

    class BoatStatusFetcherThread(QThread):
        """
        Fetch boat status from the telemetry server via HTTP polling.

        Attributes
        ----------
        data_fetched
            Signal emitted when a new boat status fetch completes. Emits a tuple containing:
                - a dictionary containing the boat status data,
                - a :class:`TelemetryStatus` enum value indicating the status of the request.

        Inherits
        --------
        :class:`QThread`
        """

        data_fetched = Signal(tuple)

        def run(self) -> None:
            """Run the thread to fetch boat status from the telemetry server."""

            while not self.isInterruptionRequested():
                instance_id = constants.SM.read_int("telemetry_server_instance_id")
                try:
                    data = constants.REQ_SESSION.get(
                        urljoin(misc.get_route("get_boat_status"), str(instance_id))
                    ).json()

                    if not isinstance(data, dict):
                        raise TypeError

                except RequestException:
                    result = ({}, constants.TelemetryStatus.FAILURE)

                except TypeError:
                    result = ({}, constants.TelemetryStatus.WRONG_FORMAT)

                else:
                    result = (data, constants.TelemetryStatus.SUCCESS)

                if instance_id != constants.SM.read_int("telemetry_server_instance_id"):
                    continue

                self.data_fetched.emit(result)


class InstanceManagerThreadRouter:
    """
    Class containing :class:`QThread` classes dealing with the ``instance_manager`` endpoint.

    Attributes
    ----------
    - :class:`InstanceFetcherThread` -> Fetches available instances.
    """

    class InstanceFetcherThread(QThread):
        """
        Fetch instances from the telemetry server.

        Attributes
        ----------
        response
            Signal to send instances to the main thread. Emits a tuple containing:
                - a list of dictionaries, where each dictionary contains information about an instance,
                - a :class:`TelemetryStatus` enum value indicating the status of the request.

        Inherits
        --------
        :class:`QThread`
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
                data = constants.REQ_SESSION.get(misc.get_route("get_all_instance_info")).json()

                if not isinstance(data, list):
                    raise TypeError

                if not all(isinstance(instance, dict) for instance in data):
                    raise TypeError

            except RequestException:
                self.response.emit(([], constants.TelemetryStatus.FAILURE))

            except TypeError:
                self.response.emit(([], constants.TelemetryStatus.WRONG_FORMAT))

            else:
                self.response.emit((data, constants.TelemetryStatus.SUCCESS))


class WaypointThreadRouter:
    """
    Class containing :class:`QThread` classes dealing with waypoints.

    Attributes
    ----------
    - :class:`RemoteFetcherThread` -> Fetches waypoints from the telemetry server.
    - :class:`LocalFetcherThread` -> Fetches waypoints from the local server.
    """

    class RemoteFetcherThread(QThread):
        """
        Thread to fetch waypoints from the telemetry server.

        Inherits
        -------
        :class:`QThread`

        Attributes
        ----------
        response
            Signal to send waypoints to the main thread. Emits a tuple containing:
                - a list of waypoints, where each waypoint is a list of `[latitude, longitude]`,
                - a :class:`TelemetryStatus` enum value indicating the status of the request.
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
                        misc.get_route("get_waypoints"),
                        str(constants.SM.read_int("telemetry_server_instance_id")),
                    )
                ).json()

                if not isinstance(data, list):
                    raise TypeError

                for waypoint in data:
                    if not isinstance(waypoint, (tuple, list)):
                        raise TypeError

                    if not all(isinstance(coord, (int, float)) for coord in waypoint):
                        raise TypeError

            except RequestException:
                self.response.emit(([], constants.TelemetryStatus.FAILURE))

            except TypeError:
                self.response.emit(([], constants.TelemetryStatus.WRONG_FORMAT))

            else:
                self.response.emit((data, constants.TelemetryStatus.SUCCESS))

    class LocalFetcherThread(QThread):
        """
        Thread to fetch waypoints from the local server.

        Attributes
        ----------
        response
            Signal to send waypoints to the main thread. Emits a tuple containing:
                - a list of waypoints, where each waypoint is a list of `[latitude, longitude]`,
                - a :class:`TelemetryStatus` enum value indicating the status of the request.

        Inherits
        --------
        :class:`QThread`
        """

        response = Signal(tuple)

        def __init__(self) -> None:
            super().__init__()

        def run(self) -> None:
            """Run the thread to fetch waypoints from the local server."""

            self.get_waypoints()

        def get_waypoints(self) -> None:
            """Fetch waypoints from the local server and emit them."""

            try:
                data = constants.REQ_SESSION.get(constants.SM.read_str("waypoints_server_url")).json()

                if not isinstance(data, list):
                    raise TypeError

                for waypoint in data:
                    if not isinstance(waypoint, (tuple, list)):
                        raise TypeError
                    if not all(isinstance(coord, (int, float)) for coord in waypoint):
                        raise TypeError

            except RequestException:
                self.response.emit(([], constants.TelemetryStatus.FAILURE))

            except TypeError:
                self.response.emit(([], constants.TelemetryStatus.WRONG_FORMAT))

            else:
                self.response.emit((data, constants.TelemetryStatus.SUCCESS))

class ImageThreadRouter:
    """
    Class containing :class:`QThread` classes dealing with image fetching.

    Attributes
    ----------
    - :class:`ImageFetcher` -> Fetches images from the telemetry server.
    """

    class ImageFetcher(QThread):
        """
        Thread to fetch images from the telemetry server.

        Attributes
        ----------
        data_fetched
            Signal to send image to the main thread.

        Inherits
        --------
        :class:`QThread`
        """

        data_fetched = Signal(bytes)

        def __init__(self) -> None:
            super().__init__()

        def run(self) -> None:
            """Run the thread to fetch images from the telemetry server."""

            self.get_image()

        def get_image(self) -> None:
            """
            Fetch an image from the telemetry server and emit it.

            Raises
            ------
            :class:`ValueError`
                If the image data is empty.
            """

            try:
                response = constants.REQ_SESSION.get(
                    urljoin(
                        misc.get_route("get_current_image"),
                        str(constants.SM.read_int("telemetry_server_instance_id")),
                    )
                )

                if response.status_code != 200:
                    raise RequestException(f"HTTP {response.status_code}: {response.text.strip()}")

                image = response.content
                if not image:
                    raise ValueError("Image data is empty")

            except RequestException as e:
                logger.warning(f"Failed to fetch image from telemetry server: {e}")
                image = pathlib.Path(constants.ASSETS_DIR / "new_logo.png").read_bytes()

            except ValueError as e:
                logger.warning(f"{e}")
                image = pathlib.Path(constants.ASSETS_DIR / "new_logo.png").read_bytes()

            image = cast("bytes", image)
            self.data_fetched.emit(image)
