"""
Module containing miscellaneous utility functions for the Groundstation application.

Functions:
- get_icons: Load and return a set of icons for the application.
- get_route: Get the full URL for a given route name.
- haversine_distance_m: Calculate the great-circle distance between two coordinates in meters.
- destination_point_m: Calculate the point reached by travelling a distance along a great circle.
- format_duration: Format a duration in seconds as a compact human-readable string.
- pushbutton_maker: Create a QPushButton with specified features.
- create_timer: Create a QTimer with specified interval and single-shot status.
- copy_qtimer: Create a copy of a QTimer with the same interval and single-shot status.
- cache_cdn_file: Download and cache a file to serve in a local CDN server.
- create_symlinks: Create symbolic links for all files in the source directory to the target directory.
- resolve_enum_name: Resolve a telemetry enum value to its member name.

Classes:
- IconProtocol: Protocol defining the expected attributes for icon objects.
"""

import math
import os
import socket
from collections.abc import Callable
from enum import Enum
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Protocol, TypeVar, cast

import numpy as np
import numpy.typing as npt
import qtawesome as qta
from requests import RequestException

from qtpy.QtCore import QTimer
from qtpy.QtGui import QIcon
from qtpy.QtWidgets import QPushButton

from utils import constants
from utils.console_logger import get_logger

__all__ = [
    "cache_cdn_file",
    "copy_qtimer",
    "create_symlinks",
    "create_timer",
    "destination_point_m",
    "format_duration",
    "get_icons",
    "get_route",
    "haversine_distance_m",
    "pushbutton_maker",
    "resolve_enum_name",
]

logger = get_logger(__name__)

T = TypeVar("T")


class IconProtocol(Protocol):
    upload: QIcon
    download: QIcon
    connect: QIcon
    disconnect: QIcon
    delete: QIcon
    add: QIcon
    home: QIcon
    save: QIcon
    pause: QIcon
    play: QIcon
    play_circle: QIcon
    play_circle_outline: QIcon
    green_play_circle_outline: QIcon
    stop: QIcon
    stop_circle: QIcon
    stop_circle_outline: QIcon
    red_stop_circle_outline: QIcon
    cog: QIcon
    pencil: QIcon
    refresh: QIcon
    hard_drive: QIcon
    boat: QIcon
    image_upload: QIcon
    notification: QIcon
    warning: QIcon
    question: QIcon


def get_icons() -> IconProtocol:
    """
    Load and return a set of icons for the application.

    Returns
    -------
    :class:`IconProtocol`
        An object containing the loaded icons as attributes.

    Examples
    --------
    >>> icons = get_icons()
    >>> icons.upload

    Notes
    -----
    The icons cannot be loaded until a :class:`QApplication` instance is created.
    """

    icons: dict[str, QIcon] = {
        "upload": qta.icon("mdi.upload", color="white"),
        "download": qta.icon("mdi.download", color="white"),
        "connect": qta.icon("mdi.connection", color="white"),
        "disconnect": qta.icon("fa6s.plug-circle-xmark", color="white"),
        "delete": qta.icon("mdi.trash-can", color="white"),
        "add": qta.icon("mdi.plus", color="white"),
        "home": qta.icon("mdi.home", color="white"),
        "save": qta.icon("mdi.content-save", color="white"),
        "pause": qta.icon("mdi.pause-circle", color="white"),
        "play": qta.icon("mdi.play-circle", color="white"),
        "play_circle": qta.icon("mdi.play-circle", color="white"),
        "play_circle_outline": qta.icon("mdi.play-circle-outline", color="white"),
        "green_play_circle_outline": qta.icon("mdi.play-circle-outline", color="green"),
        "stop": qta.icon("mdi.stop-circle", color="white"),
        "stop_circle": qta.icon("mdi.stop-circle", color="white"),
        "stop_circle_outline": qta.icon("mdi.stop-circle-outline", color="white"),
        "red_stop_circle_outline": qta.icon("mdi.stop-circle-outline", color="red"),
        "cog": qta.icon("mdi.cog", color="white"),
        "pencil": qta.icon("ei.pencil", color="white"),
        "refresh": qta.icon("mdi.refresh", color="white"),
        "hard_drive": qta.icon("fa6.hard-drive", color="white"),
        "boat": qta.icon("mdi.sail-boat", color="white"),
        "image_upload": qta.icon("mdi.image-move", color="white"),
        "notification": qta.icon("mdi.bell", color="white"),
        "warning": qta.icon("mdi.alert", color="white"),
        "question": qta.icon("mdi.help-circle", color="white"),
    }

    for icon_name, icon in icons.items():
        if not isinstance(icon, QIcon):
            raise TypeError(f"Icon '{icon_name}' is not a valid QIcon. Check the icon name or make sure the icon is available.")

    return cast("IconProtocol", SimpleNamespace(**icons))


def check_port_available(port: int) -> bool:
    """
    Check if a TCP port is available for binding.

    Parameters
    ----------
    port
        The TCP port number to check.

    Returns
    -------
    bool
        `True` if the port is available, `False` if it is already in use.
    """

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(1.0)
        return sock.connect_ex(("127.0.0.1", port)) != 0


def get_route(route_name: str) -> str:
    """
    Get the full URL for a given route name.

    Parameters
    ----------
    route_name
        The name of the route.

    Returns
    -------
    `str`
        The full URL for the given route name.

    Raises
    ------
    :class:`ValueError`
        If the route name is not found in the telemetry server endpoints.
    """

    endpoints = constants.SM.read_dict("telemetry_server_endpoints")

    if isinstance(endpoints, dict) and endpoints.get(route_name) is not None:
        return endpoints[route_name]

    raise ValueError(f"Route name '{route_name}' not found in telemetry server endpoints.")


def haversine_distance_m(
    lat1: npt.ArrayLike, lon1: npt.ArrayLike, lat2: npt.ArrayLike, lon2: npt.ArrayLike
) -> float | npt.NDArray[np.float64]:
    """
    Calculate the great-circle distance between two coordinates.

    Both scalars and arrays are accepted. Scalar inputs take a :mod:`math` fast path;
    array inputs are broadcast against each other in numpy, so every leg of a path
    can be measured in a single call.

    Examples
    --------
    >>> round(haversine_distance_m(0.0, 0.0, 0.0, 1.0))
    111195
    >>> leg_distances = haversine_distance_m(
    ...     np.array([0.0, 0.0]),
    ...     np.array([0.0, 1.0]),
    ...     np.array([0.0, 1.0]),
    ...     np.array([1.0, 2.0]),
    ... )
    >>> leg_distances.shape
    (2,)

    Parameters
    ----------
    lat1
        Latitude(s) of the first point(s) in decimal degrees.
    lon1
        Longitude(s) of the first point(s) in decimal degrees.
    lat2
        Latitude(s) of the second point(s) in decimal degrees.
    lon2
        Longitude(s) of the second point(s) in decimal degrees.

    Returns
    -------
    `float` or :class:`numpy.ndarray`
        The distance between the points in meters. A `float` is returned for scalar
        inputs, and an array of the broadcast shape for array inputs.
    """

    def scalar_m(lat_a: float, lon_a: float, lat_b: float, lon_b: float) -> float:
        """Measure one pair of coordinates with :mod:`math`, avoiding numpy's dispatch overhead."""

        delta_lat = math.radians(lat_b - lat_a)
        delta_lon = math.radians(lon_b - lon_a)

        a = math.sin(delta_lat / 2) ** 2 + (
            math.cos(math.radians(lat_a)) * math.cos(math.radians(lat_b)) * math.sin(delta_lon / 2) ** 2
        )

        return 2 * constants.EARTH_RADIUS_METERS * math.asin(math.sqrt(min(1.0, a)))

    def array_m(
        lat_a: npt.ArrayLike, lon_a: npt.ArrayLike, lat_b: npt.ArrayLike, lon_b: npt.ArrayLike
    ) -> npt.NDArray[np.float64]:
        """Measure every pair at once, broadcasting the inputs against each other."""

        latitudes_1 = np.asarray(lat_a, dtype=np.float64)
        longitudes_1 = np.asarray(lon_a, dtype=np.float64)
        latitudes_2 = np.asarray(lat_b, dtype=np.float64)
        longitudes_2 = np.asarray(lon_b, dtype=np.float64)

        delta_lat = np.radians(latitudes_2 - latitudes_1)
        delta_lon = np.radians(longitudes_2 - longitudes_1)

        a = np.sin(delta_lat / 2) ** 2 + (
            np.cos(np.radians(latitudes_1)) * np.cos(np.radians(latitudes_2)) * np.sin(delta_lon / 2) ** 2
        )

        return 2 * constants.EARTH_RADIUS_METERS * np.arcsin(np.sqrt(np.minimum(1.0, a)))

    if (
        isinstance(lat1, (int, float))
        and isinstance(lon1, (int, float))
        and isinstance(lat2, (int, float))
        and isinstance(lon2, (int, float))
    ):
        return scalar_m(lat1, lon1, lat2, lon2)

    distances = array_m(lat1, lon1, lat2, lon2)
    return float(distances) if distances.ndim == 0 else distances


def destination_point_m(lat: float, lon: float, bearing_deg: float, distance_m: float) -> tuple[float, float]:
    """
    Calculate the point reached by travelling a distance along a great circle.

    This is the inverse of :func:`haversine_distance_m`, and is useful for placing
    points a fixed distance away from a known location.

    Examples
    --------
    >>> point = destination_point_m(0.0, 0.0, 0.0, 111_195.0)
    >>> round(point[0], 3)
    1.0
    >>> round(point[1], 3)
    0.0

    Parameters
    ----------
    lat
        Latitude of the starting point in decimal degrees.
    lon
        Longitude of the starting point in decimal degrees.
    bearing_deg
        Initial bearing in degrees clockwise from true north.
    distance_m
        Distance to travel in meters.

    Returns
    -------
    `tuple[float, float]`
        The destination latitude and longitude in decimal degrees, with the longitude
        normalized to the range ``[-180, 180]``.
    """

    angular_distance = distance_m / constants.EARTH_RADIUS_METERS
    bearing = math.radians(bearing_deg)
    start_lat = math.radians(lat)
    start_lon = math.radians(lon)

    sin_start_lat = math.sin(start_lat)
    cos_start_lat = math.cos(start_lat)
    sin_angular_distance = math.sin(angular_distance)
    cos_angular_distance = math.cos(angular_distance)

    dest_lat = math.asin(
        (sin_start_lat * cos_angular_distance) + (cos_start_lat * sin_angular_distance * math.cos(bearing))
    )
    dest_lon = start_lon + math.atan2(
        math.sin(bearing) * sin_angular_distance * cos_start_lat,
        cos_angular_distance - (sin_start_lat * math.sin(dest_lat)),
    )

    # atan2 can land outside [-pi, pi], so wrap it back into a valid longitude
    normalized_lon = (math.degrees(dest_lon) + 540) % 360 - 180

    return math.degrees(dest_lat), normalized_lon


def format_duration(seconds: float) -> str:
    """
    Format a duration in seconds as a compact human-readable string.

    Examples
    --------
    >>> format_duration(45)
    '45s'
    >>> format_duration(754)
    '12m 34s'
    >>> format_duration(5400)
    '1h 30m'
    >>> format_duration(90061)
    '1d 01h'

    Parameters
    ----------
    seconds
        The duration in seconds.

    Returns
    -------
    `str`
        The formatted duration, or ``"N/A"`` if the value is not a finite, non-negative number.
    """

    if not math.isfinite(seconds) or seconds < 0:
        return "N/A"

    total_seconds = round(seconds)

    if total_seconds < 60:
        return f"{total_seconds}s"

    minutes, secs = divmod(total_seconds, 60)
    if minutes < 60:
        return f"{minutes}m {secs:02d}s"

    hours, minutes = divmod(minutes, 60)
    if hours < 24:
        return f"{hours}h {minutes:02d}m"

    days, hours = divmod(hours, 24)
    return f"{days}d {hours:02d}h"


def pushbutton_maker(
    button_text: str,
    function: Callable[[], None],
    icon: QIcon | None,
    style_sheet: str | None = None,
    max_width: int | None = None,
    min_height: int | None = None,
    is_clickable: bool = True,
    tooltip: str | None = None,
) -> QPushButton:
    """
    Create a :class:`QPushButton` with the specified features.

    Parameters
    ----------
    button_text
        The text to display on the button.
    function
        The function to connect to the button's clicked signal.
    icon
        The icon to display on the button.
    style_sheet
        An optional style sheet to apply to the button. If not specified, the default style is used.
    max_width
        The maximum width of the button. If not specified, not used.
    min_height
        The minimum height of the button. If not specified, not used.
    is_clickable
        Whether the button should be clickable. Defaults to `True`.
    tooltip
        An optional tooltip to show when hovering over the button.

    Returns
    -------
    :class:`QPushButton`
        The created button.

    Raises
    ------
    :class:`RuntimeError`
        If the button could not be created.
    """

    try:
        button = QPushButton(button_text)
        button.clicked.connect(function)

        if icon is not None:
            button.setIcon(icon)

        if style_sheet is not None:
            button.setStyleSheet(style_sheet)

        if tooltip is not None:
            button.setToolTip(tooltip)

        if max_width is not None:
            button.setMaximumWidth(max_width)

        if min_height is not None:
            button.setMinimumHeight(min_height)

        button.setDisabled(not is_clickable)

    except Exception as e:
        raise RuntimeError(f"Failed to create button '{button_text}': {e}") from e

    return button


def create_timer(interval_ms: int, single_shot: bool = False) -> QTimer:
    """
    Create a :class:`QTimer` with the specified interval and single-shot status.

    Parameters
    ----------
    interval_ms
        The interval in milliseconds for the timer.
    single_shot
        Whether the timer should be single-shot. Defaults to `False`.

    Returns
    -------
    :class:`QTimer`
    """

    timer = QTimer()
    timer.setInterval(interval_ms)
    timer.setSingleShot(single_shot)
    return timer


def copy_qtimer(original: QTimer) -> QTimer:
    """
    Create a copy of a :class:`QTimer` with the same interval and single-shot status but without copying connections.

    Parameters
    ----------
    original
        The original QTimer to copy.

    Returns
    -------
    :class:`QTimer`
    """

    new_timer = QTimer()
    new_timer.setInterval(original.interval())
    new_timer.setSingleShot(original.isSingleShot())
    return new_timer


def cache_cdn_file(url: str, save_dir: str) -> None:
    """
    Download and cache a CDN file locally.

    Parameters
    ----------
    url
        The URL of the CDN file to download.
    save_dir
        The directory to save the cached file.

    Raises
    ------
    :class:`RuntimeError`
        If the file could not be downloaded and is not already cached.
    """

    file_name = url.rsplit("/", maxsplit=1)[-1]
    save_path = Path(save_dir) / file_name

    try:
        response = constants.REQ_SESSION.get(url)
        response.raise_for_status()

        Path(save_path).write_bytes(response.content)

        logger.info(f"Cached CDN file '{file_name}' to '{save_path}'.")

    except RequestException as e:
        if file_name in os.listdir(save_dir):
            logger.warning(f"Failed to download '{file_name}' from CDN, using cached version. Error: {e}")

        else:
            raise RuntimeError(f"Failed to download and cache CDN file '{file_name}': {e}") from e


def create_symlinks(source_dir: Path, target_dir: Path) -> None:
    """
    Create symbolic links for all files in the source directory to the target directory.

    Parameters
    ----------
    source_dir
        The directory containing the original files.
    target_dir
        The directory where the symbolic links will be created.

    Raises
    ------
    :class:`RuntimeError`
        If a symbolic link cannot be created.
    """

    if not source_dir.is_dir():
        raise RuntimeError(f"Symlink source directory does not exist: '{source_dir}'")

    target_dir.mkdir(parents=True, exist_ok=True)

    for item in source_dir.iterdir():
        if item.is_file() and item.name != ".DS_Store":
            target_path = target_dir / item.name
            try:
                source_path = item.resolve(strict=True)

                if target_path.is_symlink():
                    try:
                        if target_path.resolve(strict=True) == source_path:
                            continue
                    except FileNotFoundError:
                        pass

                if target_path.exists() or target_path.is_symlink():
                    target_path.unlink()

                target_path.symlink_to(source_path)
                logger.info(f"Created symlink for '{item.name}' at '{target_path}'.")

            except PermissionError as e:
                raise RuntimeError(
                    f"Failed to create symlink for '{item.name}' at '{target_path}': "
                    f"permission denied. Ensure '{target_dir}' is writable by the current user."
                ) from e

            except Exception as e:
                raise RuntimeError(f"Failed to create symlink for '{item.name}': {e}") from e


def resolve_enum_name(enum_class: type[Enum], value: Any) -> str:
    """
    Resolve a telemetry enum value to its member name.

    The telemetry server may send either the enum's integer value (e.g., `4`) or its
    member name as a string (e.g., `"CW_TACKING"`). This helper handles both formats
    and falls back to a string representation of the raw value if it cannot be resolved,
    so that a single unexpected value does not crash the telemetry display.

    Parameters
    ----------
    enum_class
        The enum class to resolve against (e.g., :class:`SailboatAutopilotStates`).
    value
        The raw value received from the telemetry server.

    Returns
    -------
    `str`
        The resolved enum member name, or a stringified fallback.
    """

    if isinstance(value, str) and value in enum_class.__members__:
        return value

    try:
        return enum_class(int(value)).name
    except (ValueError, KeyError, TypeError):
        pass

    return str(value) if value is not None else "N/A"
