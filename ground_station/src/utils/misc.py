"""
Module containing miscellaneous utility functions for the Groundstation application.

Functions:
- get_icons: Load and return a set of icons for the application.
- get_route: Get the full URL for a given route name.
- pushbutton_maker: Create a QPushButton with specified features.
- create_timer: Create a QTimer with specified interval and single-shot status.
- copy_qtimer: Create a copy of a QTimer with the same interval and single-shot status.
- cache_cdn_file: Download and cache a file to serve in a local CDN server.
- js_load_guard: Run JavaScript after the map API has loaded.
- create_symlinks: Create symbolic links for all files in the source directory to the target directory.
"""

__all__ = [
    "cache_cdn_file",
    "copy_qtimer",
    "create_symlinks",
    "create_timer",
    "get_icons",
    "get_route",
    "js_load_guard",
    "pushbutton_maker",
]

import os
import textwrap
from collections.abc import Callable
from pathlib import Path
from requests import RequestException
from types import SimpleNamespace
from typing import Protocol, TypeVar, cast

import qtawesome as qta
from qtpy.QtCore import QTimer
from qtpy.QtGui import QIcon
from qtpy.QtWidgets import QPushButton

from utils import constants

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
    IconProtocol
        An object containing the loaded icons as attributes.

    Example
    -------
    >>> icons = get_icons()
    >>> icons.upload

    Notes
    -----
    The icons cannot be loaded until a ``QApplication`` instance is created.
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

def get_route(route_name: str) -> str:
    """
    Get the full URL for a given route name.

    Parameters
    ----------
    route_name
        The name of the route.

    Returns
    -------
    str
        The full URL for the given route name.

    Raises
    ------
    ValueError
        If the route name is not found in the telemetry server endpoints.
    """

    endpoints = constants.SM.read_dict("telemetry_server_endpoints")

    if isinstance(endpoints, dict) and endpoints.get(route_name) is not None:
        return endpoints[route_name]
    
    raise ValueError(f"Route name '{route_name}' not found in telemetry server endpoints.")


def pushbutton_maker(
    button_text: str,
    icon: QIcon,
    function: Callable[[], None],
    style_sheet: str | None = None,
    max_width: int | None = None,
    min_height: int | None = None,
    is_clickable: bool = True,
    tooltip: str | None = None,
) -> QPushButton:
    """
    Create a ``QPushButton`` with the specified features.

    Parameters
    ----------
    button_text
        The text to display on the button.
    icon
        The icon to display on the button.
    function
        The function to connect to the button's clicked signal.
    style_sheet
        An optional style sheet to apply to the button. If not specified, the default style is used.
    max_width
        The maximum width of the button. If not specified, not used.
    min_height
        The minimum height of the button. If not specified, not used.
    is_clickable
        Whether the button should be clickable. Defaults to ``True``.
    tooltip
        An optional tooltip to show when hovering over the button.

    Returns
    -------
    QPushButton
        The created button.

    Raises
    ------
    RuntimeError
        If the button could not be created.
    """

    try:
        button = QPushButton(button_text)
        button.setIcon(icon)
        button.clicked.connect(function)

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
    Create a QTimer with the specified interval and single-shot status.

    Parameters
    ----------
    interval_ms
        The interval in milliseconds for the timer.
    single_shot
        Whether the timer should be single-shot. Defaults to ``False``.

    Returns
    -------
    QTimer
        A new QTimer instance with the specified interval and single-shot status.
    """

    timer = QTimer()
    timer.setInterval(interval_ms)
    timer.setSingleShot(single_shot)
    return timer


def copy_qtimer(original: QTimer) -> QTimer:
    """
    Create a copy of a QTimer with the same interval and single-shot status but without copying connections.

    Parameters
    ----------
    original
        The original QTimer to copy.

    Returns
    -------
    QTimer
        A new QTimer instance with the same interval and single-shot status as the original.
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
    RuntimeError
        If the file could not be downloaded and is not already cached.
    """

    file_name = url.rsplit("/", maxsplit=1)[-1]
    save_path = Path(save_dir) / file_name

    try:
        response = constants.REQ_SESSION.get(url)
        response.raise_for_status()

        Path(save_path).write_bytes(response.content)
        
        print(f"[Info] Cached CDN file '{file_name}' to '{save_path}'.")
    
    except RequestException as e:
        if file_name in os.listdir(save_dir):
            print(f"[Warning] Failed to download '{file_name}' from CDN, using cached version. Error: {e}")

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
    RuntimeError
        If a symbolic link cannot be created.
    """

    for item in source_dir.iterdir():
        if item.is_file() and item.name != ".DS_Store":
            target_path = target_dir / item.name
            try:
                if target_path.exists() or target_path.is_symlink():
                    target_path.unlink()
                
                target_path.symlink_to(item.resolve())

                # make file in git_ignore unwritable to prevent accidental edits
                target_path.chmod(0o444)
                print(f"[Info] Created symlink for '{item.name}' at '{target_path}'.")
            
            except Exception as e:
                raise RuntimeError(f"Failed to create symlink for '{item.name}': {e}") from e


def js_load_guard(js_code: str) -> str:
    """Run JavaScript after the map API has loaded."""

    indented_js_code = textwrap.indent(js_code.strip(), " " * 12)

    return textwrap.dedent(
        f"""
        function __runAutoboatMapCodeWhenReady() {{
            if (
                typeof map !== "undefined" &&
                typeof map.update_boat_location_and_heading === "function"
            ) {{
{indented_js_code}
                return;
            }}

            window.__autoboatPendingMapCode = __runAutoboatMapCodeWhenReady;

            if (!window.__autoboatPendingMapListenerAdded) {{
                window.__autoboatPendingMapListenerAdded = true;

                document.addEventListener(
                    "mapLoaded",
                    function handleMapLoaded() {{
                        window.__autoboatPendingMapListenerAdded = false;

                        if (window.__autoboatPendingMapCode) {{
                            const pendingMapCode = window.__autoboatPendingMapCode;
                            window.__autoboatPendingMapCode = null;
                            pendingMapCode();
                        }}
                    }},
                    {{ once: true }}
                );
            }}
        }}

        __runAutoboatMapCodeWhenReady();
        """
    ).strip()
