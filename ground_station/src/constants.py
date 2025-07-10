import os
import sys
from pathlib import PurePath
from qtpy.QtCore import QRect, QTimer
from qtpy.QtGui import QColor, QIcon
from qtpy.QtWidgets import QPushButton
import qtawesome as qta
from types import SimpleNamespace
from typing import Optional, Callable

# region functions
def __get_icons() -> SimpleNamespace:
    """
    Load and return a set of icons for the application.

    Returns
    -------
    SimpleNamespace
        A namespace object containing the loaded icons.
        Each icon can be accessed as an attribute of the namespace.

        Example: `icons.upload` = `icons["upload"]`

    Raises
    -------
    TypeError
        If an icon fails to load or is not a QIcon.
        This indicates that the icon name is not valid or the icon could not be found.
    """

    icons = {
        "upload": qta.icon("mdi.upload"),
        "download": qta.icon("mdi.download"),
        "delete": qta.icon("mdi.trash-can"),
        "save": qta.icon("mdi.content-save"),
        "cog": qta.icon("mdi.cog"),
        "pencil": qta.icon("ei.pencil"),
        "refresh": qta.icon("mdi.refresh"),
        "hard_drive": qta.icon("fa6.hard-drive"),
        "boat": qta.icon("mdi.sail-boat"),
        "image_upload": qta.icon("mdi.image-move"),
    }

    for icon_name, icon in icons.items():
        assert isinstance(icon, QIcon), (
            f"Icon '{icon_name}' is not a valid QIcon. "
            "Please check the icon name or ensure the icon is available."
        )

    return SimpleNamespace(**icons)

def pushbutton_maker(
    button_text: str,
    icon: QIcon,
    function: Callable,
    max_width: Optional[int] = None,
    min_height: Optional[int] = None,
    is_clickable: bool = True,
) -> QPushButton:
    """
    Create a `QPushButton` with the specified features.

    Parameters
    ----------
    button_text
        The text to display on the button.
    icon
        The icon to display on the button.
    function
        The function to connect to the button's clicked signal.
    max_width
        The maximum width of the button. If not specified, not used.
    min_height
        The minimum height of the button. If not specified, not used.
    is_clickable
        Whether the button should be clickable. Defaults to `True`.

    Returns
    -------
    QPushButton
        The created button.
    """

    button = QPushButton(button_text)
    button.setIcon(icon)
    if max_width is not None:
        button.setMaximumWidth(max_width)
    if min_height is not None:
        button.setMinimumHeight(min_height)
    button.clicked.connect(function)
    button.setDisabled(not is_clickable)
    return button
# endregion functions

# see `main.py` for where this is set
ICONS = None

# colors (monokai pro color scheme)
YELLOW = QColor("#ffd866")
PURPLE = QColor("#ab9df2")
BLUE = QColor("#78dce8")
WHITE = QColor("#f8f8f2")
RED = QColor("#f76c7c")
GREY = QColor("#82878b")
GREEN = QColor("#9CD57B")

# window dimensions
WINDOW_BOX = QRect(100, 100, 800, 600)

# timers
SUPER_SLOW_TIMER = QTimer()
SUPER_SLOW_TIMER.setInterval(500)

SLOW_TIMER = QTimer()
SLOW_TIMER.setInterval(2)  # 2 ms for slow timer

FAST_TIMER = QTimer()
FAST_TIMER.setInterval(1)  # 1 ms for fast timer

# base url for telemetry server
TELEMETRY_SERVER_URL = "http://54.165.159.151:8080/"

# endpoints for telemetry server, format is `TELEMETRY_SERVER_URL` + `endpoint`
TELEMETRY_SERVER_ENDPOINTS = {
    "boat_status": TELEMETRY_SERVER_URL + "boat_status/get",
    "waypoints_test": TELEMETRY_SERVER_URL + "waypoints/test",
    "get_waypoints": TELEMETRY_SERVER_URL + "waypoints/get",
    "set_waypoints": TELEMETRY_SERVER_URL + "waypoints/set",
    "get_autopilot_parameters": TELEMETRY_SERVER_URL + "autopilot_parameters/get",
    "set_autopilot_parameters": TELEMETRY_SERVER_URL + "autopilot_parameters/set",
}

# url for local waypoints server
WAYPOINTS_SERVER_URL = "http://localhost:3001/waypoints"

try:
    # should be the path to wherever `ground_station` is located
    TOP_LEVEL_DIR = PurePath(os.getcwd())

    SRC_DIR = PurePath(TOP_LEVEL_DIR / "src")
    DATA_DIR = PurePath(TOP_LEVEL_DIR / "app_data")

    WEB_ENGINE_DIR = PurePath(SRC_DIR / "web_engine")
    HTML_MAP_PATH = PurePath(WEB_ENGINE_DIR / "map.html")
    HTML_MAP = open(HTML_MAP_PATH).read()

    CAMERA_DIR = PurePath(SRC_DIR / "widgets" / "camera_widget")
    HTML_CAMERA_PATH = PurePath(CAMERA_DIR / "camera.html")
    HTML_CAMERA = open(HTML_CAMERA_PATH).read()

    if "autopilot_params" not in os.listdir(DATA_DIR):
        os.makedirs(DATA_DIR / "autopilot_params")

    __autopilot_param_editor_dir = PurePath(
        SRC_DIR / "widgets" / "autopilot_param_editor"
    )
    if "params_temp.json" not in os.listdir(__autopilot_param_editor_dir):
        with open(__autopilot_param_editor_dir / "params_temp.json", "w") as f:
            pass

    if "boat_data" not in os.listdir(DATA_DIR):
        os.makedirs(DATA_DIR / "boat_data")

    if "boat_data_bounds" not in os.listdir(DATA_DIR):
        os.makedirs(DATA_DIR / "boat_data_bounds")

    if "buoy_data" not in os.listdir(DATA_DIR):
        os.makedirs(DATA_DIR / "buoy_data")

    if "assets" not in os.listdir(DATA_DIR):
        raise Exception(
            "Assets directory not found, please redownload the directory from GitHub."
        )

    ASSETS_DIR = PurePath(DATA_DIR / "assets")
    AUTO_PILOT_PARAMS_DIR = PurePath(DATA_DIR / "autopilot_params")
    BOAT_DATA_DIR = PurePath(DATA_DIR / "boat_data")
    BOAT_DATA_LIMITS_DIR = PurePath(DATA_DIR / "boat_data_bounds")
    BUOY_DATA_DIR = PurePath(DATA_DIR / "buoy_data")

except Exception as e:
    print(f"Error: {e}")
    sys.exit(1)
