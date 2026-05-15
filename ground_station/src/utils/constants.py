"""Module containing constants for the ground station application."""

import inspect
import json
import os
import time
from enum import Enum, auto
from pathlib import Path
from types import SimpleNamespace
from typing import Any, TypeAlias
from urllib.parse import urljoin

import requests
import requests.adapters
from numpy import number as np_number
from qtpy.QtCore import QPoint, QRect, QSize, Qt, QUrl
from qtpy.QtGui import QColor, QPalette
from qtpy.QtWebEngineWidgets import QWebEnginePage
from strenum import StrEnum

from utils import misc
from utils.data_logger import DataLogger
from utils.state_manager import StateManager


class StrictMatchEnums:
    """
    These enums MUST match those in `src/autopilot/autopilot/autopilot_library/utils/constants.py` exactly.

    If you change any of these, you MUST change the corresponding one in the autopilot repository as well, and vice versa.

    Contains
    --------
    - ``SailboatAutopilotMode``
    - ``SailboatStates``
    - ``MotorboatAutopilotMode``
    """

    class SailboatAutopilotMode(Enum):
        """An enum containing the different modes that the sailboat autopilot can be in."""

        DISABLED = 0
        FULL_RC = 1
        HOLD_BEST_SAIL = 2
        HOLD_HEADING = 3
        HOLD_HEADING_AND_BEST_SAIL = 4
        WAYPOINT_MISSION = 5


    class SailboatStates(Enum):
        """An enum containing the different states that the sailboat autopilot can be in."""

        NA = 0
        DOWNWIND_SAILING = 1
        PORT_TACK = 2         # On a tack where the wind vector is to the left of the boat (port and left both have 4 letters)
        STARBOARD_TACK = 3    # On a tack where the wind vector is to the right of the boat
        CW_TACKING = 4        # Switching tacks from starboard to port tack
        CCW_TACKING = 5       # Switching tacks from port to staboard tack
        STALL = 6
        
        
    class MotorboatAutopilotMode(Enum):
        """An enum containing the different modes that the motorboat autopilot can be in."""

        DISABLED = 0
        FULL_RC = 1
        HOLD_HEADING = 2
        WAYPOINT_MISSION = 3
        


class TelemetryStatus(StrEnum):
    """
    Enum representing the status of telemetry data fetching.

    Attributes
    ----------
    - ``SUCCESS``: Indicates that telemetry data was fetched successfully.
    - ``FAILURE``: Indicates that telemetry data fetching failed.
    - ``WRONG_FORMAT``: Indicates that the fetched telemetry data was in an incorrect format.

    Inherits
    --------
    ``StrEnum``
    """

    SUCCESS = auto()
    FAILURE = auto()
    WRONG_FORMAT = auto()

NumberType: TypeAlias = int | float | complex | np_number
FileType: TypeAlias = str | os.PathLike[str]

SM = StateManager()

# see `main.py` for where this is set
ICONS: SimpleNamespace

# colors (monokai pro color scheme)
RED = QColor("#ff6188")
ORANGE = QColor("#fc9867")
YELLOW = QColor("#ffd866")
GREEN = QColor("#a9dc76")
PURPLE = QColor("#ab9df2")
BLUE = QColor("#78dce8")
WHITE = QColor("#f8f8f2")
GREY = QColor("#82878b")

WEB_LINK_COLOR = QColor("#2a82da")
BACKGROUND_COLOR = QColor("#333333")
DARK_BASE = QColor("#252525")
ACCENT_COLOR = QColor("#AAAAAA")
FONT_COLOR = QColor("#F5F5F5")

WEB_LINK_COLOR = QColor("#2a82da")
BACKGROUND_COLOR = QColor("#333333")
ACCENT_COLOR = QColor("#AAAAAA")
FONT_COLOR = QColor("#F5F5F5")
PLOT_COLORS: list[QColor] = [ORANGE, YELLOW, GREEN, BLUE, PURPLE, RED, WHITE]

# pallette and style sheet
PALLETTE = QPalette()

PALLETTE.setColor(QPalette.Window, BACKGROUND_COLOR)
PALLETTE.setColor(QPalette.WindowText, FONT_COLOR)
PALLETTE.setColor(QPalette.Base, BACKGROUND_COLOR)
PALLETTE.setColor(QPalette.AlternateBase, ACCENT_COLOR)
PALLETTE.setColor(QPalette.ToolTipBase, FONT_COLOR)
PALLETTE.setColor(QPalette.ToolTipText, FONT_COLOR)
PALLETTE.setColor(QPalette.Text, FONT_COLOR)
PALLETTE.setColor(QPalette.Button, BACKGROUND_COLOR)
PALLETTE.setColor(QPalette.ButtonText, FONT_COLOR)
PALLETTE.setColor(QPalette.BrightText, Qt.red)
PALLETTE.setColor(QPalette.Link, WEB_LINK_COLOR)
PALLETTE.setColor(QPalette.Highlight, FONT_COLOR)
PALLETTE.setColor(QPalette.HighlightedText, Qt.black)

STYLE_SHEET = """
    QToolTip {
        color: #ffffff;
        background-color: #2a82da;
        border: 1px solid white;
    }
"""

# application window title and stuff
WINDOW_TITLE = "Groundstation"
APPLICATION_NAME = "Groundstation"
ORGANIZATION_NAME = "Autoboat @ VT"

# window size and box
MAX_WINDOW_SIZE = QSize(1800, 1080)
WINDOW_SIZE = QSize(1200, 800)
WINDOW_BOX = QRect(QPoint(100, 100), WINDOW_SIZE)

# timers
THIRTY_SECOND_TIMER = misc.create_timer(30_000)
TEN_SECOND_TIMER = misc.create_timer(10_000)
FIVE_SECOND_TIMER = misc.create_timer(5_000)
ONE_SECOND_TIMER = misc.create_timer(1_000)
HALF_SECOND_TIMER = misc.create_timer(500)
TEN_MS_TIMER = misc.create_timer(10)
ONE_MS_TIMER = misc.create_timer(1)

_start_time: float = time.time()

# server ports
ASSET_SERVER_PORT = 8000
MAP_SERVER_PORT = 3002
VITE_PORT = 5173

# url for local vite server hosting the map
MAP_URL = QUrl(f"http://127.0.0.1:{VITE_PORT}")

# see `main.py` for where this is set
MAP_PAGE: QWebEnginePage

# url for local waypoints server
_waypoints_server_url: str = f"http://localhost:{MAP_SERVER_PORT}/waypoints"

TELEMETRY_TIMEOUT_SECONDS = 10
TELEMETRY_RETRY_ATTEMPTS = 3

REQ_SESSION = requests.Session()
ADAPTER = requests.adapters.HTTPAdapter(max_retries=TELEMETRY_RETRY_ATTEMPTS)
REQ_SESSION.mount("http://", ADAPTER)
REQ_SESSION.mount("https://", ADAPTER)

# base url for telemetry server (the CIA is inside of my brain...)
_telemetry_server_url: str = "https://vt-autoboat-telemetry.uk"

_local_autopilot_param_hash: str = ""
_remote_autopilot_param_hash: str = ""
_current_autopilot_parameters: dict[str, Any] = {}

TELEMETRY_SERVER_INSTANCE_ID_INITIAL_VALUE: int = -1  # -1 means no instance selected
_telemetry_server_instance_id: int = TELEMETRY_SERVER_INSTANCE_ID_INITIAL_VALUE
_has_telemetry_server_instance_changed: bool = False

# endpoints for telemetry server, format is `_telemetry_server_url` + `endpoint` + `/`
_instance_manager_endpoints: dict[str, str] = {
    "create_instance": urljoin(_telemetry_server_url, "instance_manager/create"),
    "delete_instance": urljoin(_telemetry_server_url, "instance_manager/delete/"),
    "delete_all_instances": urljoin(_telemetry_server_url, "instance_manager/delete_all"),
    "clean_instances": urljoin(_telemetry_server_url, "instance_manager/clean_instances"),
    "set_instance_user": urljoin(_telemetry_server_url, "instance_manager/set_user/"),
    "set_instance_name": urljoin(_telemetry_server_url, "instance_manager/set_name/"),
    "get_user_from_id": urljoin(_telemetry_server_url, "instance_manager/get_user/"),
    "get_instance_name_from_id": urljoin(_telemetry_server_url, "instance_manager/get_name/"),
    "get_instance_id_from_name": urljoin(_telemetry_server_url, "instance_manager/get_id/"),
    "get_instance_info": urljoin(_telemetry_server_url, "instance_manager/get_instance_info/"),
    "get_all_instance_info": urljoin(_telemetry_server_url, "instance_manager/get_all_instance_info"),
    "get_all_ids": urljoin(_telemetry_server_url, "instance_manager/get_ids"),
}

_boat_status_endpoints: dict[str, str] = {
    "get_boat_status": urljoin(_telemetry_server_url, "boat_status/get/"),
    "get_new_boat_status": urljoin(_telemetry_server_url, "boat_status/get_new/"),
    "test_boat_status": urljoin(_telemetry_server_url, "boat_status/test/"),
}

_autopilot_parameters_endpoints: dict[str, str] = {
    "get_autopilot_parameters": urljoin(_telemetry_server_url, "autopilot_parameters/get/"),
    "get_new_autopilot_parameters": urljoin(_telemetry_server_url, "autopilot_parameters/get_new/"),
    "get_default_autopilot_parameters": urljoin(_telemetry_server_url, "autopilot_parameters/get_default/"),
    "get_current_hash": urljoin(_telemetry_server_url, "autopilot_parameters/get_hash/"),
    "get_config_from_hash": urljoin(_telemetry_server_url, "autopilot_parameters/get_config/"),
    "get_hash_description": urljoin(_telemetry_server_url, "autopilot_parameters/get_hash_description/"),
    "get_all_hashes": urljoin(_telemetry_server_url, "autopilot_parameters/get_all_hashes"),
    "get_hash_exists": urljoin(_telemetry_server_url, "autopilot_parameters/get_hash_exists/"),
    "set_autopilot_parameters": urljoin(_telemetry_server_url, "autopilot_parameters/set/"),
    "set_default_autopilot_parameters": urljoin(_telemetry_server_url, "autopilot_parameters/set_default/"),
    "set_default_from_hash": urljoin(_telemetry_server_url, "autopilot_parameters/set_default_from_hash/"),
    "set_hash_description": urljoin(_telemetry_server_url, "autopilot_parameters/set_hash_description/"),
    "create_config": urljoin(_telemetry_server_url, "autopilot_parameters/create_config"),
    "delete_config": urljoin(_telemetry_server_url, "autopilot_parameters/delete_config/"),
    "test_autopilot_parameters": urljoin(_telemetry_server_url, "autopilot_parameters/test/"),
}

_waypoints_endpoints: dict[str, str] = {
    "get_waypoints": urljoin(_telemetry_server_url, "waypoints/get/"),
    "get_new_waypoints": urljoin(_telemetry_server_url, "waypoints/get_new/"),
    "set_waypoints": urljoin(_telemetry_server_url, "waypoints/set/"),
    "test_waypoints": urljoin(_telemetry_server_url, "waypoints/test/"),
}

_camera_endpoints: dict[str, str] = {
    "get_current_camera_image": urljoin(_telemetry_server_url, "camera/get_current_image/"),
}

_telemetry_server_endpoints: dict[str, str] = dict(
    **_instance_manager_endpoints,
    **_boat_status_endpoints,
    **_autopilot_parameters_endpoints,
    **_waypoints_endpoints,
    **_camera_endpoints,
)

_map_features: dict[str, dict[str, str | bool]] = {
    "waypoints_popup": {
        "name": "Waypoints Popup",
        "description": "Show a popup when the waypoints on the telemetry server change.",
        "feedback_text": "Updated Waypoints Popup Config.",
        "status": False
    },
    "sailboat_debug_symbols": {
        "name": "Sailboat Debugging Symbols",
        "description": "Show sailboat debugging symbols on the map.",
        "feedback_text": (
            "Updated Debugging Symbols Config.\n"
            "orange, wind\n"
            "black, velocity\n"
            "red, no-go zone\n"
            "pink, decision zone 2"
        ),
        "status": False
    }
}

_data_logging_active: bool = False
_initial_log_file_path: str = ""

STATE_FILE_CONTENTS: dict[str, Any] = {
    "start_time": _start_time,
    "telemetry_server_url": _telemetry_server_url,
    "waypoints_server_url": _waypoints_server_url,
    "local_autopilot_param_hash": _local_autopilot_param_hash,
    "remote_autopilot_param_hash": _remote_autopilot_param_hash,
    "current_autopilot_parameters": _current_autopilot_parameters,
    "telemetry_server_instance_id": _telemetry_server_instance_id,
    "telemetry_server_instance_user": "",
    "has_telemetry_server_instance_changed": _has_telemetry_server_instance_changed,
    "telemetry_server_endpoints": _telemetry_server_endpoints,
    "map_features": _map_features,
    "data_logging_active": _data_logging_active,
    "data_log_file_path": _initial_log_file_path,
}

try:
    # should be the path to wherever `ground_station` is located
    TOP_LEVEL_DIR = Path(os.getcwd())

    SRC_DIR = Path(TOP_LEVEL_DIR / "src")
    UTILS_DIR = Path(SRC_DIR / "utils")
    WIDGETS_DIR = Path(SRC_DIR / "widgets")

    DATA_DIR = Path(TOP_LEVEL_DIR / "app_data")
    GIT_KEEP_DIR = Path(DATA_DIR / "git_keep")
    GIT_IGNORE_DIR = Path(DATA_DIR / "git_ignore")
    os.makedirs(GIT_IGNORE_DIR, exist_ok=True)

    DEFAULTS_EXAMPLES_DIR = Path(GIT_KEEP_DIR / "defaults_examples")
    ASSETS_DIR = Path(GIT_KEEP_DIR / "assets")
    APP_LOGO_PATH = Path(ASSETS_DIR / "logo.png")

    CAMERA_WIDGET_DIR = Path(WIDGETS_DIR / "camera_widget")
    HTML_CAMERA_PATH = Path(CAMERA_WIDGET_DIR / "camera.html")
    
    APP_STATE_PATH = Path(GIT_IGNORE_DIR / "app_state.json")

    stack = inspect.stack()
    active_flag: bool = stack[0].filename == Path(UTILS_DIR / "constants.py").as_posix()

    # will not break if moved outside of if block, but prevents redundant checks
    if active_flag:
        if "assets" not in os.listdir(GIT_KEEP_DIR):
            raise Exception("Assets directory not found, please redownload the directory from GitHub.")
        
        if "defaults_examples" not in os.listdir(GIT_KEEP_DIR):
            raise Exception("Defaults/examples directory not found, please redownload the directory from GitHub.")
        
        if "autopilot_params" not in os.listdir(GIT_IGNORE_DIR):
            print("[Info] Creating autopilot parameters directory...")
            os.makedirs(GIT_IGNORE_DIR / "autopilot_params")

        if "buoy_data" not in os.listdir(GIT_IGNORE_DIR):
            print("[Info] Creating buoy data directory...")
            os.makedirs(GIT_IGNORE_DIR / "buoy_data")

        if "data_logs" not in os.listdir(GIT_IGNORE_DIR):
            print("[Info] Creating data logs directory...")
            os.makedirs(GIT_IGNORE_DIR / "data_logs")

        if not APP_STATE_PATH.exists():
            print("[Info] Creating app state file...")
            APP_STATE_PATH.touch()
            with open(APP_STATE_PATH, "w") as f:
                json.dump({}, f, indent=4)

        if json.load(open(file=APP_STATE_PATH, mode="r", encoding="utf-8")) == {}:
            print("[Info] Initializing app state file...")
            with open(APP_STATE_PATH, "w", encoding="utf-8") as f:
                json.dump(STATE_FILE_CONTENTS, f, indent=4)

        else:
            raise RuntimeError(
                f"Stale app state file found at {APP_STATE_PATH}, please delete this file "
                "and restart the application."
            )
        
    DATA_LOGS_DIR = Path(GIT_IGNORE_DIR / "data_logs")
    os.makedirs(DATA_LOGS_DIR, exist_ok=True)

    DL = DataLogger()

    AUTOPILOT_PARAMS_DIR = Path(GIT_IGNORE_DIR / "autopilot_params")
    misc.create_symlinks(DEFAULTS_EXAMPLES_DIR / "autopilot_params", AUTOPILOT_PARAMS_DIR)

    BUOY_DATA_DIR = Path(GIT_IGNORE_DIR / "buoy_data")
    misc.create_symlinks(DEFAULTS_EXAMPLES_DIR / "buoy_data", BUOY_DATA_DIR)

except Exception as e:
    raise RuntimeError(f"Initialization error: {e}") from e
