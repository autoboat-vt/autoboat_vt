"""Module containing constants for the ground station application."""

import inspect
import json
import os
import time
from enum import auto
from pathlib import Path
from types import SimpleNamespace
from typing import Any, TypeAlias
from urllib.parse import urljoin

import requests
import requests.adapters
from qtpy.QtCore import QPoint, QRect, QSize, Qt
from qtpy.QtGui import QColor, QPalette
from strenum import StrEnum

from utils import misc
from utils.state_manager import StateManager


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

NumberType: TypeAlias = int | float

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

# window size and box
WINDOW_SIZE = QSize(800, 600)
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

JS_LIBRARIES: tuple[str, ...] = (
    "https://unpkg.com/leaflet@1.9.4/dist/leaflet.css",
    "https://unpkg.com/leaflet@1.9.4/dist/leaflet.js",
    "https://cdn.jsdelivr.net/gh/bbecquet/Leaflet.RotatedMarker@master/leaflet.rotatedMarker.js",
)

# server ports
ASSET_SERVER_PORT = 8000
GO_SERVER_PORT = 3001

# url for local waypoints server
_waypoints_server_url: str = f"http://localhost:{GO_SERVER_PORT}/waypoints"

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
}

try:
    # should be the path to wherever `ground_station` is located
    TOP_LEVEL_DIR = Path(os.getcwd())

    SRC_DIR = Path(TOP_LEVEL_DIR / "src")
    UTILS_DIR = Path(SRC_DIR / "utils")
    WIDGETS_DIR = Path(SRC_DIR / "widgets")

    DATA_DIR = Path(TOP_LEVEL_DIR / "app_data")

    MAP_WIDGET_DIR = Path(WIDGETS_DIR / "map_widget")
    HTML_MAP_PATH = Path(MAP_WIDGET_DIR / "map.html")

    CAMERA_WIDGET_DIR = Path(WIDGETS_DIR / "camera_widget")
    HTML_CAMERA_PATH = Path(CAMERA_WIDGET_DIR / "camera.html")
    
    APP_STATE_PATH = Path(DATA_DIR / "app_state.json")

    stack = inspect.stack()
    active_flag: bool = stack[0].filename == Path(UTILS_DIR / "constants.py").as_posix()

    # will not break if moved outside of if block, but prevents redundant checks
    if active_flag:
        if "assets" not in os.listdir(DATA_DIR):
            raise Exception("Assets directory not found, pleacse redownload the directory from GitHub.")
        
        if not APP_STATE_PATH.exists():
            print("[Info] Creating app state file...")
            APP_STATE_PATH.touch()
            with open(APP_STATE_PATH, "w") as f:
                json.dump({}, f, indent=4)

        if json.load(open(file=APP_STATE_PATH, mode="r", encoding="utf-8")) == {}:
            print("[Info] Initializing app state file...")
            with open(APP_STATE_PATH, "w", encoding="utf-8") as f:
                json.dump(STATE_FILE_CONTENTS, f, indent=4)
        
        if "autopilot_params" not in os.listdir(DATA_DIR):
            print("[Info] Creating autopilot parameters directory...")
            os.makedirs(DATA_DIR / "autopilot_params")

        if "example_params.json" not in os.listdir(DATA_DIR / "autopilot_params"):
            print("[Warning] Missing example autopilot parameters file.")

        if "boat_data" not in os.listdir(DATA_DIR):
            print("[Info] Creating boat data directory...")
            os.makedirs(DATA_DIR / "boat_data")

        if "boat_data_bounds" not in os.listdir(DATA_DIR):
            print("[Info] Creating boat data bounds directory...")
            os.makedirs(DATA_DIR / "boat_data_bounds")

        if "buoy_data" not in os.listdir(DATA_DIR):
            print("[Info] Creating buoy data directory...")
            os.makedirs(DATA_DIR / "buoy_data")

    ASSETS_DIR = Path(DATA_DIR / "assets")
    AUTOPILOT_PARAMS_DIR = Path(DATA_DIR / "autopilot_params")
    BOAT_DATA_DIR = Path(DATA_DIR / "boat_data")
    BOAT_DATA_LIMITS_DIR = Path(DATA_DIR / "boat_data_bounds")
    BUOY_DATA_DIR = Path(DATA_DIR / "buoy_data")

except Exception as e:
    raise RuntimeError(f"Initialization error: {e}") from e
