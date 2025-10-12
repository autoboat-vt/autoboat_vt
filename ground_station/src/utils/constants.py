"""
Module containing constants for the ground station application.

Constants:
- TelemetryStatus: Enum representing the status of telemetry data fetching.
- ICONS: A namespace containing application icons.
- YELLOW, PURPLE, BLUE, WHITE, RED, GREY, GREEN: Color constants for the application.
- PALLETTE: A QPalette object for the application's color scheme.
- STYLE_SHEET: A string containing the application's style sheet.
- WINDOW_BOX: QRect defining the main window dimensions.
- TEN_SECOND_TIMER, HALF_SECOND_TIMER, TEN_MS_TIMER, ONE_MS_TIMER: QTimer objects for various intervals.
- TELEMETRY_SERVER_URL: Base URL for the telemetry server.
- TELEMETRY_SERVER_ENDPOINTS: Dictionary of endpoints for the telemetry server.
- WAYPOINTS_SERVER_URL: URL for the local waypoints server.
- TOP_LEVEL_DIR, SRC_DIR, DATA_DIR: Paths to the main directories of the application.
- HTML_MAP_PATH, HTML_MAP: Path and content of the HTML file used by the map widget in the ground station.
- HTML_CAMERA_PATH, HTML_CAMERA: Path and content of the HTML file used by the camera widget.
- ASSETS_DIR, AUTO_PILOT_PARAMS_DIR, BOAT_DATA_DIR, BOAT_DATA_LIMITS_DIR, BUOY_DATA_DIR: Paths to various data directories.
"""

import os
import shutil
import requests
from utils import misc
from requests.adapters import HTTPAdapter
from urllib.parse import urljoin
from pathlib import PurePath
from qtpy.QtCore import Qt, QRect, QTimer
from qtpy.QtGui import QColor, QPalette
from types import SimpleNamespace
from enum import auto
from strenum import StrEnum


class TelemetryStatus(StrEnum):
    """
    Enum representing the status of telemetry data fetching.

    Attributes
    ----------
    `SUCCESS`: Indicates that telemetry data was fetched successfully. <br>
    `FAILURE`: Indicates that telemetry data fetching failed.

    Inherits
    --------
    `StrEnum`
    """

    SUCCESS: str = auto()
    FAILURE: str = auto()


# see `main.py` for where this is set
ICONS: SimpleNamespace

# colors (monokai pro color scheme)
YELLOW = QColor("#ffd866")
PURPLE = QColor("#ab9df2")
BLUE = QColor("#78dce8")
WHITE = QColor("#f8f8f2")
RED = QColor("#f76c7c")
GREY = QColor("#82878b")
GREEN = QColor("#9CD57B")

# pallette and style sheet
PALLETTE = QPalette()

PALLETTE.setColor(QPalette.Window, QColor(53, 53, 53))
PALLETTE.setColor(QPalette.WindowText, Qt.white)
PALLETTE.setColor(QPalette.Base, QColor(25, 25, 25))
PALLETTE.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
PALLETTE.setColor(QPalette.ToolTipBase, Qt.white)
PALLETTE.setColor(QPalette.ToolTipText, Qt.white)
PALLETTE.setColor(QPalette.Text, Qt.white)
PALLETTE.setColor(QPalette.Button, QColor(53, 53, 53))
PALLETTE.setColor(QPalette.ButtonText, Qt.white)
PALLETTE.setColor(QPalette.BrightText, Qt.red)
PALLETTE.setColor(QPalette.Link, QColor(42, 130, 218))
PALLETTE.setColor(QPalette.Highlight, Qt.white)
PALLETTE.setColor(QPalette.HighlightedText, Qt.black)

STYLE_SHEET = """
    QToolTip {
        color: #ffffff;
        background-color: #2a82da;
        border: 1px solid white;
    }
"""

# window dimensions
WINDOW_BOX = QRect(100, 100, 800, 600)

# timers
THIRTY_SECOND_TIMER = misc.create_timer(30_000)

TEN_SECOND_TIMER = misc.create_timer(10_000)

FIVE_SECOND_TIMER = misc.create_timer(5_000)

ONE_SECOND_TIMER = misc.create_timer(1_000)

HALF_SECOND_TIMER = misc.create_timer(500)

TEN_MS_TIMER = misc.create_timer(10)

ONE_MS_TIMER = misc.create_timer(1)


# url for local waypoints server
WAYPOINTS_SERVER_URL = "http://localhost:3001/waypoints"

# base url for telemetry server (the CIA is inside of my brain...)
TELEMETRY_SERVER_URL = "https://vt-autoboat-telemetry.uk:8443"

TELEMETRY_SERVER_INSTANCE_ID: int = -1  # -1 means no instance selected
HAS_TELEMETRY_SERVER_INSTANCE_CHANGED: bool = False

# endpoints for telemetry server, format is `TELEMETRY_SERVER_URL` + `endpoint` + `/`
_instance_manager_endpoints = {
    "create_instance": urljoin(TELEMETRY_SERVER_URL, "instance_manager/create"),
    "delete_instance": urljoin(TELEMETRY_SERVER_URL, "instance_manager/delete/"),
    "delete_all_instances": urljoin(TELEMETRY_SERVER_URL, "instance_manager/delete_all"),
    "set_instance_name": urljoin(TELEMETRY_SERVER_URL, "instance_manager/set_name/"),
    "get_instance_name_from_id": urljoin(TELEMETRY_SERVER_URL, "instance_manager/get_name/"),
    "get_instance_id_from_name": urljoin(TELEMETRY_SERVER_URL, "instance_manager/get_id/"),
    "get_instance_info": urljoin(TELEMETRY_SERVER_URL, "instance_manager/get_instance_info/"),
    "get_all_instance_info": urljoin(TELEMETRY_SERVER_URL, "instance_manager/get_all_instance_info"),
    "get_all_ids": urljoin(TELEMETRY_SERVER_URL, "instance_manager/get_ids"),
}

_boat_status_endpoints = {
    "get_boat_status": urljoin(TELEMETRY_SERVER_URL, "boat_status/get/"),
    "get_new_boat_status": urljoin(TELEMETRY_SERVER_URL, "boat_status/get_new/"),
    "test_boat_status": urljoin(TELEMETRY_SERVER_URL, "boat_status/test/"),
}

_autopilot_parameters_endpoints = {
    "get_autopilot_parameters": urljoin(TELEMETRY_SERVER_URL, "autopilot_parameters/get/"),
    "get_new_autopilot_parameters": urljoin(TELEMETRY_SERVER_URL, "autopilot_parameters/get_new/"),
    "get_default_autopilot_parameters": urljoin(TELEMETRY_SERVER_URL, "autopilot_parameters/get_default/"),
    "set_autopilot_parameters": urljoin(TELEMETRY_SERVER_URL, "autopilot_parameters/set/"),
    "set_default_autopilot_parameters": urljoin(TELEMETRY_SERVER_URL, "autopilot_parameters/set_default/"),
    "test_autopilot_parameters": urljoin(TELEMETRY_SERVER_URL, "autopilot_parameters/test/"),
}

_waypoints_endpoints = {
    "get_waypoints": urljoin(TELEMETRY_SERVER_URL, "waypoints/get/"),
    "get_new_waypoints": urljoin(TELEMETRY_SERVER_URL, "waypoints/get_new/"),
    "set_waypoints": urljoin(TELEMETRY_SERVER_URL, "waypoints/set/"),
    "test_waypoints": urljoin(TELEMETRY_SERVER_URL, "waypoints/test/"),
}

TELEMETRY_SERVER_ENDPOINTS = dict(
    **_instance_manager_endpoints,
    **_boat_status_endpoints,
    **_autopilot_parameters_endpoints,
    **_waypoints_endpoints,
)

TELEMETRY_TIMEOUT_SECONDS = 30
TELEMETRY_RETRY_ATTEMPTS = 3

REQ_SESSION = requests.Session()
REQ_SESSION.mount(TELEMETRY_SERVER_URL, HTTPAdapter(max_retries=TELEMETRY_RETRY_ATTEMPTS))

try:
    # should be the path to wherever `ground_station` is located
    TOP_LEVEL_DIR = PurePath(os.getcwd())

    SRC_DIR = PurePath(TOP_LEVEL_DIR / "src")
    DATA_DIR = PurePath(TOP_LEVEL_DIR / "app_data")

    MAP_DIR = PurePath(SRC_DIR / "widgets" / "map_widget")
    HTML_MAP_PATH = PurePath(MAP_DIR / "map.html")
    HTML_MAP = open(HTML_MAP_PATH).read()

    CAMERA_DIR = PurePath(SRC_DIR / "widgets" / "camera_widget")
    HTML_CAMERA_PATH = PurePath(CAMERA_DIR / "camera.html")
    HTML_CAMERA = open(HTML_CAMERA_PATH).read()

    if __name__ == "__main__":
        if "params_default.jsonc" not in os.listdir(DATA_DIR / "autopilot_params"):
            raise Exception("Default autopilot parameters file not found, please redownload the directory from GitHub.")

        if "autopilot_params" not in os.listdir(DATA_DIR):
            os.makedirs(DATA_DIR / "autopilot_params")

        _autopilot_param_editor_dir = PurePath(SRC_DIR / "widgets" / "autopilot_param_editor")
        if "params_temp.json" not in os.listdir(_autopilot_param_editor_dir):
            shutil.copyfile(
                PurePath(DATA_DIR / "autopilot_params" / "params_default.jsonc"),
                PurePath(_autopilot_param_editor_dir / "params_temp.json"),
            )

            with open(PurePath(_autopilot_param_editor_dir / "params_temp.json"), "r") as f:
                lines = f.readlines()

            # remove comments and empty lines
            with open(PurePath(_autopilot_param_editor_dir / "params_temp.json"), "w") as f:
                for line in lines:
                    if not line.strip().startswith("//"):
                        f.write(line)

        if "boat_data" not in os.listdir(DATA_DIR):
            os.makedirs(DATA_DIR / "boat_data")

        if "boat_data_bounds" not in os.listdir(DATA_DIR):
            os.makedirs(DATA_DIR / "boat_data_bounds")

        if "buoy_data" not in os.listdir(DATA_DIR):
            os.makedirs(DATA_DIR / "buoy_data")

        if "assets" not in os.listdir(DATA_DIR):
            raise Exception("Assets directory not found, please redownload the directory from GitHub.")

    ASSETS_DIR = PurePath(DATA_DIR / "assets")
    AUTO_PILOT_PARAMS_DIR = PurePath(DATA_DIR / "autopilot_params")
    BOAT_DATA_DIR = PurePath(DATA_DIR / "boat_data")
    BOAT_DATA_LIMITS_DIR = PurePath(DATA_DIR / "boat_data_bounds")
    BUOY_DATA_DIR = PurePath(DATA_DIR / "buoy_data")

except Exception as e:
    raise RuntimeError(f"Initialization error: {e}") from e
