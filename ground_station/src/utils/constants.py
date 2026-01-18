"""Module containing constants for the ground station application."""

import os
import shutil
import time
from enum import auto
from pathlib import Path
from types import SimpleNamespace
from urllib.parse import urljoin

import requests
import requests.adapters
from qtpy.QtCore import QPoint, QRect, QSize, Qt
from qtpy.QtGui import QColor, QPalette
from strenum import StrEnum

from utils import misc


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

START_TIME: float = time.time()

# server ports
ASSET_SERVER_PORT = 8000
CDN_SERVER_PORT = 8081
GO_SERVER_PORT = 3001

JS_LIBRARIES: tuple[str, ...] = (
    "https://unpkg.com/leaflet@1.9.4/dist/leaflet.css",
    "https://unpkg.com/leaflet@1.9.4/dist/leaflet.js",
    "https://cdn.jsdelivr.net/gh/bbecquet/Leaflet.RotatedMarker@master/leaflet.rotatedMarker.js",
)

# url for local waypoints server
WAYPOINTS_SERVER_URL: str = f"http://localhost:{GO_SERVER_PORT}/waypoints"

# base url for telemetry server (the CIA is inside of my brain...)
TELEMETRY_SERVER_URL: str = "https://vt-autoboat-telemetry.uk"

TELEMETRY_SERVER_INSTANCE_ID_INITIAL_VALUE: int = -1  # -1 means no instance selected
TELEMETRY_SERVER_INSTANCE_ID: int = TELEMETRY_SERVER_INSTANCE_ID_INITIAL_VALUE
HAS_TELEMETRY_SERVER_INSTANCE_CHANGED: bool = False

# endpoints for telemetry server, format is `TELEMETRY_SERVER_URL` + `endpoint` + `/`
_instance_manager_endpoints: dict[str, str] = {
    "create_instance": urljoin(TELEMETRY_SERVER_URL, "instance_manager/create"),
    "delete_instance": urljoin(TELEMETRY_SERVER_URL, "instance_manager/delete/"),
    "delete_all_instances": urljoin(TELEMETRY_SERVER_URL, "instance_manager/delete_all"),
    "clean_instances": urljoin(TELEMETRY_SERVER_URL, "instance_manager/clean_instances"),
    "set_instance_user": urljoin(TELEMETRY_SERVER_URL, "instance_manager/set_user/"),
    "set_instance_name": urljoin(TELEMETRY_SERVER_URL, "instance_manager/set_name/"),
    "get_user_from_id": urljoin(TELEMETRY_SERVER_URL, "instance_manager/get_user/"),
    "get_instance_name_from_id": urljoin(TELEMETRY_SERVER_URL, "instance_manager/get_name/"),
    "get_instance_id_from_name": urljoin(TELEMETRY_SERVER_URL, "instance_manager/get_id/"),
    "get_instance_info": urljoin(TELEMETRY_SERVER_URL, "instance_manager/get_instance_info/"),
    "get_all_instance_info": urljoin(TELEMETRY_SERVER_URL, "instance_manager/get_all_instance_info"),
    "get_all_ids": urljoin(TELEMETRY_SERVER_URL, "instance_manager/get_ids"),
}

_boat_status_endpoints: dict[str, str] = {
    "get_boat_status": urljoin(TELEMETRY_SERVER_URL, "boat_status/get/"),
    "get_new_boat_status": urljoin(TELEMETRY_SERVER_URL, "boat_status/get_new/"),
    "test_boat_status": urljoin(TELEMETRY_SERVER_URL, "boat_status/test/"),
}

_autopilot_parameters_endpoints: dict[str, str] = {
    "get_autopilot_parameters": urljoin(TELEMETRY_SERVER_URL, "autopilot_parameters/get/"),
    "get_new_autopilot_parameters": urljoin(TELEMETRY_SERVER_URL, "autopilot_parameters/get_new/"),
    "get_default_autopilot_parameters": urljoin(TELEMETRY_SERVER_URL, "autopilot_parameters/get_default/"),
    "set_autopilot_parameters": urljoin(TELEMETRY_SERVER_URL, "autopilot_parameters/set/"),
    "set_default_autopilot_parameters": urljoin(TELEMETRY_SERVER_URL, "autopilot_parameters/set_default/"),
    "test_autopilot_parameters": urljoin(TELEMETRY_SERVER_URL, "autopilot_parameters/test/"),
}

_waypoints_endpoints: dict[str, str] = {
    "get_waypoints": urljoin(TELEMETRY_SERVER_URL, "waypoints/get/"),
    "get_new_waypoints": urljoin(TELEMETRY_SERVER_URL, "waypoints/get_new/"),
    "set_waypoints": urljoin(TELEMETRY_SERVER_URL, "waypoints/set/"),
    "test_waypoints": urljoin(TELEMETRY_SERVER_URL, "waypoints/test/"),
}

TELEMETRY_SERVER_ENDPOINTS: dict[str, str] = dict(
    **_instance_manager_endpoints,
    **_boat_status_endpoints,
    **_autopilot_parameters_endpoints,
    **_waypoints_endpoints,
)

TELEMETRY_TIMEOUT_SECONDS = 10
TELEMETRY_RETRY_ATTEMPTS = 3

REQ_SESSION = requests.Session()
ADAPTER = requests.adapters.HTTPAdapter(max_retries=TELEMETRY_RETRY_ATTEMPTS)
REQ_SESSION.mount("http://", ADAPTER)
REQ_SESSION.mount("https://", ADAPTER)

try:
    # should be the path to wherever `ground_station` is located
    TOP_LEVEL_DIR = Path(os.getcwd())

    SRC_DIR = Path(TOP_LEVEL_DIR / "src")
    DATA_DIR = Path(TOP_LEVEL_DIR / "app_data")

    MAP_DIR = Path(SRC_DIR / "widgets" / "map_widget")
    HTML_MAP_PATH = Path(MAP_DIR / "map.html")
    HTML_MAP = open(HTML_MAP_PATH, encoding="utf-8").read()

    CAMERA_DIR = Path(SRC_DIR / "widgets" / "camera_widget")
    HTML_CAMERA_PATH = Path(CAMERA_DIR / "camera.html")
    HTML_CAMERA = open(HTML_CAMERA_PATH, encoding="utf-8").read()

    if __name__ == "__main__":
        if "params_default.jsonc" not in os.listdir(DATA_DIR / "autopilot_params"):
            raise Exception("Default autopilot parameters file not found, please redownload the directory from GitHub.")

        if "autopilot_params" not in os.listdir(DATA_DIR):
            os.makedirs(DATA_DIR / "autopilot_params")

        _autopilot_param_editor_dir = Path(SRC_DIR / "widgets" / "autopilot_param_editor")
        if "params_temp.json" not in os.listdir(_autopilot_param_editor_dir):
            shutil.copyfile(
                Path(DATA_DIR / "autopilot_params" / "params_default.jsonc"),
                Path(_autopilot_param_editor_dir / "params_temp.json"),
            )

            with open(Path(_autopilot_param_editor_dir / "params_temp.json"), mode="r", encoding="utf-8") as f:
                lines = f.readlines()

            # remove comments and empty lines
            with open(Path(_autopilot_param_editor_dir / "params_temp.json"), mode="w", encoding="utf-8") as f:
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

    ASSETS_DIR = Path(DATA_DIR / "assets")
    CDN_DIR = Path(DATA_DIR / "js_libraries_cache")
    AUTO_PILOT_PARAMS_DIR = Path(DATA_DIR / "autopilot_params")
    BOAT_DATA_DIR = Path(DATA_DIR / "boat_data")
    BOAT_DATA_LIMITS_DIR = Path(DATA_DIR / "boat_data_bounds")
    BUOY_DATA_DIR = Path(DATA_DIR / "buoy_data")

except Exception as e:
    raise RuntimeError(f"Initialization error: {e}") from e
