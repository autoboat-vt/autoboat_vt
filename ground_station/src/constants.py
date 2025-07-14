import os
import sys
from pathlib import PurePath
from PyQt5.QtCore import QRect, QTimer
from PyQt5.QtGui import QColor

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
# TELEMETRY_SERVER_URL = "http://54.165.159.151:8080/"
TELEMETRY_SERVER_URL = "http://3.138.35.188:5000/"


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
