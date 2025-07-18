"""
Module containing constants and utility functions for the ground station application.

Functions:
- pushbutton_maker: Creates a styled QPushButton.
- show_message_box: Displays a message box with customizable options.
- show_input_dialog: Displays an input dialog for user input.

Constants:
- ICONS: A namespace containing application icons.
- YELLOW, PURPLE, BLUE, WHITE, RED, GREY, GREEN: Color constants for the application.
- PALLETTE: A QPalette object for the application's color scheme.
- STYLE_SHEET: A string containing the application's style sheet.
- WINDOW_BOX: QRect defining the main window dimensions.
- TEN_SECOND_TIMER, SUPER_SLOW_TIMER, SLOW_TIMER, FAST_TIMER: QTimer objects for various intervals.
- TELEMETRY_SERVER_URL: Base URL for the telemetry server.
- TELEMETRY_SERVER_ENDPOINTS: Dictionary of endpoints for the telemetry server.
- WAYPOINTS_SERVER_URL: URL for the local waypoints server.
- TOP_LEVEL_DIR, SRC_DIR, DATA_DIR: Paths to the main directories of the application.
- HTML_MAP_PATH, HTML_MAP: Path and content of the HTML file used by the map widget in the ground station.
- HTML_CAMERA_PATH, HTML_CAMERA: Path and content of the HTML file used by the camera widget.
- ASSETS_DIR, AUTO_PILOT_PARAMS_DIR, BOAT_DATA_DIR, BOAT_DATA_LIMITS_DIR, BUOY_DATA_DIR: Paths to various data directories.
"""

import os
import sys
import shutil
from pathlib import PurePath
from qtpy.QtCore import QRect, QTimer, Qt
from qtpy.QtGui import QColor, QIcon, QPalette
from qtpy.QtWidgets import QPushButton, QMessageBox, QInputDialog, QCheckBox
import qtawesome as qta
from types import SimpleNamespace
from typing import Optional, Callable, Union
from typing import TypeVar


T = TypeVar("T")


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
        "notification": qta.icon("mdi.bell"),
        "warning": qta.icon("mdi.alert"),
        "question": qta.icon("mdi.help-circle"),
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


def show_message_box(
    title: str,
    message: str,
    icon: Optional[QIcon] = None,
    buttons: Optional[list[QMessageBox.StandardButton]] = None,
    remember_choice_option: Optional[bool] = False,
) -> Union[QMessageBox.StandardButton, tuple[QMessageBox.StandardButton, bool]]:
    """
    Show a message box with the specified title, message, and optional icon and buttons.
    Returns the button that was clicked by the user. If the user closes the message box
    without clicking a button, it returns `QMessageBox.StandardButton.NoButton`.

    Parameters
    ----------
    title
        The title of the message box.
    message
        The message to display in the message box.
    icon
        An optional icon to display in the message box.
    buttons
        A list of standard buttons to show. Defaults to `[QMessageBox.Ok]`. <br>
        Example: `[QMessageBox.Yes, QMessageBox.No]`
    remember_choice_option
        If `True`, adds a "Remember my choice" checkbox to the message box.

    Returns
    -------
    QMessageBox.StandardButton
        The button that was clicked by the user.
    bool
        If `remember_choice_option` is `True`, returns whether the user checked the "Remember my choice" checkbox.
    """

    if buttons is None:
        buttons = [QMessageBox.Ok]

    msg_box = QMessageBox()
    msg_box.setWindowTitle(title)
    msg_box.setText(message)

    if icon:
        msg_box.setIconPixmap(icon.pixmap(64, 64))

    for button in buttons:
        msg_box.addButton(button)

    if remember_choice_option:
        remember_checkbox = QCheckBox("Remember my decision")
        msg_box.setCheckBox(remember_checkbox)
        clicked_button = msg_box.exec_()
        return QMessageBox.StandardButton(clicked_button), remember_checkbox.isChecked()

    else:
        clicked_button = msg_box.exec_()
        return QMessageBox.StandardButton(clicked_button)


def show_input_dialog(
    title: str,
    label: str,
    default_value: Optional[str] = None,
    input_type: T = str,
) -> Union[T, None]:
    """
    Show an input dialog to get user input.

    Parameters
    ----------
    title
        The title of the input dialog.
    label
        The label for the input field.
    default_value
        The default value to show in the input field.
    input_type
        The type to convert the input to. Defaults to `str`. <br>
        Example: `int`, `float`, etc.

    Returns
    -------
    Union[T, None]
        The user input converted to the specified type, or `None` if the dialog was cancelled.
    """

    text, ok = QInputDialog.getText(None, title, label, text=default_value)
    if ok:
        return input_type(text)
    else:
        return None


# endregion functions

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
TEN_SECOND_TIMER = QTimer()
TEN_SECOND_TIMER.setInterval(10000)

SUPER_SLOW_TIMER = QTimer()
SUPER_SLOW_TIMER.setInterval(500)

SLOW_TIMER = QTimer()
SLOW_TIMER.setInterval(2)  # 2 ms for slow timer

FAST_TIMER = QTimer()
FAST_TIMER.setInterval(1)  # 1 ms for fast timer

# base url for telemetry server
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

    if "params_default.jsonc" not in os.listdir(DATA_DIR / "autopilot_params"):
        raise Exception(
            "Default autopilot parameters file not found, please redownload the directory from GitHub."
        )

    if "autopilot_params" not in os.listdir(DATA_DIR):
        os.makedirs(DATA_DIR / "autopilot_params")

    _autopilot_param_editor_dir = PurePath(
        SRC_DIR / "widgets" / "autopilot_param_editor"
    )
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
        raise Exception(
            "Assets directory not found, please redownload the directory from GitHub."
        )

    ASSETS_DIR = PurePath(DATA_DIR / "assets")
    AUTO_PILOT_PARAMS_DIR = PurePath(DATA_DIR / "autopilot_params")
    BOAT_DATA_DIR = PurePath(DATA_DIR / "boat_data")
    BOAT_DATA_LIMITS_DIR = PurePath(DATA_DIR / "boat_data_bounds")
    BUOY_DATA_DIR = PurePath(DATA_DIR / "buoy_data")

except Exception as e:
    print(f"[Error] {e}")
    sys.exit(1)
