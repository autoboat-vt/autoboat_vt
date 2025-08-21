# region imports
import os
import time
import requests
import json

import constants
import thread_classes
from syntax_highlighters.json import JsonHighlighter
from widgets.popup_edit import TextEditWindow

from pathlib import PurePath
from functools import partial
from urllib.parse import urljoin
from typing import Literal, Any

from qtpy.QtCore import Qt
from qtpy.QtWebEngineWidgets import QWebEngineView
from qtpy.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QLabel,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
    QFileDialog,
    QMessageBox,
)
# endregion imports


class GroundStationWidget(QWidget):
    """
    Main widget for the ground station application.

    Inherits
    -------
    `QWidget`
    """

    def __init__(self) -> None:
        super().__init__()
        self.waypoints: list[list[float]] = []
        self.num_waypoints: int = 0

        # buoy_name => {"lat": float, "lon": float}
        self.buoys: dict[str, dict[str, float]] = {}

        self.all_boat_data: dict[str, dict[str, dict]] = {}
        self.boat_data: dict[str, Any] = {}
        self.telemetry_data_limits: dict[str, float] = {}

        # should we remember the status of the user's last response to the
        # dialog that asks if the telemetry server URL should be changed?
        self.remember_telemetry_server_url_status: bool = False

        # should we remember the status of the user's last response to the
        # dialog that asks if the user wants to pull waypoints from the telemetry server?
        self.remember_waypoints_pull_service_status: bool = False

        self.instance_id: int = 0
        self.instance_name: str = ""

        # region timers
        self.ten_ms_timer = constants.copy_qtimer(constants.TEN_MS_TIMER)
        self.one_ms_timer = constants.copy_qtimer(constants.ONE_MS_TIMER)
        self.thirty_second_timer = constants.copy_qtimer(constants.THIRTY_SECOND_TIMER)
        self.timers = [self.thirty_second_timer, self.ten_ms_timer, self.one_ms_timer]

        # region define layouts
        self.main_layout = QGridLayout()
        self.main_layout.setObjectName("main_layout")

        self.left_layout = QVBoxLayout()
        self.left_layout.setObjectName("left_layout")
        self.left_widget = QWidget()

        self.middle_layout = QGridLayout()
        self.middle_layout.setObjectName("middle_layout")

        self.right_layout = QTabWidget()
        self.right_layout.setObjectName("right_layout")
        self.right_tab1_layout = QGridLayout()
        self.right_tab2_layout = QGridLayout()
        self.right_tab1 = QWidget()
        self.right_tab2 = QWidget()
        # endregion define layouts

        # region setup UI
        # region left section
        self.left_width = 300
        self.left_label = QLabel("Telemetry Data")
        self.left_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.left_text_section = QTextEdit()
        self.left_text_section.setReadOnly(True)
        self.left_text_section.setText("Awaiting telemetry data...")

        self.save_boat_data_button = constants.pushbutton_maker(
            "Save Boat Data to File",
            constants.ICONS.save,
            self.save_boat_data,
            self.left_width,
            50,
        )

        self.edit_boat_data_limits_button = constants.pushbutton_maker(
            "Edit Limits",
            constants.ICONS.cog,
            self.edit_boat_data_limits,
            self.left_width // 2,
            50,
        )

        self.side_buttons_layout = QVBoxLayout()

        self.load_boat_data_limits_button = constants.pushbutton_maker(
            "Load Limits from File",
            constants.ICONS.hard_drive,
            self.load_boat_data_limits,
            self.left_width // 2,
            25,
        )

        self.save_boat_data_limits_button = constants.pushbutton_maker(
            "Save Limits to File",
            constants.ICONS.save,
            self.save_boat_data_limits,
            self.left_width // 2,
            25,
        )

        self.side_buttons_layout.addWidget(self.load_boat_data_limits_button)
        self.side_buttons_layout.addWidget(self.save_boat_data_limits_button)

        self.left_button_groupbox = QGroupBox()
        self.left_button_layout = QGridLayout()

        self.left_button_layout.addWidget(self.save_boat_data_button, 0, 0, 1, 2)
        self.left_button_layout.addWidget(self.edit_boat_data_limits_button, 1, 0)
        self.left_button_layout.addLayout(self.side_buttons_layout, 1, 1)
        self.left_button_groupbox.setLayout(self.left_button_layout)

        self.left_layout.addWidget(self.left_label)
        self.left_layout.addWidget(self.left_text_section)
        self.left_layout.addWidget(self.left_button_groupbox)

        self.left_widget.setLayout(self.left_layout)
        self.left_widget.setMaximumWidth(self.left_width)
        # self.left_layout.setContentsMargins(0, 0, 0, self.left_width)
        self.main_layout.addWidget(self.left_widget, 0, 0)

        # endregion left section

        # region middle section
        self.browser = QWebEngineView()
        self.browser.setHtml(constants.HTML_MAP)
        self.browser.setMinimumWidth(700)
        self.browser.setMinimumHeight(700)
        self.middle_layout.addWidget(self.browser, 0, 1)
        self.main_layout.addLayout(self.middle_layout, 0, 1)
        # endregion middle section

        # region right section
        self.right_width = 320

        # region tab1: waypoint data
        self.right_tab1_label = QLabel("Waypoints")
        self.right_tab1_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.right_tab1_table = QTableWidget()
        self.right_tab1_table.setMinimumWidth(self.right_width)
        self.right_tab1_table.cellClicked.connect(partial(self.zoom_to_marker, table="waypoints"))
        self.can_send_waypoints = True
        self.send_waypoints_button = constants.pushbutton_maker(
            "Send Waypoints",
            constants.ICONS.upload,
            self.send_waypoints,
            self.right_width // 2,
            50,
            self.can_send_waypoints,
        )

        self.can_reset_waypoints = False
        self.clear_waypoints_button = constants.pushbutton_maker(
            "Clear Waypoints",
            constants.ICONS.delete,
            self.clear_waypoints,
            self.right_width // 2,
            50,
            self.can_send_waypoints,
        )

        self.can_pull_waypoints = True
        self.pull_waypoints_button = constants.pushbutton_maker(
            "Pull Waypoints",
            constants.ICONS.download,
            self.pull_waypoints,
            self.right_width // 2,
            50,
            self.can_pull_waypoints,
        )

        self.focus_boat_button = constants.pushbutton_maker(
            "Zoom to Boat",
            constants.ICONS.boat,
            self.zoom_to_boat,
            self.right_width // 2,
            50,
        )

        self.right_tab1_layout.addWidget(self.right_tab1_label, 0, 0, 1, 2)
        self.right_tab1_layout.addWidget(self.right_tab1_table, 1, 0, 1, 2)
        self.right_tab1_layout.addWidget(self.send_waypoints_button, 2, 0)
        self.right_tab1_layout.addWidget(self.clear_waypoints_button, 2, 1)
        self.right_tab1_layout.addWidget(self.focus_boat_button, 3, 0)
        self.right_tab1_layout.addWidget(self.pull_waypoints_button, 3, 1)
        self.right_tab1.setLayout(self.right_tab1_layout)
        # endregion tab1: waypoint data

        # region tab2: buoy data
        self.right_tab2_label = QLabel("Buoy Data")
        self.right_tab2_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.right_tab2_table = QTableWidget()
        self.right_tab2_table.setMinimumWidth(self.right_width)
        self.right_tab2_table.cellClicked.connect(partial(self.zoom_to_marker, table="buoys"))

        self.edit_buoy_data_button = constants.pushbutton_maker(
            "Edit Buoy Data",
            constants.ICONS.cog,
            self.edit_buoy_data,
            self.right_width,
            50,
        )

        self.save_buoy_data_button = constants.pushbutton_maker(
            "Save Buoy Data",
            constants.ICONS.save,
            self.save_buoy_data,
            self.right_width // 2,
            50,
        )

        self.load_buoy_data_button = constants.pushbutton_maker(
            "Load Buoy Data",
            constants.ICONS.hard_drive,
            self.load_buoy_data,
            self.right_width // 2,
            50,
        )

        self.right_tab2_layout.addWidget(self.right_tab2_label, 0, 0, 1, 2)
        self.right_tab2_layout.addWidget(self.right_tab2_table, 1, 0, 1, 2)
        self.right_tab2_layout.addWidget(self.edit_buoy_data_button, 2, 0, 1, 2)
        self.right_tab2_layout.addWidget(self.save_buoy_data_button, 3, 0)
        self.right_tab2_layout.addWidget(self.load_buoy_data_button, 3, 1)
        self.right_tab2.setLayout(self.right_tab2_layout)
        # endregion tab2: buoy data

        self.right_layout.addTab(self.right_tab1, "Waypoints")
        self.right_layout.addTab(self.right_tab2, "Buoy Data")
        self.right_layout.setMaximumWidth(self.right_width)
        self.main_layout.addWidget(self.right_layout, 0, 2)
        # endregion right section

        self.setLayout(self.main_layout)
        # endregion setup UI

        self.telemetry_handler = thread_classes.TelemetryUpdater()
        self.local_waypoint_handler = thread_classes.LocalWaypointFetcher()
        self.remote_waypoint_handler = thread_classes.RemoteWaypointFetcher()

        self.thirty_second_timer.timeout.connect(self.remote_waypoint_handler_starter)
        self.ten_ms_timer.timeout.connect(self.update_telemetry_starter)
        self.one_ms_timer.timeout.connect(self.local_waypoint_handler_starter)

        # updating displays
        self.telemetry_handler.boat_data_fetched.connect(self.update_telemetry_display)
        self.local_waypoint_handler.waypoints_fetched.connect(self.update_waypoints_display)

        # telemetry server URL change
        self.telemetry_handler.request_url_change.connect(self.change_telemetry_server_url)
        self.remote_waypoint_handler.request_url_change.connect(self.change_telemetry_server_url)

        # misc
        self.remote_waypoint_handler.waypoints_fetched.connect(self.check_telemetry_waypoints)

        for timer in self.timers:
            timer.start()

    # region button functions
    def send_waypoints(self, test: bool = False) -> None:
        """
        Send waypoints to the server.

        Parameters
        ----------
        test
            If `True`, use the test waypoint endpoint. Defaults to `False`.
        """

        if not test:
            try:
                response = constants.REQ_SESSION.post(
                    urljoin(constants.TELEMETRY_SERVER_ENDPOINTS["set_waypoints"], str(constants.TELEMETRY_SERVER_INSTANCE_ID)),
                    json={"waypoints": self.waypoints},
                    timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
                )
                response.raise_for_status()

                js_code = "map.change_color_waypoints('red')"
                self.browser.page().runJavaScript(js_code)
                print(f"[Info] Waypoints sent successfully. Waypoints: {self.waypoints}")

            except requests.exceptions.RequestException as e:
                print(f"[Error] Failed to send waypoints: {e}\nWaypoints: {self.waypoints}")

        else:
            try:
                response = constants.REQ_SESSION.post(
                    urljoin(constants.TELEMETRY_SERVER_ENDPOINTS["waypoints_test"], str(constants.TELEMETRY_SERVER_INSTANCE_ID)),
                    json={"waypoints": self.waypoints},
                    timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
                )
                response.raise_for_status()

            except requests.exceptions.RequestException as e:
                print(f"[Error] Failed to send waypoints: {e}\nWaypoints: {self.waypoints}")

    def pull_waypoints(self) -> None:
        """Pull waypoints from the telemetry server and add them to the map."""

        try:
            remote_waypoints: list[list[float]] = constants.REQ_SESSION.get(
                urljoin(constants.TELEMETRY_SERVER_ENDPOINTS["get_waypoints"], str(constants.TELEMETRY_SERVER_INSTANCE_ID)),
                timeout=constants.TELEMETRY_TIMEOUT_SECONDS,
            ).json()

            if remote_waypoints:
                print(f"[Info] Fetched waypoints from server: {remote_waypoints}")
                existing_waypoints = self.waypoints.copy()
                self.browser.page().runJavaScript("map.clear_waypoints()")
                for waypoint in remote_waypoints:
                    self.browser.page().runJavaScript(f"map.add_waypoint({waypoint[0]}, {waypoint[1]})")
                self.browser.page().runJavaScript("map.change_color_waypoints('red')")
                for waypoint in existing_waypoints:
                    self.browser.page().runJavaScript(f"map.add_waypoint({waypoint[0]}, {waypoint[1]})")

            else:
                print("[Warning] No waypoints found on the server.")

            self.can_pull_waypoints = False
            self.pull_waypoints_button.setDisabled(not self.can_pull_waypoints)

        except requests.exceptions.RequestException as e:
            print(f"[Error] Failed to pull waypoints. Exception: {e}")

    def clear_waypoints(self) -> None:
        """Clear waypoints from the table."""

        self.can_reset_waypoints = False
        self.can_pull_waypoints = True
        self.pull_waypoints_button.setDisabled(not self.can_pull_waypoints)
        js_code = "map.clear_waypoints()"
        self.browser.page().runJavaScript(js_code)

    def save_boat_data(self) -> None:
        """
        Saves latest entry in the `self.boat_data` array to a file.

        Files are stored in the `boat_data` directory and are named `boat_data_<timestamp>.json`
        where `<timestamp>` is nanoseconds since unix epoch.
        """

        try:
            file_path = PurePath(constants.BOAT_DATA_DIR / f"boat_data_{time.time_ns()}.json")
            with open(file_path, "w") as f:
                json.dump(self.boat_data, f, indent=4)

        except Exception as e:
            print(f"[Error] Failed to save boat data: {e}")

        print(f"[Info] Boat data saved to {file_path}")

    def edit_boat_data_limits(self) -> None:
        """
        Opens a text edit window to edit the telemetry data limits.

        `self.edit_boat_data_limits_callback` is called when the user closes or clicks the save button in the text edit window.
        `self.edit_boat_data_limits_callback` recieves the text from the text edit window when the user clicks the save button,
        otherwise it recieves the text without any changes.
        """

        try:
            initial_config = json.dumps(self.telemetry_data_limits, indent=4)
            self.text_edit_window = TextEditWindow(highlighter=JsonHighlighter, initial_text=initial_config)
            self.text_edit_window.setWindowTitle("Edit Boat Data Limits")
            self.text_edit_window.user_text_emitter.connect(self.edit_boat_data_limits_callback)
            self.text_edit_window.show()

        except Exception as e:
            print(f"[Error] Failed to open boat data limits edit window: {e}")

    def edit_boat_data_limits_callback(self, text: str) -> None:
        """
        Callback function for the `edit_boat_data_limits` function.

        This function is called when the user closes the text edit window.
        It retrieves the edited text and saves it to the `self.telemetry_data_limits` variable and closes the window.

        Parameters
        ----------
        text
            The text entered by the user in the text edit window.
        """

        try:
            edited_config = text
            self.telemetry_data_limits = json.loads(edited_config)

        except Exception as e:
            print(f"[Error] Failed to edit boat data limits: {e}")

    def load_boat_data_limits(self) -> None:
        """
        Load upper and lower bounds for some of the telemetry data, if no file selected use `default.json`.

        Files are stored in the `boat_data_bounds` directory and are named `boat_data_bounds_<timestamp>.json`
        where `<timestamp>` is nanoseconds since unix epoch.
        """

        try:
            chosen_file = QFileDialog.getOpenFileName(
                self,
                "Select Parameter File",
                constants.BOAT_DATA_LIMITS_DIR.as_posix(),
                "*.json",
            )
            if chosen_file == ("", ""):
                chosen_file = [PurePath(constants.BOAT_DATA_LIMITS_DIR / "default.json")]
            with open(chosen_file[0], "r") as f:
                self.telemetry_data_limits = json.load(f)

        except Exception as e:
            print(f"[Error] Failed to load boat data limits: {e}")

        print(f"[Info] Boat data limits loaded from {chosen_file[0]}")

    def save_boat_data_limits(self) -> None:
        """
        Save upper and lower bounds for some of the telemetry data.

        Files are stored in the `boat_data_bounds` directory and are named `boat_data_bounds_<timestamp>.json`
        where `<timestamp>` is nanoseconds since unix epoch.
        """

        try:
            file_path = PurePath(
                constants.BOAT_DATA_LIMITS_DIR / f"boat_data_bounds_{time.time_ns()}.json",
            )
            with open(file_path, "w") as f:
                json.dump(self.telemetry_data_limits, f, indent=4)

        except Exception as e:
            print(f"[Error] Failed to save boat data limits: {e}")

        print(f"[Info] Boat data limits saved to {file_path}")

    def edit_buoy_data(self) -> None:
        """
        Opens a text edit window to edit the buoy data.

        `self.edit_buoy_data_callback` is called when the user closes or clicks the save button in the text edit window.
        `self.edit_buoy_data_callback` recieves the text from the text edit window when the user clicks the save button,
        otherwise it recieves the text without any changes.
        """

        try:
            buoy_json = json.dumps(self.buoys, indent=4)
            self.text_edit_window = TextEditWindow(highlighter=JsonHighlighter, initial_text=buoy_json)
            self.text_edit_window.setWindowTitle("Edit Buoy GPS Coordinates")
            self.text_edit_window.user_text_emitter.connect(self.edit_buoy_data_callback)
            self.text_edit_window.show()

        except Exception as e:
            print(f"[Error] Failed to open buoy data edit window: {e}")

    def edit_buoy_data_callback(self, text: str) -> None:
        """
        Callback function for the `edit_buoy_data` function.

        This function is called when the user closes the text edit window.
        It retrieves the edited text and saves it to the `self.buoys` variable and closes the window.

        Parameters
        ----------
        text
            The text entered by the user in the text edit window.
        """

        try:
            edited_bouys = text
            if self.buoys != json.loads(edited_bouys):
                self.buoys = json.loads(edited_bouys)
                self.update_buoy_table()

        except Exception as e:
            print(f"[Error] Failed to edit buoy data: {e}")

    def update_buoy_table(self) -> None:
        """Update the buoy table with the latest buoy data."""

        self.right_tab2_table.clear()
        self.right_tab2_table.setRowCount(0)
        self.right_tab2_table.setColumnCount(2)
        self.right_tab2_table.setHorizontalHeaderLabels(["Latitude", "Longitude"])

        clear_js_buoys = "map.clear_buoys()"
        self.browser.page().runJavaScript(clear_js_buoys)

        for buoy in self.buoys:
            self.right_tab2_table.insertRow(self.right_tab2_table.rowCount())
            add_js_buoy = f"map.add_buoy({self.buoys[buoy]['lat']}, {self.buoys[buoy]['lon']})"
            self.browser.page().runJavaScript(add_js_buoy)
            for i, coord in enumerate(["lat", "lon"]):
                item = QTableWidgetItem(f"{float(self.buoys[buoy][coord]):.13f}")
                item.setFlags(Qt.ItemFlag.ItemIsEnabled)
                self.right_tab2_table.setItem(self.right_tab2_table.rowCount() - 1, i, item)
        self.right_tab2_table.resizeColumnsToContents()
        self.right_tab2_table.resizeRowsToContents()

    def save_buoy_data(self) -> None:
        """
        Saves latest entry in the `self.buoys` array to a file.

        Files are stored in the `buoy_data` directory and are named `buoy_data_<timestamp>.json`
        where `<timestamp>` is nanoseconds since unix epoch.
        """

        try:
            file_path = PurePath(constants.BUOY_DATA_DIR / f"buoy_data_{time.time_ns()}.json")
            with open(file_path, "w") as f:
                json.dump(self.buoys, f, indent=4)

        except Exception as e:
            print(f"[Error] Failed to save buoy data: {e}")

        print(f"[Info] Buoy data saved to {file_path}")

    def load_buoy_data(self) -> None:
        """
        Load buoy data from the `buoy_data` directory, if none selected use `default.json`.

        Files are stored in the `buoy_data` directory and are named `buoy_data_<timestamp>.json`
        where `<timestamp>` is nanoseconds since unix epoch.
        """

        try:
            buoy_files = os.listdir(constants.BUOY_DATA_DIR)
            if not buoy_files:
                print("[Warning] No buoy data files found.")

            else:
                chosen_file = QFileDialog.getOpenFileName(
                    self,
                    "Select Buoy Data File",
                    constants.BUOY_DATA_DIR.as_posix(),
                    "*.json",
                )
                if chosen_file == ("", ""):
                    chosen_file = [PurePath(constants.BUOY_DATA_DIR / "default.json")]

                with open(chosen_file[0], "r") as f:
                    self.buoys = json.load(f)

                self.update_buoy_table()

        except Exception as e:
            print(f"[Error] Failed to load buoy data: {e}")

        print(f"[Info] Buoy data loaded from {chosen_file[0]}")

    def zoom_to_boat(self) -> None:
        """Center the view on the boat's position."""

        if isinstance(self.boat_data.get("position"), list):
            js_code = "map.focus_map_on_boat()"
            self.browser.page().runJavaScript(js_code)

        else:
            print("[Warning] Boat position not available.")

    def zoom_to_marker(self, row: int, table: Literal["waypoints", "buoys"] = "waypoints") -> None:
        """
        Center the view on the selected waypoint in the table.

        Parameters
        ----------
        row
            The row index of the waypoint in the table.

        table
            The table to zoom in on, either "waypoints" or "buoys". Defaults to "waypoints".
        """

        if table == "waypoints":
            if self.right_tab1_table.rowCount() > 0:
                try:
                    approx_lat = float(self.right_tab1_table.item(row, 0).text())
                    approx_lon = float(self.right_tab1_table.item(row, 1).text())

                    lat, lon = None, None
                    for waypoint in self.waypoints:
                        if abs(waypoint[0] - approx_lat) < 1e-6 and abs(waypoint[1] - approx_lon) < 1e-6:
                            lat, lon = waypoint
                            break

                    js_code = f"map.focus_map_on_marker({lat}, {lon})"
                    self.browser.page().runJavaScript(js_code)

                except (ValueError, TypeError) as e:
                    print(f"[Error] Invalid waypoint data: {e}")
            else:
                print("[Warning] No waypoints available to zoom to.")

        elif table == "buoys":
            if self.right_tab2_table.rowCount() > 0:
                try:
                    approx_lat = float(self.right_tab2_table.item(row, 0).text())
                    approx_lon = float(self.right_tab2_table.item(row, 1).text())

                    lat, lon = None, None
                    for buoy in self.buoys.values():
                        if abs(buoy["lat"] - approx_lat) < 1e-6 and abs(buoy["lon"] - approx_lon) < 1e-6:
                            lat, lon = buoy["lat"], buoy["lon"]
                            break

                    js_code = f"map.focus_map_on_marker({lat}, {lon})"
                    self.browser.page().runJavaScript(js_code)

                except (ValueError, TypeError) as e:
                    print(f"[Error] Invalid buoy data: {e}")
            else:
                print("[Warning] No buoys available to zoom to.")

        else:
            print(f"[Error] Invalid table specified: {table}. Use 'waypoints' or 'buoys'.")

    # endregion button functions

    # region pyqt thread functions
    def local_waypoint_handler_starter(self) -> None:
        """Starts the JS waypoint handler thread."""

        if not self.local_waypoint_handler.isRunning():
            self.local_waypoint_handler.start()

    def remote_waypoint_handler_starter(self) -> None:
        """Starts the telemetry waypoint handler thread."""

        if not self.remote_waypoint_handler.isRunning() and constants.TELEMETRY_SERVER_INSTANCE_ID != -1:
            self.remote_waypoint_handler.start()

    def update_telemetry_starter(self) -> None:
        """Starts the telemetry handler thread."""

        if not self.telemetry_handler.isRunning() and constants.TELEMETRY_SERVER_INSTANCE_ID != -1:
            self.telemetry_handler.start()

    def update_waypoints_display(self, waypoints: list[list[float]]) -> None:
        """
        Update waypoints display with fetched waypoints.

        Parameters
        ----------
        waypoints
            List of waypoints fetched from the server.
        """

        self.waypoints = waypoints
        self.send_waypoints_button.setDisabled(not self.can_send_waypoints)
        self.clear_waypoints_button.setDisabled(not self.can_reset_waypoints)
        self.pull_waypoints_button.setDisabled(not self.can_pull_waypoints)
        if self.num_waypoints != len(self.waypoints):
            self.num_waypoints = len(self.waypoints)
            if self.num_waypoints == 0:
                self.can_pull_waypoints = True
                self.can_reset_waypoints = False
            else:
                self.can_pull_waypoints = False
                self.can_reset_waypoints = True
            self.can_send_waypoints = True

            self.right_tab1_table.clear()
            self.right_tab1_table.setRowCount(0)
            self.right_tab1_table.setColumnCount(2)
            self.right_tab1_table.setHorizontalHeaderLabels(["Latitude", "Longitude"])

            for waypoint in waypoints:
                self.right_tab1_table.insertRow(self.right_tab1_table.rowCount())
                for i, coord in enumerate(waypoint):
                    item = QTableWidgetItem(f"{coord:.13f}")
                    item.setFlags(Qt.ItemFlag.ItemIsEnabled)
                    self.right_tab1_table.setItem(self.right_tab1_table.rowCount() - 1, i, item)
            self.right_tab1_table.resizeColumnsToContents()
            self.right_tab1_table.resizeRowsToContents()

    def check_telemetry_waypoints(self, waypoints: list[list[float]]) -> None:
        """
        Check if the waypoints on the telemetry server are the same as the local waypoints.
        If they are different, show a dialog and let user decide whether to update the local waypoints.

        Parameters
        ----------
        waypoints
            List of waypoints to check.
        """

        equal_flag = sorted(self.waypoints) == sorted(waypoints)

        if not equal_flag and not self.remember_waypoints_pull_service_status and self.can_pull_waypoints:
            for timer in self.timers:
                timer.stop()

            response, temp_pull_waypoints_reminder = constants.show_message_box(
                "Local Waypoints Mismatch",
                "The local waypoints are different from the telemetry server waypoints. Do you want to update the local waypoints?",
                constants.ICONS.warning,
                [
                    QMessageBox.StandardButton.Yes,
                    QMessageBox.StandardButton.No,
                ],
                True,
            )
            if response == QMessageBox.StandardButton.Yes:
                not_uploaded_waypoints = [waypoint for waypoint in self.waypoints if waypoint not in waypoints]
                self.waypoints = waypoints.copy()
                self.browser.page().runJavaScript("map.clear_waypoints()")
                for waypoint in self.waypoints:
                    self.browser.page().runJavaScript(f"map.add_waypoint({waypoint[0]}, {waypoint[1]})")
                self.browser.page().runJavaScript("map.change_color_waypoints('red')")
                for waypoint in not_uploaded_waypoints:
                    self.browser.page().runJavaScript(f"map.add_waypoint({waypoint[0]}, {waypoint[1]})")
                print("[Info] Local waypoints updated from telemetry server.")

            else:
                self.remember_waypoints_pull_service_status = temp_pull_waypoints_reminder
                print("[Info] Local waypoints not updated.")

            for timer in self.timers:
                timer.start()

        elif not equal_flag and self.remember_waypoints_pull_service_status:
            print("[Info] Local waypoints do not match telemetry server waypoints, but not prompting user.")

        else:
            print("[Info] Local waypoints match telemetry server waypoints, but not prompting user.")

    def change_telemetry_server_url(self, telemetry_status: constants.TelemetryStatus) -> None:
        """
        Prompt the user to change the telemetry server URL if fetching waypoints fails.

        Parameters
        ----------
        telemetry_status
            <ul>
            <li> <code>SUCCESS</code> indicates that the telemetry server is reachable and waypoints were fetched successfully.</li>
            <li> <code>FAILURE</code> indicates that the telemetry server is not reachable and waypoints could not be fetched.</li>
            </ul>
        """

        if telemetry_status == constants.TelemetryStatus.FAILURE and not self.remember_telemetry_server_url_status:
            for timer in self.timers:
                timer.stop()

            response, temp_remember_telemetry_server_url_status = constants.show_message_box(
                "Failed to fetch waypoints",
                "Do you want to change the telemetry server URL?",
                constants.ICONS.question,
                [
                    QMessageBox.StandardButton.Yes,
                    QMessageBox.StandardButton.No,
                ],
                True,
            )

            if response == QMessageBox.StandardButton.Yes:
                new_url = constants.show_input_dialog(
                    "Change Telemetry Server URL",
                    "Enter the new telemetry server URL:",
                    default_value=constants.TELEMETRY_SERVER_URL,
                    input_type=str,
                )

                if new_url:
                    print(f"[Info] Changed telemetry server URL to {new_url}, was {constants.TELEMETRY_SERVER_URL}.")
                    constants.TELEMETRY_SERVER_URL = new_url
                    for endpoint in constants.TELEMETRY_SERVER_ENDPOINTS:
                        path_tail = "/".join(constants.TELEMETRY_SERVER_ENDPOINTS[endpoint].split("/")[-2:])
                        new_endpoint_url = constants.TELEMETRY_SERVER_URL + path_tail
                        constants.TELEMETRY_SERVER_ENDPOINTS[endpoint] = new_endpoint_url

                else:
                    print("[Warning] No new telemetry server URL provided, keeping old one.")

            elif response == QMessageBox.StandardButton.No:
                self.remember_telemetry_server_url_status = temp_remember_telemetry_server_url_status
                print("[Info] Telemetry server URL not changed.")

            else:
                print(f"[Error] Received unexpected response from user dialog. Got: {response}, expected Yes or No.")

            for timer in self.timers:
                timer.start()

    def update_telemetry_display(
        self,
        boat_data: dict[str, float | str | tuple[float, float] | list[tuple[float, float]]],
    ) -> None:
        """
        Update telemetry display with boat data.

        Parameters
        ----------
        boat_data
            Dictionary containing boat data fetched from the telemetry server.
        """

        def fix_formatting(data_item: float | None) -> str:
            """
            Applies some formatting rules that multiple keys have in common.

            <ol>
            <li> If the value is None, it is replaced with -69.420.
            <li> If the value is negative, it is multiplied by -1.
            <li> The value is rounded to 5 decimal places.
            </ol>

            Examples
            -------
            >>> fix_formatting(-69.420)
            '69.42000'
            >>> fix_formatting(None)
            '-69.42000'

            Parameters
            ----------
            data_item
                The float value to format.

            Returns
            -------
            str
                The formatted value.
            """

            return f"{abs(data_item):.5f}" if data_item is not None else f"{-69.420:.5f}"

        def convert_to_seconds(ms: float) -> float:
            """
            Converts milliseconds to seconds. 1000 milliseconds = 1 second.

            Parameters
            ----------
            ms
                The time in milliseconds.

            Returns
            -------
            float
                The time in seconds.
            """

            return ms * 1000

        self.boat_data = boat_data
        self.all_boat_data[f"{time.time_ns()}"] = boat_data

        try:
            if not isinstance(self.boat_data.get("position"), list):
                raise TypeError("Boat position is not a list. Expected format: [latitude, longitude]")
            if not isinstance(self.boat_data.get("heading"), float):
                raise TypeError("Boat heading is not a float. Expected format: float")

            if not all(isinstance(coord, (float, int)) for coord in self.boat_data["position"]):
                raise TypeError("Boat position coordinates are not all floats or ints.")

            if len(self.boat_data["position"]) != 2:
                raise ValueError("Boat position list does not contain exactly two elements.")

            js_code = (
                f"map.update_boat_location({self.boat_data['position'][0]}, {self.boat_data['position'][1]});"
                f"map.update_boat_heading({self.boat_data['heading']});"
            )
            self.browser.page().runJavaScript(js_code)

        except TypeError:
            self.boat_data["position"] = [-69.420, -69.420]
            self.boat_data["heading"] = -69.420

        telemetry_text = (
            "Position: "
            f"{self.boat_data.get('position', [-69.420, -69.420])[0]:.8f}, "
            f"{self.boat_data.get('position', [-69.420, -69.420])[1]:.8f}\n"
            f"State: {self.boat_data.get('state', 'N/A')}\n"
            f"Current Maneuver: {self.boat_data.get('full_autonomy_maneuver', 'N/A')}\n"
            f"Velocity Vector: {self.boat_data.get('velocity_vector', [-69.420, -69.420])[0]:.5f}, {self.boat_data.get('velocity_vector', [-69.420, -69.420])[1]:.5f}\n"
            f"Speed: {self.boat_data.get('speed', -69.420):.5f} knots\n"
            f"Distance To Next WP: {fix_formatting(self.boat_data.get('distance_to_next_waypoint'))} meters\n"
            f"Bearing: {self.boat_data.get('bearing', -69.420):.5f}°\n"
            f"Heading: {self.boat_data.get('heading', -69.420):.5f}°\n"
            f"True Wind Speed: {self.boat_data.get('true_wind_speed', -69.420):.5f} knots\n"
            f"True Wind Angle: {self.boat_data.get('true_wind_angle', -69.420):.5f}°\n"
            f"Apparent Wind Speed: {self.boat_data.get('apparent_wind_speed', -69.420):.5f} knots\n"
            f"Apparent Wind Angle: {self.boat_data.get('apparent_wind_angle', -69.420):.5f}°\n"
            f"Sail Angle: {self.boat_data.get('sail_angle', -69.420):.5f}°\n"
            f"Rudder Angle: {self.boat_data.get('rudder_angle', -69.420):.5f}°\n"
            f"Current Waypoint Index: {self.boat_data.get('current_waypoint_index', 'N/A')}\n"
            f"Current Waypoint: {self.waypoints[self.boat_data.get('current_waypoint_index', -1)] if isinstance(self.boat_data.get('current_waypoint_index'), int) and 0 <= self.boat_data.get('current_waypoint_index', -1) < len(self.waypoints) else 'N/A'}"
        )

        self.left_text_section.setText(telemetry_text)

    # endregion pyqt thread functions

    # region helper functions
    def safe_convert_to_float(self, x: object) -> float | Literal[0]:
        """
        Safely convert a value to float, returning 0 if conversion fails.

        Parameters
        ----------
        x
            The value to convert to float.

        Returns
        -------
        float or Literal[0]
            The converted float value, or `0` if conversion fails.
        """

        try:
            return float(x)
        except ValueError:
            return 0

    # endregion helper functions
