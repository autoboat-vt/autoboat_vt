import json
import os
import time
from functools import partial
from pathlib import Path
from typing import Any, Literal
from urllib.parse import urljoin

import numpy as np
import svg
from qtpy.QtCore import Qt, QUrl, Signal
from qtpy.QtWebEngineWidgets import QWebEngineView
from qtpy.QtWidgets import (
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QLabel,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from requests.exceptions import RequestException
from syntax_highlighters import JsonHighlighter
from utils import constants, misc, thread_classes

from widgets.popup_edit import TextEditWindow
from widgets.popup_telemetry_config import EditTelemetryConfigWindow


class GroundStationWidget(QWidget):
    """
    Main widget for the ground station application.

    Parameters
    ----------
    boat_status_source
        A ``Signal`` that provides boat status updates.

    Inherits
    -------
    ``QWidget``
    """

    def __init__(self, boat_status_source: Signal) -> None:
        super().__init__()
        self.fake_position: tuple[float] = (0.0, 0.0)
        self.fake_heading: float = 180.0

        self.waypoints: list[list[float]] = []
        self.num_waypoints: int = 0

        # buoy_name => {"lat": float, "lon": float}
        self.buoys: dict[str, dict[str, float]] = {}

        self.boat_data: dict[str, Any] = {}
        self.telemetry_data_limits: dict[str, float] = {}

        # should we remember the status of the user's last response to the
        # dialog that asks if the telemetry server URL should be changed?
        self.remember_telemetry_server_url_status: bool = False

        # should we check for changes in the telemetry server waypoints?
        self.waypoints_checker_status: bool = False

        # should we display sailboat debugging symbols?
        self.sailboat_debugging_symbols_status: bool = False

        # should we remember the status of the user's last response to the
        # dialog that asks if the user wants to pull waypoints from the telemetry server?
        self.remember_waypoints_pull_service_status: bool = False

        # region timers
        self.one_ms_timer = misc.copy_qtimer(constants.ONE_MS_TIMER)
        self.thirty_second_timer = misc.copy_qtimer(constants.THIRTY_SECOND_TIMER)
        self.timers = [self.one_ms_timer, self.thirty_second_timer]

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
        self.left_text_section.highlighter = JsonHighlighter(self.left_text_section.document())
        self.left_text_section.setReadOnly(True)
        self.left_text_section.setText("Awaiting telemetry data...")

        self.save_boat_data_button = misc.pushbutton_maker(
            "Save Boat Data to File",
            constants.ICONS.save,
            self.save_boat_data,
            max_width=self.left_width,
            min_height=50,
        )

        self.edit_boat_data_limits_button = misc.pushbutton_maker(
            "Edit Limits",
            constants.ICONS.cog,
            self.edit_boat_data_limits,
            max_width=self.left_width // 2,
            min_height=50,
        )

        self.side_buttons_layout = QVBoxLayout()

        self.load_boat_data_limits_button = misc.pushbutton_maker(
            "Load Limits from File",
            constants.ICONS.hard_drive,
            self.load_boat_data_limits,
            max_width=self.left_width // 2,
            min_height=25,
        )

        self.save_boat_data_limits_button = misc.pushbutton_maker(
            "Save Limits to File",
            constants.ICONS.save,
            self.save_boat_data_limits,
            max_width=self.left_width // 2,
            min_height=25,
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
        self.browser.setUrl(QUrl(f"http://127.0.0.1:{constants.VITE_PORT}"))
        self.browser.setMinimumWidth(700)
        self.browser.setMinimumHeight(700)

        def handle_waypoints_callback(state: Qt.CheckState) -> None:
            self.waypoints_checker_status = state == Qt.CheckState.Checked
        
        def handle_sailboat_debugging_callback(state: Qt.CheckState) -> None:
            if state == Qt.CheckState.Checked:
                self.sailboat_debugging_symbols_status = True
            else:
                self.sailboat_debugging_symbols_status = False
                self.browser.page().runJavaScript("map.remove_all_svgs()")

        self.edit_telemetry_config_window = EditTelemetryConfigWindow(handle_waypoints_callback,
                                                                      handle_sailboat_debugging_callback)
        self.edit_telemetry_config_window.setWindowTitle("Edit Telemetry Config")
        
        self.telemetry_config_button = QPushButton("Map Appearance Config")
        self.telemetry_config_button.setToolTip(
            "If enabled, a popup will appear where you can alter the telemetry configuration.",
        )
        self.telemetry_config_button.clicked.connect(
            lambda: self.edit_telemetry_config_window.show() or self.edit_telemetry_config_window.raise_()
        )

        self.middle_layout.addWidget(self.browser, 0, 1)
        self.middle_layout.setRowStretch(0, 1)
        self.middle_layout.addWidget(self.telemetry_config_button, 1, 1, Qt.AlignCenter)
        self.middle_layout.setRowStretch(1, 0)
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
        self.send_waypoints_button = misc.pushbutton_maker(
            "Send Waypoints",
            constants.ICONS.upload,
            self.send_waypoints,
            max_width=self.right_width // 2,
            min_height=50,
            is_clickable=self.can_send_waypoints,
        )

        self.can_reset_waypoints = False
        self.clear_waypoints_button = misc.pushbutton_maker(
            "Clear Waypoints",
            constants.ICONS.delete,
            self.clear_waypoints,
            max_width=self.right_width // 2,
            min_height=50,
            is_clickable=self.can_send_waypoints,
        )

        self.can_pull_waypoints = True
        self.pull_waypoints_button = misc.pushbutton_maker(
            "Pull Waypoints",
            constants.ICONS.download,
            self.pull_waypoints,
            max_width=self.right_width // 2,
            min_height=50,
            is_clickable=self.can_pull_waypoints,
        )

        self.focus_boat_button = misc.pushbutton_maker(
            "Zoom to Boat",
            constants.ICONS.boat,
            self.zoom_to_boat,
            max_width=self.right_width // 2,
            min_height=50,
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

        self.edit_buoy_data_button = misc.pushbutton_maker(
            "Edit Buoy Data",
            constants.ICONS.cog,
            self.edit_buoy_data,
            max_width=self.right_width,
            min_height=50,
        )

        self.save_buoy_data_button = misc.pushbutton_maker(
            "Save Buoy Data",
            constants.ICONS.save,
            self.save_buoy_data,
            max_width=self.right_width // 2,
            min_height=50,
        )

        self.load_buoy_data_button = misc.pushbutton_maker(
            "Load Buoy Data",
            constants.ICONS.hard_drive,
            self.load_buoy_data,
            max_width=self.right_width // 2,
            min_height=50,
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

        self.local_waypoint_handler = thread_classes.WaypointThreadRouter.LocalFetcherThread()
        self.local_waypoint_handler.response.connect(self.update_waypoints_display)
        self.one_ms_timer.timeout.connect(self.local_waypoint_handler_starter)

        self.remote_waypoint_handler = thread_classes.WaypointThreadRouter.RemoteFetcherThread()
        self.remote_waypoint_handler.response.connect(self.check_telemetry_waypoints)
        self.thirty_second_timer.timeout.connect(self.remote_waypoint_handler_starter)

        for timer in self.timers:
            timer.start()

        self.boat_status_source: Signal = boat_status_source
        self.boat_status_source.connect(self.update_telemetry_display)

    # region button functions
    def send_waypoints(self, test: bool = False) -> None:
        """
        Send waypoints to the server.

        Parameters
        ----------
        test
            If ``True``, use the test waypoint endpoint. Defaults to ``False``.
        """

        if not test:
            try:
                instance_id = constants.SM.read("telemetry_server_instance_id")
                constants.REQ_SESSION.post(
                    urljoin(misc.get_route("set_waypoints"), str(instance_id)),
                    json=self.waypoints,
                )

                js_code = "map.change_color_waypoints('red')"
                self.browser.page().runJavaScript(js_code)
                print(f"[Info] Waypoints sent successfully. Waypoints: {self.waypoints}")

            except RequestException as e:
                print(f"[Error] Failed to send waypoints: {e}\nWaypoints: {self.waypoints}")

        else:
            try:
                constants.REQ_SESSION.post(
                    urljoin(
                        constants.SM.read("test_waypoints"),
                        str(constants.SM.read("telemetry_server_instance_id")),
                    ),
                    json=self.waypoints,
                )

            except RequestException as e:
                print(f"[Error] Failed to send waypoints: {e}\nWaypoints: {self.waypoints}")

    def pull_waypoints(self) -> None:
        """Pull waypoints from the telemetry server and add them to the map."""

        try:
            instance_id = constants.SM.read("telemetry_server_instance_id")
            remote_waypoints: list[list[float]] = constants.REQ_SESSION.get(
                urljoin(misc.get_route("get_waypoints"), str(instance_id)),
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

        except RequestException as e:
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
        Saves latest entry in the ``self.boat_data`` array to a file.

        Files are stored in the ``boat_data`` directory and are named ``boat_data_<timestamp>.json``
        where ``<timestamp>`` is nanoseconds since unix epoch.
        """

        try:
            file_path = Path(constants.BOAT_DATA_DIR / f"boat_data_{time.time_ns()}.json")
            with open(file_path, mode="w", encoding="utf-8") as f:
                json.dump(self.boat_data, f, indent=4)

        except Exception as e:
            print(f"[Error] Failed to save boat data: {e}")

        print(f"[Info] Boat data saved to {file_path}")

    def edit_boat_data_limits(self) -> None:
        """
        Opens a text edit window to edit the telemetry data limits.

        ``self.edit_boat_data_limits_callback`` is called when the user closes or clicks the save button in the text edit window.
        ``self.edit_boat_data_limits_callback`` recieves the text from the text edit window when the user clicks the save button,
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
        Callback function for the ``edit_boat_data_limits`` function.

        This function is called when the user closes the text edit window.
        It retrieves the edited text and saves it to the ``self.telemetry_data_limits`` variable and closes the window.

        Parameters
        ----------
        text
            The text entered by the user in the text edit window.
        """

        try:
            self.telemetry_data_limits = json.loads(text)

        except Exception as e:
            print(f"[Error] Failed to edit boat data limits: {e}")

    def load_boat_data_limits(self) -> None:
        """
        Load upper and lower bounds for some of the telemetry data, if no file selected use ``default.json``.

        Files are stored in the ``boat_data_bounds`` directory and are named ``boat_data_bounds_<timestamp>.json``
        where ``<timestamp>`` is nanoseconds since unix epoch.
        """

        try:
            chosen_file = QFileDialog.getOpenFileName(
                self,
                "Select Parameter File",
                constants.BOAT_DATA_LIMITS_DIR.as_posix(),
                "*.json",
            )
            if chosen_file == ("", ""):
                chosen_file = [Path(constants.BOAT_DATA_LIMITS_DIR / "default.json")]
            with open(chosen_file[0], mode="r", encoding="utf-8") as f:
                self.telemetry_data_limits = json.load(f)

        except Exception as e:
            print(f"[Error] Failed to load boat data limits: {e}")

        print(f"[Info] Boat data limits loaded from {chosen_file[0]}")

    def save_boat_data_limits(self) -> None:
        """
        Save upper and lower bounds for some of the telemetry data.

        Files are stored in the ``boat_data_bounds`` directory and are named ``boat_data_bounds_<timestamp>.json``
        where ``<timestamp>`` is nanoseconds since unix epoch.
        """

        try:
            file_path = Path(
                constants.BOAT_DATA_LIMITS_DIR / f"boat_data_bounds_{time.time_ns()}.json",
            )
            with open(file_path, mode="w", encoding="utf-8") as f:
                json.dump(self.telemetry_data_limits, f, indent=4)

        except Exception as e:
            print(f"[Error] Failed to save boat data limits: {e}")

        print(f"[Info] Boat data limits saved to {file_path}")

    def edit_buoy_data(self) -> None:
        """
        Opens a text edit window to edit the buoy data.

        ``self.edit_buoy_data_callback`` is called when the user closes or clicks the save button in the text edit window.
        ``self.edit_buoy_data_callback`` recieves the text from the text edit window when the user clicks the save button,
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
        Callback function for the ``edit_buoy_data`` function.

        This function is called when the user closes the text edit window.
        It retrieves the edited text and saves it to the ``self.buoys`` variable and closes the window.

        Parameters
        ----------
        text
            The text entered by the user in the text edit window.
        """

        try:
            edited_bouys = json.loads(text)
            if self.buoys != edited_bouys:
                self.buoys = edited_bouys
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
        Saves latest entry in the ``self.buoys`` array to a file.

        Files are stored in the ``buoy_data`` directory and are named ``buoy_data_<timestamp>.json``
        where ``<timestamp>`` is nanoseconds since unix epoch.
        """

        try:
            file_path = Path(constants.BUOY_DATA_DIR / f"buoy_data_{time.time_ns()}.json")
            with open(file_path, mode="w", encoding="utf-8") as f:
                json.dump(self.buoys, f, indent=4)

        except Exception as e:
            print(f"[Error] Failed to save buoy data: {e}")

        print(f"[Info] Buoy data saved to {file_path}")

    def load_buoy_data(self) -> None:
        """
        Load buoy data from the ``buoy_data`` directory, if none selected use ``default.json``.

        Files are stored in the ``buoy_data`` directory and are named ``buoy_data_<timestamp>.json``
        where ``<timestamp>`` is nanoseconds since unix epoch.
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
                    chosen_file = [Path(constants.BUOY_DATA_DIR / "default.json")]

                with open(chosen_file[0], mode="r", encoding="utf-8") as f:
                    self.buoys = json.load(f)

                self.update_buoy_table()

        except Exception as e:
            print(f"[Error] Failed to load buoy data: {e}")

        print(f"[Info] Buoy data loaded from {chosen_file[0]}")

    def zoom_to_boat(self) -> None:
        """Center the view on the boat's position."""

        self.browser.page().runJavaScript("map.focus_map_on_boat()")

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

    def remote_waypoint_handler_starter(self) -> None:
        """Starts the telemetry waypoint handler thread."""

        if not self.waypoints_checker_status:
            self.remember_waypoints_pull_service_status = False
            print("[Info] Waypoint checker disabled, not checking for waypoint updates.")
            return

        if not self.remote_waypoint_handler.isRunning():
            self.remote_waypoint_handler.start()

    def local_waypoint_handler_starter(self) -> None:
        """Starts the local waypoint handler thread."""

        if not self.local_waypoint_handler.isRunning():
            self.local_waypoint_handler.start()

    def update_waypoints_display(self, request_result: tuple[list[list[int | float]], constants.TelemetryStatus]) -> None:
        """
        Update waypoints display with waypoints fetched from the local server.

        Parameters
        ----------
        request_result
            A tuple containing:
            - a list of waypoints fetched from the local server.
            - a ``TelemetryStatus`` enum value indicating the status of the request.
        """

        waypoints, _ = request_result
        self.waypoints = waypoints
        num_new_waypoints = len(waypoints)

        self.send_waypoints_button.setDisabled(not self.can_send_waypoints)
        self.clear_waypoints_button.setDisabled(not self.can_reset_waypoints)
        self.pull_waypoints_button.setDisabled(not self.can_pull_waypoints)

        if self.num_waypoints != num_new_waypoints:
            self.num_waypoints = num_new_waypoints

            # if we have no local waypoints, we can't reset them
            if self.num_waypoints == 0:
                self.can_pull_waypoints = True
                self.can_reset_waypoints = False

            # if we have local waypoints, we cannot pull from the
            # remote server until we send them or reset them
            # as we cannot pull without overwriting local waypoints
            else:
                self.can_pull_waypoints = False
                self.can_reset_waypoints = True

            # we can always send waypoints to the remote server
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

    def check_telemetry_waypoints(self, request_result: tuple[list[list[int | float]], constants.TelemetryStatus]) -> None:
        """
        Check if the waypoints on the telemetry server are the same as the local waypoints.
        If they are different, show a dialog and let user decide whether to update the local waypoints.

        Parameters
        ----------
        request_result
            A tuple containing:
            - a list of waypoints fetched from the telemetry server.
            - a ``TelemetryStatus`` enum value indicating the status of the request.
        """

        waypoints, _ = request_result
        equal_flag = sorted(self.waypoints) == sorted(waypoints)

        if not (equal_flag and self.remember_waypoints_pull_service_status) and self.can_pull_waypoints:
            for timer in self.timers:
                timer.stop()

            response, temp_pull_waypoints_reminder = misc.show_message_box(
                "Local Waypoints Mismatch",
                "The local waypoints are different from the telemetry server waypoints. Do you want to update the local waypoints?",  # noqa: E501
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
            A ``TelemetryStatus`` enum value indicating the status of the request. Possible values are:
            - ``SUCCESS`` indicates that the telemetry server is reachable and waypoints were fetched successfully.
            - ``FAILURE`` indicates that the telemetry server is not reachable and waypoints could not be fetched.
        """

        if telemetry_status == constants.TelemetryStatus.FAILURE and not self.remember_telemetry_server_url_status:
            for timer in self.timers:
                timer.stop()

            response, temp_remember_telemetry_server_url_status = misc.show_message_box(
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
                new_url = misc.show_input_dialog(
                    "Change Telemetry Server URL",
                    "Enter the new telemetry server URL:",
                    default_value=constants.SM.read("telemetry_server_url"),
                    input_type=str,
                )

                if new_url:
                    print(f"[Info] Changed telemetry server URL to {new_url}, was {constants.SM.read('telemetry_server_url')}.")
                    
                    constants.SM.write("telemetry_server_url", new_url)
                    tmp_dict = {}
                    for endpoint, value in constants.SM.read("telemetry_server_endpoints").items():
                        path_tail = "/".join(value.split("/")[-2:])
                        new_endpoint_url = constants.SM.read("telemetry_server_url") + path_tail
                        tmp_dict[endpoint] = new_endpoint_url

                    constants.SM.write("telemetry_server_endpoints", tmp_dict)

                else:
                    print("[Warning] No new telemetry server URL provided, keeping old one.")

            elif response == QMessageBox.StandardButton.No:
                self.remember_telemetry_server_url_status = temp_remember_telemetry_server_url_status
                print("[Info] Telemetry server URL not changed.")

            else:
                print(f"[Error] Received unexpected response from user dialog. Got: {response}, expected Yes or No.")

            for timer in self.timers:
                timer.start()

    def update_telemetry_display(self, request_result: tuple[dict[str, Any], constants.TelemetryStatus]) -> None:
        """
        Update telemetry display with boat data.

        Parameters
        ----------
        request_result
            A tuple containing:
            - a dictionary with the latest boat telemetry data.
            - a ``TelemetryStatus`` enum value indicating the status of the request.
        """

        def fix_formatting(data_item: float | None) -> str:
            """
            Applies some formatting rules that multiple keys have in common.

            <ol>
            <li> If the value is None, displays "N/A".
            <li> Otherwise, the value is rounded to 1 decimal places.
            </ol>

            Examples
            --------
            >>> fix_formatting(-69.420)
            '-69.42000'
            >>> fix_formatting(None)
            'N/A'

            Parameters
            ----------
            data_item
                The float value to format.

            Returns
            -------
            str
                The formatted value.
            """

            return "N/A" if data_item is None else f"{float(data_item):.1f}"

        def sailboat_mode(boat_data: dict[str, Any]) -> str:
            self.boat_data["full_autonomy_maneuver"] = constants.SailboatStates(boat_data["full_autonomy_maneuver"]).name
            self.boat_data["autopilot_mode"] = constants.SailboatAutopilotMode(boat_data["autopilot_mode"]).name

            return (
                "Position: "
                f"[{self.boat_data.get('position', self.fake_position)[0]:.8f}, "
                f"{self.boat_data.get('position', self.fake_position)[1]:.8f}]\n"
                f"State: {self.boat_data.get('autopilot_mode', 'N/A')}\n"
                f"Connection Status: {connection_status.name}\n"
                f"Current Maneuver: {self.boat_data.get('full_autonomy_maneuver', 'N/A')}\n"
                f"Current Waypoint Index: {self.boat_data.get('current_waypoint_index') + 1 if isinstance(self.boat_data.get('current_waypoint_index'), int) else 'N/A'}\n"  # noqa: E501
                f"Velocity Vector: [{fix_formatting(self.boat_data.get('velocity_x', -69.420))}, {fix_formatting(self.boat_data.get('velocity_y', -69.420))}]\n"  # noqa: E501
                f"Speed: {fix_formatting(self.boat_data.get('speed'))} knots\n"
                f"Distance To Next WP: {fix_formatting(self.boat_data.get('distance_to_next_waypoint'))} meters\n"
                f"Heading: {fix_formatting(self.boat_data.get('heading', self.fake_heading))}°\n"
                f"True Wind Speed: {fix_formatting(self.boat_data.get('true_wind_speed'))} knots\n"
                f"True Wind Angle: {fix_formatting(self.boat_data.get('true_wind_angle'))}°\n"
                f"Apparent Wind Speed: {fix_formatting(self.boat_data.get('apparent_wind_speed'))} knots\n"
                f"Apparent Wind Angle: {fix_formatting(self.boat_data.get('apparent_wind_angle'))}°\n"
                f"Desired Heading: {fix_formatting(self.boat_data.get('desired_heading'))}°\n"
                f"Desired Sail Angle: {fix_formatting(self.boat_data.get('desired_sail_angle'))}°\n"
                f"Current Sail Angle: {fix_formatting(self.boat_data.get('current_sail_angle'))}°\n"
                f"Sail Angle Error: {fix_formatting(self.boat_data.get('sail_angle_error'))}°\n"
                f"Desired Rudder Angle: {fix_formatting(self.boat_data.get('desired_rudder_angle'))}°\n"
                f"Current Rudder Angle: {fix_formatting(self.boat_data.get('current_rudder_angle'))}°\n"
                f"Rudder Angle Error: {fix_formatting(self.boat_data.get('rudder_angle_error'))}°\n"
            )


        def motorboat_mode(boat_data: dict[str, Any]) -> str:
            self.boat_data["autopilot_mode"] = constants.MotorboatAutopilotMode(boat_data["autopilot_mode"]).name

            return (
                "Position: "
                f"{self.boat_data.get('position', self.fake_position)[0]:.8f}, "
                f"{self.boat_data.get('position', self.fake_position)[1]:.8f}\n"
                f"State: {self.boat_data.get('autopilot_mode', 'N/A')}\n"
                f"Connection Status: {connection_status.name}\n"
                f"Current Maneuver: {self.boat_data.get('full_autonomy_maneuver', 'N/A')}\n"
                f"Current Waypoint Index: {self.boat_data.get('current_waypoint_index') + 1 if isinstance(self.boat_data.get('current_waypoint_index'), int) else 'N/A'}\n"  # noqa: E501
                f"Velocity Vector: [{fix_formatting(self.boat_data.get('velocity_x', -69.420))}, {fix_formatting(self.boat_data.get('velocity_y', -69.420))}]\n"  # noqa: E501
                f"Speed: {fix_formatting(self.boat_data.get('speed'))} knots\n"
                f"Distance To Next WP: {fix_formatting(self.boat_data.get('distance_to_next_waypoint'))} meters\n"
                f"Heading: {fix_formatting(self.boat_data.get('heading', self.fake_heading))}°\n"
                f"Desired Rudder Angle: {fix_formatting(self.boat_data.get('desired_rudder_angle'))}°\n"
                f"Current Rudder Angle: {fix_formatting(self.boat_data.get('current_rudder_angle'))}°\n"
                f"Rudder Angle Error: {fix_formatting(self.boat_data.get('rudder_angle_error'))}°\n"
                f"Motor RPM: {fix_formatting(self.boat_data.get('rpm'))} RPM\n"
                f"Duty Cycle: {fix_formatting(self.boat_data.get('duty_cycle'))}%\n"
                f"Amp Hours Charged: {fix_formatting(self.boat_data.get('amp_hours_charged'))} Ah\n"
                f"VESC Current: {fix_formatting(self.boat_data.get('current_to_vesc'))} A\n"
                f"Motor Voltage: {fix_formatting(self.boat_data.get('voltage_to_motor'))} V\n"
                f"VESC Voltage: {fix_formatting(self.boat_data.get('voltage_to_vesc'))} V\n"
                f"Motor Wattage: {fix_formatting(self.boat_data.get('wattage_to_motor'))} W\n"
                f"VESC Online: {fix_formatting(self.boat_data.get('time_since_vesc_startup'))}\n"
                f"Motor Temperature: {fix_formatting(self.boat_data.get('motor_temperature'))} °C\n"
                f"VESC Temperature: {fix_formatting(self.boat_data.get('vesc_temperature'))} °C\n"
            )
        
        boat_data, connection_status = request_result
        self.boat_data = boat_data

        def draw_map_diagnostics(heading: float) -> None:
            """
            Draw diagnostics on the map, such as no sail zone and wind direction.
            
            Parameters
            ----------
            heading
                The heading of the boat, used to orient the diagnostics correctly on the map.
            """

            no_sail_zone_size_dict: dict[str, str | float] | None = constants.SM.read("current_autopilot_parameters").get(
                "no_sail_zone_size"
            )

            if no_sail_zone_size_dict is None:
                print("[Warning] `no_sail_zone_size` not found in current autopilot parameters, not drawing the no sail zone.")
                return

            no_sail_size: float = 0
            if "current" in no_sail_zone_size_dict:
                no_sail_size = no_sail_zone_size_dict["current"]
            else:
                no_sail_size = no_sail_zone_size_dict["default"]

            wind_direction: float | None = self.boat_data.get("true_wind_angle")
            if wind_direction is None:
                print("[Warning] `true_wind_angle` not found in boat data, defaulting to 0.")
                wind_direction = 0

            head = heading + wind_direction + 180  # opposite the direction of wind
            size = 0.2

            # don't think about it too hard
            x1: float = 2 + np.cos(np.deg2rad(head - no_sail_size / 2))
            y1: float = 2 - np.sin(np.deg2rad(head - no_sail_size / 2))
            x2: float = 2 + np.cos(np.deg2rad(head + no_sail_size / 2))
            y2: float = 2 - np.sin(np.deg2rad(head + no_sail_size / 2))

            no_go_path_shape: list[svg.PathData] = [
                svg.MoveTo(2, 2),
                svg.LineTo(x1, y1),
                svg.Arc(1, 1, 0, 0, 0, x2, y2),
                svg.LineTo(2, 2),
            ]
            no_go_html = svg.Path(d=no_go_path_shape, fill="#c9140a")

            tack_distance_dict: dict[str, str | float] | None = constants.SM.read("current_autopilot_parameters").get(
                "tack_distance"
            )

            if tack_distance_dict is None:
                print("[Warning] `tack_distance` not found in current autopilot parameters, not drawing the no sail zone.")
                return

            tack_distance = tack_distance_dict["current"] if "current" in tack_distance_dict else tack_distance_dict["default"]

            distance_to_waypoint: float | None = self.boat_data.get("distance_to_next_waypoint")
            if distance_to_waypoint is None:
                print("[Warning] `distance_to_next_waypoint` not found in boat data, defaulting to 200.")
                distance_to_waypoint = 200
            
            if tack_distance > distance_to_waypoint:
                # we can't draw the line!
                decision_zone_path: list[svg.PathData] = []
                distance_to_waypoint = 200
            else:
            
                # in radians
                decision_zone_size: float = np.rad2deg(
                    np.arcsin((tack_distance/distance_to_waypoint)*np.sin(np.deg2rad(no_sail_size/2)))
                )

                # don't think about it too hard
                x1: float = 2 + np.cos(np.deg2rad(head - (no_sail_size / 2 - decision_zone_size / 2)))
                y1: float = 2 - np.sin(np.deg2rad(head - (no_sail_size / 2 - decision_zone_size / 2)))
                x2: float = 2 + np.cos(np.deg2rad(head + (no_sail_size / 2 - decision_zone_size / 2)))
                y2: float = 2 - np.sin(np.deg2rad(head + (no_sail_size / 2 - decision_zone_size / 2)))

                decision_zone_path: list[svg.PathData] = [
                    svg.MoveTo(2, 2),
                    svg.LineTo(x1, y1),
                    svg.Arc(1, 1, 0, 0, 0, x2, y2),
                    svg.LineTo(2, 2),
                ]

            decision_zone_html = svg.Path(d=decision_zone_path, fill="pink")

            wind_direction_shape: list[svg.PathData] = [
                svg.MoveTo(1, 1),
                svg.LineTo(
                    1 + np.cos(np.deg2rad(heading + wind_direction)),
                    1 - np.sin(np.deg2rad(heading + wind_direction)),
                ),
            ]
            wind_html = svg.Path(
                d=wind_direction_shape,
                stroke="orange",
                stroke_width="0.1",
            )

            speed: float | None = self.boat_data.get("speed")
            if speed is None:
                print("[Warning] `speed` not found in boat data, defaulting to 1e-3.")
                speed = 1e-3

            elif np.isclose(speed, 0.0, rtol=1e-5, atol=1e-8):
                print("[Warning] `speed` is very close to 0, defaulting to 1e-3 to avoid division by zero.")
                speed = 1e-3
            
            vx: float = self.boat_data.get("velocity_x", -69.420)
            vy: float = self.boat_data.get("velocity_y", -69.420)

            radius: float = 4*speed
            x1: float = 2 + radius * vx / speed
            y1: float = 2 + radius * vy / speed
            head = heading

            velocity_arrow_shape: list[svg.PathData] = [
                svg.MoveTo(2, 2),
                svg.LineTo(x1, y1)
            ]
            velocity_arrow_transform: list[svg.Transform] = [
                svg.Rotate(-head, 2, 2),
            ]
            velocity_html = svg.Path(
                d=velocity_arrow_shape,
                stroke="black",
                stroke_width="0.1",
                transform=velocity_arrow_transform
            )

            svg_str = no_go_html.as_str() + decision_zone_html.as_str()
            self.browser.page().runJavaScript(
                f"map.update_no_sail_svg('{svg_str}', {size})"
            )
            self.browser.page().runJavaScript(
                f"map.update_velocity_svg('{velocity_html.as_str()}', '{size}')"
            )
            self.browser.page().runJavaScript(
                f"map.update_wind_svg('{wind_html.as_str()}')"
            )

        try:
            heading = self.boat_data.get("heading")
            assert isinstance(heading, (float, int)), "heading is not a number."

        except AssertionError:
            heading = self.fake_heading

        try:
            lat = self.boat_data.get("latitude")
            assert isinstance(lat, (float, int)), "latitude is not a number."

            lon = self.boat_data.get("longitude")
            assert isinstance(lon, (float, int)), "longitude is not a number."

            self.boat_data.setdefault("position", (lat, lon))

        except AssertionError:
            lat, lon = self.fake_position

        if constants.SM.read("has_telemetry_server_instance_changed"):
            constants.SM.write("remote_autopilot_param_hash", "")
            self.clear_waypoints()
            constants.SM.write("has_telemetry_server_instance_changed", False)

        self.browser.page().runJavaScript(f"map.update_boat_location_and_heading({lat}, {lon}, {heading})")

        if self.sailboat_debugging_symbols_status:
            draw_map_diagnostics(heading)

        if "full_autonomy_maneuver" in self.boat_data:
            telemetry_text = sailboat_mode(boat_data)
        
        elif "rpm" in self.boat_data:
            telemetry_text = motorboat_mode(boat_data)

        else:
            telemetry_text = "Boat data received, but could not determine boat type."

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
            The converted float value, or ``0`` if conversion fails.
        """

        try:
            return float(x)
        except ValueError:
            return 0

    # endregion helper functions
