import json
import os
import time
from functools import partial
from pathlib import Path
from typing import Any, Literal
from urllib.parse import urljoin

from qtpy.QtCore import Qt, Signal
from qtpy.QtWebEngineWidgets import QWebEngineView
from qtpy.QtWidgets import (
    QCheckBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QLabel,
    QMessageBox,
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
        self.browser.setHtml(constants.HTML_MAP)
        self.browser.setMinimumWidth(700)
        self.browser.setMinimumHeight(700)

        self.waypoints_checker_toggle = QCheckBox("Enable popup when waypoints change?")
        self.waypoints_checker_toggle.setChecked(False)
        self.waypoints_checker_toggle.setToolTip(
            "If enabled, a popup will appear when the waypoints on the telemetry server change.",
        )
        self.waypoints_checker_toggle.stateChanged.connect(
            lambda state: setattr(self, "waypoints_checker_status", state == Qt.CheckState.Checked),
        )

        self.middle_layout.addWidget(self.browser, 0, 1)
        self.middle_layout.setRowStretch(0, 1)
        self.middle_layout.addWidget(self.waypoints_checker_toggle, 1, 1, Qt.AlignCenter)
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
                constants.REQ_SESSION.post(
                    urljoin(
                        constants.TELEMETRY_SERVER_ENDPOINTS["set_waypoints"],
                        str(constants.TELEMETRY_SERVER_INSTANCE_ID),
                    ),
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
                        constants.TELEMETRY_SERVER_ENDPOINTS["waypoints_test"],
                        str(constants.TELEMETRY_SERVER_INSTANCE_ID),
                    ),
                    json=self.waypoints,
                )

            except RequestException as e:
                print(f"[Error] Failed to send waypoints: {e}\nWaypoints: {self.waypoints}")

    def pull_waypoints(self) -> None:
        """Pull waypoints from the telemetry server and add them to the map."""

        try:
            remote_waypoints: list[list[float]] = constants.REQ_SESSION.get(
                urljoin(constants.TELEMETRY_SERVER_ENDPOINTS["get_waypoints"], str(constants.TELEMETRY_SERVER_INSTANCE_ID))
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
            <li> Otherwise, the value is rounded to 5 decimal places.
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

            return "N/A" if data_item is None else f"{float(data_item):.5f}"

        boat_data, connection_status = request_result
        self.boat_data = boat_data

        try:
            heading = self.boat_data.get("heading")
            assert isinstance(heading, (float, int)), "heading is not a number."

        except AssertionError:
            heading = self.fake_heading

        try:
            position = self.boat_data.get("position")
            assert isinstance(position, list), "position is not a list."
            assert len(position) == 2, "position does not have length 2."
            lat, lon = position
            assert isinstance(lat, (float, int)), "latitude is not a number."
            assert isinstance(lon, (float, int)), "longitude is not a number."

        except AssertionError:
            lat, lon = self.fake_position

        if constants.HAS_TELEMETRY_SERVER_INSTANCE_CHANGED:
            self.clear_waypoints()
            constants.HAS_TELEMETRY_SERVER_INSTANCE_CHANGED = False

        self.browser.page().runJavaScript(f"map.update_boat_location_and_heading({lat}, {lon}, {heading})")

        telemetry_text = (
            "Position: "
            f"{self.boat_data.get('position', self.fake_position)[0]:.8f}, "
            f"{self.boat_data.get('position', self.fake_position)[1]:.8f}\n"
            f"State: {self.boat_data.get('state', 'N/A')}\n"
            f"Connection Status: {connection_status.name}\n"
            f"Current Maneuver: {self.boat_data.get('full_autonomy_maneuver', 'N/A')}\n"
            f"Velocity Vector: [{self.boat_data.get('velocity_vector', [-69.420, -69.420])[0]:.5f}, {self.boat_data.get('velocity_vector', [-69.420, -69.420])[1]:.5f}]\n"  # noqa: E501
            f"Speed: {fix_formatting(self.boat_data.get('speed'))} knots\n"
            f"Distance To Next WP: {fix_formatting(self.boat_data.get('distance_to_next_waypoint'))} meters\n"
            f"Bearing: {fix_formatting(self.boat_data.get('bearing'))}°\n"
            f"Heading: {fix_formatting(self.boat_data.get('heading', self.fake_heading))}°\n"
            f"True Wind Speed: {fix_formatting(self.boat_data.get('true_wind_speed'))} knots\n"
            f"True Wind Angle: {fix_formatting(self.boat_data.get('true_wind_angle'))}°\n"
            f"Apparent Wind Speed: {fix_formatting(self.boat_data.get('apparent_wind_speed'))} knots\n"
            f"Apparent Wind Angle: {fix_formatting(self.boat_data.get('apparent_wind_angle'))}°\n"
            f"Sail Angle: {fix_formatting(self.boat_data.get('sail_angle'))}°\n"
            f"Rudder Angle: {fix_formatting(self.boat_data.get('rudder_angle'))}°\n"
            f"Current Waypoint Index: {self.boat_data.get('current_waypoint_index') + 1 if isinstance(self.boat_data.get('current_waypoint_index'), int) else 'N/A'}\n"  # noqa: E501
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
            The converted float value, or ``0`` if conversion fails.
        """

        try:
            return float(x)
        except ValueError:
            return 0

    # endregion helper functions
