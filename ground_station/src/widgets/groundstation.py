import gzip
import json
import os
import time
from pathlib import Path
from typing import Any, Literal, cast
from urllib.parse import urljoin

import numpy as np
import svg
from requests.exceptions import RequestException

from qtpy.QtCore import QEvent, QObject, QSize, Qt, QTimer, Signal, Slot
from qtpy.QtGui import QKeyEvent, QKeySequence, QShowEvent
from qtpy.QtWebEngineWidgets import QWebEngineView
from qtpy.QtWidgets import (
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QLabel,
    QMessageBox,
    QPushButton,
    QShortcut,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from utils import TextEditWindow, constants, misc, thread_classes
from utils.console_logger import get_logger
from utils.constants import StrictMatchEnums
from utils.dialog_templates import CoordinateInputDialog, InputDialog, show_message_box
from utils.syntax_highlighters import JsonHighlighter

from .easter_eggs import PongDialog, SnakeDialog, TetrisDialog
from .keybind_widget import (
    KeybindConfigDialog,
    get_keybind_manager,
    normalize_key_string,
    qt_key_event_to_string,
)
from .map_widget import MapBridge, MapOptionsHandler
from .map_widget.land_click_prompt import LAND_CLICK_PROMPT

logger = get_logger(__name__)

MotorboatControlModes = StrictMatchEnums.MotorboatControlModes
SailboatAutopilotStates = StrictMatchEnums.SailboatAutopilotStates
SailboatControlModes = StrictMatchEnums.SailboatControlModes


class GroundStationWidget(QWidget):
    """
    Main widget for the ground station application.

    Parameters
    ----------
    boat_status_source
        A :class:`Signal` that provides boat status updates.

    Attributes
    ----------
    refresh_autopilot_config_signal
        Signal emitted when the autopilot configuration needs to be refreshed.

    Inherits
    --------
    :class:`QWidget`
    """

    refresh_autopilot_config_signal = Signal(bool)

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

        # do we need to clear the sailboat diagnostics svgs on the next telemetry update?
        self.need_to_clear_diagnostics: bool = False

        # should we remember the status of the user's last response to the
        # dialog that asks if the telemetry server URL should be changed?
        self.remember_telemetry_server_url_status: bool = False

        # should we remember the status of the user's last response to the
        # dialog that asks if the user wants to pull waypoints from the telemetry server?
        self.remember_waypoints_pull_service_status: bool = False

        # region timers
        self.one_ms_timer = misc.copy_qtimer(constants.ONE_MS_TIMER)
        self.thirty_second_timer = misc.copy_qtimer(constants.THIRTY_SECOND_TIMER)
        self.timers = [self.one_ms_timer, self.thirty_second_timer]
        # endregion timers

        # region define layouts
        self.main_layout = QGridLayout()
        self.main_layout.setObjectName("main_layout")

        self.left_width = 300
        self.left_layout = QVBoxLayout()
        self.left_layout.setObjectName("left_layout")
        self.left_widget = QWidget()

        self.middle_width_min = 2 * self.left_width
        self.middle_width_max = 4 * self.left_width
        self.middle_layout = QGridLayout()
        self.middle_layout.setObjectName("middle_layout")

        self.right_width = self.left_width + 30
        self.right_layout = QTabWidget()
        self.right_layout.setObjectName("right_layout")
        self.right_tab1_layout = QGridLayout()
        self.right_tab2_layout = QGridLayout()
        self.right_tab1 = QWidget()
        self.right_tab2 = QWidget()

        self.setMaximumWidth(self.left_width + self.middle_width_max + self.right_width)
        # endregion define layouts

        # region setup UI
        # region left section
        self.left_label = QLabel("Telemetry Data")
        self.left_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.left_text_section = QTextEdit()
        self.left_text_section.highlighter = JsonHighlighter(self.left_text_section.document())
        self.left_text_section.setReadOnly(True)
        self.left_text_section.setText("Awaiting telemetry data...")

        self.start_data_logging_button = misc.pushbutton_maker(
            "Start Data Logging",
            self.start_data_logging,
            constants.ICONS.play_circle_outline,
            max_width=self.left_width,
            min_height=50,
        )
        self.start_data_logging_button.setIconSize(QSize(20, 20))

        self.stop_data_logging_button = misc.pushbutton_maker(
            "End Data Logging",
            self.stop_data_logging,
            constants.ICONS.stop_circle_outline,
            max_width=self.left_width,
            min_height=50,
        )
        self.stop_data_logging_button.setIconSize(QSize(20, 20))

        self.left_button_groupbox = QGroupBox()
        self.left_button_layout = QGridLayout()

        self.left_button_layout.addWidget(self.start_data_logging_button, 0, 0)
        self.left_button_layout.addWidget(self.stop_data_logging_button, 1, 0)
        self.left_button_groupbox.setLayout(self.left_button_layout)

        self.left_layout.addWidget(self.left_label)
        self.left_layout.addWidget(self.left_text_section)
        self.left_layout.addWidget(self.left_button_groupbox)

        self.left_widget.setLayout(self.left_layout)
        self.left_widget.setFixedWidth(self.left_width)
        self.main_layout.addWidget(self.left_widget, 0, 0)

        # endregion left section

        # region middle sections
        self.browser = QWebEngineView()
        self.browser.setPage(constants.MAP_PAGE)

        self.browser.setMinimumWidth(self.middle_width_min)
        self.browser.setMaximumWidth(self.middle_width_max)

        self.middle_layout.addWidget(self.browser, 0, 1)
        self.middle_layout.setRowStretch(0, 1)

        self.map_bridge = MapBridge(self.browser)
        QTimer.singleShot(0, self.map_bridge.verify_api)

        LAND_CLICK_PROMPT.agreement_requested.connect(self._handle_land_click_prompt, Qt.ConnectionType.QueuedConnection)

        self.middle_button_groupbox = QGroupBox()
        self.middle_button_layout = QGridLayout()

        self.edit_telemetry_config_window = MapOptionsHandler(self.on_map_feature_toggled)
        self.telemetry_config_button = QPushButton("Map Appearance Configuration")
        self.telemetry_config_button.setToolTip(
            "If enabled, a popup will appear where you can alter the telemetry configuration.",
        )
        self.telemetry_config_button.clicked.connect(self.edit_telemetry_config_window.exec)

        self.keybind_config_window = KeybindConfigDialog()
        self.keybind_config_button = QPushButton("Keybind Configuration")
        self.keybind_config_button.setToolTip("View and edit keyboard shortcuts.")
        self.keybind_config_button.clicked.connect(self.keybind_config_window.exec)

        self.tetris_window = TetrisDialog()
        self.snake_window = SnakeDialog()
        self.pong_window = PongDialog()

        self.test_waypoint_rng = np.random.default_rng(69420)
        self.add_500_test_waypoints_button = QPushButton("Add 500 Test Waypoints?")
        self.add_500_test_waypoints_button.clicked.connect(self.add_500_test_waypoints)

        self.manual_waypoint_button = QPushButton("Add Waypoint by Coordinates")
        self.manual_waypoint_button.setToolTip("Manually enter latitude and longitude to add a waypoint.")
        self.manual_waypoint_button.clicked.connect(self.add_manual_waypoint)

        self.middle_button_layout.addWidget(self.telemetry_config_button, 0, 0)
        self.middle_button_layout.addWidget(self.keybind_config_button, 0, 1)
        self.middle_button_layout.addWidget(self.add_500_test_waypoints_button, 0, 2)
        self.middle_button_layout.addWidget(self.manual_waypoint_button, 0, 3)
        self.middle_button_groupbox.setLayout(self.middle_button_layout)

        self.middle_layout.addWidget(self.middle_button_groupbox, 1, 1, Qt.AlignmentFlag.AlignCenter)
        self.middle_layout.setRowStretch(1, 0)
        self.main_layout.addLayout(self.middle_layout, 0, 1)
        # endregion middle section

        # region right section
        # region tab1: waypoint data
        self.right_tab1_label = QLabel("Waypoints")
        self.right_tab1_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.right_tab1_table = QTableWidget()
        self.right_tab1_table.setMinimumWidth(self.right_width - 20)
        self.right_tab1_table.cellClicked.connect(lambda row, _column: self.zoom_to_marker(row, table="waypoints"))
        self.can_send_waypoints = True
        self.send_waypoints_button = misc.pushbutton_maker(
            "Send Waypoints",
            self.send_waypoints,
            constants.ICONS.upload,
            max_width=self.right_width // 2,
            min_height=50,
            is_clickable=self.can_send_waypoints,
        )

        self.can_reset_waypoints = False
        self.clear_waypoints_button = misc.pushbutton_maker(
            "Clear Waypoints",
            self.clear_waypoints,
            constants.ICONS.delete,
            max_width=self.right_width // 2,
            min_height=50,
            is_clickable=self.can_reset_waypoints,
        )

        self.can_pull_waypoints = True
        self.pull_waypoints_button = misc.pushbutton_maker(
            "Pull Waypoints",
            self.pull_waypoints,
            constants.ICONS.download,
            max_width=self.right_width // 2,
            min_height=50,
            is_clickable=self.can_pull_waypoints,
        )

        self.focus_boat_button = misc.pushbutton_maker(
            "Zoom to Boat",
            self.zoom_to_boat,
            constants.ICONS.boat,
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
        self.right_tab2_table.setMinimumWidth(self.right_width - 20)
        self.right_tab2_table.cellClicked.connect(lambda row, _column: self.zoom_to_marker(row, table="buoys"))

        self.edit_buoy_data_button = misc.pushbutton_maker(
            "Edit Buoy Data",
            self.edit_buoy_data,
            constants.ICONS.cog,
            max_width=self.right_width,
            min_height=50,
        )

        self.save_buoy_data_button = misc.pushbutton_maker(
            "Save Buoy Data",
            self.save_buoy_data,
            constants.ICONS.save,
            max_width=self.right_width // 2,
            min_height=50,
        )

        self.load_buoy_data_button = misc.pushbutton_maker(
            "Load Buoy Data",
            self.load_buoy_data,
            constants.ICONS.hard_drive,
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
        self.right_layout.setFixedWidth(self.right_width)
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

        self.boat_status_source = boat_status_source
        self.boat_status_source.connect(self.update_telemetry_display)

        # region keybinds
        self._keybind_manager = get_keybind_manager()
        self._shortcuts: dict[str, QShortcut] = {}

        self._keybind_manager.register_handler("open_keybind_config", self.keybind_config_window.exec)
        self._keybind_manager.register_handler("pull_waypoints", self.pull_waypoints)
        self._keybind_manager.register_handler("send_waypoints", self.send_waypoints)
        self._keybind_manager.register_handler("toggle_data_logging", self.toggle_data_logging)
        self._keybind_manager.register_handler("undo_waypoint", self._trigger_undo_waypoint)
        self._keybind_manager.register_handler("open_tetris", self._show_tetris)
        self._keybind_manager.register_handler("open_snake", self._show_snake)
        self._keybind_manager.register_handler("open_pong", self._show_pong)

        self._rebuild_shortcuts()
        self._push_map_keybinds()

        self._keybind_manager.bindings_changed.connect(self._on_keybinds_changed)
        # endregion keybinds

        # we need to "install" an event filter on the QWebEngineView's internal
        # Chromium render widget so we can intercept Ctrl+Z before it swallows it
        self.browser.installEventFilter(self)
        QTimer.singleShot(0, self._install_render_widget_filter)

    # region keybind functions

    def _rebuild_shortcuts(self) -> None:
        """Recreate every app-scope :class:`QShortcut` from the current bindings."""

        for shortcut in self._shortcuts.values():
            shortcut.setEnabled(False)
            shortcut.deleteLater()

        self._shortcuts.clear()
        for action, info in self._keybind_manager.get_actions_by_scope("app").items():
            key = info.get("key")
            if not key:
                continue

            sequence = QKeySequence(key, QKeySequence.SequenceFormat.PortableText)
            if sequence.isEmpty():
                continue

            shortcut = QShortcut(sequence, self)
            shortcut.setContext(Qt.ShortcutContext.WindowShortcut)
            shortcut.activated.connect(lambda _checked=False, a=action: self._keybind_manager.trigger(a))
            self._shortcuts[action] = shortcut

        undo_info = self._keybind_manager.get_binding("undo_waypoint")
        if undo_info and undo_info.get("key"):
            sequence = QKeySequence(undo_info["key"], QKeySequence.SequenceFormat.PortableText)
            if not sequence.isEmpty():
                shortcut = QShortcut(sequence, self)
                shortcut.setContext(Qt.ShortcutContext.WindowShortcut)
                shortcut.activated.connect(lambda _checked=False: self._keybind_manager.trigger("undo_waypoint"))
                self._shortcuts["undo_waypoint"] = shortcut

    def _push_map_keybinds(self) -> None:
        """Push the current map-scope bindings into the TS frontend."""

        frontend_bindings = self._keybind_manager.to_frontend_dict()
        self.map_bridge.set_keybinds(frontend_bindings)

    def _trigger_undo_waypoint(self) -> None:
        """Trigger the undo waypoint action in the TS frontend."""

        self.map_bridge.undo_last_waypoint()

    # region easter egg functions
    def _show_tetris(self) -> None:
        """Show the hidden Tetris easter egg."""

        self.tetris_window.show()
        self.tetris_window.raise_()
        self.tetris_window.activateWindow()
        self.tetris_window._board.setFocus()

    def _show_snake(self) -> None:
        """Show the hidden Snake easter egg."""

        self.snake_window.show()
        self.snake_window.raise_()
        self.snake_window.activateWindow()
        self.snake_window._board.setFocus()

    def _show_pong(self) -> None:
        """Show the hidden Pong easter egg."""

        self.pong_window.show()
        self.pong_window.raise_()
        self.pong_window.activateWindow()
        self.pong_window._board.setFocus()

    # endregion easter egg functions

    def _install_render_widget_filter(self) -> None:
        """Install the event filter on the focus policy of the :class:`QWebEngineView`'s internal Chromium render widget."""

        proxy = self.browser.focusProxy()
        if proxy is not None:
            proxy.installEventFilter(self)

    def eventFilter(self, obj: QObject, event: QEvent) -> bool:
        """
        Wrapper around :meth:`_handle_undo_keypress` to intercept Ctrl+Z keypresses.

        Note
        ----
        This method exists to satisfy the Qt event filter interface.
        See :meth:`_handle_undo_keypress` for details on why this is necessary.

        Parameters
        ----------
        obj
            The object that received the event.
        event
            The event that was received.

        Returns
        -------
        `bool`
            `True` if the event was consumed, `False` otherwise.
        """

        if self._handle_undo_keypress(event):
            return True

        return super().eventFilter(obj, event)

    def _handle_undo_keypress(self, event: QEvent) -> bool:
        """
        Intercept Ctrl+Z keypresses and trigger the undo waypoint action.

        We need to do this because the QWebEngineView's internal Chromium render widget
        swallows Ctrl+Z keypresses before they reach the TS frontend, so we have to
        handle it in Python and trigger the action manually.

        Returns
        -------
        `bool`
            `True` if the event was consumed, `False` otherwise.
        """

        # only care about key presses that include the Control modifier — the
        # undo binding is always a Ctrl combo, so skip everything else early
        if not (event.type() == QEvent.Type.KeyPress and event.modifiers() & Qt.KeyboardModifier.ControlModifier):
            return False

        event = cast("QKeyEvent", event)
        combo = qt_key_event_to_string(event)
        if not combo:
            return False

        undo_info = self._keybind_manager.get_binding("undo_waypoint")
        if not (undo_info and undo_info.get("key")):
            return False

        configured = normalize_key_string(undo_info["key"])
        if combo.lower() != configured.lower():
            return False

        self._trigger_undo_waypoint()
        return True

    @Slot(dict)
    def _on_keybinds_changed(self, _bindings: dict) -> None:
        """
        Handle the ``bindings_changed`` signal from the keybind manager.

        Parameters
        ----------
        _bindings
            The new keybinds dictionary. This parameter is unused because we
            always fetch the latest bindings from the keybind manager directly.
        """

        self._rebuild_shortcuts()
        self._push_map_keybinds()

    @Slot()
    def toggle_data_logging(self) -> None:
        """Toggle telemetry data logging on or off based on current state."""

        if constants.SM.read_bool("data_logging_active"):
            self.stop_data_logging()
        else:
            self.start_data_logging()

    # endregion keybind functions

    # region focus handling

    def showEvent(self, event: QShowEvent) -> None:
        """
        Give the map webview keyboard focus when the widget is shown.

        The TS frontend's ``keydown`` listener (which dispatches map-scope
        keybinds like `f` or `c`) only fires when the :class:`QWebEngineView`
        has focus. Without this, the user would have to click on the map
        before any map-scope keybind works.

        Parameters
        ----------
        event
            The show event forwarded to the parent implementation.
        """

        super().showEvent(event)

        # defer the focus request to the next event loop tick so the webview
        # has fully finished laying out before we steal focus into it
        QTimer.singleShot(0, self.browser.setFocus)

    # endregion focus handling

    # region button functions

    @Slot()
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
                instance_id = constants.SM.read_int("telemetry_server_instance_id")
                constants.REQ_SESSION.post(
                    urljoin(misc.get_route("set_waypoints"), str(instance_id)),
                    json=self.waypoints,
                )

                self.map_bridge.change_color_waypoints("red")
                logger.info(f"Waypoints sent successfully. Waypoints: {self.waypoints}")

            except RequestException as e:
                logger.error(f"Failed to send waypoints: {e}\nWaypoints: {self.waypoints}")

        else:
            try:
                constants.REQ_SESSION.post(
                    urljoin(
                        constants.SM.read_str("test_waypoints"),
                        str(constants.SM.read_int("telemetry_server_instance_id")),
                    ),
                    json=self.waypoints,
                )

            except RequestException as e:
                logger.error(f"Failed to send waypoints: {e}\nWaypoints: {self.waypoints}")

    @Slot()
    def pull_waypoints(self) -> None:
        """Pull waypoints from the telemetry server and add them to the map."""

        try:
            instance_id = constants.SM.read_int("telemetry_server_instance_id")
            remote_waypoints: list[list[float]] = constants.REQ_SESSION.get(
                urljoin(misc.get_route("get_waypoints"), str(instance_id)),
            ).json()

            if remote_waypoints:
                if len(remote_waypoints) > 10:
                    logger.info(
                        f"Pulled {len(remote_waypoints)} waypoints from server. "
                        f"Displaying first 10 waypoints: {remote_waypoints[:10]}"
                    )
                else:
                    logger.info(f"Fetched waypoints from server: {remote_waypoints}")

                existing_waypoints = self.waypoints.copy()
                self.map_bridge.clear_waypoints()

                for waypoint in remote_waypoints:
                    self.map_bridge.add_waypoint(waypoint[0], waypoint[1])

                self.map_bridge.change_color_waypoints("red")

                for waypoint in existing_waypoints:
                    self.map_bridge.add_waypoint(waypoint[0], waypoint[1])

            else:
                logger.warning("No waypoints found on the server.")

            self.can_pull_waypoints = False
            self.pull_waypoints_button.setDisabled(not self.can_pull_waypoints)

        except RequestException as e:
            logger.error(f"Failed to pull waypoints. Exception: {e}")

    @Slot()
    def clear_waypoints(self) -> None:
        """Clear waypoints from the table."""

        self.can_reset_waypoints = False
        self.can_pull_waypoints = True
        self.pull_waypoints_button.setDisabled(not self.can_pull_waypoints)
        self.map_bridge.clear_waypoints()

    @Slot()
    def add_500_test_waypoints(self) -> None:
        """Add 500 test waypoints to the map."""

        for _ in range(500):
            latitude = self.test_waypoint_rng.uniform(-90, 90)
            longitude = self.test_waypoint_rng.uniform(-180, 180)
            self.map_bridge.add_waypoint(latitude, longitude)

        logger.info("Added 500 test waypoints to the map, LOL.")

    @Slot()
    def add_manual_waypoint(self) -> None:
        """Open a coordinate-entry dialog and add a waypoint at the entered location."""

        dialog = CoordinateInputDialog(self)
        if dialog.exec() != CoordinateInputDialog.DialogCode.Accepted:
            return

        try:
            latitude, longitude = dialog.get_coordinates()
        except ValueError as exc:
            show_message_box(title="Invalid Coordinates", message=str(exc))
            return

        self.map_bridge.add_waypoint(latitude, longitude)
        logger.info(f"Manually added waypoint at ({latitude}, {longitude}).")

    @Slot(float, float)
    def _handle_land_click_prompt(self, latitude: float, longitude: float) -> None:
        """
        Ask the user to confirm adding a waypoint that falls on land.

        Runs on the Qt main thread; invoked via the queued connection from
        :attr:`land_click_prompt.LAND_CLICK_PROMPT` when a map click lands on
        land. The resulting answer is posted back to the blocked HTTP thread
        via :meth:`land_click_prompt.LandClickPrompt.answer`.

        Parameters
        ----------
        latitude
            Latitude of the clicked point.
        longitude
            Longitude of the clicked point.
        """

        response = show_message_box(
            title="Waypoint on Land",
            message=f"The point ({latitude:.5f}, {longitude:.5f}) appears to be on land. Add the waypoint anyway?",
            icon=constants.ICONS.warning,
            buttons=[QMessageBox.StandardButton.Yes, QMessageBox.StandardButton.No],
        )

        LAND_CLICK_PROMPT.answer(response == QMessageBox.StandardButton.Yes)

    @Slot()
    def start_data_logging(self) -> None:
        """Start logging telemetry data to a file."""

        data_log_file = Path(constants.DATA_LOGS_DIR / f"data_log_{time.time_ns()}.csv")
        data_log_file.touch(exist_ok=True)

        constants.SM.write("data_log_file_path", data_log_file.as_posix())
        constants.SM.write("data_logging_active", True)
        constants.DL.start()
        self.boat_status_source.connect(constants.DL.write_from_qthread)

        self.start_data_logging_button.setDisabled(True)
        self.stop_data_logging_button.setDisabled(False)

        logger.info("Data logging started.")

    @Slot()
    def stop_data_logging(self) -> None:
        """Stop logging telemetry data to a file."""

        constants.SM.write("data_logging_active", False)
        self.boat_status_source.disconnect(constants.DL.write_from_qthread)
        constants.DL.stop()

        self.start_data_logging_button.setDisabled(False)
        self.stop_data_logging_button.setDisabled(True)

        non_compressed_path = Path(constants.SM.read_str("data_log_file_path"))
        file_size = os.path.getsize(non_compressed_path) / (1024 * 1024)

        if file_size > 20:
            compressed_file_path = Path(constants.SM.read_str("data_log_file_path").replace(".csv", ".csv.gz"))

            with open(non_compressed_path, "rb") as f_in, gzip.open(compressed_file_path, mode="wb", compresslevel=9) as f_out:
                f_out.writelines(f_in)

            compressed_file_size = os.path.getsize(compressed_file_path) / (1024 * 1024)
            file_size_percent_difference = (file_size - compressed_file_size) / file_size * 100
            logger.info(
                f"Data logging stopped. Log file compressed to {compressed_file_path}, "
                f"reduced file size by {file_size_percent_difference:.2f}%."
            )

    @Slot()
    def edit_buoy_data(self) -> None:
        """
        Opens a text edit window to edit the buoy data.

        ``self.edit_buoy_data_callback`` is called when the user closes or clicks the save button in the text edit window.
        ``self.edit_buoy_data_callback`` recieves the text from the text edit window when the user clicks the save button,
        otherwise it recieves the text without any changes.
        """

        try:
            buoy_json = json.dumps(self.buoys, indent=4)
            if hasattr(self, "text_edit_window") and self.text_edit_window is not None:
                self.text_edit_window.close()
            self.text_edit_window = TextEditWindow(highlighter=JsonHighlighter, initial_text=buoy_json)
            self.text_edit_window.setWindowTitle("Edit Buoy GPS Coordinates")
            self.text_edit_window.user_text_emitter.connect(self.edit_buoy_data_callback)
            self.text_edit_window.show()

        except Exception as e:
            logger.error(f"Failed to open buoy data edit window: {e}")

    @Slot(str)
    def edit_buoy_data_callback(self, text: str) -> None:
        """
        Callback function for :meth:`edit_buoy_data`.

        This function is called when the user closes the text edit window.
        It retrieves the edited text and saves it to the ``self.buoys`` variable and closes the window.

        Parameters
        ----------
        text
            The text entered by the user in the text edit window.
        """

        try:
            edited_buoys = json.loads(text)
            if self.buoys != edited_buoys:
                self.buoys = edited_buoys
                self.update_buoy_table()

        except Exception as e:
            logger.error(f"Failed to edit buoy data: {e}")

    def update_buoy_table(self) -> None:
        """Update the buoy table with the latest buoy data."""

        self.right_tab2_table.clear()
        self.right_tab2_table.setRowCount(0)
        self.right_tab2_table.setColumnCount(2)
        self.right_tab2_table.setHorizontalHeaderLabels(["Latitude", "Longitude"])

        self.map_bridge.clear_buoys()

        for buoy in self.buoys:
            self.right_tab2_table.insertRow(self.right_tab2_table.rowCount())
            self.map_bridge.add_buoy(self.buoys[buoy]["lat"], self.buoys[buoy]["lon"])

            for i, coord in enumerate(["lat", "lon"]):
                item = QTableWidgetItem(f"{float(self.buoys[buoy][coord]):.13f}")
                item.setFlags(Qt.ItemFlag.ItemIsEnabled)
                self.right_tab2_table.setItem(self.right_tab2_table.rowCount() - 1, i, item)

        self.right_tab2_table.resizeColumnsToContents()
        self.right_tab2_table.resizeRowsToContents()

    @Slot()
    def save_buoy_data(self) -> None:
        """
        Saves latest entry in the ``self.buoys`` array to a file.

        Files are stored in the `buoy_data` directory and are named `buoy_data_<timestamp>.json`
        where `<timestamp>` is nanoseconds since unix epoch.
        """

        try:
            file_path = Path(constants.BUOY_DATA_DIR / f"buoy_data_{time.time_ns()}.json")
            with open(file_path, mode="w", encoding="utf-8") as f:
                json.dump(self.buoys, f, indent=4)
            logger.info(f"Buoy data saved to {file_path}")

        except Exception as e:
            logger.error(f"Failed to save buoy data: {e}")

    @Slot()
    def load_buoy_data(self) -> None:
        """
        Load buoy data from the `buoy_data` directory, if none selected use `default.json`.

        Files are stored in the `buoy_data` directory and are named `buoy_data_<timestamp>.json`
        where `<timestamp>` is nanoseconds since unix epoch.
        """

        try:
            buoy_files = os.listdir(constants.BUOY_DATA_DIR)
            if not buoy_files:
                logger.warning("No buoy data files found.")

            else:
                chosen_file = QFileDialog.getOpenFileName(
                    self,
                    "Select Buoy Data File",
                    constants.BUOY_DATA_DIR.as_posix(),
                    "*.json",
                )
                if chosen_file == ("", ""):
                    chosen_file_path = Path(constants.BUOY_DATA_DIR / "default.json")
                else:
                    chosen_file_path = Path(chosen_file[0])

                with open(chosen_file_path, mode="r", encoding="utf-8") as f:
                    self.buoys = json.load(f)

                self.update_buoy_table()
                logger.info(f"Buoy data loaded from {chosen_file_path}")

        except Exception as e:
            logger.error(f"Failed to load buoy data: {e}")

    @Slot()
    def zoom_to_boat(self) -> None:
        """Center the view on the boat's position."""

        self.map_bridge.focus_map_on_boat()

    def on_map_feature_toggled(self, feature: str, enabled: bool) -> None:
        """
        Handle a map feature toggle from the Map Appearance Configuration dialog.

        Parameters
        ----------
        feature
            The feature key that was toggled.
        enabled
            Whether the feature was enabled or disabled.
        """

        if feature == "boat_track":
            self.map_bridge.set_track_visible(enabled)
        elif feature == "bathymetry":
            self.map_bridge.set_bathymetry_visible(enabled)

    @Slot(int, str)
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
                    item_lat = self.right_tab1_table.item(row, 0)
                    item_lon = self.right_tab1_table.item(row, 1)
                    if item_lat is None or item_lon is None:
                        raise ValueError(f"Missing cell data at row {row}")

                    approx_lat = float(item_lat.text())
                    approx_lon = float(item_lon.text())

                    lat, lon = None, None
                    for waypoint in self.waypoints:
                        if abs(waypoint[0] - approx_lat) < 1e-6 and abs(waypoint[1] - approx_lon) < 1e-6:
                            lat, lon = waypoint
                            break

                    if lat is not None and lon is not None:
                        logger.info(f"Zooming to waypoint at ({lat}, {lon})")
                        self.map_bridge.focus_map_on_marker(lat, lon)
                    else:
                        logger.warning(f"Waypoint not found for coordinates ({approx_lat}, {approx_lon})")

                except (ValueError, TypeError) as e:
                    logger.error(f"Invalid waypoint data: {e}")
            else:
                logger.warning("No waypoints available to zoom to.")

        elif table == "buoys":
            if self.right_tab2_table.rowCount() > 0:
                try:
                    item_lat = self.right_tab2_table.item(row, 0)
                    item_lon = self.right_tab2_table.item(row, 1)
                    if item_lat is None or item_lon is None:
                        raise ValueError(f"Missing cell data at row {row}")

                    approx_lat = float(item_lat.text())
                    approx_lon = float(item_lon.text())

                    lat, lon = None, None
                    for buoy in self.buoys.values():
                        if abs(buoy["lat"] - approx_lat) < 1e-6 and abs(buoy["lon"] - approx_lon) < 1e-6:
                            lat, lon = buoy["lat"], buoy["lon"]
                            break

                    if lat is not None and lon is not None:
                        logger.info(f"Zooming to buoy at ({lat}, {lon})")
                        self.map_bridge.focus_map_on_marker(lat, lon)
                    else:
                        logger.warning(f"Buoy not found for coordinates ({approx_lat}, {approx_lon})")

                except (ValueError, TypeError) as e:
                    logger.error(f"Invalid buoy data: {e}")
            else:
                logger.warning("No buoys available to zoom to.")

        else:
            logger.error(f"Invalid table specified: {table}. Use 'waypoints' or 'buoys'.")

    # endregion button functions

    # region pyqt thread functions

    @Slot()
    def remote_waypoint_handler_starter(self) -> None:
        """Starts the telemetry waypoint handler thread."""

        if not constants.SM.read_dict("map_features")["waypoints_popup"]["status"]:
            self.remember_waypoints_pull_service_status = False
            logger.info("Waypoint checker disabled, not checking for waypoint updates.")
            return

        if not self.remote_waypoint_handler.isRunning():
            self.remote_waypoint_handler.start()

    @Slot()
    def local_waypoint_handler_starter(self) -> None:
        """Starts the local waypoint handler thread."""

        if not self.local_waypoint_handler.isRunning():
            self.local_waypoint_handler.start()

    @Slot(tuple)
    def update_waypoints_display(self, request_result: tuple[list[list[int | float]], constants.TelemetryStatus]) -> None:
        """
        Update waypoints display with waypoints fetched from the local server.

        Parameters
        ----------
        request_result
            A tuple containing:
                - a list of waypoints fetched from the local server.
                - a :class:`TelemetryStatus` enum value indicating the status of the request.
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

    @Slot(tuple)
    def check_telemetry_waypoints(self, request_result: tuple[list[list[int | float]], constants.TelemetryStatus]) -> None:
        """
        Check if the waypoints on the telemetry server are the same as the local waypoints.
        If they are different, show a dialog and let user decide whether to update the local waypoints.

        Parameters
        ----------
        request_result
            A tuple containing:
                - a list of waypoints fetched from the telemetry server.
                - a :class:`TelemetryStatus` enum value indicating the status of the request.
        """

        waypoints, _ = request_result
        equal_flag = sorted(self.waypoints) == sorted(waypoints)

        if not (equal_flag and self.remember_waypoints_pull_service_status) and self.can_pull_waypoints:
            for timer in self.timers:
                timer.stop()

            response, temp_pull_waypoints_reminder = show_message_box(
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
                self.map_bridge.clear_waypoints()

                for waypoint in self.waypoints:
                    self.map_bridge.add_waypoint(waypoint[0], waypoint[1])

                self.map_bridge.change_color_waypoints("red")

                for waypoint in not_uploaded_waypoints:
                    self.map_bridge.add_waypoint(waypoint[0], waypoint[1])

                logger.info("Local waypoints updated from telemetry server.")

            else:
                self.remember_waypoints_pull_service_status = temp_pull_waypoints_reminder
                logger.info("Local waypoints not updated.")

            for timer in self.timers:
                timer.start()

        elif not equal_flag and self.remember_waypoints_pull_service_status:
            logger.info("Local waypoints do not match telemetry server waypoints, but not prompting user.")

        else:
            logger.info("Local waypoints match telemetry server waypoints, but not prompting user.")

    def change_telemetry_server_url(self, telemetry_status: constants.TelemetryStatus) -> None:
        """
        Prompt the user to change the telemetry server URL if fetching waypoints fails.

        Parameters
        ----------
        telemetry_status
            A :class:`TelemetryStatus` enum value indicating the status of the request. Possible values are:
                - ``SUCCESS`` indicates that the telemetry server is reachable and waypoints were fetched successfully.
                - ``FAILURE`` indicates that the telemetry server is not reachable and waypoints could not be fetched.
        """

        if telemetry_status == constants.TelemetryStatus.FAILURE and not self.remember_telemetry_server_url_status:
            for timer in self.timers:
                timer.stop()

            response, temp_remember_telemetry_server_url_status = show_message_box(
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
                new_url = InputDialog(
                    "Change Telemetry Server URL",
                    "Enter the new telemetry server URL:",
                    default_value=constants.SM.read_str("telemetry_server_url"),
                    input_type=str,
                ).get_input()

                if new_url:
                    logger.info(
                        f"Changed telemetry server URL to {new_url}, was {constants.SM.read_str('telemetry_server_url')}."
                    )

                    constants.SM.write("telemetry_server_url", new_url)
                    tmp_dict = {}
                    for endpoint, value in constants.SM.read_dict("telemetry_server_endpoints").items():
                        path_tail = "/".join(value.split("/")[-2:])
                        new_endpoint_url = constants.SM.read_str("telemetry_server_url") + path_tail
                        tmp_dict[endpoint] = new_endpoint_url

                    constants.SM.write("telemetry_server_endpoints", tmp_dict)

                else:
                    logger.warning("No new telemetry server URL provided, keeping old one.")

            elif response == QMessageBox.StandardButton.No:
                self.remember_telemetry_server_url_status = temp_remember_telemetry_server_url_status
                logger.info("Telemetry server URL not changed.")

            else:
                logger.error(f"Received unexpected response from user dialog. Got: {response}, expected Yes or No.")

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
                - a :class:`TelemetryStatus` enum value indicating the status of the request.
        """

        boat_data, connection_status = request_result
        self.boat_data = boat_data

        def fix_formatting(data_item: float | None) -> str:
            """
            Applies some formatting rules that multiple keys have in common.

            If the value is `None`, displays "N/A".
            Otherwise, the value is rounded to 1 decimal places.

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
            `str`
                The formatted value.
            """

            return "N/A" if data_item is None else f"{float(data_item):.1f}"

        # region mode dependent print functions
        def sailboat_mode(boat_data: dict[str, Any]) -> str:
            self.boat_data["boat_autopilot_state"] = misc.resolve_enum_name(
                SailboatAutopilotStates, boat_data["boat_autopilot_state"]
            )
            self.boat_data["boat_control_mode"] = misc.resolve_enum_name(SailboatControlModes, boat_data["boat_control_mode"])

            return (
                "Position: "
                f"[{self.boat_data.get('position', self.fake_position)[0]:.8f}, "
                f"{self.boat_data.get('position', self.fake_position)[1]:.8f}]\n"
                f"Control Mode: {self.boat_data.get('boat_control_mode', 'N/A')}\n"
                f"Autopilot State: {self.boat_data.get('boat_autopilot_state', 'N/A')}\n"
                f"Connection Status: {connection_status.name}\n"
                f"Current Waypoint Index: {self.boat_data.get('current_waypoint_index') + 1 if isinstance(self.boat_data.get('current_waypoint_index'), int) else 'N/A'}\n"  # noqa: E501
                f"Velocity Vector: [{fix_formatting(self.boat_data.get('velocity_x', -69.420))}, {fix_formatting(self.boat_data.get('velocity_y', -69.420))}]\n"  # noqa: E501
                f"Speed: {fix_formatting(self.boat_data.get('speed'))} m/s\n"
                f"Distance To Next WP: {fix_formatting(self.boat_data.get('distance_to_next_waypoint'))} meters\n"
                f"True Wind Speed: {fix_formatting(self.boat_data.get('true_wind_speed'))} m/s\n"
                f"True Wind Angle: {fix_formatting(self.boat_data.get('true_wind_angle'))}°\n"
                f"Apparent Wind Speed: {fix_formatting(self.boat_data.get('apparent_wind_speed'))} m/s\n"
                f"Apparent Wind Angle: {fix_formatting(self.boat_data.get('apparent_wind_angle'))}°\n"
                f"Heading: {fix_formatting(self.boat_data.get('heading', self.fake_heading))}°\n"
                f"Desired Heading: {fix_formatting(self.boat_data.get('desired_heading'))}°\n"
                f"Desired Sail Angle: {fix_formatting(self.boat_data.get('desired_sail_angle'))}°\n"
                f"Current Sail Angle: {fix_formatting(self.boat_data.get('current_sail_angle'))}°\n"
                f"Sail Angle Error: {fix_formatting(self.boat_data.get('sail_angle_error'))}°\n"
                f"Desired Rudder Angle: {fix_formatting(self.boat_data.get('desired_rudder_angle'))}°\n"
                f"Current Rudder Angle: {fix_formatting(self.boat_data.get('current_rudder_angle'))}°\n"
                f"Rudder Angle Error: {fix_formatting(self.boat_data.get('rudder_angle_error'))}°\n"
            )

        def motorboat_mode(boat_data: dict[str, Any]) -> str:
            self.boat_data["boat_control_mode"] = misc.resolve_enum_name(MotorboatControlModes, boat_data["boat_control_mode"])

            return (
                "Position: "
                f"{self.boat_data.get('position', self.fake_position)[0]:.8f}, "
                f"{self.boat_data.get('position', self.fake_position)[1]:.8f}\n"
                f"Boat Control Mode: {self.boat_data.get('boat_control_mode', 'N/A')}\n"
                f"Connection Status: {connection_status.name}\n"
                f"Autopilot State: {self.boat_data.get('boat_autopilot_state', 'N/A')}\n"
                f"Current Waypoint Index: {self.boat_data.get('current_waypoint_index') + 1 if isinstance(self.boat_data.get('current_waypoint_index'), int) else 'N/A'}\n"  # noqa: E501
                f"Velocity Vector: [{fix_formatting(self.boat_data.get('velocity_x', -69.420))}, {fix_formatting(self.boat_data.get('velocity_y', -69.420))}]\n"  # noqa: E501
                f"Speed: {fix_formatting(self.boat_data.get('speed'))} m/s\n"
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

        # endregion mode dependent print functions

        def draw_map_diagnostics(heading: float) -> None:
            """
            Draw diagnostics on the map, such as no sail zone and wind direction.

            Parameters
            ----------
            heading
                The heading of the boat, used to orient the diagnostics correctly on the map.
            """

            current_autopilot_parameters = constants.SM.read_dict("current_autopilot_parameters")

            no_sail_zone_size_dict: dict[str, str | float] | None = current_autopilot_parameters.get("no_sail_zone_size")
            if no_sail_zone_size_dict is None:
                return

            if "current" in no_sail_zone_size_dict:
                no_sail_size: float = no_sail_zone_size_dict["current"]
            else:
                no_sail_size: float = no_sail_zone_size_dict["default"]

            wind_direction: float | None = self.boat_data.get("true_wind_angle")
            if wind_direction is None:
                return

            heading_opposite_wind = heading + (wind_direction + 180)

            # don't think about it too hard
            x1: float = 2 + np.cos(np.deg2rad(heading_opposite_wind - no_sail_size / 2))
            y1: float = 2 - np.sin(np.deg2rad(heading_opposite_wind - no_sail_size / 2))
            x2: float = 2 + np.cos(np.deg2rad(heading_opposite_wind + no_sail_size / 2))
            y2: float = 2 - np.sin(np.deg2rad(heading_opposite_wind + no_sail_size / 2))

            no_go_path_shape: list[svg.PathData] = [
                svg.MoveTo(2, 2),
                svg.LineTo(x1, y1),
                svg.Arc(1, 1, 0, 0, 0, x2, y2),
                svg.LineTo(2, 2),
            ]
            no_go_html = svg.Path(d=no_go_path_shape, fill="#c9140a")

            tack_distance_dict: dict[str, str | float] | None = current_autopilot_parameters.get("tack_distance")
            if tack_distance_dict is None:
                return

            if "current" in tack_distance_dict:
                tack_distance: float = tack_distance_dict["current"]
            else:
                tack_distance: float = tack_distance_dict["default"]

            distance_to_waypoint: float | None = self.boat_data.get("distance_to_next_waypoint")
            if distance_to_waypoint is None:
                return

            if tack_distance > distance_to_waypoint:
                # we can't draw the line!
                decision_zone_path: list[svg.PathData] = []
                distance_to_waypoint = 200

            else:
                # ratio of the tack distance to the distance to the waypoint, used to scale the decision zone size
                tack_distance_ratio = tack_distance / distance_to_waypoint

                # in radians
                decision_zone_size: float = np.rad2deg(np.arcsin(tack_distance_ratio * np.sin(np.deg2rad(no_sail_size / 2))))

                # don't think about it too hard
                x1: float = 2 + np.cos(np.deg2rad(heading_opposite_wind - (no_sail_size / 2 - decision_zone_size / 2)))
                y1: float = 2 - np.sin(np.deg2rad(heading_opposite_wind - (no_sail_size / 2 - decision_zone_size / 2)))
                x2: float = 2 + np.cos(np.deg2rad(heading_opposite_wind + (no_sail_size / 2 - decision_zone_size / 2)))
                y2: float = 2 - np.sin(np.deg2rad(heading_opposite_wind + (no_sail_size / 2 - decision_zone_size / 2)))

                decision_zone_path: list[svg.PathData] = [
                    svg.MoveTo(2, 2),
                    svg.LineTo(x1, y1),
                    svg.Arc(1, 1, 0, 0, 0, x2, y2),
                    svg.LineTo(2, 2),
                ]

            decision_zone_html = svg.Path(d=decision_zone_path, fill="pink")

            wind_direction_shape: list[svg.PathData] = [
                svg.MoveTo(50, 50),
                svg.LineTo(
                    50 + 50 * np.cos(np.deg2rad(heading + wind_direction)),
                    50 - 50 * np.sin(np.deg2rad(heading + wind_direction)),
                ),
            ]
            wind_html = svg.Path(
                d=wind_direction_shape,
                stroke="orange",
                stroke_width="0.1",
            )

            speed: float | None = self.boat_data.get("speed")
            if speed is None:
                return

            elif np.isclose(speed, 0.0, rtol=1e-5, atol=1e-8):
                logger.warning("`speed` is very close to 0, defaulting to 1e-3 to avoid division by zero.")
                speed = 1e-3

            vx: float = self.boat_data.get("velocity_x", -69.420)
            vy: float = self.boat_data.get("velocity_y", -69.420)

            radius: float = 4 * speed
            x1: float = 2 + radius * vx / speed
            y1: float = 2 + radius * vy / speed

            velocity_arrow_shape: list[svg.PathData] = [svg.MoveTo(2, 2), svg.LineTo(x1, y1)]
            velocity_arrow_transform: list[svg.Transform] = [
                svg.Rotate(-heading, 2, 2),
            ]
            velocity_html = svg.Path(
                d=velocity_arrow_shape, stroke="black", stroke_width="0.1", transform=velocity_arrow_transform
            )

            size = 0.2
            svg_str = no_go_html.as_str() + decision_zone_html.as_str()

            self.map_bridge.update_no_sail_svg(svg_str, size)
            self.map_bridge.update_velocity_svg(velocity_html.as_str(), size)
            self.map_bridge.update_wind_svg(wind_html.as_str())
            self.map_bridge.update_compass_svg(heading + wind_direction)

        # region data validation and defaulting
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

        instance_changed = constants.SM.read_bool("has_telemetry_server_instance_changed")
        if instance_changed:
            constants.SM.write("remote_autopilot_param_hash", "")
            constants.SM.write("data_logging_active", False)
            constants.SM.write("data_log_file_path", "")
            self.refresh_autopilot_config_signal.emit(True)

            self.start_data_logging_button.setDisabled(False)
            self.stop_data_logging_button.setDisabled(True)
            self.clear_waypoints()

            constants.SM.write("has_telemetry_server_instance_changed", False)

        else:
            self.refresh_autopilot_config_signal.emit(False)

        # endregion data validation and defaulting

        self.map_bridge.update_boat_location_and_heading(lat, lon, heading, record_track=not instance_changed)

        if constants.SM.read_dict("map_features")["sailboat_debug_symbols"]["status"]:
            draw_map_diagnostics(heading)
            self.need_to_clear_diagnostics = True

        elif self.need_to_clear_diagnostics:
            self.map_bridge.remove_all_svgs()
            self.need_to_clear_diagnostics = False

        if "desired_sail_angle" in self.boat_data:
            telemetry_text = sailboat_mode(boat_data)

        elif "rpm" in self.boat_data:
            telemetry_text = motorboat_mode(boat_data)

        else:
            telemetry_text = "Boat data received, but could not determine boat type."

        self.left_text_section.setText(telemetry_text)

    # endregion pyqt thread functions
