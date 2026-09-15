from __future__ import annotations

import os
import sys
from pathlib import Path


def _resource_root() -> Path:
    """
    Return the directory that owns ``app_data`` for this process.

    Returns
    -------
    :class:`Path`
        Frozen onedir: the folder containing the executable
        (``sys.executable.parent``). Frozen macOS BUNDLE: the folder
        containing ``GroundStation.app`` (a sibling of the bundle, NOT a
        folder inside it), so the release zip's shipped
        ``ground_station/app_data/`` sits next to ``GroundStation.app``.
        In development: the parent directory of this file (the
        ``ground_station`` folder).
    """

    if getattr(sys, "frozen", False):
        exe_dir = Path(sys.executable).resolve().parent
        if exe_dir.parent.name == "Contents" and exe_dir.name == "MacOS":
            return exe_dir.parents[2]

        return exe_dir

    return Path(__file__).resolve().parent.parent


_GS_HOME = _resource_root()
os.environ.setdefault("GROUND_STATION_HOME", _GS_HOME.as_posix())
os.chdir(_GS_HOME)

# Select the Qt binding before any qtpy import. PySide6 ships prebuilt
# wheels for Windows/macOS/Linux (unlike PySide2/PyQt5), so it is the only
# binding that works everywhere out of the box.
os.environ.setdefault("QT_API", "pyside6")

# The ground station talks to its bundled servers over plain loopback HTTP;
# bypass any configured proxy for those requests so corporate proxy settings
# cannot break the local map/asset servers.
os.environ.setdefault("NO_PROXY", "localhost,127.0.0.1,::1")

import qtpy

if qtpy.API_NAME != "PySide6":
    message = (
        f"qtpy selected {qtpy.API_NAME!r} but the Ground Station requires "
        "PySide6. Uninstall the other Qt bindings (PyQt5/PyQt6/PySide2) or "
        "pyqtgraph before launching."
    )
    raise ImportError(message)

import http.server
import mimetypes
import socketserver
import threading
from collections.abc import Callable
from typing import Any, NoReturn

from qtpy.QtCore import QThread, QtMsgType, qInstallMessageHandler
from qtpy.QtGui import QCloseEvent, QIcon
from qtpy.QtWebEngineWidgets import QWebEnginePage
from qtpy.QtWidgets import QApplication, QMainWindow, QTabWidget

from utils import constants, misc
from utils.console_logger import get_logger
from widgets import (
    AutopilotConfigWidget,
    CameraWidget,
    ConsoleOutputWidget,
    GraphViewer,
    GroundStationWidget,
    InstanceHandler,
    UserGuideWidget,
)
from widgets.map_widget import frontend_server, server as map_server

logger = get_logger(__name__)


class MainWindow(QMainWindow):
    """Main window for the ground station application."""

    def start_asset_server(self) -> None:
        """Start a quiet HTTP server for static assets (and the built frontend, when present)."""

        mimetypes.add_type("image/png", ".png")
        mimetypes.add_type("text/plain", ".txt")

        asset_dir = constants.ASSETS_DIR

        class _QuietHandler(http.server.SimpleHTTPRequestHandler):
            def log_message(self, format_string: str, *args: object) -> None:
                pass

        def handler(*args: tuple, **kwargs: dict) -> _QuietHandler:
            return _QuietHandler(*args, directory=asset_dir.as_posix(), **kwargs)

        socketserver.TCPServer.allow_reuse_address = True
        self.asset_server = socketserver.TCPServer(("", constants.ASSET_SERVER_PORT), handler)
        logger.info(f"Serving HTTP assets from {asset_dir} on port {constants.ASSET_SERVER_PORT}...")
        self.asset_server.serve_forever()

    def start_frontend_server(self) -> None:
        """Serve the built map frontend on the Vite port from an in-process server."""

        frontend_dir = constants.FRONTEND_SERV_DIR
        if frontend_dir is None:
            logger.error("Built frontend not found; map widget will not load.")
            return

        try:
            frontend_server.run(constants.VITE_PORT, frontend_dir)
        except OSError as exc:
            logger.error(f"Failed to serve frontend on port {constants.VITE_PORT}: {exc}")

    def start_map_server(self) -> None:
        """Start the local map widget server."""

        try:
            map_server.run()
        except OSError as exc:
            logger.error(f"Failed to start map server on port {constants.MAP_SERVER_PORT}: {exc}")

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(constants.WINDOW_TITLE)
        self.setGeometry(constants.WINDOW_BOX)
        self.setMaximumSize(constants.MAX_WINDOW_SIZE)
        self.setUnifiedTitleAndToolBarOnMac(True)

        constants.MAP_PAGE = QWebEnginePage()
        constants.MAP_PAGE.load(constants.MAP_URL)

        self.main_widget = QTabWidget()
        self.setCentralWidget(self.main_widget)

        try:
            self.console_widget = ConsoleOutputWidget()
            logger.info(f"Starting the {self.windowTitle()}...")
            self.instance_handler = InstanceHandler()
            self.main_widget.addTab(UserGuideWidget(), "Documentation")
            self.main_widget.addTab(self.console_widget, "Console Output")
            self.main_widget.setCurrentIndex(1)

            if constants.SM.read_bool("has_telemetry_server_instance_changed"):
                self.load_main_tabs()

            else:
                self.main_widget.addTab(self.instance_handler, "Instance Handler")
                self.check_timer = misc.copy_qtimer(constants.TEN_MS_TIMER)
                self.check_timer.timeout.connect(self.check_instance_connection)
                self.check_timer.start()

        except Exception as e:
            logger.error(f"Failed to initialize main window: {e}")

    def load_main_tabs(self) -> None:
        """Load the main application tabs after an instance connection is detected."""

        try:
            self.main_widget.addTab(self.instance_handler, "Instance Handler")

            graph_viewer = GraphViewer()
            groundstation_widget = GroundStationWidget(graph_viewer.boat_data_signal)
            self.main_widget.addTab(groundstation_widget, "Ground Station")
            self.main_widget.addTab(graph_viewer, "Graph Viewer")

            autopilot_config_widget = AutopilotConfigWidget(groundstation_widget.refresh_autopilot_config_signal)
            self.main_widget.addTab(autopilot_config_widget, "Autopilot Configuration")

            self.main_widget.addTab(CameraWidget(), "Camera Feed")
            self.main_widget.setCurrentIndex(3)
            logger.info("Main application tabs loaded.")

        except Exception as e:
            logger.error(f"Failed to load main tabs: {e}")

    def check_instance_connection(self) -> None:
        """Check if an instance connection has been established."""

        if constants.SM.read_bool("has_telemetry_server_instance_changed"):
            self.check_timer.stop()
            self.load_main_tabs()

    def closeEvent(self, event: QCloseEvent) -> NoReturn:
        """Handle the window close event."""

        logger.info("Shutting down background threads...")
        for thread in self.findChildren(QThread):
            thread.requestInterruption()
            thread.wait()

        logger.info("Shutting down asset server...")
        if hasattr(self, "asset_server"):
            self.asset_server.shutdown()
            self.asset_server.server_close()

        logger.info("Releasing map page...")
        if hasattr(constants, "MAP_PAGE") and isinstance(constants.MAP_PAGE, QWebEnginePage):
            constants.MAP_PAGE.deleteLater()

        logger.info("Closing the application...")
        event.accept()


if __name__ == "__main__":
    _default_handler: list[Callable[[QtMsgType, Any, str], None] | None] = [None]

    def _filter_qt_messages(msg_type: QtMsgType, _context: Any, message: str) -> None:
        spam_prefixes = (
            "FFmpeg log:",
            "mp3float",
            "audio device has unrecognized channel",
        )

        if msg_type in (QtMsgType.QtDebugMsg, QtMsgType.QtWarningMsg) and any(
            message.startswith(p) or p in message for p in spam_prefixes
        ):
            return

        handler = _default_handler[0]
        if handler is not None:
            handler(msg_type, _context, message)

    # qInstallMessageHandler returns the previously installed handler (or None),
    # which we store so _filter_qt_messages can forward non-spam messages to it.
    _default_handler[0] = qInstallMessageHandler(_filter_qt_messages)

    app = QApplication(sys.argv)
    constants.ICONS = misc.get_icons()

    window = MainWindow()
    if constants.APP_LOGO_PATH.is_file():
        logger.info(f"Setting application icon from {constants.APP_LOGO_PATH}...")
        logo_icon = QIcon(constants.APP_LOGO_PATH.as_posix())
        app.setWindowIcon(logo_icon)
        window.setWindowIcon(logo_icon)

    else:
        logger.warning(f"Application logo not found at {constants.APP_LOGO_PATH}. Using default icon.")
        app.setWindowIcon(constants.ICONS.boat)
        window.setWindowIcon(constants.ICONS.boat)

    threading.Thread(target=window.start_asset_server, daemon=True).start()
    threading.Thread(target=window.start_frontend_server, daemon=True).start()
    threading.Thread(target=window.start_map_server, daemon=True).start()

    app.setStyleSheet(constants.STYLE_SHEET)
    app.setPalette(constants.PALLETTE)
    app.setStyle("Fusion")

    app.setApplicationName(constants.APPLICATION_NAME)
    app.setOrganizationName(constants.ORGANIZATION_NAME)

    window.show()
    sys.exit(app.exec())
