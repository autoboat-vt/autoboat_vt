import http.server
import mimetypes
import socketserver
import sys
import threading
import time
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
from widgets.map_widget.callback_server.server import assemble_map_callback_server

logger = get_logger(__name__)


class MainWindow(QMainWindow):
    """Main window for the ground station application."""

    def start_asset_server(self) -> None:
        """Start a quiet HTTP server for static assets."""

        mimetypes.add_type("image/png", ".png")
        mimetypes.add_type("text/plain", ".txt")

        class _QuietHandler(http.server.SimpleHTTPRequestHandler):
            def log_message(self, format_string: str, *args: object) -> None:
                pass

        def handler(*args: tuple, **kwargs: dict) -> _QuietHandler:
            return _QuietHandler(*args, directory=constants.ASSETS_DIR.as_posix(), **kwargs)

        socketserver.TCPServer.allow_reuse_address = True
        while not misc.check_port_available(constants.ASSET_SERVER_PORT):
            logger.warning(
                f"Port {constants.ASSET_SERVER_PORT} is already in use. Retrying in 1 second..."
            )
            time.sleep(1)

        self.asset_server = socketserver.TCPServer(("127.0.0.1", constants.ASSET_SERVER_PORT), handler)
        logger.info(f"Serving HTTP assets on port {constants.ASSET_SERVER_PORT}...")
        self.asset_server.serve_forever()

    def start_map_control_server(self) -> None:
        """Start the map control server for handling waypoints, land checks, and bathymetry requests."""

        self.map_callback_server = assemble_map_callback_server()
        try:
            logger.info(f"Hosting map callback server on port {constants.MAP_CALLBACK_PORT}...")
            self.map_callback_server.serve_forever()

        except OSError as exc:
            logger.error(f"Failed to start map callback server on port {constants.MAP_CALLBACK_PORT}: {exc}")

    def start_map_viewer_server(self) -> None:
        """Start the map viewer server for rendering the map widget."""

        try:
            waiting_for_assets = True
            while misc.check_port_available(constants.ASSET_SERVER_PORT):
                if waiting_for_assets:
                    waiting_for_assets = False
                time.sleep(0.1)

            logger.info(f"Starting map viewer server on port {constants.MAP_VIEWER_PORT}...")
            self.map_viewer_server.start()
            self.map_viewer_server.wait_until_ready()

        except Exception as exc:
            logger.error(f"Failed to start map viewer server: {exc}")

    def load_map_page(self) -> None:
        """
        Navigate the map page to the map viewer URL.

        Called only once the map viewer server is accepting connections. Making
        the web view navigate before Vite is listening makes the first load fail
        with ``ERR_CONNECTION_REFUSED`` even though a manual reload then works.
        """

        if self.map_page_loaded:
            return

        self.map_page_loaded = True
        constants.MAP_PAGE.load(constants.MAP_URL)
        logger.info(f"Loading map from {constants.MAP_URL.toString()}...")

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(constants.WINDOW_TITLE)
        self.setGeometry(constants.WINDOW_BOX)
        self.setMaximumSize(constants.MAX_WINDOW_SIZE)
        self.setUnifiedTitleAndToolBarOnMac(True)

        # The page object is created eagerly so widgets can call setPage() on it,
        # but navigation is deferred until the viewer server is up (load_map_page).
        self.map_viewer_server = misc.MapViewerServer()
        self.map_page_loaded = False
        constants.MAP_PAGE = QWebEnginePage()

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

        logger.info("Releasing map page...")
        if hasattr(constants, "MAP_PAGE") and isinstance(constants.MAP_PAGE, QWebEnginePage):
            constants.MAP_PAGE.deleteLater()

        logger.info("Shutting down map viewer server...")
        if hasattr(self, "map_viewer_server"):
            self.map_viewer_server.stop()

        logger.info("Shutting down map callback server...")
        if hasattr(self, "map_callback_server"):
            self.map_callback_server.shutdown()

        logger.info("Closing the application...")
        event.accept()


if __name__ == "__main__":
    _default_handler: list[Callable[[QtMsgType, Any, str], None] | None] = [None]

    def _filter_qt_messages(msg_type: QtMsgType, _context: Any, message: str) -> None:
        spam_prefixes = (
            "FFmpeg log:",
            "QXcbIntegration:",
            "mp3float",
            "audio device has unrecognized channel",
        )
        is_spam_message = any(message.startswith(prefix) or prefix in message for prefix in spam_prefixes)

        if msg_type in (QtMsgType.QtDebugMsg, QtMsgType.QtWarningMsg) and is_spam_message:
            return

        handler = _default_handler[0]
        if handler is not None:
            handler(msg_type, _context, message)

    _default_handler[0] = qInstallMessageHandler(_filter_qt_messages)

    app = QApplication(sys.argv)
    constants.ICONS = misc.get_icons()

    window = MainWindow()

    threading.Thread(target=window.start_asset_server, daemon=True).start()
    threading.Thread(target=window.start_map_control_server, daemon=True).start()
    threading.Thread(target=window.start_map_viewer_server, daemon=True).start()

    map_load_timer = misc.create_timer(100)
    def load_map_when_ready() -> None:
        if window.map_viewer_server.is_ready:
            map_load_timer.stop()
            window.load_map_page()

    map_load_timer.timeout.connect(load_map_when_ready)
    map_load_timer.start()

    if constants.APP_LOGO_PATH.is_file():
        logger.info(f"Setting application icon from {constants.APP_LOGO_PATH}...")
        logo_icon = QIcon(constants.APP_LOGO_PATH.as_posix())
        app.setWindowIcon(logo_icon)
        window.setWindowIcon(logo_icon)

    else:
        logger.warning(f"Application logo not found at {constants.APP_LOGO_PATH}. Using default icon.")
        app.setWindowIcon(constants.ICONS.boat)
        window.setWindowIcon(constants.ICONS.boat)

    app.setStyleSheet(constants.STYLE_SHEET)
    app.setPalette(constants.PALLETTE)
    app.setStyle("Fusion")

    app.setApplicationName(constants.APPLICATION_NAME)
    app.setOrganizationName(constants.ORGANIZATION_NAME)

    window.show()
    sys.exit(app.exec())
