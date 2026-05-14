import http.server
import mimetypes
import socketserver
import sys
import threading
from typing import NoReturn

from qtpy.QtGui import QIcon
from qtpy.QtWebEngineWidgets import QWebEnginePage
from qtpy.QtWidgets import QApplication, QMainWindow, QTabWidget
from utils import constants, misc
from widgets import AutopilotConfigWidget, CameraWidget, ConsoleOutputWidget, GraphViewer, GroundStationWidget, InstanceHandler
from widgets.map_widget import server as map_server


class MainWindow(QMainWindow):
    """Main window for the ground station application."""

    def start_asset_server(self) -> None:
        """Start a quiet HTTP server for static assets."""

        mimetypes.add_type("image/png", ".png")
        mimetypes.add_type("text/plain", ".txt")

        def handler(*args: tuple, **kwargs: dict) -> http.server.SimpleHTTPRequestHandler:
            return http.server.SimpleHTTPRequestHandler(*args, directory=constants.ASSETS_DIR.as_posix(), **kwargs)

        socketserver.TCPServer.allow_reuse_address = True
        self.asset_server = socketserver.TCPServer(("", constants.ASSET_SERVER_PORT), handler)
        print(f"[Info] Serving HTTP assets on port {constants.ASSET_SERVER_PORT}...")
        self.asset_server.serve_forever()

    def start_map_server(self) -> None:
        """Start the local map widget server."""

        try:
            map_server.run()
        except OSError as exc:
            print(f"[Error] Failed to start map server on port {constants.MAP_SERVER_PORT}: {exc}")

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
            print(f"[Info] Starting the {self.windowTitle()}...")
            self.instance_handler = InstanceHandler()
            self.main_widget.addTab(self.console_widget, "Console Output")

            if constants.SM.read("has_telemetry_server_instance_changed"):
                self.load_main_tabs()

            else:
                self.main_widget.addTab(self.instance_handler, "Instance Handler")
                self.check_timer = misc.copy_qtimer(constants.TEN_MS_TIMER)
                self.check_timer.timeout.connect(self.check_instance_connection)
                self.check_timer.start()

        except Exception as e:
            print(f"[Error] Failed to initialize main window: {e}")

    def closeEvent(self, event: object) -> NoReturn:
        """Handle the window close event."""

        print("[Info] Shutting down asset server...")
        if hasattr(self, "asset_server"):
            self.asset_server.shutdown()

        print("[Info] Closing the application...")
        event.accept()

    def check_instance_connection(self) -> None:
        """Check if an instance connection has been established."""

        if constants.SM.read("has_telemetry_server_instance_changed"):
            self.check_timer.stop()
            self.load_main_tabs()

    def load_main_tabs(self) -> None:
        """Load the main application tabs after an instance connection is detected."""

        try:
            self.main_widget.addTab(self.instance_handler, "Instance Handler")
            graph_viewer = GraphViewer()
            self.main_widget.addTab(GroundStationWidget(graph_viewer.boat_data_signal), "Ground Station")
            self.main_widget.addTab(graph_viewer, "Graph Viewer")
            self.main_widget.addTab(AutopilotConfigWidget(), "Autopilot Configuration")
            self.main_widget.addTab(CameraWidget(), "Camera Feed")
            self.main_widget.setCurrentIndex(2)
            print("[Info] Main application tabs loaded.")

        except Exception as e:
            print(f"[Error] Failed to load main tabs: {e}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    constants.ICONS = misc.get_icons()
    
    window = MainWindow()
    threading.Thread(target=window.start_asset_server, daemon=True).start()
    threading.Thread(target=window.start_map_server, daemon=True).start()

    app.setStyleSheet(constants.STYLE_SHEET)
    app.setPalette(constants.PALLETTE)
    app.setStyle("Fusion")

    app.setApplicationName(constants.APPLICATION_NAME)
    app.setOrganizationName(constants.ORGANIZATION_NAME)

    if constants.APP_LOGO_PATH.is_file():
        print(f"[Info] Setting application icon from {constants.APP_LOGO_PATH}...")
        logo_icon = QIcon(constants.APP_LOGO_PATH.as_posix())
        app.setWindowIcon(logo_icon)
        window.setWindowIcon(logo_icon)

    else:
        print(f"[Warning] Application logo not found at {constants.APP_LOGO_PATH}. Using default icon.")
        app.setWindowIcon(constants.ICONS.boat)
        window.setWindowIcon(constants.ICONS.boat)

    window.show()
    sys.exit(app.exec())
