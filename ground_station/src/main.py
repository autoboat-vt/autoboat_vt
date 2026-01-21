import http.server
import mimetypes
import socketserver
import sys
import threading

from qtpy.QtWidgets import QApplication, QMainWindow, QTabWidget
from utils import constants, misc
from widgets import (
    AutopilotConfigEditor,
    AutopilotConfigManager,
    CameraWidget,
    ConsoleOutputWidget,
    GraphViewer,
    GroundStationWidget,
    InstanceHandler,
)


class MainWindow(QMainWindow):
    """Main window for the ground station application."""

    def start_asset_server(self) -> None:
        """Start a quiet HTTP server for static assets."""

        mimetypes.add_type("image/png", ".png")
        mimetypes.add_type("text/plain", ".txt")

        def handler(*args: tuple, **kwargs: dict) -> http.server.SimpleHTTPRequestHandler:
            return http.server.SimpleHTTPRequestHandler(*args, directory=constants.ASSETS_DIR.as_posix(), **kwargs)

        self.asset_server = socketserver.TCPServer(("", constants.ASSET_SERVER_PORT), handler)
        print(f"[Info] Serving HTTP assets on port {constants.ASSET_SERVER_PORT}...")
        self.asset_server.serve_forever()

    def start_cdn_server(self) -> None:
        """Start a quiet HTTP server for CDN assets."""

        mimetypes.add_type("text/javascript", ".js")
        mimetypes.add_type("text/css", ".css")

        def handler(*args: tuple, **kwargs: dict) -> http.server.SimpleHTTPRequestHandler:
            return http.server.SimpleHTTPRequestHandler(*args, directory=constants.CDN_DIR.as_posix(), **kwargs)

        for link in constants.JS_LIBRARIES:
            misc.cache_cdn_file(link, constants.CDN_DIR)

        self.cdn_server = socketserver.TCPServer(("", constants.CDN_SERVER_PORT), handler)
        print(f"[Info] Serving CDN assets on port {constants.CDN_SERVER_PORT}...")
        self.cdn_server.serve_forever()

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("SailBussy Ground Station")
        self.setGeometry(constants.WINDOW_BOX)

        self.main_widget = QTabWidget()
        self.setCentralWidget(self.main_widget)

        try:
            self.console_widget = ConsoleOutputWidget()
            print(f"[Info] Starting the {self.windowTitle()}...")
            self.instance_handler = InstanceHandler()
            self.main_widget.addTab(self.console_widget, "Console Output")

            if constants.HAS_TELEMETRY_SERVER_INSTANCE_CHANGED:
                self.load_main_tabs()

            else:
                self.main_widget.addTab(self.instance_handler, "Instance Handler")
                self.check_timer = misc.copy_qtimer(constants.TEN_MS_TIMER)
                self.check_timer.timeout.connect(self.check_instance_connection)
                self.check_timer.start()

        except Exception as e:
            print(f"[Error] Failed to initialize main window: {e}")

    def closeEvent(self, event: object) -> None:
        """Handle the window close event."""

        print("[Info] Shutting down servers...")
        
        if hasattr(self, "asset_server"):
            self.asset_server.shutdown()
        
        if hasattr(self, "cdn_server"):
            self.cdn_server.shutdown()
        
        print("[Info] Servers shut down.")
        event.accept()


    def check_instance_connection(self) -> None:
        """Check if an instance connection has been established."""

        if constants.HAS_TELEMETRY_SERVER_INSTANCE_CHANGED:
            self.check_timer.stop()
            self.load_main_tabs()

    def load_main_tabs(self) -> None:
        """Load the main application tabs after an instance connection is detected."""

        try:
            self.main_widget.addTab(self.instance_handler, "Instance Handler")
            graph_viewer = GraphViewer()
            self.main_widget.addTab(GroundStationWidget(graph_viewer.boat_data_signal), "Ground Station")
            self.main_widget.addTab(graph_viewer, "Graph Viewer")
            self.main_widget.addTab(AutopilotConfigEditor(), "Autopilot Parameters")
            self.main_widget.addTab(AutopilotConfigManager(), "Config Manager")
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
    threading.Thread(target=window.start_cdn_server, daemon=True).start()
    app.setStyleSheet(constants.STYLE_SHEET)
    app.setPalette(constants.PALLETTE)
    app.setStyle("Fusion")
    app.setWindowIcon(constants.ICONS.boat)

    window.show()
    sys.exit(app.exec())
