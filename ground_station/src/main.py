import sys
import threading
import http.server
import socketserver

from utils import constants, misc
from widgets import GroundStationWidget, ConsoleOutputWidget, AutopilotParamEditor, CameraWidget, InstanceHandler
from qtpy.QtWidgets import QApplication, QMainWindow, QTabWidget


class MainWindow(QMainWindow):
    """
    Main window for the ground station application.
    """

    @staticmethod
    def start_asset_server() -> None:
        """Start a quiet HTTP server for static assets."""

        Handler = lambda *args, **kwargs: http.server.SimpleHTTPRequestHandler(*args, directory=constants.ASSETS_DIR.as_posix(), **kwargs)
        with socketserver.TCPServer(("", constants.ASSET_SERVER_PORT), Handler) as httpd:
            print(f"[Info] Serving HTTP assets on port {constants.ASSET_SERVER_PORT}...")
            httpd.serve_forever()

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
            print(f"Error: {e}")

    def check_instance_connection(self) -> None:
        if constants.HAS_TELEMETRY_SERVER_INSTANCE_CHANGED:
            self.check_timer.stop()
            self.load_main_tabs()

    def load_main_tabs(self) -> None:
        try:
            self.main_widget.addTab(GroundStationWidget(), "Ground Station")
            self.main_widget.addTab(self.instance_handler, "Instance Handler")
            self.main_widget.addTab(AutopilotParamEditor(), "Autopilot Parameters")
            self.main_widget.addTab(CameraWidget(), "Camera Feed")
            print("[Info] Main application tabs loaded.")
        except Exception as e:
            print(f"Error loading main tabs: {e}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    constants.ICONS = misc.get_icons()
    window = MainWindow()
    threading.Thread(target=MainWindow.start_asset_server, daemon=True).start()
    app.setStyleSheet(constants.STYLE_SHEET)
    app.setPalette(constants.PALLETTE)
    app.setStyle("Fusion")
    app.setWindowIcon(constants.ICONS.boat)

    window.show()
    sys.exit(app.exec())
