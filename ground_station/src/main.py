import sys
from utils import constants, misc
from widgets import GroundStationWidget, ConsoleOutputWidget, AutopilotParamEditor, CameraWidget, InstanceHandler
from qtpy.QtWidgets import QApplication, QMainWindow, QTabWidget


class MainWindow(QMainWindow):
    """
    Main window for the ground station application.

    Inherits
    -------
    `QMainWindow`
    """

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("SailBussy Ground Station")
        self.setGeometry(constants.WINDOW_BOX)

        self.main_widget = QTabWidget()
        self.setCentralWidget(self.main_widget)

        self.tabs_loaded = False

        try:
            # load console first to capture any startup messages
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

    def load_main_tabs(self) -> None:
        """Load the main application tabs after an instance is connected."""

        if not self.tabs_loaded:
            try:
                self.main_widget.addTab(GroundStationWidget(), "Ground Station")
                self.main_widget.addTab(self.instance_handler, "Instance Handler")
                self.main_widget.addTab(AutopilotParamEditor(), "Autopilot Parameters")
                self.main_widget.addTab(CameraWidget(), "Camera Feed")
                self.tabs_loaded = True
                print("[Info] Main application tabs loaded.")

            except Exception as e:
                print(f"Error loading main tabs: {e}")

    def check_instance_connection(self) -> None:
        """Check if an instance has been connected and load tabs if needed."""

        if constants.HAS_TELEMETRY_SERVER_INSTANCE_CHANGED and not self.tabs_loaded:
            self.check_timer.stop()
            self.load_main_tabs()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    constants.ICONS = misc.get_icons()
    window = MainWindow()
    app.setStyleSheet(constants.STYLE_SHEET)
    app.setPalette(constants.PALLETTE)
    app.setStyle("Fusion")
    app.setWindowIcon(constants.ICONS.boat)

    window.show()
    sys.exit(app.exec())
