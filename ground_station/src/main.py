import sys
import constants
from widgets.groundstation import GroundStationWidget
from widgets.camera_widget.camera import CameraWidget
from widgets.console_output import ConsoleOutputWidget
from widgets.autopilot_param_editor.editor import AutopilotParamEditor

from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget
from PyQt5.QtGui import QIcon


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
        try:
            # load console first to capture any startup messages
            self.main_widget.addTab(ConsoleOutputWidget(), "Console Output")

            self.main_widget.addTab(GroundStationWidget(), "Ground Station")
            self.main_widget.addTab(AutopilotParamEditor(), "Autopilot Parameters")
            self.main_widget.addTab(CameraWidget(), "Camera Feed")
        except Exception as e:
            print(f"Error: {e}")
        self.main_widget.setCurrentIndex(1)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setApplicationName("SailBussy Ground Station")
    app.setStyle("Fusion")
    constants.ICONS = constants.__get_icons()
    app_icon: QIcon = constants.ICONS.boat
    app.setWindowIcon(app_icon)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
