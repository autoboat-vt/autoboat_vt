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

        try:
            # load console first to capture any startup messages
            self.main_widget.addTab(ConsoleOutputWidget(), "Console Output")

            self.main_widget.addTab(GroundStationWidget(), "Ground Station")
            self.main_widget.addTab(InstanceHandler(), "Instance Handler")
            self.main_widget.addTab(AutopilotParamEditor(), "Autopilot Parameters")
            self.main_widget.addTab(CameraWidget(), "Camera Feed")

        except Exception as e:
            print(f"Error: {e}")

        self.main_widget.setCurrentIndex(2)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    constants.ICONS = misc.__get_icons()
    window = MainWindow()
    app.setStyleSheet(constants.STYLE_SHEET)
    app.setPalette(constants.PALLETTE)
    app.setStyle("Fusion")
    app.setWindowIcon(constants.ICONS.boat)

    window.show()
    sys.exit(app.exec())
