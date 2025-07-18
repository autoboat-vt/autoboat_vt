import sys
import constants
from widgets.groundstation import GroundStationWidget
from widgets.camera_widget.camera import CameraWidget
from widgets.autopilot_param_editor.editor import AutopilotParamEditor
from widgets.console_output import ConsoleOutputWidget

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
        self.setStyleSheet(constants.STYLE_SHEET)
        self.setPalette(constants.PALLETTE)

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
    constants.ICONS = constants.__get_icons()
    window = MainWindow()
    app.setStyle("Fusion")
    app.setWindowIcon(constants.ICONS.boat)

    window.show()
    sys.exit(app.exec())
