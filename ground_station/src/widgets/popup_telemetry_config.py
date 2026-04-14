from collections.abc import Callable

from qtpy.QtWidgets import QCheckBox, QLabel, QVBoxLayout, QWidget


class EditTelemetryConfigWindow(QWidget):
    def __init__(self, waypoints_checker_callback: Callable, debugging_symbols_callback: Callable) -> None:
        super().__init__()
        self.layout = QVBoxLayout(self)
        self.setLayout(self.layout)

        self.feedback_text = QLabel("")


        self.waypoints_checker_toggle = QCheckBox("Enable popup when waypoints change?")
        self.waypoints_checker_toggle.setChecked(False)
        self.waypoints_checker_toggle.setToolTip(
            "If enabled, a popup will appear when the waypoints on the telemetry server change.",
        )

        self.waypoints_checker_toggle.stateChanged.connect(
            lambda state: waypoints_checker_callback(state) or self.feedback_text.setText("Updated Waypoints Popup Config.")
        )


        self.debugging_symbols_toggle = QCheckBox("Enable sailboat debugging symbols?")
        self.debugging_symbols_toggle.setChecked(False)
        self.debugging_symbols_toggle.setToolTip(
            "If enabled, the sailboat debugging symbols will appear on the screen.",
        )
        self.debugging_symbols_toggle.stateChanged.connect(
            lambda state: debugging_symbols_callback(state) or self.feedback_text.setText(
"""Updated Debugging Symbols Config.
orange - wind
black - velocity
red - no-go zone
pink - decision zone 2"""
            )
        )

        self.layout.addWidget(self.waypoints_checker_toggle)
        self.layout.addWidget(self.debugging_symbols_toggle)
        self.layout.addWidget(self.feedback_text)
    
