from qtpy.QtWidgets import QTabWidget

from .config_editor import AutopilotConfigEditor
from .config_manager import AutopilotConfigManager


class AutopilotConfigWidget(QTabWidget):
    """Widget for configuring the autopilot settings."""

    def __init__(self) -> None:
        super().__init__()

        self.setTabPosition(QTabWidget.South)
        self.addTab(AutopilotConfigEditor(), "Edit Parameters")
        self.addTab(AutopilotConfigManager(), "Manage Configurations")
        self.setWindowTitle("Autopilot Configuration")
