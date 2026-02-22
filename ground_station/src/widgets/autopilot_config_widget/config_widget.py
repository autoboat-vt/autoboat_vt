from qtpy.QtWidgets import QTabWidget

from .config_editor import AutopilotConfigEditor
from .config_manager import AutopilotConfigManager


class AutopilotConfigWidget(QTabWidget):
    """Widget for configuring the autopilot settings."""

    def __init__(self) -> None:
        super().__init__()

        self.setTabPosition(QTabWidget.South)
        autopilot_config_manager = AutopilotConfigManager()
        self.addTab(AutopilotConfigEditor(), "Edit Parameters")
        self.addTab(autopilot_config_manager, "Manage Configurations")
        self.setWindowTitle("Autopilot Configuration")
