from qtpy.QtCore import Signal
from qtpy.QtWidgets import QTabWidget

from .config_editor import AutopilotConfigEditor
from .config_manager import AutopilotConfigManager


class AutopilotConfigWidget(QTabWidget):
    """
    Widget for configuring the autopilot settings.
    
    Parameters
    ----------
    refresh_signal: ``Signal``
        Signal emitted when the autopilot configuration needs to be refreshed.

    Inherits
    -------
    ``QTabWidget``
    """

    def __init__(self, refresh_signal: Signal) -> None:
        super().__init__()

        self.setTabPosition(QTabWidget.South)
        autopilot_config_manager = AutopilotConfigManager()
        self.addTab(AutopilotConfigEditor(refresh_signal), "Edit Parameters")
        self.addTab(autopilot_config_manager, "Manage Configurations")
        self.setWindowTitle("Autopilot Configuration")
