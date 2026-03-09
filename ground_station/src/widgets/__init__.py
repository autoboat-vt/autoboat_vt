"""
Package for widgets in the ground station application.

Contains:
- GroundStationWidget: Main widget for the ground station interface.
- ConsoleOutputWidget: Widget for displaying console output.
- GraphViewer: Widget for viewing telemetry data.
- InstanceHandler: Manages instances of the application, displaying their information and allowing interaction.
- CameraWidget: Widget for displaying a camera feed.
- AutopilotConfigEditor: Widget for editing autopilot parameters.
- AutopilotConfigManager: Widget for managing autopilot configuration files.
- AutopilotConfigWidget: Tabbed widget combining the config editor and manager.
- TextEditWindow: Popup window for editing text.
"""

__all__ = [
    "AutopilotConfigEditor",
    "AutopilotConfigManager",
    "AutopilotConfigWidget",
    "CameraWidget",
    "ConsoleOutputWidget",
    "GraphViewer",
    "GroundStationWidget",
    "InstanceHandler",
    "TextEditWindow",
]

from .autopilot_config_widget.config_editor import AutopilotConfigEditor
from .autopilot_config_widget.config_manager import AutopilotConfigManager
from .autopilot_config_widget.config_widget import AutopilotConfigWidget
from .camera_widget.camera import CameraWidget
from .console_output import ConsoleOutputWidget
from .graph_viewer import GraphViewer
from .groundstation import GroundStationWidget
from .instance_handler import InstanceHandler
from .popup_edit import TextEditWindow