"""
Module for widgets in the ground station application.

Contains:
- GroundStationWidget: Main widget for the ground station interface.
- ConsoleOutputWidget: Widget for displaying console output.
- InstanceHandler: Manages instances of the application, displaying their information and allowing interaction.
- CameraWidget: Widget for displaying a camera feed.
- AutopilotParamEditor: Widget for editing autopilot parameters.
- TextEditWindow: Popup window for editing text.
"""

__all__ = ["AutopilotParamEditor", "CameraWidget", "ConsoleOutputWidget", "GroundStationWidget", "InstanceHandler", "TextEditWindow"]

from .autopilot_param_editor.editor import AutopilotParamEditor
from .camera_widget.camera import CameraWidget
from .console_output import ConsoleOutputWidget
from .groundstation import GroundStationWidget
from .instance_handler import InstanceHandler
from .popup_edit import TextEditWindow
