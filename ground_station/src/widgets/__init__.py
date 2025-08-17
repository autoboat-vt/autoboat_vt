"""
Module for widgets in the ground station application.

Contains:
- GroundStationWidget: Main widget for the ground station interface.
- ConsoleOutputWidget: Widget for displaying console output.
- CameraWidget: Widget for displaying a camera feed.
- AutopilotParamEditor: Widget for editing autopilot parameters.
- TextEditWindow: Popup window for editing text.
"""

__all__ = ["AutopilotParamEditor", "CameraWidget", "ConsoleOutputWidget", "GroundStationWidget", "TextEditWindow"]

from .groundstation import GroundStationWidget
from .console_output import ConsoleOutputWidget
from .camera_widget.camera import CameraWidget
from .autopilot_param_editor.editor import AutopilotParamEditor
from .popup_edit import TextEditWindow
