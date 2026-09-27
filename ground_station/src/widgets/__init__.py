"""
Provide all top-level widgets for the ground station application.

This package includes the main :class:`GroundStationWidget`, console output,
map, camera, autopilot configuration, keybind, instance handler, and user guide
widgets, along with several easter egg games.
"""

__all__ = [
    "LAND_CLICK_PROMPT",
    "AutopilotConfigWidget",
    "CameraWidget",
    "ConsoleOutputWidget",
    "GraphViewer",
    "GroundStationWidget",
    "InstanceHandler",
    "KeybindConfigDialog",
    "LandClickPrompt",
    "MapOptionsHandler",
    "UserGuideWidget",
    "get_keybind_manager"
]

from .autopilot_config_widget import AutopilotConfigWidget
from .camera import CameraWidget
from .console_output import ConsoleOutputWidget
from .graph_viewer import GraphViewer
from .groundstation import GroundStationWidget
from .instance_handler import InstanceHandler
from .keybind_widget import KeybindConfigDialog, get_keybind_manager
from .map_widget import LAND_CLICK_PROMPT, LandClickPrompt, MapOptionsHandler
from .user_guide import UserGuideWidget
