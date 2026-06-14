"""
Package for widgets in the ground station application.

Subpackages:
- ``map_widget``: Contains the map widget and related classes for managing waypoints and map options.
- ``autopilot_config_widget``: Contains the autopilot configuration widget and related classes for managing autopilot settings.
- ``camera_widget``: Contains the camera widget for displaying camera feeds.

Modules:
- GroundStationWidget: Main widget for the ground station interface.
- ConsoleOutputWidget: Widget for displaying console output.
- GraphViewer: Widget for viewing telemetry data.
- InstanceHandler: Manages instances of the application, displaying their information and allowing interaction.
- CameraWidget: Widget for displaying a camera feed.
- AutopilotConfigWidget: Tabbed widget combining the config editor and manager.
- UserGuideWidget: Widget for displaying the documentation.
"""

__all__ = [
    "AutopilotConfigWidget",
    "CameraWidget",
    "ConsoleOutputWidget",
    "GraphViewer",
    "GroundStationWidget",
    "InstanceHandler",
    "MapOptionsHandler",
    "UserGuideWidget",
    "run",
]

from .autopilot_config_widget import AutopilotConfigWidget
from .camera_widget import CameraWidget
from .console_output import ConsoleOutputWidget
from .graph_viewer import GraphViewer
from .groundstation import GroundStationWidget
from .instance_handler import InstanceHandler
from .map_widget import MapOptionsHandler, run
from .user_guide import UserGuideWidget
