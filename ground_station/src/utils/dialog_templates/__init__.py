"""
Package for dialog templates used across the application.

Contains:
- `base_dialog.py`: Base class for custom dialogs.
- `coordinate_input_dialog.py`: Dialog for entering lat/lon in multiple formats.
- `custom_buttons_dialog.py`: Message boxes with custom buttons.
- `random_waypoints_dialog.py`: Dialog for generating random waypoints around the boat.
- `text_input_dialog.py`: Simple typed input dialog.

"""

from .base_dialog import BaseDialog
from .coordinate_input_dialog import CoordinateInputDialog
from .custom_buttons_dialog import MessageBoxButton, show_custom_message_box, show_message_box
from .random_waypoints_dialog import RandomWaypointsDialog
from .text_input_dialog import InputDialog

__all__ = [
    "BaseDialog",
    "CoordinateInputDialog",
    "InputDialog",
    "MessageBoxButton",
    "RandomWaypointsDialog",
    "show_custom_message_box",
    "show_message_box",
]
