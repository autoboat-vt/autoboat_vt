"""
Package for dialog templates used across the application.

Contains:
- `base_dialog.py`: Base class for custom dialogs.
- `coordinate_input_dialog.py`: Dialog for entering lat/lon in multiple formats.
- `custom_buttons_dialog.py`: Message boxes with custom buttons.
- `text_input_dialog.py`: Simple typed input dialog.

"""

from .base_dialog import BaseDialog
from .coordinate_input_dialog import CoordinateInputDialog
from .custom_buttons_dialog import MessageBoxButton, show_custom_message_box, show_message_box
from .text_input_dialog import InputDialog

__all__ = [
    "BaseDialog",
    "CoordinateInputDialog",
    "InputDialog",
    "MessageBoxButton",
    "show_custom_message_box",
    "show_message_box",
]
