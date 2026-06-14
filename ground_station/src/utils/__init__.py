"""
This package contains various utility classes and functions used throughout the application.

Subpackages
-----------
- ``syntax_highlighters``: Contains syntax highlighters for various text formats.
- ``widget_size_controllers``: Contains classes for controlling the size of widgets.
- ``dialog_templates``: Contains template classes for creating dialog windows.

Modules
-------
- ``constants``: Contains constant values used throughout the application.
- ``misc``: Contains miscellaneous utility functions.
- ``thread_classes``: Contains classes for managing threads in the application.

Classes
-------
- ``TextEditWindow``: A simple text edit window that emits the entered text when closed.
- ``StateManager``: A class for managing shared application state stored in a JSON file.
- ``TextEditWindow``: A simple text edit window that emits the entered text when closed.
- ``DataLogger``: A class for logging data entries to a file.
"""

__all__ = [
    "DataLogger",
    "StateManager",
    "TextEditWindow",
    "constants",
    "dialog_templates",
    "misc",
    "syntax_highlighters",
    "thread_classes",
    "widget_size_controllers",
]

from . import constants, dialog_templates, misc, syntax_highlighters, thread_classes, widget_size_controllers
from .data_logger import DataLogger
from .popup_edit import TextEditWindow
from .state_manager import StateManager
