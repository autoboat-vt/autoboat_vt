"""
Package housing the keybind configuration widget and supporting classes.

Exposes:
- KeybindConfigDialog: Dialog for viewing and editing keybinds.
- get_keybind_manager: Function to retrieve the shared KeybindManager instance.
- normalize_key_string: Function to normalize a key combination string for storage and comparison.
- qt_key_event_to_string: Function to convert a QKeyEvent into a normalized combination string.

Contains:
- `keybind_manager.py`: Module containing the keybind manager (registry, persistence, and dispatch).
- `keybind_widget.py`: Module containing the :class:`KeybindConfigDialog` and the
    :class:`KeyCaptureItem` used to capture new bindings.
"""

__all__ = [
    "KeybindConfigDialog",
    "get_keybind_manager",
    "normalize_key_string",
    "qt_key_event_to_string",
]

from .keybind_manager import get_keybind_manager, normalize_key_string, qt_key_event_to_string
from .keybind_widget import KeybindConfigDialog
