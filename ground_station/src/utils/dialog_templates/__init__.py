"""
Package for dialog templates used across the application.

Provides reusable dialog widgets and convenience functions for common dialog patterns.
"""

from .base_dialog import BaseDialog
from .custom_buttons_dialog import MessageBoxButton, show_custom_message_box, show_message_box
from .text_input_dialog import show_input_dialog

__all__ = [
    "BaseDialog",
    "InputDialog",
    "MessageBoxButton",
    "show_custom_message_box",
    "show_input_dialog",
    "show_message_box",
]
