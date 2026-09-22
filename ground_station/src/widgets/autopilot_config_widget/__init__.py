"""
Package housing the autopilot configuration widget and supporting classes.

Exposes:
- AutopilotConfigWidget: The main widget for configuring the autopilot.

Contains:
- `config_widget.py`: File containing the main `AutopilotConfigWidget` class.
- `config_editor.py`: File containing classes enabling editing of individual configuration parameters.
- `config_manager.py`: File containing classes for managing the loading and saving of autopilot parameter configurations.
"""

__all__ = ["AutopilotConfigWidget"]

from .config_widget import AutopilotConfigWidget
