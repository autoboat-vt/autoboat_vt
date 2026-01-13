"""
Package containing modules and classes which aid in the implementation of the autopilot logic.

Contains:
- constants: Module containing constant values used throughout the autopilot library.
- function_library: Module with miscellaneous utility functions.
- DiscretePID: Class implementing a discrete PID controller.
- Position: Class representing a geographical position with latitude and longitude.
"""

__all__ = [
    "constants",
    "function_library",
    "DiscretePID",
    "Position",
]

from . import constants, function_library
from .discrete_pid import DiscretePID
from .position import Position
