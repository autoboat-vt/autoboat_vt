"""
Package containing modules and classes which aid in the implementation of the autopilot logic.

Contains:
- constants: Module containing constant values used throughout the autopilot library.
- utils_function_library: Module with miscellaneous utility functions.
- DiscretePID: Class implementing a discrete PID controller.
- Position: Class representing a geographical position with latitude and longitude.
- telemetry_payloads: Module containing classes representing the structs that will be
  sent to the telemetry server to provide information about the boat's current state.
"""

# used to specify what is available to import from this file
__all__ = [
    "DiscretePID",
    "Position",
    "constants",
    "telemetry_payloads",
    "utils_function_library",
]

from . import constants, telemetry_payloads, utils_function_library
from .discrete_pid import DiscretePID
from .position import Position
