"""
Package containing the core autopilot logic and supporting code.

Includes:
- MotorboatAutopilot: Class implementing the autopilot logic for motorboats.
- SailboatAutopilot: Class implementing the autopilot logic for sailboats.
- DiscretePID: Class implementing a discrete PID controller.
- Position: Class representing a geographical position with latitude and longitude.
- misc: Module with miscellaneous utility functions.
- constants: Module containing constant values used throughout the autopilot library.
"""

__all__ = [
    "DiscretePID",
    "MotorboatAutopilot",
    "Position",
    "SailboatAutopilot",
    "misc",
    "constants",
]

from .motorboat_autopilot import MotorboatAutopilot
from .sailboat_autopilot import SailboatAutopilot
from .utils import constants, misc
from .utils.discrete_pid import DiscretePID
from .utils.position import Position
