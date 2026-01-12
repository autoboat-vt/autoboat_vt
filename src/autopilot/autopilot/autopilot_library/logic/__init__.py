"""
Package containing classes responsible for implementing the autopilot logic.

Contains:
- MotorboatAutopilot: Module implementing the autopilot logic for motorboats.
- SailboatAutopilot: Module implementing the autopilot logic for sailboats.
"""

__all__ = [
    "MotorboatAutopilot",
    "SailboatAutopilot",
]

from .motorboat_autopilot import MotorboatAutopilot
from .sailboat_autopilot import SailboatAutopilot
