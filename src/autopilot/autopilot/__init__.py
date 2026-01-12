"""
Package containing packages and classes responsible for managing communication with telemetry server, autopilot logic, and boat sensors.

This package also houses the configuration files for the autopilot nodes.

Contains:
- autopilot_library: Package containing the core autopilot logic and supporting code.
- MotorboatAutopilotNode: Class implementing the ROS2 node for motorboat autopilot.
- SailboatAutopilotNode: Class implementing the ROS2 node for sailboat autopilot.
- TelemetryNode: Class implementing the ROS2 node for communication between the autopilot nodes and the telemetry server.
"""

__all__ = [
    "autopilot_library",
    "MotorboatAutopilotNode",
    "SailboatAutopilotNode",
    "TelemetryNode",
]

from . import autopilot_library
from .motorboat_autopilot_node import MotorboatAutopilotNode
from .sailboat_autopilot_node import SailboatAutopilotNode
from .telemetry_node import TelemetryNode
