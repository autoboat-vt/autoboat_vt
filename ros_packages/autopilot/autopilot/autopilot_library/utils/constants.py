from __future__ import annotations

from enum import Enum
from pathlib import Path

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# used to specify what is available to import from this file
__all__ = [
    "BASE_DIRECTORY",
    "CONFIG_DIRECTORY",
    "QOS_AUTOPILOT_PARAMETER_CONFIG_PATH",
    "TELEMETRY_SERVER_URL",
    "MotorboatControlModes",
    "PropellerMotorControlMode",
    "SailboatAutopilotStates",
    "SailboatControlModes",
    "TelemetryStatus",
]


class SailboatControlModes(Enum):
    """An enum containing the different control modes that the sailboat can be in."""

    DISABLED = 0
    FULL_RC = 1
    HOLD_BEST_SAIL = 2
    HOLD_HEADING = 3
    HOLD_HEADING_AND_BEST_SAIL = 4
    WAYPOINT_MISSION = 5
    EMERGENCY_STOP = 6


class SailboatAutopilotStates(Enum):
    """
    An enum containing the different states that the sailboat can be in.

    NOTE
    ----
    If you would like to learn more about the difference between a port/ starboard tack
    and other similar terms, please look at the following resource:
    https://rpayc.com.au/wp-content/uploads/2020/11/Basic_Terminology.pdf?srsltid=AfmBOop6ujPja2Fu5hKn8OYW-YlFAZVxAZTDO8zgEOzT3R4wk-7PCQzf

    For more information about what tacking and jibing are, please read the following:
    https://captainsword.com/tacking-and-jibing
    """

    NA = 0
    DOWNWIND_SAILING = 1
    PORT_TACK = 2          # On a tack where the wind vector is to the left of the boat (port and left both have 4 letters)
    STARBOARD_TACK = 3     # On a tack where the wind vector is to the right of the boat
    CW_TACKING = 4         # Switching tacks from starboard to port tack
    CCW_TACKING = 5        # Switching tacks from port to staboard tack
    STALL_WIGGLE_TO_PORT_TACK = 6         # We have stalled in the no sail zone and need to wiggle to port tack
    STALL_WIGGLE_TO_STARBOARD_TACK = 7    # We have stalled in the no sail zone and need to wiggle to the starboard tack






class MotorboatControlModes(Enum):
    """An enum containing the different control modes that the motorboat can be in."""

    DISABLED = 0
    FULL_RC = 1
    HOLD_HEADING = 2
    WAYPOINT_MISSION = 3
    EMERGENCY_STOP = 4

class PropellerMotorControlMode(Enum):
    """
    An enum containing the different propeller control modes for the motorboat.

    RPM directly commands the rotations per minute of the propeller, the duty cycle
    directly controls the voltage going to the motor, and the current, obviously,
    controls the current going to the propeller motor.
    """

    RPM = 0
    DUTY_CYCLE = 1
    CURRENT = 2





class TelemetryNodeModes(Enum):
    """
    An enum containing whether or not we are relaying the telemety data of a sailboat
    or of a motorboat. The things that we need to send to the telemetry server changes
    based on whether or not we are using a sailboat or motorboat.
    """

    SAILBOAT = 0
    MOTORBOAT = 1


class TelemetryStatus(Enum):
    """Enum representing the status of communication with the telemetry server."""

    SUCCESS = 0
    FAILURE = 1




QOS_AUTOPILOT_PARAMETER_CONFIG_PATH = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# don't put '/' at the end of the URL
TELEMETRY_SERVER_URL = "https://vt-autoboat-telemetry.uk"

BASE_DIRECTORY = Path(__file__).resolve().parent.parent.parent
CONFIG_DIRECTORY = BASE_DIRECTORY / "config"
