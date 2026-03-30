from __future__ import annotations

from enum import Enum
from pathlib import Path

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# used to specify what is available to import from this file
__all__ = [
    "BASE_DIRECTORY",
    "CONFIG_DIRECTORY",
    "QOS_AUTOPILOT_PARAM_CONFIG_PATH",
    "TELEMETRY_SERVER_URL",
    "MotorboatAutopilotMode",
    "MotorboatControls",
    "SailboatAutopilotMode",
    "SailboatManeuvers",
    "SailboatStates",
    "TelemetryStatus",
]


class SailboatAutopilotMode(Enum):
    """An enum containing the different modes that the sailboat autopilot can be in."""

    DISABLED = 0
    FULL_RC = 1
    HOLD_BEST_SAIL = 2
    HOLD_HEADING = 3
    HOLD_HEADING_AND_BEST_SAIL = 4
    WAYPOINT_MISSION = 5


class SailboatStates(Enum):
    """An enum containing the different states that the sailboat autopilot can be in."""

    NA = -1
    NORMAL = 0
    CW_TACKING = 1
    CCW_TACKING = 2
    STALL = 3
    # JIBE = 4

class SailboatManeuvers(Enum):
    """
    An enum containing the different sailing maneuvers that the sailboat autopilot can perform.

    Note
    ----
    For more information about what tacking and jibing are, please read the following:
    https://captainsword.com/tacking-and-jibing
    """

    AUTOPILOT_DISABLED = 0
    STANDARD = 1
    TACK = 2
    JIBE = 3


class MotorboatAutopilotMode(Enum):
    """An enum containing the different modes that the motorboat autopilot can be in."""

    DISABLED = 0
    FULL_RC = 1
    HOLD_HEADING = 2
    WAYPOINT_MISSION = 3

class MotorboatControls(Enum):
    """An enum containing the different motorboat control types."""

    RPM = 0
    DUTY_CYCLE = 1
    CURRENT = 2


class TelemetryStatus(Enum):
    """Enum representing the status of communication with the telemetry server."""

    SUCCESS = 0
    FAILURE = 1


class DiagnosticMessageIntensity(Enum):
    """
    Enumeration representing the intensity levels of diagnostic messages.

    Attributes
    ----------
    - ```INFO```: Represents informational messages that indicate normal operation or
        provide general information.

    - ```WARNING```: Represents warning messages that indicate a potential issue or a
        situation that may require attention but does not necessarily indicate an immediate problem.

    - ```ERROR```: Represents error messages that indicate a significant problem or
        failure that requires immediate attention and may impact the functionality of the system.
    """

    INFO = 1
    WARNING = 2
    ERROR = 3


QOS_AUTOPILOT_PARAM_CONFIG_PATH = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# don't put '/' at the end of the URL
TELEMETRY_SERVER_URL = "https://vt-autoboat-telemetry.uk"

BASE_DIRECTORY = Path(__file__).resolve().parent.parent.parent
CONFIG_DIRECTORY = BASE_DIRECTORY / "config"
