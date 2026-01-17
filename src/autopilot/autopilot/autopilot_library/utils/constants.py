from enum import Enum
from pathlib import Path


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


# don't put '/' at the end of the URL
TELEMETRY_SERVER_URL = "https://vt-autoboat-telemetry.uk:8443"

BASE_DIRECTORY = Path(__file__).resolve().parent.parent.parent
CONFIG_DIRECTORY = BASE_DIRECTORY / "config"
