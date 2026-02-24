import ctypes
from enum import Enum
from pathlib import Path
from typing import ClassVar, Final

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# used to specify what is available to import from this file
__all__ = [
    "BASE_DIRECTORY",
    "BOAT_STATUS_MAPPING",
    "CONFIG_DIRECTORY",
    "QOS_AUTOPILOT_CONFIG",
    "TELEMETRY_SERVER_URL",
    "BoatStatusPayload",
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

class BoatStatusPayload(ctypes.Structure):
    """
    A class representing the payload that will be sent to the telemetry server
    to provide information about the boat's current state.

    Inherits
    --------
    ``ctypes.Structure``
    """

    CType = type[ctypes._SimpleCData]

    _pack_: ClassVar[int] = 1

    four_byte_fields: Final[tuple[tuple[str, CType], ...]] = (
        ("latitude", ctypes.c_int32),
        ("longitude", ctypes.c_int32),
        ("distance_to_next_waypoint", ctypes.c_uint32),
    )

    two_byte_fields: Final[tuple[tuple[str, CType], ...]] = (
        ("speed", ctypes.c_int16),
        ("velocity_x", ctypes.c_int16),
        ("velocity_y", ctypes.c_int16),
        ("desired_heading", ctypes.c_int16),
        ("heading", ctypes.c_int16),
        ("true_wind_speed", ctypes.c_uint16),
        ("true_wind_angle", ctypes.c_int16),
        ("apparent_wind_speed", ctypes.c_uint16),
        ("apparent_wind_angle", ctypes.c_int16),
        ("desired_sail_angle", ctypes.c_int16),
        ("desired_rudder_angle", ctypes.c_int16),
    )

    one_byte_fields: Final[tuple[tuple[str, CType], ...]] = (
        ("current_waypoint_index", ctypes.c_uint8),
        ("autopilot_mode", ctypes.c_uint8),
        ("full_autonomy_maneuver", ctypes.c_uint8),
    )

    _fields_: ClassVar[tuple[tuple[str, CType], ...]] = four_byte_fields + two_byte_fields + one_byte_fields
    _field_names: ClassVar[tuple[str, ...]] = tuple(name for name, _ in _fields_)

    def __str__(self) -> str:
        """
        Return a string representation of the telemetry payload, which includes the name and value of each field in the payload.

        Returns
        -------
        str
            A string representation of the telemetry payload.
        """

        return "\n".join(f"{field_name}: {getattr(self, field_name)}" for field_name in self._field_names)

QOS_AUTOPILOT_PARAM_CONFIG_PATH = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

BOAT_STATUS_MAPPING: list[str] = [
    "position",
    "state",
    "full_autonomy_maneuver",
    "speed",
    "velocity_vector",
    "bearing",
    "heading",
    "true_wind_speed",
    "true_wind_angle",
    "apparent_wind_speed",
    "apparent_wind_angle",
    "sail_angle",
    "rudder_angle",
    "current_waypoint_index",
    "distance_to_next_waypoint",
]

# don't put '/' at the end of the URL
TELEMETRY_SERVER_URL = "https://vt-autoboat-telemetry.uk"

BASE_DIRECTORY = Path(__file__).resolve().parent.parent.parent
CONFIG_DIRECTORY = BASE_DIRECTORY / "config"
