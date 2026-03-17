from __future__ import annotations

import ctypes
from typing import ClassVar, Final

from typing_extensions import override

__all__ = [
    "BoatStatusPayload",
    "MotorboatStatusPayload",
    "SailboatStatusPayload",
]


class BoatStatusPayload(ctypes.LittleEndianStructure):
    """
    A class representing the payload that will be sent to the telemetry server
    to provide information about the boat's current state.

    Must inherit ``ctypes.LittleEndianStructure`` to ensure that the data is packed correctly when sent over the network.

    Inherits
    --------
    ``ctypes.LittleEndianStructure``
    """

    # 1 byte alignment (no padding)
    _pack_: ClassVar[int] = 1

    four_byte_fields: Final[tuple[tuple[str, ctypes._SimpleCData], ...]] = (
        ("latitude", ctypes.c_float),
        ("longitude", ctypes.c_float),
        ("distance_to_next_waypoint", ctypes.c_float),
        ("speed", ctypes.c_float),
        ("velocity_x", ctypes.c_float),
        ("velocity_y", ctypes.c_float),
        ("desired_heading", ctypes.c_float),
        ("heading", ctypes.c_float),
        ("desired_rudder_angle", ctypes.c_float),
    )

    one_byte_fields: Final[tuple[tuple[str, ctypes._SimpleCData], ...]] = (
        ("current_waypoint_index", ctypes.c_ubyte),
        ("autopilot_mode", ctypes.c_ubyte),
    )

    _fields_: ClassVar[tuple[tuple[str, ctypes._SimpleCData], ...]] = four_byte_fields + one_byte_fields
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
    
    @classmethod
    def construct_mapping(cls) -> list[list[str]]:
        """
        Constructs a mapping of the boat status payload field names to their corresponding data types.

        Returns
        -------
        list[list[str]]
            A list of lists, where each inner list contains the field name and data type of a field in the boat status payload.
        """
        
        return [[field_name, field_type.__name__] for field_name, field_type in cls._fields_]


class SailboatStatusPayload(BoatStatusPayload):
    """
    A class representing the payload that will be sent to the telemetry server
    to provide information about the sailboat's current state.

    Must inherit ``BoatStatusPayload`` to ensure that it has all the necessary fields for the telemetry payload.

    Inherits
    --------
    ``BoatStatusPayload``
    """

    # 1 byte alignment (no padding)
    _pack_: ClassVar[int] = 1

    sail_four_byte_fields: Final[tuple[tuple[str, ctypes._SimpleCData], ...]] = (
        ("true_wind_speed", ctypes.c_float),
        ("true_wind_angle", ctypes.c_float),
        ("apparent_wind_speed", ctypes.c_float),
        ("apparent_wind_angle", ctypes.c_float),
        ("desired_sail_angle", ctypes.c_float),
    )

    sail_one_byte_fields: Final[tuple[tuple[str, ctypes._SimpleCData], ...]] = (("full_autonomy_maneuver", ctypes.c_ubyte),)

    _fields_: ClassVar[tuple[tuple[str, ctypes._SimpleCData], ...]] = (
        sail_four_byte_fields + sail_one_byte_fields
    )
    _field_names: ClassVar[tuple[str, ...]] = tuple(name for name, _ in _fields_)

    @classmethod
    @override
    def construct_mapping(cls) -> list[list[str]]:
        """
        Constructs a mapping of the sailboat status payload field names to
        their corresponding data types.

        Returns
        -------
        list[list[str]]
            A list of lists, where each inner list contains the field name
            and data type of a field in the sailboat status payload.
        """
        
        parent_mapping = BoatStatusPayload.construct_mapping()
        child_mapping = [[field_name, field_type.__name__] for field_name, field_type in cls._fields_]
        return parent_mapping + child_mapping


class MotorboatStatusPayload(BoatStatusPayload):
    """
    A class representing the payload that will be sent to the telemetry server
    to provide information about the motorboat's current state.

    Must inherit ``BoatStatusPayload`` to ensure that it has all the necessary fields for the telemetry payload.

    Inherits
    --------
    ``BoatStatusPayload``
    """

    # 1 byte alignment (no padding)
    _pack_: ClassVar[int] = 1

    motor_four_byte_fields: Final[tuple[tuple[str, ctypes._SimpleCData], ...]] = (
        ("rpm", ctypes.c_float),
        ("duty_cycle", ctypes.c_float),
        ("amp_hours", ctypes.c_float),
        ("amp_hours_charged", ctypes.c_float),
        ("current_to_vesc", ctypes.c_float),
        ("voltage_to_motor", ctypes.c_float),
        ("voltage_to_vesc", ctypes.c_float),
        ("wattage_to_motor", ctypes.c_float),
        ("motor_temperature", ctypes.c_float),
        ("vesc_temperature", ctypes.c_float),
        ("time_since_vesc_startup", ctypes.c_float),
    )

    motor_one_byte_fields: Final[tuple[tuple[str, ctypes._SimpleCData], ...]] = ()

    _fields_: ClassVar[tuple[tuple[str, ctypes._SimpleCData], ...]] = motor_four_byte_fields + motor_one_byte_fields
    _field_names: ClassVar[tuple[str, ...]] = tuple(name for name, _ in _fields_)

    @classmethod
    def construct_mapping(cls) -> list[list[str]]:
        """
        Constructs a mapping of the motorboat status payload field names to
        their corresponding data types.

        Returns
        -------
        list[list[str]]
            A list of lists, where each inner list contains the field name
            and data type of a field in the motorboat status payload.
        """
        
        parent_mapping = BoatStatusPayload.construct_mapping()
        child_mapping = [[field_name, field_type.__name__] for field_name, field_type in cls._fields_]
        return parent_mapping + child_mapping
