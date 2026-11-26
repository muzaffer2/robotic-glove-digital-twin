"""Digital Twin package for the STM32U545 textile-based pneumatic robotic glove."""

from .pneumatics import (
    FingerKinematics,
    PneumaticChamberModel,
    PneumaticParameters,
    angle_to_pressure_kpa,
    pressure_to_finger_angles,
)
from .protocol import TelemetrySample, parse_stm32_line
from .serial_bridge import STM32Bridge, list_serial_ports

__all__ = [
    "FingerKinematics",
    "PneumaticChamberModel",
    "PneumaticParameters",
    "STM32Bridge",
    "TelemetrySample",
    "angle_to_pressure_kpa",
    "list_serial_ports",
    "parse_stm32_line",
    "pressure_to_finger_angles",
]
