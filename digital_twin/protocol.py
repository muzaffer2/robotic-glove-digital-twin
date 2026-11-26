"""ASCII serial protocol parser and command builders for STM32U545 valve firmware."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


T150PGSA3XX_MAX_KPA = 1034.2136  # 150 psi Gage in kPa


@dataclass(slots=True)
class TelemetrySample:
    """Single 50 Hz telemetry sample emitted by the STM32 firmware."""

    timestamp_s: float
    s0_kpa: float
    s1_kpa: float
    v1: int
    v2: int
    pwm1: int = 0
    pwm2: int = 0
    temp1_c: float = 25.0
    is_legacy_normalized: bool = False

    @property
    def s0_connected(self) -> bool:
        return abs(self.s0_kpa + 1.0) > 0.01

    @property
    def s1_connected(self) -> bool:
        return abs(self.s1_kpa + 1.0) > 0.01

    @property
    def duty1(self) -> float:
        return max(0.0, min(1.0, self.pwm1 / 999.0))

    @property
    def duty2(self) -> float:
        return max(0.0, min(1.0, self.pwm2 / 999.0))


@dataclass(slots=True)
class EventMessage:
    """Non-telemetry line (ACK, PONG, STATUS, WARN, ERROR, SYSTEM) from STM32."""

    timestamp_s: float
    kind: str
    raw: str


def parse_stm32_line(
    line: str, timestamp_s: float
) -> tuple[Optional[TelemetrySample], Optional[EventMessage]]:
    """Parse one newline-stripped line from STM32 into telemetry or event."""
    text = line.strip()
    if not text:
        return None, None

    if text.startswith("DATA,"):
        fields: dict[str, str] = {}
        for part in text[5:].split(","):
            if ":" in part:
                k, v = part.split(":", 1)
                fields[k.strip()] = v.strip()
        try:
            s0 = float(fields["S0"])
            s1 = float(fields["S1"])
            v1 = int(fields["V1"])
            v2 = int(fields["V2"])
            is_legacy = "PWM1" not in fields
            pwm1 = int(fields["PWM1"]) if "PWM1" in fields else (999 if v1 else 0)
            pwm2 = int(fields["PWM2"]) if "PWM2" in fields else (999 if v2 else 0)
            temp1 = float(fields["T1"]) if "T1" in fields else 25.0
        except (KeyError, ValueError):
            return None, EventMessage(timestamp_s=timestamp_s, kind="MALFORMED", raw=text)

        return (
            TelemetrySample(
                timestamp_s=timestamp_s,
                s0_kpa=s0,
                s1_kpa=s1,
                v1=v1,
                v2=v2,
                pwm1=pwm1,
                pwm2=pwm2,
                temp1_c=temp1,
                is_legacy_normalized=is_legacy,
            ),
            None,
        )

    kind = text.split(",", 1)[0].strip().upper()
    return None, EventMessage(timestamp_s=timestamp_s, kind=kind, raw=text)


def cmd_ping() -> str:
    return "PING\n"


def cmd_status() -> str:
    return "STATUS\n"


def cmd_valve(valve_id: int, enabled: bool) -> str:
    if valve_id not in (1, 2):
        raise ValueError("valve_id must be 1 or 2")
    return f"VALVE,{valve_id},{'ON' if enabled else 'OFF'}\n"


def cmd_pwm(valve_id: int, duty_0_to_999: int) -> str:
    if valve_id not in (1, 2):
        raise ValueError("valve_id must be 1 or 2")
    duty = max(0, min(999, int(duty_0_to_999)))
    return f"PWM,{valve_id},{duty}\n"


def cmd_target_pressure(target_kpa: Optional[float]) -> str:
    if target_kpa is None:
        return "TARGET,OFF\n"
    clamped = max(0.0, float(target_kpa))
    return f"TARGET,{clamped:.2f}\n"


def cmd_zero(reset: bool = False) -> str:
    return "ZERO,RESET\n" if reset else "ZERO\n"


def cmd_max_pressure(max_kpa: float) -> str:
    clamped = max(1.0, min(1000.0, float(max_kpa)))
    return f"MAXP,{clamped:.2f}\n"
