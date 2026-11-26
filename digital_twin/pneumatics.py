"""Pneumatic actuator dynamics and soft robotic finger kinematics models."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(slots=True)
class PneumaticParameters:
    """Physical parameters for a single textile pneumatic glove actuator channel."""

    supply_pressure_kpa: float = 160.0
    ambient_pressure_kpa: float = 0.0  # Gage reference
    tau_charge_s: float = 0.45
    tau_discharge_s: float = 0.35
    tau_leak_s: float = 45.0

    # Nonlinear pressure-to-flexion calibration (soft textile bending actuator)
    p_threshold_kpa: float = 12.0
    p_half_kpa: float = 75.0
    steepness_kpa: float = 22.0
    max_mcp_deg: float = 75.0
    max_pip_deg: float = 95.0
    max_dip_deg: float = 60.0


@dataclass(slots=True)
class FingerKinematics:
    """Joint angles for one articulated glove finger (in degrees and radians)."""

    mcp_deg: float
    pip_deg: float
    dip_deg: float

    @property
    def total_curl_deg(self) -> float:
        return self.mcp_deg + self.pip_deg + self.dip_deg

    @property
    def mcp_rad(self) -> float:
        return math.radians(self.mcp_deg)

    @property
    def pip_rad(self) -> float:
        return math.radians(self.pip_deg)

    @property
    def dip_rad(self) -> float:
        return math.radians(self.dip_deg)


def _soft_bend_ratio(pressure_kpa: float, params: PneumaticParameters) -> float:
    """Map gage pressure (kPa) to normalized finger flexion [0.0, 1.0]."""
    if pressure_kpa <= params.p_threshold_kpa:
        return 0.0

    # Shifted logistic curve normalized so ratio(p_threshold) == 0.0
    z = (pressure_kpa - params.p_half_kpa) / max(1e-3, params.steepness_kpa)
    z0 = (params.p_threshold_kpa - params.p_half_kpa) / max(1e-3, params.steepness_kpa)
    sig = 1.0 / (1.0 + math.exp(-max(-30.0, min(30.0, z))))
    sig0 = 1.0 / (1.0 + math.exp(-max(-30.0, min(30.0, z0))))
    ratio = (sig - sig0) / max(1e-6, 1.0 - sig0)
    return max(0.0, min(1.0, ratio))


def pressure_to_finger_angles(
    pressure_kpa: float,
    params: PneumaticParameters | None = None,
) -> FingerKinematics:
    """Compute MCP/PIP/DIP flexion angles from actuator gage pressure (kPa)."""
    cfg = params or PneumaticParameters()
    ratio = _soft_bend_ratio(pressure_kpa, cfg)
    return FingerKinematics(
        mcp_deg=ratio * cfg.max_mcp_deg,
        pip_deg=ratio * cfg.max_pip_deg,
        dip_deg=ratio * cfg.max_dip_deg,
    )


def angle_to_pressure_kpa(
    curl_fraction: float,
    params: PneumaticParameters | None = None,
) -> float:
    """Invert the bending curve: normalized finger curl [0..1] -> target pressure (kPa)."""
    cfg = params or PneumaticParameters()
    r = max(0.0, min(0.985, float(curl_fraction)))
    if r <= 0.0:
        return 0.0

    z0 = (cfg.p_threshold_kpa - cfg.p_half_kpa) / max(1e-3, cfg.steepness_kpa)
    sig0 = 1.0 / (1.0 + math.exp(-max(-30.0, min(30.0, z0))))
    sig = sig0 + r * (1.0 - sig0)
    sig = max(1e-6, min(1.0 - 1e-6, sig))
    p = cfg.p_half_kpa + cfg.steepness_kpa * math.log(sig / (1.0 - sig))
    return max(0.0, min(cfg.supply_pressure_kpa, p))


class PneumaticChamberModel:
    """First-order lumped-parameter pneumatic chamber model for Digital Twin prediction."""

    def __init__(self, params: PneumaticParameters | None = None) -> None:
        self.params = params or PneumaticParameters()
        self.pressure_kpa: float = self.params.ambient_pressure_kpa

    def reset(self, pressure_kpa: float = 0.0) -> None:
        self.pressure_kpa = max(0.0, float(pressure_kpa))

    def step(self, dt_s: float, valve1_duty: float, valve2_duty: float) -> float:
        """Advance chamber pressure over dt_s given inflate (v1) and exhaust (v2) duties."""
        if dt_s <= 0.0:
            return self.pressure_kpa

        u1 = max(0.0, min(1.0, float(valve1_duty)))
        u2 = max(0.0, min(1.0, float(valve2_duty)))
        p = self.pressure_kpa
        cfg = self.params

        # Sub-step for numerical stability even at large UI frame intervals
        substeps = max(1, int(math.ceil(dt_s / 0.005)))
        h = dt_s / substeps
        for _ in range(substeps):
            dp_charge = u1 * (cfg.supply_pressure_kpa - p) / max(1e-3, cfg.tau_charge_s)
            dp_discharge = u2 * (p - cfg.ambient_pressure_kpa) / max(1e-3, cfg.tau_discharge_s)
            dp_leak = (p - cfg.ambient_pressure_kpa) / max(1e-3, cfg.tau_leak_s)
            p = p + h * (dp_charge - dp_discharge - dp_leak)
            p = max(0.0, min(cfg.supply_pressure_kpa, p))

        self.pressure_kpa = p
        return self.pressure_kpa
