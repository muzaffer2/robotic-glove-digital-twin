"""Real-time physical-virtual synchronization engine and evaluation metrics."""

from __future__ import annotations

import csv
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Optional

from .pneumatics import (
    FingerKinematics,
    PneumaticChamberModel,
    PneumaticParameters,
    angle_to_pressure_kpa,
    pressure_to_finger_angles,
)
from .protocol import (
    T150PGSA3XX_MAX_KPA,
    EventMessage,
    TelemetrySample,
    cmd_max_pressure,
    cmd_ping,
    cmd_pwm,
    cmd_target_pressure,
    cmd_valve,
    cmd_zero,
)
from .serial_bridge import STM32Bridge


@dataclass(slots=True)
class SyncSnapshot:
    """Thread-safe snapshot of current physical, virtual, and synchronization states."""

    connected: bool
    port: str
    s0_kpa: float
    s1_kpa: float
    sim_kpa: float
    target_kpa: Optional[float]
    temp1_c: float
    v1: int
    v2: int
    pwm1: int
    pwm2: int
    finger_kinematics: FingerKinematics
    sample_rate_hz: float
    jitter_ms: float
    rtt_ms: Optional[float]
    rmse_kpa: float
    mae_kpa: float
    correlation_r: float
    sample_count: int
    logging_active: bool
    log_path: Optional[str]
    legacy_firmware: bool = False


class DigitalTwinEngine:
    """Coordinates STM32 telemetry, pneumatic twin simulation, logging, and sync metrics."""

    def __init__(
        self,
        params: PneumaticParameters | None = None,
        history_len: int = 500,
    ) -> None:
        self.params = params or PneumaticParameters()
        self.chamber_model = PneumaticChamberModel(self.params)
        self._lock = threading.Lock()

        self._bridge: Optional[STM32Bridge] = None
        self._port = "SIM"
        self._last_sample: Optional[TelemetrySample] = None
        self._last_ts: Optional[float] = None
        self._target_kpa: Optional[float] = None

        self._history_t: Deque[float] = deque(maxlen=history_len)
        self._history_s1: Deque[float] = deque(maxlen=history_len)
        self._history_sim: Deque[float] = deque(maxlen=history_len)
        self._history_s0: Deque[float] = deque(maxlen=history_len)
        self._dt_history: Deque[float] = deque(maxlen=200)
        self._events: Deque[EventMessage] = deque(maxlen=50)

        self._t0: Optional[float] = None
        self._sample_count = 0
        self._sq_err_sum = 0.0
        self._abs_err_sum = 0.0
        self._legacy_firmware = False
        self._sw_s0_offset_kpa = 0.0
        self._sw_s1_offset_kpa = 0.0
        self._last_raw_s0_kpa = 0.0
        self._last_raw_s1_kpa = 0.0

        self._csv_file = None
        self._csv_writer = None
        self._log_path: Optional[Path] = None

    def connect(self, port: str = "SIM", baudrate: int = 115200) -> None:
        self._stop_bridge()
        with self._lock:
            self._port = port
            self.chamber_model.reset(0.0)
            self._last_sample = None
            self._last_ts = None
            self._t0 = None
            self._sample_count = 0
            self._sq_err_sum = 0.0
            self._abs_err_sum = 0.0
            self._legacy_firmware = False
            self._sw_s0_offset_kpa = 0.0
            self._sw_s1_offset_kpa = 0.0
            self._last_raw_s0_kpa = 0.0
            self._last_raw_s1_kpa = 0.0
            self._history_t.clear()
            self._history_s1.clear()
            self._history_sim.clear()
            self._history_s0.clear()
            self._dt_history.clear()

        bridge = STM32Bridge(
            port=port,
            baudrate=baudrate,
            params=self.params,
            on_telemetry=self._on_telemetry,
            on_event=self._on_event,
        )
        bridge.start()
        with self._lock:
            self._bridge = bridge

    def _stop_bridge(self) -> None:
        with self._lock:
            bridge = self._bridge
            self._bridge = None
        if bridge is not None:
            bridge.stop()

    def disconnect(self) -> None:
        self._stop_bridge()
        self.stop_logging()

    def start_logging(self, directory: Path | str = "logs") -> Path:
        self.stop_logging()
        out_dir = Path(directory)
        out_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = out_dir / f"sync_log_{stamp}.csv"
        f = path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(f)
        writer.writerow(
            [
                "time_s",
                "s0_kpa",
                "s1_kpa",
                "sim_kpa",
                "target_kpa",
                "v1",
                "v2",
                "pwm1",
                "pwm2",
                "mcp_deg",
                "pip_deg",
                "dip_deg",
                "temp1_c",
                "rtt_ms",
            ]
        )
        with self._lock:
            self._csv_file = f
            self._csv_writer = writer
            self._log_path = path
        return path

    def stop_logging(self) -> None:
        with self._lock:
            f = self._csv_file
            self._csv_file = None
            self._csv_writer = None
        if f is not None:
            f.close()

    def _on_telemetry(self, sample: TelemetrySample) -> None:
        legacy_cmds: list[str] = []
        with self._lock:
            if self._t0 is None:
                self._t0 = sample.timestamp_s

            dt = 0.02 if self._last_ts is None else max(0.001, min(0.25, sample.timestamp_s - self._last_ts))
            if self._last_ts is not None:
                self._dt_history.append(sample.timestamp_s - self._last_ts)
            self._last_ts = sample.timestamp_s
            self._legacy_firmware = sample.is_legacy_normalized

            scale = T150PGSA3XX_MAX_KPA if sample.is_legacy_normalized else 1.0
            raw_s0 = (sample.s0_kpa * scale) if sample.s0_connected else -1.0
            raw_s1 = (sample.s1_kpa * scale) if sample.s1_connected else -1.0
            if sample.s0_connected:
                self._last_raw_s0_kpa = raw_s0
            if sample.s1_connected:
                self._last_raw_s1_kpa = raw_s1

            s0_adj = (raw_s0 - self._sw_s0_offset_kpa) if sample.s0_connected else -1.0
            s1_adj = (raw_s1 - self._sw_s1_offset_kpa) if sample.s1_connected else -1.0

            adj_sample = TelemetrySample(
                timestamp_s=sample.timestamp_s,
                s0_kpa=s0_adj,
                s1_kpa=s1_adj,
                v1=sample.v1,
                v2=sample.v2,
                pwm1=sample.pwm1,
                pwm2=sample.pwm2,
                temp1_c=sample.temp1_c,
                is_legacy_normalized=sample.is_legacy_normalized,
            )
            self._last_sample = adj_sample

            # Adapt supply pressure estimate if S0 is measuring pressurized manifold (> 20 kPa)
            if sample.s0_connected and s0_adj > 20.0:
                self.params.supply_pressure_kpa = (
                    0.95 * self.params.supply_pressure_kpa + 0.05 * s0_adj
                )

            sim_kpa = self.chamber_model.step(dt, sample.duty1, sample.duty2)
            phys_kpa = s1_adj if sample.s1_connected else sim_kpa

            # Software closed-loop fallback when connected to original 4-field firmware
            if self._legacy_firmware and self._target_kpa is not None and sample.s1_connected:
                hyst = 3.0
                if phys_kpa < self._target_kpa - hyst:
                    if sample.v1 == 0:
                        legacy_cmds.append(cmd_valve(1, True))
                    if sample.v2 != 0:
                        legacy_cmds.append(cmd_valve(2, False))
                elif phys_kpa > self._target_kpa + hyst:
                    if sample.v1 != 0:
                        legacy_cmds.append(cmd_valve(1, False))
                    if sample.v2 == 0:
                        legacy_cmds.append(cmd_valve(2, True))
                else:
                    if sample.v1 != 0:
                        legacy_cmds.append(cmd_valve(1, False))
                    if sample.v2 != 0:
                        legacy_cmds.append(cmd_valve(2, False))

            err = phys_kpa - sim_kpa
            self._sample_count += 1
            self._sq_err_sum += err * err
            self._abs_err_sum += abs(err)

            t_rel = sample.timestamp_s - self._t0
            self._history_t.append(t_rel)
            self._history_s1.append(phys_kpa)
            self._history_sim.append(sim_kpa)
            self._history_s0.append(s0_adj)

            if self._csv_writer is not None:
                kin = pressure_to_finger_angles(phys_kpa, self.params)
                rtt = self._bridge.last_rtt_ms if self._bridge is not None else None
                self._csv_writer.writerow(
                    [
                        f"{t_rel:.4f}",
                        f"{s0_adj:.4f}",
                        f"{phys_kpa:.4f}",
                        f"{sim_kpa:.4f}",
                        "" if self._target_kpa is None else f"{self._target_kpa:.2f}",
                        sample.v1,
                        sample.v2,
                        sample.pwm1,
                        sample.pwm2,
                        f"{kin.mcp_deg:.2f}",
                        f"{kin.pip_deg:.2f}",
                        f"{kin.dip_deg:.2f}",
                        f"{sample.temp1_c:.2f}",
                        "" if rtt is None else f"{rtt:.2f}",
                    ]
                )

        for c in legacy_cmds:
            self.send_raw(c)

    def _on_event(self, event: EventMessage) -> None:
        with self._lock:
            self._events.append(event)

    def send_raw(self, command: str) -> None:
        with self._lock:
            bridge = self._bridge
        if bridge is not None:
            bridge.send_command(command)

    def ping(self) -> None:
        with self._lock:
            bridge = self._bridge
        if bridge is not None:
            bridge.send_command(cmd_ping())

    def set_valve(self, valve_id: int, enabled: bool) -> None:
        with self._lock:
            self._target_kpa = None
        self.send_raw(cmd_valve(valve_id, enabled))

    def set_pwm(self, valve_id: int, duty: int) -> None:
        with self._lock:
            self._target_kpa = None
        self.send_raw(cmd_pwm(valve_id, duty))

    def set_target_pressure(self, target_kpa: Optional[float]) -> None:
        with self._lock:
            self._target_kpa = target_kpa
            legacy = self._legacy_firmware
        if legacy:
            if target_kpa is None:
                self.send_raw(cmd_valve(1, False))
                self.send_raw(cmd_valve(2, False))
        else:
            self.send_raw(cmd_target_pressure(target_kpa))

    def set_target_curl(self, curl_fraction: float) -> float:
        target = angle_to_pressure_kpa(curl_fraction, self.params)
        self.set_target_pressure(target)
        return target

    def tare(self, reset: bool = False) -> None:
        with self._lock:
            legacy = self._legacy_firmware
            if legacy:
                if reset:
                    self._sw_s0_offset_kpa = 0.0
                    self._sw_s1_offset_kpa = 0.0
                else:
                    self._sw_s0_offset_kpa = self._last_raw_s0_kpa
                    self._sw_s1_offset_kpa = self._last_raw_s1_kpa
            else:
                self._sw_s0_offset_kpa = 0.0
                self._sw_s1_offset_kpa = 0.0
            self.chamber_model.reset(0.0)
        if not legacy:
            self.send_raw(cmd_zero(reset=reset))

    def set_max_pressure(self, max_kpa: float) -> None:
        self.send_raw(cmd_max_pressure(max_kpa))

    def get_snapshot(self) -> SyncSnapshot:
        with self._lock:
            connected = self._bridge is not None and self._bridge.is_connected
            rtt_ms = self._bridge.last_rtt_ms if self._bridge is not None else None
            sample = self._last_sample
            s0 = sample.s0_kpa if sample is not None else 0.0
            s1 = sample.s1_kpa if (sample is not None and sample.s1_connected) else 0.0
            sim_kpa = self.chamber_model.pressure_kpa
            v1 = sample.v1 if sample is not None else 0
            v2 = sample.v2 if sample is not None else 0
            pwm1 = sample.pwm1 if sample is not None else 0
            pwm2 = sample.pwm2 if sample is not None else 0
            temp1 = sample.temp1_c if sample is not None else 25.0

            dts = list(self._dt_history)
            if dts:
                mean_dt = sum(dts) / len(dts)
                rate_hz = 1.0 / mean_dt if mean_dt > 0 else 0.0
                var = sum((d - mean_dt) ** 2 for d in dts) / len(dts)
                jitter_ms = math.sqrt(var) * 1000.0
            else:
                rate_hz = 0.0
                jitter_ms = 0.0

            n = self._sample_count
            rmse = math.sqrt(self._sq_err_sum / n) if n > 0 else 0.0
            mae = (self._abs_err_sum / n) if n > 0 else 0.0
            corr = self._pearson_r(self._history_s1, self._history_sim)

            kin = pressure_to_finger_angles(s1 if (sample and sample.s1_connected) else sim_kpa, self.params)
            return SyncSnapshot(
                connected=connected,
                port=self._port,
                s0_kpa=s0,
                s1_kpa=s1,
                sim_kpa=sim_kpa,
                target_kpa=self._target_kpa,
                temp1_c=temp1,
                v1=v1,
                v2=v2,
                pwm1=pwm1,
                pwm2=pwm2,
                finger_kinematics=kin,
                sample_rate_hz=rate_hz,
                jitter_ms=jitter_ms,
                rtt_ms=rtt_ms,
                rmse_kpa=rmse,
                mae_kpa=mae,
                correlation_r=corr,
                sample_count=n,
                logging_active=self._csv_writer is not None,
                log_path=str(self._log_path) if self._log_path is not None else None,
                legacy_firmware=self._legacy_firmware,
            )

    def get_history(self) -> tuple[list[float], list[float], list[float]]:
        with self._lock:
            return list(self._history_t), list(self._history_s1), list(self._history_sim)

    def get_events(self) -> list[EventMessage]:
        with self._lock:
            return list(self._events)

    @staticmethod
    def _pearson_r(xs: Deque[float], ys: Deque[float]) -> float:
        n = min(len(xs), len(ys))
        if n < 3:
            return 1.0
        xl = list(xs)[-n:]
        yl = list(ys)[-n:]
        mx = sum(xl) / n
        my = sum(yl) / n
        num = sum((x - mx) * (y - my) for x, y in zip(xl, yl))
        dx = math.sqrt(sum((x - mx) ** 2 for x in xl))
        dy = math.sqrt(sum((y - my) ** 2 for y in yl))
        if dx < 1e-6 or dy < 1e-6:
            return 1.0 if abs(mx - my) < 1.0 else 0.0
        return max(-1.0, min(1.0, num / (dx * dy)))
