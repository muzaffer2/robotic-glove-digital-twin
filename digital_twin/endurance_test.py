"""Long-term cyclic endurance, leak-rate, and drift characterization for pneumatic actuators."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import List, Optional

from .pneumatics import PneumaticChamberModel, PneumaticParameters
from .protocol import (
    TelemetrySample,
    cmd_max_pressure,
    cmd_ping,
    cmd_target_pressure,
    cmd_valve,
)
from .serial_bridge import STM32Bridge


@dataclass(slots=True)
class CycleMetrics:
    """Extracted mechanical/pneumatic health indicators for one actuation cycle."""

    cycle_index: int
    elapsed_s: float
    baseline_kpa: float
    peak_kpa: float
    hold_start_kpa: float
    hold_end_kpa: float
    hold_leak_kpa_s: float
    rise_time_90_s: Optional[float]
    fall_time_10_s: Optional[float]
    residual_kpa: float
    twin_rmse_kpa: float
    temp1_c: float


@dataclass(slots=True)
class EnduranceConfig:
    """Configuration for an automated cyclic endurance test."""

    port: str = "SIM"
    cycles: int = 50
    mode: str = "valve"  # "valve" (timed inflate) or "target" (closed-loop kPa)
    target_kpa: float = 80.0
    max_safe_kpa: float = 200.0
    inflate_s: float = 1.2
    hold_s: float = 1.5
    deflate_s: float = 1.2
    rest_s: float = 0.5
    tare_at_start: bool = True
    save_raw_samples: bool = False
    output_dir: str = "logs"


def compute_cycle_metrics(
    cycle_index: int,
    elapsed_s: float,
    samples: List[tuple[float, str, TelemetrySample]],
    chamber_model: PneumaticChamberModel,
) -> CycleMetrics:
    """Compute leak rate, rise/fall times, residual pressure, and twin RMSE for one cycle."""
    if not samples:
        raise ValueError("Cycle has no telemetry samples")

    t_start = samples[0][0]
    p_vals = [s.s1_kpa for _, _, s in samples]
    baseline_kpa = p_vals[0]
    peak_kpa = max(p_vals)

    # Hold phase metrics (leak rate under closed valves)
    hold_pts = [(t, s.s1_kpa) for t, phase, s in samples if phase == "hold"]
    if len(hold_pts) >= 2:
        t_h0, p_h0 = hold_pts[0]
        t_h1, p_h1 = hold_pts[-1]
        dt_h = max(1e-3, t_h1 - t_h0)
        hold_leak_kpa_s = (p_h0 - p_h1) / dt_h
    else:
        p_h0 = peak_kpa
        p_h1 = peak_kpa
        hold_leak_kpa_s = 0.0

    # Rise time to 90% of (peak - baseline) during inflate/hold
    span = peak_kpa - baseline_kpa
    rise_time_90_s: Optional[float] = None
    if span > 5.0:
        thresh_90 = baseline_kpa + 0.90 * span
        for t, phase, s in samples:
            if phase in ("inflate", "hold") and s.s1_kpa >= thresh_90:
                rise_time_90_s = t - t_start
                break

    # Fall time from start of deflate down to 10% above baseline
    deflate_pts = [(t, s.s1_kpa) for t, phase, s in samples if phase in ("deflate", "rest")]
    fall_time_10_s: Optional[float] = None
    if deflate_pts and span > 5.0:
        t_def0 = deflate_pts[0][0]
        thresh_10 = baseline_kpa + 0.10 * span
        for t, p_kpa in deflate_pts:
            if p_kpa <= thresh_10:
                fall_time_10_s = t - t_def0
                break

    residual_kpa = p_vals[-1]

    # Compare physical S1 trajectory against nominal Digital Twin chamber model
    sq_err = 0.0
    prev_t: Optional[float] = None
    chamber_model.reset(baseline_kpa)
    for t, _, s in samples:
        dt = 0.02 if prev_t is None else max(0.001, min(0.2, t - prev_t))
        prev_t = t
        p_sim = chamber_model.step(dt, s.duty1, s.duty2)
        diff = s.s1_kpa - p_sim
        sq_err += diff * diff
    twin_rmse = math.sqrt(sq_err / len(samples))

    avg_temp = sum(s.temp1_c for _, _, s in samples) / len(samples)
    return CycleMetrics(
        cycle_index=cycle_index,
        elapsed_s=elapsed_s,
        baseline_kpa=baseline_kpa,
        peak_kpa=peak_kpa,
        hold_start_kpa=p_h0,
        hold_end_kpa=p_h1,
        hold_leak_kpa_s=hold_leak_kpa_s,
        rise_time_90_s=rise_time_90_s,
        fall_time_10_s=fall_time_10_s,
        residual_kpa=residual_kpa,
        twin_rmse_kpa=twin_rmse,
        temp1_c=avg_temp,
    )


def summarize_drift(cycles: List[CycleMetrics]) -> dict[str, float]:
    """Compare early cycles vs late cycles to quantify actuator fatigue/drift."""
    if not cycles:
        return {}
    k = max(1, len(cycles) // 5)  # First 20% vs last 20%
    early = cycles[:k]
    late = cycles[-k:]

    def mean(vals: List[float]) -> float:
        return sum(vals) / len(vals) if vals else 0.0

    early_peak = mean([c.peak_kpa for c in early])
    late_peak = mean([c.peak_kpa for c in late])
    early_leak = mean([c.hold_leak_kpa_s for c in early])
    late_leak = mean([c.hold_leak_kpa_s for c in late])
    early_rmse = mean([c.twin_rmse_kpa for c in early])
    late_rmse = mean([c.twin_rmse_kpa for c in late])
    early_res = mean([c.residual_kpa for c in early])
    late_res = mean([c.residual_kpa for c in late])

    return {
        "cycles_completed": float(len(cycles)),
        "early_peak_kpa": early_peak,
        "late_peak_kpa": late_peak,
        "peak_drift_kpa": late_peak - early_peak,
        "early_leak_kpa_s": early_leak,
        "late_leak_kpa_s": late_leak,
        "leak_drift_kpa_s": late_leak - early_leak,
        "early_residual_kpa": early_res,
        "late_residual_kpa": late_res,
        "early_twin_rmse_kpa": early_rmse,
        "late_twin_rmse_kpa": late_rmse,
    }


def run_endurance_test(
    cfg: EnduranceConfig,
    params: PneumaticParameters | None = None,
) -> tuple[List[CycleMetrics], dict[str, float], Path]:
    """Execute a safe multi-cycle actuator characterization run and save CSV/JSON logs."""
    twin_params = params or PneumaticParameters()
    chamber_model = PneumaticChamberModel(twin_params)

    out_dir = Path(cfg.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    cycles_csv_path = out_dir / f"endurance_{stamp}_cycles.csv"
    summary_json_path = out_dir / f"endurance_{stamp}_summary.json"
    raw_csv_path = out_dir / f"endurance_{stamp}_raw.csv"

    current_phase = "init"
    cycle_samples: List[tuple[float, str, TelemetrySample]] = []
    last_rx_ts = time.perf_counter()
    s1_disconnected_streak = 0

    import threading

    lock = threading.Lock()

    def on_telemetry(sample: TelemetrySample) -> None:
        nonlocal last_rx_ts, s1_disconnected_streak
        with lock:
            last_rx_ts = time.perf_counter()
            if not sample.s1_connected:
                s1_disconnected_streak += 1
            else:
                s1_disconnected_streak = 0
            cycle_samples.append((sample.timestamp_s, current_phase, sample))

    bridge = STM32Bridge(port=cfg.port, params=twin_params, on_telemetry=on_telemetry)
    bridge.start()

    results: List[CycleMetrics] = []
    raw_file = raw_csv_path.open("w", newline="", encoding="utf-8") if cfg.save_raw_samples else None
    raw_writer = csv.writer(raw_file) if raw_file else None
    if raw_writer:
        raw_writer.writerow(
            ["cycle", "phase", "time_s", "s0_kpa", "s1_kpa", "v1", "v2", "pwm1", "pwm2", "temp1_c"]
        )

    def safe_vent_and_close() -> None:
        try:
            bridge.send_command(cmd_target_pressure(None))
            bridge.send_command(cmd_valve(1, False))
            bridge.send_command(cmd_valve(2, True))
            time.sleep(min(0.6, cfg.deflate_s))
            bridge.send_command(cmd_valve(2, False))
        except Exception:
            pass

    try:
        time.sleep(0.3)
        bridge.send_command(cmd_ping())
        bridge.send_command(cmd_max_pressure(cfg.max_safe_kpa))
        if cfg.tare_at_start:
            bridge.send_command("ZERO\n")
        time.sleep(0.25)

        t_run_start = time.perf_counter()
        with cycles_csv_path.open("w", newline="", encoding="utf-8") as cf:
            cwriter = csv.writer(cf)
            cwriter.writerow(
                [
                    "cycle",
                    "elapsed_s",
                    "baseline_kpa",
                    "peak_kpa",
                    "hold_start_kpa",
                    "hold_end_kpa",
                    "hold_leak_kpa_s",
                    "rise_time_90_s",
                    "fall_time_10_s",
                    "residual_kpa",
                    "twin_rmse_kpa",
                    "temp1_c",
                ]
            )

            for idx in range(1, cfg.cycles + 1):
                with lock:
                    cycle_samples.clear()
                    current_phase = "inflate"

                if cfg.mode == "target":
                    bridge.send_command(cmd_target_pressure(cfg.target_kpa))
                else:
                    bridge.send_command(cmd_valve(2, False))
                    bridge.send_command(cmd_valve(1, True))
                _sleep_with_watchdog(cfg.inflate_s, lambda: last_rx_ts, lambda: s1_disconnected_streak)

                with lock:
                    current_phase = "hold"
                bridge.send_command(cmd_target_pressure(None))
                bridge.send_command(cmd_valve(1, False))
                bridge.send_command(cmd_valve(2, False))
                _sleep_with_watchdog(cfg.hold_s, lambda: last_rx_ts, lambda: s1_disconnected_streak)

                with lock:
                    current_phase = "deflate"
                bridge.send_command(cmd_valve(1, False))
                bridge.send_command(cmd_valve(2, True))
                _sleep_with_watchdog(cfg.deflate_s, lambda: last_rx_ts, lambda: s1_disconnected_streak)

                with lock:
                    current_phase = "rest"
                bridge.send_command(cmd_valve(2, False))
                _sleep_with_watchdog(cfg.rest_s, lambda: last_rx_ts, lambda: s1_disconnected_streak)

                with lock:
                    snapshot_samples = list(cycle_samples)

                if raw_writer:
                    for t_s, phase, s in snapshot_samples:
                        raw_writer.writerow(
                            [
                                idx,
                                phase,
                                f"{t_s - t_run_start:.4f}",
                                f"{s.s0_kpa:.4f}",
                                f"{s.s1_kpa:.4f}",
                                s.v1,
                                s.v2,
                                s.pwm1,
                                s.pwm2,
                                f"{s.temp1_c:.2f}",
                            ]
                        )

                if snapshot_samples:
                    metrics = compute_cycle_metrics(
                        cycle_index=idx,
                        elapsed_s=time.perf_counter() - t_run_start,
                        samples=snapshot_samples,
                        chamber_model=chamber_model,
                    )
                    results.append(metrics)
                    cwriter.writerow(
                        [
                            metrics.cycle_index,
                            f"{metrics.elapsed_s:.2f}",
                            f"{metrics.baseline_kpa:.3f}",
                            f"{metrics.peak_kpa:.3f}",
                            f"{metrics.hold_start_kpa:.3f}",
                            f"{metrics.hold_end_kpa:.3f}",
                            f"{metrics.hold_leak_kpa_s:.4f}",
                            "" if metrics.rise_time_90_s is None else f"{metrics.rise_time_90_s:.3f}",
                            "" if metrics.fall_time_10_s is None else f"{metrics.fall_time_10_s:.3f}",
                            f"{metrics.residual_kpa:.3f}",
                            f"{metrics.twin_rmse_kpa:.3f}",
                            f"{metrics.temp1_c:.2f}",
                        ]
                    )
                    cf.flush()
                    print(
                        f"[Cycle {idx:4d}/{cfg.cycles}] peak={metrics.peak_kpa:6.2f} kPa | "
                        f"leak={metrics.hold_leak_kpa_s:+.3f} kPa/s | "
                        f"res={metrics.residual_kpa:5.2f} kPa | "
                        f"twin_RMSE={metrics.twin_rmse_kpa:5.2f} kPa"
                    )
    finally:
        safe_vent_and_close()
        bridge.stop()
        if raw_file:
            raw_file.close()

    summary = summarize_drift(results)
    summary_payload = {
        "config": asdict(cfg),
        "summary": summary,
        "cycles_csv": str(cycles_csv_path),
    }
    summary_json_path.write_text(json.dumps(summary_payload, indent=2), encoding="utf-8")
    return results, summary, cycles_csv_path


def _sleep_with_watchdog(duration_s: float, get_last_rx, get_disc_streak) -> None:
    deadline = time.perf_counter() + max(0.0, duration_s)
    while time.perf_counter() < deadline:
        time.sleep(min(0.05, max(0.0, deadline - time.perf_counter())))
        if time.perf_counter() - get_last_rx() > 3.0:
            raise TimeoutError("Telemetry timeout (>3s) — venting valves for safety.")
        if get_disc_streak() >= 25:
            raise RuntimeError("S1 sensor disconnected during endurance run — venting for safety.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Long-term pneumatic actuator endurance & drift test")
    parser.add_argument("--port", default="SIM", help="Serial COM port (e.g. COM7) or SIM")
    parser.add_argument("--cycles", type=int, default=20, help="Number of inflate/hold/deflate cycles")
    parser.add_argument("--mode", choices=("valve", "target"), default="valve", help="Open-loop valve or closed-loop kPa")
    parser.add_argument("--target-kpa", type=float, default=80.0, help="Target kPa when --mode target")
    parser.add_argument("--max-kpa", type=float, default=200.0, help="Overpressure safety cutoff (kPa)")
    parser.add_argument("--inflate-s", type=float, default=1.2, help="Inflation duration per cycle (s)")
    parser.add_argument("--hold-s", type=float, default=1.5, help="Closed-valve hold duration to measure leak (s)")
    parser.add_argument("--deflate-s", type=float, default=1.2, help="Deflation duration per cycle (s)")
    parser.add_argument("--rest-s", type=float, default=0.5, help="Rest duration between cycles (s)")
    parser.add_argument("--raw-log", action="store_true", help="Also save full 50 Hz sample CSV")
    parser.add_argument("--output-dir", default="logs", help="Directory for CSV and summary JSON")
    args = parser.parse_args()

    cfg = EnduranceConfig(
        port=args.port,
        cycles=args.cycles,
        mode=args.mode,
        target_kpa=args.target_kpa,
        max_safe_kpa=args.max_kpa,
        inflate_s=args.inflate_s,
        hold_s=args.hold_s,
        deflate_s=args.deflate_s,
        rest_s=args.rest_s,
        save_raw_samples=args.raw_log,
        output_dir=args.output_dir,
    )

    _, summary, csv_path = run_endurance_test(cfg)
    print("\n=== Endurance & Drift Summary ===")
    for k, v in summary.items():
        print(f"{k:22s}: {v:.4f}")
    print(f"Per-cycle CSV saved to: {csv_path}")


if __name__ == "__main__":
    main()
