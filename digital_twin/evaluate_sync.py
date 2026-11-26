"""Headless synchronization evaluation script for the physical/simulated STM32 glove."""

from __future__ import annotations

import argparse
import time

from .pneumatics import PneumaticParameters
from .sync_engine import DigitalTwinEngine


def run_evaluation(port: str = "SIM", duration_s: float = 8.0, save_csv: bool = True) -> dict[str, object]:
    """Execute an inflation -> hold -> target -> vent profile and print sync metrics."""
    engine = DigitalTwinEngine(PneumaticParameters())
    log_path = engine.start_logging() if save_csv else None
    engine.connect(port=port)

    try:
        time.sleep(0.3)
        engine.ping()
        engine.tare()
        time.sleep(0.2)

        # Stage 1: Inflate via Valve 1
        engine.set_valve(1, True)
        engine.set_valve(2, False)
        time.sleep(duration_s * 0.28)

        # Stage 2: Hold
        engine.set_valve(1, False)
        time.sleep(duration_s * 0.17)

        # Stage 3: Closed-loop target (approx 55% finger curl)
        engine.set_target_curl(0.55)
        time.sleep(duration_s * 0.30)

        # Stage 4: Deflate via Valve 2
        engine.set_target_pressure(None)
        engine.set_valve(1, False)
        engine.set_valve(2, True)
        time.sleep(duration_s * 0.25)
        engine.set_valve(2, False)

        snap = engine.get_snapshot()
        return {
            "samples": float(snap.sample_count),
            "sample_rate_hz": snap.sample_rate_hz,
            "jitter_ms": snap.jitter_ms,
            "rtt_ms": snap.rtt_ms or 0.0,
            "rmse_kpa": snap.rmse_kpa,
            "mae_kpa": snap.mae_kpa,
            "correlation_r": snap.correlation_r,
            "log_path": str(log_path or ""),
        }
    finally:
        engine.disconnect()


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate real-time Digital Twin synchronization")
    parser.add_argument("--port", default="SIM", help="Serial COM port (e.g. COM3) or SIM")
    parser.add_argument("--duration", type=float, default=8.0, help="Experiment duration in seconds")
    args = parser.parse_args()

    results = run_evaluation(port=args.port, duration_s=args.duration, save_csv=True)
    print("=== Digital Twin Synchronization Evaluation ===")
    print(f"Samples        : {int(results['samples'])}")
    print(f"Sample Rate    : {results['sample_rate_hz']:.2f} Hz")
    print(f"Timing Jitter  : {results['jitter_ms']:.2f} ms")
    print(f"Ping RTT       : {results['rtt_ms']:.2f} ms")
    print(f"Pressure RMSE  : {results['rmse_kpa']:.2f} kPa")
    print(f"Pressure MAE   : {results['mae_kpa']:.2f} kPa")
    print(f"Correlation r  : {results['correlation_r']:.4f}")
    if results["log_path"]:
        print(f"CSV Log Saved  : {results['log_path']}")


if __name__ == "__main__":
    main()
