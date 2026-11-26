"""End-to-end unit and integration tests for the robotic glove Digital Twin."""

from __future__ import annotations

import math
import tempfile
import time
import unittest
from pathlib import Path

from digital_twin.endurance_test import EnduranceConfig, run_endurance_test
from digital_twin.glove_model import FINGER_NAMES, HAS_PYBULLET, PyBulletGloveTwin
from digital_twin.pneumatics import (
    PneumaticChamberModel,
    PneumaticParameters,
    angle_to_pressure_kpa,
    pressure_to_finger_angles,
)
from digital_twin.protocol import (
    cmd_max_pressure,
    cmd_pwm,
    cmd_target_pressure,
    cmd_valve,
    cmd_zero,
    parse_stm32_line,
)
from digital_twin.sync_engine import DigitalTwinEngine


class TestProtocol(unittest.TestCase):
    def test_parse_legacy_and_extended_data(self) -> None:
        s1, ev1 = parse_stm32_line("DATA,S0:101.2500,S1:45.6789,V1:1,V2:0", 1.0)
        self.assertIsNone(ev1)
        self.assertIsNotNone(s1)
        assert s1 is not None
        self.assertAlmostEqual(s1.s0_kpa, 101.25)
        self.assertAlmostEqual(s1.s1_kpa, 45.6789)
        self.assertEqual(s1.pwm1, 999)
        self.assertEqual(s1.pwm2, 0)

        s2, ev2 = parse_stm32_line(
            "DATA,S0:158.0,S1:82.5,V1:1,V2:0,PWM1:500,PWM2:0,T1:26.40", 2.0
        )
        self.assertIsNone(ev2)
        assert s2 is not None
        self.assertEqual(s2.pwm1, 500)
        self.assertAlmostEqual(s2.duty1, 500.0 / 999.0, places=4)
        self.assertAlmostEqual(s2.temp1_c, 26.4)

    def test_parse_events_and_commands(self) -> None:
        _, ev = parse_stm32_line("ACK,PWM,1,500", 1.2)
        assert ev is not None
        self.assertEqual(ev.kind, "ACK")
        self.assertEqual(cmd_valve(1, True), "VALVE,1,ON\n")
        self.assertEqual(cmd_pwm(2, 1200), "PWM,2,999\n")
        self.assertEqual(cmd_target_pressure(75.5), "TARGET,75.50\n")
        self.assertEqual(cmd_target_pressure(None), "TARGET,OFF\n")
        self.assertEqual(cmd_zero(), "ZERO\n")
        self.assertEqual(cmd_max_pressure(250), "MAXP,250.00\n")


class TestPneumaticsAndKinematics(unittest.TestCase):
    def test_chamber_charge_and_discharge(self) -> None:
        model = PneumaticChamberModel(PneumaticParameters(supply_pressure_kpa=160.0))
        for _ in range(50):
            model.step(0.02, 1.0, 0.0)
        charged = model.pressure_kpa
        self.assertGreater(charged, 100.0)

        for _ in range(50):
            model.step(0.02, 0.0, 1.0)
        self.assertLess(model.pressure_kpa, charged * 0.25)

    def test_pressure_angle_roundtrip(self) -> None:
        params = PneumaticParameters()
        for frac in (0.1, 0.35, 0.65, 0.9):
            p_kpa = angle_to_pressure_kpa(frac, params)
            kin = pressure_to_finger_angles(p_kpa, params)
            recovered = kin.mcp_deg / params.max_mcp_deg
            self.assertAlmostEqual(recovered, frac, places=3)


class TestPyBulletGloveTwin(unittest.TestCase):
    @unittest.skipUnless(HAS_PYBULLET, "pybullet not installed")
    def test_urdf_loads_and_moves_joints(self) -> None:
        twin = PyBulletGloveTwin(gui=False)
        try:
            for name in FINGER_NAMES:
                for joint in ("mcp", "pip", "dip"):
                    self.assertIn(f"{name}_{joint}", twin.joints)
            kin = pressure_to_finger_angles(95.0)
            twin.update_from_pressures(
                {name: kin for name in FINGER_NAMES},
                active_kpa=95.0,
                mirror_all=True,
            )
            twin.reset_camera("front")
            twin.reset_camera("default")
            mcp, pip, dip = twin.get_joint_angles_deg("index")
            self.assertAlmostEqual(mcp, kin.mcp_deg, places=2)
            self.assertAlmostEqual(pip, kin.pip_deg, places=2)
            self.assertAlmostEqual(dip, kin.dip_deg, places=2)
            self.assertFalse(math.isnan(mcp))
        finally:
            twin.close()


class TestSimulatedSynchronization(unittest.TestCase):
    def test_end_to_end_sim_sync_and_csv_log(self) -> None:
        engine = DigitalTwinEngine()
        with tempfile.TemporaryDirectory() as tmpdir:
            log_path = engine.start_logging(tmpdir)
            engine.connect("SIM")
            try:
                time.sleep(0.25)
                engine.ping()
                engine.set_target_curl(0.6)
                time.sleep(1.2)
                snap = engine.get_snapshot()

                self.assertTrue(snap.connected)
                self.assertGreater(snap.sample_count, 40)
                self.assertGreater(snap.sample_rate_hz, 35.0)
                self.assertIsNotNone(snap.rtt_ms)
                self.assertGreater(snap.s1_kpa, 30.0)
                self.assertGreater(snap.correlation_r, 0.85)
                self.assertLess(snap.rmse_kpa, 20.0)
            finally:
                engine.disconnect()

            lines = Path(log_path).read_text(encoding="utf-8").strip().splitlines()
            self.assertGreater(len(lines), 40)
            self.assertTrue(lines[0].startswith("time_s,s0_kpa,s1_kpa,sim_kpa"))

    def test_legacy_firmware_autoscaling_and_software_tare(self) -> None:
        engine = DigitalTwinEngine()
        s_init, _ = parse_stm32_line("DATA,S0:0.0100,S1:0.0100,V1:0,V2:0", 1.0)
        assert s_init is not None
        engine._on_telemetry(s_init)
        snap0 = engine.get_snapshot()
        self.assertTrue(snap0.legacy_firmware)
        self.assertAlmostEqual(snap0.s1_kpa, 0.01 * 1034.2136, places=2)

        engine.tare()
        s_next, _ = parse_stm32_line("DATA,S0:0.0100,S1:0.1100,V1:1,V2:0", 1.02)
        assert s_next is not None
        engine._on_telemetry(s_next)
        snap1 = engine.get_snapshot()
        self.assertAlmostEqual(snap1.s1_kpa, 0.10 * 1034.2136, places=2)


class TestEnduranceCharacterization(unittest.TestCase):
    def test_short_sim_endurance_run_produces_cycle_csv_and_summary(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            cfg = EnduranceConfig(
                port="SIM",
                cycles=3,
                mode="valve",
                inflate_s=0.45,
                hold_s=0.45,
                deflate_s=0.45,
                rest_s=0.20,
                save_raw_samples=True,
                output_dir=tmpdir,
            )
            cycles, summary, csv_path = run_endurance_test(cfg)
            self.assertEqual(len(cycles), 3)
            self.assertEqual(int(summary["cycles_completed"]), 3)
            self.assertGreater(cycles[0].peak_kpa, 40.0)
            self.assertTrue(Path(csv_path).exists())
            lines = Path(csv_path).read_text(encoding="utf-8").strip().splitlines()
            self.assertEqual(len(lines), 4)  # header + 3 cycles


if __name__ == "__main__":
    unittest.main()
