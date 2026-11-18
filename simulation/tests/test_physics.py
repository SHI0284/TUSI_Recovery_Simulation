import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest

from simulation.inputs import DATA, read_eng, read_ork
from simulation.model import (
    Actuator,
    G,
    simulate as simulate_model,
    trim,
    validate as validate_model,
)


def simulate(overrides=None):
    return simulate_model(
        {
            "body_heading_lag_s": 0,
            "servo_latency_s": 0,
            "servo_accel_deg_s2": 0,
            "servo_resolution_deg": 0,
            "wind_inertia_enabled": 0,
            "mode": "manual",
            "wind_east_mps": 0,
            "wind_north_mps": 0,
            "gust_std_mps": 0,
            "wind_shear_east_mps_100m": 0,
            "wind_shear_north_mps_100m": 0,
            **(overrides or {}),
        }
    )


def validate(overrides):
    return validate_model(
        {
            "body_heading_lag_s": 0,
            "servo_latency_s": 0,
            "servo_accel_deg_s2": 0,
            "servo_resolution_deg": 0,
            **overrides,
        }
    )


class InputsTests(unittest.TestCase):
    def test_input_hashes(self):
        for entry in json.loads((DATA / "manifest.json").read_text()):
            self.assertEqual(
                hashlib.sha256((DATA / entry["file"]).read_bytes()).hexdigest(),
                entry["sha256"],
            )

    def test_measured_impulse_matches_file_header(self):
        motor = read_eng(DATA / "measured.eng")
        self.assertAlmostEqual(motor.impulse_ns, 427.991227, places=5)
        self.assertEqual(motor.points[0], (0, 0))
        self.assertEqual(motor.thrust(-1), 0)
        self.assertEqual(motor.thrust(100), 0)

    def test_linear_thrust_interpolation(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "test.eng"
            p.write_text("Test 10 10 0 0.1 0.2 Test\n0 0\n1 10\n2 0\n")
            c = read_eng(p)
            self.assertEqual(c.thrust(0.5), 5)
            self.assertEqual(c.thrust(1.5), 5)
            self.assertEqual(c.impulse_ns, 10)
            p.write_text("Test 10 10 0 0.1 0.2 Test\n1 0\n0 10\n")
            with self.assertRaises(ValueError):
                read_eng(p)

    def test_ork_recovery_mass_not_filename_mass(self):
        r = read_ork()
        self.assertAlmostEqual(r["saved_recovery_mass_kg"], 4.276)
        self.assertAlmostEqual(r["saved_apogee_m"], 390.242)
        self.assertEqual(r["canopies"][1]["overridemass"], "0.454")
        self.assertEqual(r["canopies"][1]["deployevent"], "apogee")


class PhysicsTests(unittest.TestCase):
    def test_supply_voltage_mismatch_is_explicit_not_silently_scaled(self):
        r = simulate({"duration_s": 1})
        self.assertEqual(r["config"]["servo_supply_voltage_v"], 6.5)
        self.assertFalse(r["motor_voltage_matches_performance"])
        self.assertEqual(r["config"]["no_load_speed_deg_s"], 270)
        self.assertTrue(any("VOLTAGE MISMATCH" in w for w in r["warnings"]))
        reference = simulate({"duration_s": 1, "servo_supply_voltage_v": 12})
        self.assertTrue(reference["motor_voltage_matches_performance"])

    def test_st3235_reference_units_and_overrides(self):
        r = simulate({"duration_s": 1})
        self.assertAlmostEqual(r["config"]["stall_torque_nm"], 30 * 0.0980665)
        self.assertEqual(r["config"]["no_load_speed_deg_s"], 45 * 6)
        self.assertTrue(r["uses_reference_torque_speed"])
        self.assertIsNone(r["servo_reference"]["actual_supply_voltage_v"])
        custom = simulate({"duration_s": 1, "stall_torque_nm": 1})
        self.assertFalse(custom["uses_reference_torque_speed"])
        self.assertEqual(custom["config"]["stall_torque_nm"], 1)

    def test_drum_geometry(self):
        c = validate({"line_slack_m": 0, "brake_stiffness_n_m": 0})
        a = Actuator(math.pi)
        o = a.observe(c, 0)
        self.assertAlmostEqual(o["takeup_m"], math.pi * 0.01)
        self.assertAlmostEqual(o["brake_m"], math.pi * 0.01)

    def test_compliance_force_balance_and_pulley(self):
        c = validate({"pulley_ratio": 2})
        a = Actuator(math.pi)
        o = a.observe(c, 5)
        self.assertAlmostEqual(o["stroke_m"], math.pi * 0.01 / 2)
        self.assertAlmostEqual(
            o["brake_m"] + c["line_slack_m"] + o["tension_n"] / c["line_stiffness_n_m"],
            o["stroke_m"],
        )
        self.assertAlmostEqual(o["torque_nm"], o["tension_n"] * 0.01 / (0.8 * 2))

    def test_slack_removes_authority(self):
        c = validate({"line_slack_m": 0.1})
        self.assertEqual(Actuator(math.pi).observe(c, 5)["brake_m"], 0)
        r = simulate({"line_slack_m": 0.1, "duration_s": 20})
        self.assertEqual(r["summary"]["heading_change_deg"], 0)

    def test_stall_and_release(self):
        c = validate({"stall_torque_nm": 0.01})
        a = Actuator()
        o = a.step(180, c, 10, 0.1)
        self.assertTrue(o["stalled"])
        self.assertEqual(a.angle_rad, 0)
        a.angle_rad = 1
        a.step(0, c, 10, 0.1)
        self.assertLess(a.angle_rad, 1)

    def test_rotation_speed_and_travel_limits(self):
        c = validate({})
        a = Actuator()
        a.step(1e6, c, 0, 0.02)
        self.assertLessEqual(
            math.degrees(a.angle_rad), c["no_load_speed_deg_s"] * 0.02 + 1e-12
        )
        for _ in range(1000):
            a.step(1e6, c, 0, 0.02)
        self.assertLessEqual(math.degrees(a.angle_rad), c["max_angle_deg"] + 1e-9)

    def test_trim_equilibrium(self):
        c = validate({})
        v, gamma = trim(c, 4.276)
        q = 0.5 * c["rho_kg_m3"] * v * v * c["canopy_area_m2"]
        self.assertAlmostEqual(q * c["cl"], 4.276 * G * math.cos(gamma))
        self.assertAlmostEqual(q * c["cd"], -4.276 * G * math.sin(gamma))
        r = simulate({"mode": "neutral", "duration_s": 20})
        first, last = r["trajectory"][0], r["trajectory"][-1]
        self.assertAlmostEqual(first["airspeed_mps"], last["airspeed_mps"], places=8)
        self.assertAlmostEqual(
            first["altitude_m"] - last["altitude_m"], 20 * first["sink_mps"], places=6
        )

    def test_mirror_symmetry(self):
        right = simulate({"duration_s": 30})
        left = simulate(
            {"duration_s": 30, "left_command_deg": 180, "right_command_deg": 0}
        )
        for a, b in zip(right["trajectory"], left["trajectory"]):
            self.assertAlmostEqual(a["heading_deg"], -b["heading_deg"], places=8)
            self.assertAlmostEqual(
                a["east_m"] - 21.53, -(b["east_m"] - 21.53), places=8
            )
            self.assertAlmostEqual(a["north_m"], b["north_m"], places=8)

    def test_symmetric_pull_does_not_turn(self):
        r = simulate(
            {"duration_s": 20, "left_command_deg": 180, "right_command_deg": 180}
        )
        self.assertEqual(r["summary"]["heading_change_deg"], 0)
        self.assertGreater(r["summary"]["max_left_brake_mm"], 0)

    def test_wind_advection(self):
        calm = simulate({"duration_s": 20})["trajectory"][-1]
        wind = simulate({"duration_s": 20, "wind_east_mps": 3, "wind_north_mps": -2})[
            "trajectory"
        ][-1]
        self.assertAlmostEqual(wind["east_m"] - calm["east_m"], 60, places=7)
        self.assertAlmostEqual(wind["north_m"] - calm["north_m"], -40, places=7)
        self.assertEqual(wind["heading_deg"], calm["heading_deg"])

    def test_touchdown_interpolation(self):
        r = simulate({"mode": "neutral", "duration_s": 300})
        first, last = r["trajectory"][0], r["trajectory"][-1]
        self.assertEqual(last["altitude_m"], 0)
        self.assertTrue(r["summary"]["landed"])
        self.assertAlmostEqual(
            last["t_s"], first["altitude_m"] / first["sink_mps"], places=6
        )

    def test_timestep_convergence(self):
        a = simulate({"duration_s": 30, "dt_s": 0.02})["summary"]
        b = simulate({"duration_s": 30, "dt_s": 0.01})["summary"]
        self.assertLess(abs(a["heading_change_deg"] - b["heading_change_deg"]), 0.15)
        self.assertLess(
            math.hypot(a["east_m"] - b["east_m"], a["north_m"] - b["north_m"]), 0.5
        )

    def test_heading_controller_reaches_line_deadband(self):
        r = simulate({"duration_s": 80, "mode": "heading", "target_heading_deg": 90})
        c = r["config"]
        base = r["summary"]["recovery_mass_kg"] * G * c["control_load_fraction"] / 2
        dead_angle = (
            (c["line_slack_m"] + base / c["line_stiffness_n_m"])
            * c["pulley_ratio"]
            / c["drum_radius_m"]
        )
        dead_error_deg = math.degrees(
            dead_angle / math.radians(c["max_angle_deg"]) / c["heading_kp"]
        )
        # PD without integral/dead-zone compensation has a finite steady heading error.
        self.assertLess(
            abs(r["trajectory"][-1]["heading_deg"] - 90), dead_error_deg + 0.25
        )

    def test_ascent_sources_and_finite_output(self):
        results = [
            simulate({"initialization": name, "duration_s": 1})
            for name in ["measured", "theoretical"]
        ]
        self.assertLess(
            results[0]["summary"]["apogee_m"], results[1]["summary"]["apogee_m"]
        )
        for r in results:
            self.assertAlmostEqual(r["summary"]["recovery_mass_kg"], 4.2757, places=4)
            json.dumps(r, allow_nan=False)

    def test_validation_and_truncation(self):
        for config in [
            {"cl": 0},
            {"dt_s": float("nan")},
            {"mode": "missing"},
            {"what": 1},
            {"controller_period_s": 0.01, "dt_s": 0.02},
            {"right_command_deg": -1},
            {"duration_s": 1e9},
        ]:
            with self.assertRaises(ValueError):
                simulate(config)
        r = simulate({"duration_s": 1})
        self.assertFalse(r["summary"]["landed"])
        self.assertGreater(r["trajectory"][-1]["altitude_m"], 0)


if __name__ == "__main__":
    unittest.main()
