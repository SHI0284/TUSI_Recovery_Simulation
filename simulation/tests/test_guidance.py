"""Closed-loop behavior, sensor timing and separation from ground truth."""

import math
import unittest
from simulation.environment import WindField, Sensors
from simulation.guidance import TargetPD
from simulation.model import validate, simulate


def observation(east=0, north=0, heading=0, yaw=0, stamp=1):
    return dict(
        gps_valid=True,
        imu_valid=True,
        baro_valid=True,
        gps_age_s=0.2,
        gps=dict(east_m=east, north_m=north, ve_mps=0, vn_mps=6.3, sample_time_s=stamp),
        imu=dict(heading_rad=math.radians(heading), yaw_rad_s=math.radians(yaw)),
        baro=dict(altitude_m=100),
    )


class GuidanceTests(unittest.TestCase):
    def test_turn_direction_and_heading_wrap(self):
        c = validate(
            {"wind_compensation": 0, "target_east_m": 100, "target_north_m": 0}
        )
        out = TargetPD(c, 4.276).update(1, observation())
        self.assertGreater(out["right_command_deg"], 0)
        self.assertEqual(out["left_command_deg"], 0)
        c.update(target_east_m=-100)
        out = TargetPD(c, 4.276).update(1, observation())
        self.assertGreater(out["left_command_deg"], 0)
        c.update(target_east_m=0, target_north_m=100)
        out = TargetPD(c, 4.276).update(1, observation(heading=359))
        self.assertAlmostEqual(out["heading_error_deg"], 1, places=6)

    def test_derivative_damping_and_separate_plant_calibration(self):
        c = validate({"wind_compensation": 0})
        base = TargetPD(c, 4.276).update(1, observation())
        yaw = TargetPD(c, 4.276).update(1, observation(yaw=10))
        self.assertLess(yaw["desired_yaw_rate_deg_s"], base["desired_yaw_rate_deg_s"])
        altered = {
            **c,
            "turn_gain_rad_s_m": -20,
            "wind_east_mps": 20,
            "gust_std_mps": 7,
        }
        self.assertEqual(TargetPD(altered, 4.276).update(1, observation()), base)

    def test_slew_limit_and_saturation(self):
        c = validate({"target_east_m": 100, "target_north_m": 0})
        ctl = TargetPD(c, 4.276)
        last = 0
        for i in range(50):
            o = ctl.update(i * 0.05, observation())
            value = o["requested_right_brake_m"]
            self.assertLessEqual(abs(value - last), c["brake_slew_m_s"] * 0.05 + 1e-12)
            self.assertLessEqual(value, c["control_brake_limit_m"])
            self.assertLessEqual(o["right_command_deg"], c["max_angle_deg"])
            self.assertTrue(o["command_saturated"])
            last = value

    def test_no_false_success_before_touchdown(self):
        r = simulate(
            {
                "duration_s": 1,
                "target_east_m": 21.53,
                "target_north_m": 0,
                "target_radius_m": 500,
            }
        )
        self.assertFalse(r["summary"]["landed"])
        self.assertFalse(r["summary"]["landed_in_target"])

    def test_stale_sensor_neutral_and_recovery(self):
        ctl = TargetPD(validate({}), 4.276)
        o = observation()
        self.assertTrue(ctl.update(1, o)["guidance_valid"])
        o["gps_valid"] = False
        failed = ctl.update(2, o)
        self.assertFalse(failed["guidance_valid"])
        self.assertEqual(failed["right_command_deg"], 0)
        self.assertEqual(failed["left_command_deg"], 0)
        o["gps_valid"] = True
        self.assertTrue(ctl.update(3, o)["guidance_valid"])

    def test_zero_coordinate_is_valid(self):
        out = TargetPD(validate({}), 4.276).update(1, observation())
        self.assertTrue(out["guidance_valid"])

    def test_unreachable_estimate(self):
        c = validate(
            {"target_east_m": 0, "target_north_m": 100, "wind_estimator_tau_s": 0.1}
        )
        ctl = TargetPD(c, 4.276)
        o = observation()
        o["gps"]["vn_mps"] = -15
        self.assertTrue(ctl.update(1, o)["estimated_upwind_unreachable"])


class EnvironmentTests(unittest.TestCase):
    def test_wind_seed_and_timestep_independence(self):
        c = validate({"duration_s": 3})
        a = WindField(c)
        b = WindField({**c, "dt_s": 0.01, "mode": "neutral"})
        for t in [0, 0.03, 0.2, 1.1, 3]:
            self.assertEqual(a.at(t, 300), b.at(t, 300))
        self.assertNotEqual(a.at(1, 300), WindField({**c, "random_seed": 8}).at(1, 300))
        zero = WindField({**c, "gust_std_mps": 0})
        self.assertAlmostEqual(
            zero.at(2, 200)[0], c["wind_east_mps"] + 2 * c["wind_shear_east_mps_100m"]
        )

    def test_latency_sample_hold_and_freshness(self):
        c = validate(
            {
                "gps_period_s": 0.2,
                "gps_latency_s": 0.1,
                "gps_position_std_m": 0,
                "gps_velocity_std_mps": 0,
                "gps_bias_std_m": 0,
                "sensor_timeout_s": 0.3,
            }
        )
        s = Sensors(c)

        def tick(t):
            return s.update(t, t, 0, 100, 0, 0, 1, 0)

        self.assertFalse(tick(0)["gps_valid"])
        self.assertFalse(tick(0.02)["gps_valid"])
        self.assertTrue(tick(0.1)["gps_valid"])
        self.assertEqual(tick(0.2)["gps"]["east_m"], 0)
        self.assertEqual(tick(0.3)["gps"]["east_m"], 0.2)
        self.assertFalse(s.observe(0.6)["gps_valid"])

    def test_reproducible_closed_loop_and_dropout(self):
        cfg = {"duration_s": 8, "gps_dropout_start_s": 2, "gps_dropout_end_s": 5}
        a = simulate(cfg)
        b = simulate(cfg)
        self.assertEqual(a, b)
        log = a["control_log"]
        stale = [r for r in log if 3.6 < r["t_s"] < 5.1]
        self.assertTrue(stale)
        self.assertTrue(
            all(
                not r["guidance_valid"]
                and r["right_command_deg"] == r["left_command_deg"] == 0
                for r in stale
            )
        )
        self.assertTrue(all(r["guidance_valid"] for r in log if r["t_s"] > 5.4))
        self.assertGreater(len(log), 150)
        self.assertLess(a["sensor_counts"]["gps"], len(log))
        self.assertEqual(set(a["trajectory"][0]), set(a["trajectory"][-1]))

    def test_default_guidance_improves_seeded_landing_error(self):
        target = simulate()
        neutral = simulate({"mode": "neutral"})
        self.assertTrue(target["summary"]["landed"])
        self.assertLess(
            target["summary"]["target_error_m"],
            neutral["summary"]["target_error_m"] / 2,
        )
        self.assertGreater(target["summary"]["control_updates"], 2000)
        self.assertGreater(target["summary"]["max_left_brake_mm"], 1)
        self.assertGreater(target["summary"]["max_right_brake_mm"], 1)

    def test_new_parameter_validation(self):
        for cfg in [
            {"random_seed": 0.5},
            {"controller_turn_gain_rad_s_m": 0},
            {"gps_period_s": 0.001},
            {"gps_dropout_start_s": 5, "gps_dropout_end_s": 2},
            {"gust_std_mps": -1},
        ]:
            with self.assertRaises(ValueError):
                validate(cfg)


if __name__ == "__main__":
    unittest.main()
