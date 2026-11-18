import math
import unittest
from simulation.attitude import axis_fields, ork_attitude
from simulation.inputs import read_ork
from simulation.model import Actuator, simulate, validate
from simulation.environment import WindField, Sensors


class RealismTests(unittest.TestCase):
    def test_ork_axis_uses_elevation_and_clockwise_north_azimuth(self):
        f = axis_fields(math.pi / 6, math.pi / 2, "test")
        self.assertAlmostEqual(f["body_axis_east"], math.sqrt(3) / 2)
        self.assertAlmostEqual(f["body_axis_north"], 0)
        self.assertAlmostEqual(f["body_axis_up"], 0.5)
        self.assertAlmostEqual(f["body_tilt_deg"], 60)
        a = ork_attitude(read_ork()["apogee"])
        self.assertAlmostEqual(a["body_tilt_deg"], 22.276388, places=4)
        self.assertTrue(a["angular_rates_available"])
        self.assertGreater(a["body_axis_east"], 0.37)

    def test_servo_delay_quantization_and_startup_acceleration(self):
        c = validate(
            {
                "servo_latency_s": 0.06,
                "servo_accel_deg_s2": 100,
                "servo_resolution_deg": 1,
                "line_slack_m": 0,
            }
        )
        servo = Actuator()
        for _ in range(3):
            o = servo.step(10.4, c, 0, 0.02)
            self.assertEqual(o["angle_deg"], 0)
        o = servo.step(10.4, c, 0, 0.02)
        self.assertEqual(o["applied_target_deg"], 10)
        self.assertAlmostEqual(o["velocity_deg_s"], 2)
        self.assertAlmostEqual(o["angle_deg"], 0.02)
        for _ in range(100):
            o = servo.step(10.4, c, 0, 0.02)
        self.assertAlmostEqual(o["angle_deg"], 10)
        self.assertEqual(o["velocity_deg_s"], 0)

    def test_delayed_commands_keep_order(self):
        c = validate({"servo_latency_s": 0.04})
        servo = Actuator()
        self.assertEqual(servo.step(30, c, 0, 0.02)["applied_target_deg"], 0)
        self.assertEqual(servo.step(60, c, 0, 0.02)["applied_target_deg"], 0)
        self.assertEqual(
            servo.step(90, c, 0, 0.02)["applied_target_deg"],
            round(30 / c["servo_resolution_deg"]) * c["servo_resolution_deg"],
        )
        self.assertAlmostEqual(
            servo.step(90, c, 0, 0.02)["applied_target_deg"],
            round(60 / c["servo_resolution_deg"]) * c["servo_resolution_deg"],
        )

    def test_motor_overload_stops_winding(self):
        c = validate({"servo_latency_s": 0, "stall_torque_nm": 0.01})
        o = Actuator().step(180, c, 100, 0.02)
        self.assertTrue(o["stalled"])
        self.assertEqual(o["angle_deg"], 0)

    def test_material_wind_derivative_matches_moving_sample(self):
        c = validate({})
        w = WindField(c)
        t = 0.035
        z = 300
        vz = -3
        epsilon = 1e-6
        before = w.at(t - epsilon, z - vz * epsilon)
        after = w.at(t + epsilon, z + vz * epsilon)
        for actual, low, high in zip(w.acceleration(t, z, vz), before, after):
            self.assertAlmostEqual(actual, (high - low) / (2 * epsilon), places=6)

    def test_changing_wind_does_not_instantly_accelerate_ground_velocity(self):
        # At the initial trim, no steering: aerodynamic+gravity horizontal force is zero.
        cfg = {
            "mode": "neutral",
            "duration_s": 1,
            "dt_s": 0.005,
            "imu_period_s": 0.005,
            "gust_std_mps": 0,
            "wind_shear_east_mps_100m": 2,
            "wind_shear_north_mps_100m": 1,
        }

        def acceleration(enabled):
            r = simulate({**cfg, "wind_inertia_enabled": enabled})["trajectory"]

            def ground(v):
                vh = math.sqrt(v["airspeed_mps"] ** 2 - v["sink_mps"] ** 2)
                a = math.radians(v["air_heading_deg"])
                return (
                    vh * math.sin(a) + v["wind_east_mps"],
                    vh * math.cos(a) + v["wind_north_mps"],
                )

            return math.hypot(
                *[(b - a) / 0.005 for a, b in zip(ground(r[0]), ground(r[1]))]
            )

        self.assertLess(acceleration(1), 0.001)
        self.assertGreater(acceleration(0), 0.04)

    def test_sensor_bias_is_repeatable_and_distinct_from_white_noise(self):
        c = validate(
            {
                "gps_latency_s": 0,
                "imu_latency_s": 0,
                "gps_position_std_m": 0,
                "gyro_std_deg_s": 0,
                "gyro_bias_deg_s": 2,
            }
        )
        a, b = Sensors(c), Sensors(c)
        for i in range(20):
            x = a.update(i * 0.2, 0, 0, 100, 0, 0, 0, 0)
            y = b.update(i * 0.2, 0, 0, 100, 0, 0, 0, 0)
            self.assertEqual(x, y)
        self.assertNotEqual(x["gps"]["east_m"], 0)
        self.assertAlmostEqual(math.degrees(x["imu"]["yaw_rad_s"]), 2)

    def test_air_velocity_heading_is_not_immediate_body_heading(self):
        r = simulate({"duration_s": 3, "mode": "neutral"})
        self.assertGreater(
            max(abs(x["air_heading_deg"] - x["heading_deg"]) for x in r["trajectory"]),
            0.1,
        )
        self.assertLess(
            max(abs(x["yaw_rate_deg_s"]) for x in r["trajectory"]),
            max(abs(x["air_heading_rate_deg_s"]) for x in r["trajectory"]),
        )
        self.assertFalse(r["model_assessment"]["calibrated_for_this_canopy"])

    def test_no_turn_authority_cannot_turn_even_with_brake_input(self):
        r = simulate(
            {
                "duration_s": 5,
                "mode": "manual",
                "turn_gain_rad_s_m": 0,
                "gust_std_mps": 0,
                "wind_shear_east_mps_100m": 0,
            }
        )
        self.assertGreater(r["summary"]["max_right_brake_mm"], 10)
        self.assertEqual(
            max(abs(x["steering_yaw_rate_deg_s"]) for x in r["trajectory"]), 0
        )
        self.assertAlmostEqual(r["summary"]["heading_change_deg"], 0)


if __name__ == "__main__":
    unittest.main()
