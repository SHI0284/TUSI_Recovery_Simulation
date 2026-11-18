import json
import math
import unittest
from simulation.model import simulate
from simulation.mission import build_mission, mission_rows


class MissionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.flight = simulate()
        cls.flight["mission"] = build_mission(cls.flight)
        cls.rows = list(mission_rows(cls.flight))

    def test_ordered_phases_and_actual_timing(self):
        m = self.flight["mission"]
        self.assertEqual(
            [e["key"] for e in m["events"]],
            [
                "PAD",
                "BOOST",
                "COAST",
                "DEPLOY",
                "STABILIZATION",
                "GUIDED_DESCENT",
                "GROUND",
            ],
        )
        self.assertEqual(m["burnout_s"], 1.635)
        self.assertAlmostEqual(m["apogee_sample_s"], 9.506)
        self.assertAlmostEqual(m["ork_apogee_event_s"], 9.556)
        self.assertAlmostEqual(m["recovery_start_s"], 12.506)
        self.assertEqual(self.rows[0]["t_s"], -3)
        self.assertTrue(
            all(a["t_s"] < b["t_s"] for a, b in zip(self.rows, self.rows[1:]))
        )
        self.assertEqual(len({tuple(sorted(r)) for r in self.rows}), 1)
        json.dumps(self.rows, allow_nan=False)

    def test_no_guidance_or_open_canopy_during_ascent(self):
        ascending = [r for r in self.rows if r["phase"] in ("PAD", "BOOST", "COAST")]
        self.assertTrue(
            all(
                r["canopy_fraction"] == 0
                and not r["recovery_active"]
                and not r["guidance_valid"]
                for r in ascending
            )
        )
        self.assertTrue(
            all(
                r["control_tick"] == 0
                and r["left_command_deg"] == r["right_command_deg"] == 0
                for r in ascending
            )
        )
        boost = [r for r in ascending if r["phase"] == "BOOST"]
        self.assertGreater(max(r["thrust_n"] for r in boost), 0)

    def test_bridge_continuous_position_and_labelled_assumption(self):
        m = self.flight["mission"]
        deployment = [r for r in self.rows if r["phase"] in ("DEPLOY", "STABILIZATION")]
        first = self.flight["trajectory"][0]
        self.assertEqual(
            deployment[0]["altitude_m"], self.flight["summary"]["apogee_m"]
        )
        self.assertTrue(
            all(
                not r["flight_model_valid"]
                and r["data_source"] == "ILLUSTRATIVE_DEPLOYMENT"
                for r in deployment
            )
        )
        heights = [r["altitude_m"] for r in deployment] + [first["altitude_m"]]
        self.assertTrue(all(a >= b for a, b in zip(heights, heights[1:])))
        self.assertLess(abs(heights[-2] - heights[-1]), 0.2)
        self.assertEqual(deployment[-1]["east_m"], first["east_m"])
        self.assertEqual(deployment[-1]["north_m"], first["north_m"])
        self.assertEqual(deployment[-1]["canopy_fraction"], 1)

    def test_recovery_clock_and_original_solution_are_preserved(self):
        raw = self.flight["trajectory"][500]
        assembled = next(r for r in self.rows if r["recovery_t_s"] == raw["t_s"])
        self.assertAlmostEqual(
            assembled["t_s"] - raw["t_s"], self.flight["mission"]["recovery_start_s"]
        )
        for key in [
            "east_m",
            "north_m",
            "left_brake_m",
            "right_brake_m",
            "control_tick",
            "gps_age_s",
        ]:
            self.assertEqual(assembled[key], raw[key])
        final = self.rows[-1]
        self.assertEqual(final["phase"], "GROUND")
        self.assertFalse(final["guidance_valid"])
        self.assertEqual(final["east_m"], self.flight["summary"]["east_m"])
        self.assertEqual(final["data_source"], "ILLUSTRATIVE_GROUND_HOLD")
        self.assertEqual(final["altitude_m"], 0)

    def test_timeout_is_not_landing(self):
        flight = simulate({"duration_s": 1})
        m = build_mission(flight)
        rows = list(mission_rows(flight))
        self.assertIsNone(m["touchdown_s"])
        self.assertEqual(m["events"][-1]["key"], "TIME_LIMIT")
        self.assertEqual(rows[-1]["phase"], "TIME_LIMIT")
        self.assertGreater(rows[-1]["altitude_m"], 0)
        self.assertFalse(rows[-1]["guidance_valid"])

    def test_measured_ascent_burnout_and_zero_countdown(self):
        flight = simulate(
            {
                "initialization": "measured",
                "duration_s": 1,
                "mission_countdown_s": 0,
                "stabilization_height_loss_m": 0,
            }
        )
        m = build_mission(flight)
        rows = list(mission_rows(flight))
        self.assertEqual(rows[0]["t_s"], 0)
        self.assertEqual(m["burnout_s"], 3.1)
        self.assertIsNone(m["ork_apogee_event_s"])
        deployment = [r for r in rows if r["data_source"] == "ILLUSTRATIVE_DEPLOYMENT"]
        self.assertTrue(
            all(
                math.isclose(r["altitude_m"], flight["summary"]["apogee_m"])
                for r in deployment
            )
        )
        self.assertTrue(all(a["t_s"] < b["t_s"] for a, b in zip(rows, rows[1:])))

    def test_scenario_parameter_bounds(self):
        for c in [
            {"deployment_transition_s": 0},
            {"mission_countdown_s": -1},
            {"landing_hold_s": 11},
        ]:
            with self.assertRaises(ValueError):
                simulate(c)


if __name__ == "__main__":
    unittest.main()
