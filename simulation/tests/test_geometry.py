import unittest
from simulation.inputs import read_ork


class RocketGeometryTests(unittest.TestCase):
    def test_external_stack_matches_saved_ork(self):
        g = read_ork()["geometry"]
        self.assertAlmostEqual(g["length_m"], 1.604375)
        nose, body, tail = [
            next(p for p in g["parts"] if p["kind"] == k)
            for k in ["nosecone", "bodytube", "transition"]
        ]
        self.assertEqual(nose["shape"], "ellipsoid")
        self.assertAlmostEqual(nose["length_m"], 0.28)
        self.assertAlmostEqual(body["start_m"], 0.28)
        self.assertAlmostEqual(body["radius_m"], 0.052)
        self.assertAlmostEqual(tail["start_m"], body["start_m"] + body["length_m"])
        self.assertAlmostEqual(tail["aft_radius_m"], 0.04375)

    def test_fins_end_at_body_aft_and_preserve_planform(self):
        g = read_ork()["geometry"]
        f = next(p for p in g["parts"] if p["kind"] == "trapezoidfinset")
        self.assertEqual(f["count"], 4)
        self.assertAlmostEqual(f["start_m"] + f["root_chord_m"], 1.58)
        self.assertAlmostEqual(f["root_chord_m"], 0.39993597120046603)
        self.assertAlmostEqual(f["span_m"], 0.08972466421127322)
        self.assertAlmostEqual(f["thickness_m"], 0.003)
        self.assertGreater(f["sweep_m"], 0)
        self.assertLess(f["sweep_m"] + f["tip_chord_m"], f["root_chord_m"])
        self.assertFalse(any(p["kind"] == "masscomponent" for p in g["parts"]))


if __name__ == "__main__":
    unittest.main()
