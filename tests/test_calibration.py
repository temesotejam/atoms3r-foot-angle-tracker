import json
import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
CONFIG = (ROOT / "include" / "app_config.h").read_text(encoding="utf-8")
CAL = json.loads(
    (ROOT / "calibration" / "foot_angle_v1.json").read_text(encoding="utf-8")
)


def cpp_float(name: str) -> float:
    m = re.search(
        rf"{re.escape(name)}\s*=\s*([-+0-9.eE]+)f?\s*;",
        CONFIG,
    )
    if not m:
        raise AssertionError(f"missing C++ constant: {name}")
    return float(m.group(1))


class CalibrationSyncTest(unittest.TestCase):
    def test_marker_a_constants_match_json(self):
        self.assertAlmostEqual(
            cpp_float("kFootAngleAZeroXPx"),
            CAL["marker_a"]["zero_x_px"],
            places=5,
        )
        self.assertAlmostEqual(
            cpp_float("kFootAngleADegPerPx"),
            CAL["marker_a"]["deg_per_px"],
            places=7,
        )

    def test_marker_b_constants_match_json(self):
        self.assertAlmostEqual(
            cpp_float("kFootAngleBZeroXPx"),
            CAL["marker_b"]["zero_x_px"],
            places=5,
        )
        self.assertAlmostEqual(
            cpp_float("kFootAngleBDegPerPx"),
            CAL["marker_b"]["deg_per_px"],
            places=7,
        )

    def test_upright_is_zero(self):
        for marker, zero_key, slope_key in (
            ("marker_a", "kFootAngleAZeroXPx", "kFootAngleADegPerPx"),
            ("marker_b", "kFootAngleBZeroXPx", "kFootAngleBDegPerPx"),
        ):
            x0 = cpp_float(zero_key)
            slope = cpp_float(slope_key)
            angle = slope * (x0 - x0)
            self.assertAlmostEqual(angle, 0.0, places=7)

    def test_positive_direction_is_left(self):
        for zero_key, slope_key in (
            ("kFootAngleAZeroXPx", "kFootAngleADegPerPx"),
            ("kFootAngleBZeroXPx", "kFootAngleBDegPerPx"),
        ):
            x0 = cpp_float(zero_key)
            slope = cpp_float(slope_key)
            self.assertGreater(slope * (x0 - (x0 - 10.0)), 0.0)


if __name__ == "__main__":
    unittest.main()
