from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from ics_flight_envelope import (  # noqa: E402
    LandingFlapConfiguration,
    alpha_prot_deg,
    approach_limits,
    detect_landing_flaps,
    measured_landing_flaps,
    roll_limit_deg,
)


class ApproachEnvelopeTests(unittest.TestCase):
    def test_uses_next_conservative_weight_row(self) -> None:
        full = approach_limits(69277.0, LandingFlapConfiguration.FULL)
        flaps_3 = approach_limits(69277.0, LandingFlapConfiguration.FLAPS_3)

        self.assertEqual(full.table_weight_kg, 70000.0)
        self.assertEqual(full.vapp_kt, 136.0)
        self.assertEqual(full.vsr1_kt, 110.5)
        self.assertEqual(full.vfe_kt, 178.0)
        self.assertEqual(full.alpha_sw_deg, 13.2)
        self.assertEqual(flaps_3.vapp_kt, 140.0)
        self.assertEqual(flaps_3.vsr1_kt, 113.6)
        self.assertEqual(flaps_3.vfe_kt, 183.0)
        self.assertEqual(flaps_3.alpha_sw_deg, 13.6)

    def test_touchdown_speed_band_follows_document_formula(self) -> None:
        limits = approach_limits(69277.0, LandingFlapConfiguration.FULL)
        self.assertAlmostEqual(limits.touchdown_speed_min_kt, 130.56)
        self.assertEqual(limits.touchdown_speed_max_kt, 146.0)

    def test_detects_landing_flap_configuration(self) -> None:
        self.assertEqual(detect_landing_flaps(36.0), LandingFlapConfiguration.FULL)
        self.assertEqual(detect_landing_flaps(27.0), LandingFlapConfiguration.FLAPS_3)
        self.assertEqual(detect_landing_flaps(0.0), LandingFlapConfiguration.FULL)
        self.assertIsNone(measured_landing_flaps(0.0))

    def test_reduces_roll_limit_close_to_ground(self) -> None:
        self.assertEqual(roll_limit_deg(250.0, 15.0), 15.0)
        self.assertEqual(roll_limit_deg(75.0, 15.0), 10.0)
        self.assertEqual(roll_limit_deg(20.0, 15.0), 5.0)

    def test_interpolates_alpha_protection_by_mach(self) -> None:
        self.assertAlmostEqual(alpha_prot_deg(0.20, LandingFlapConfiguration.FULL), 13.1)
        self.assertAlmostEqual(alpha_prot_deg(0.32, LandingFlapConfiguration.FULL), 12.8)
        self.assertAlmostEqual(alpha_prot_deg(0.36, LandingFlapConfiguration.FULL), 12.55)


if __name__ == "__main__":
    unittest.main()
