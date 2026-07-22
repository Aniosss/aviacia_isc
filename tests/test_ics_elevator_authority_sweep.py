from __future__ import annotations

import argparse
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from ics_elevator_authority_sweep import parse_step  # noqa: E402


class ElevatorAuthoritySweepTests(unittest.TestCase):
    def test_parses_positive_step(self) -> None:
        self.assertEqual(parse_step("2.0"), 2.0)

    def test_rejects_non_positive_steps(self) -> None:
        with self.assertRaises(argparse.ArgumentTypeError):
            parse_step("0")

    def test_rejects_non_finite_step(self) -> None:
        with self.assertRaises(argparse.ArgumentTypeError):
            parse_step("nan")


if __name__ == "__main__":
    unittest.main()
