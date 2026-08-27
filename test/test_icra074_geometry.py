#!/usr/bin/env python3
"""Focused V2 geometry-amendment contract for ICRA-074."""

import copy
import importlib.util
import math
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
FIXTURE_TOOL = (
    REPO / "scripts/dev_planner/icra073_inverse_corridor_fixture.py")


class Icra074GeometryAmendmentTest(unittest.TestCase):
    @staticmethod
    def _load_tool():
        spec = importlib.util.spec_from_file_location(
            "icra073_inverse_corridor_fixture", FIXTURE_TOOL)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module

    @staticmethod
    def _dense_curve_to_cuboid_clearance(amplitude_y_m):
        minimum = math.inf
        bounds = ((-2.0, 2.0), (-0.75, 0.75), (0.0, 2.8))
        for index in range(1_000_001):
            u = index / 1_000_000
            point = (
                -12.0 + 24.0 * u,
                amplitude_y_m * math.sin(math.pi * u),
                1.5,
            )
            squared = 0.0
            for coordinate, (lower, upper) in zip(point, bounds):
                if coordinate < lower:
                    squared += (lower - coordinate) ** 2
                elif coordinate > upper:
                    squared += (coordinate - upper) ** 2
            minimum = min(minimum, math.sqrt(squared))
        return minimum

    def test_v2_changes_only_risky_amplitude_and_descriptor_identities(self):
        module = self._load_tool()
        expected_risky_amplitudes = {
            "PRIMARY": -2.20,
            "EXACT_MIRROR": 2.20,
            "FLAT_NULL": -2.20,
        }
        expected_hashes = {
            "PRIMARY": (
                "41ab7001c7c60e78cdcf8a670efdcba7545bcd7c93e06081745a090b57f0f06d"),
            "EXACT_MIRROR": (
                "3bd25208dba3bbd03857104758878e30d647b7f8bfdfc43e7654f78b569caf33"),
            "FLAT_NULL": (
                "89074561a7263268e121c2439ceccaca2b3c7285f665432ebdc07ab5668e8513"),
        }

        for variant, expected_amplitude in expected_risky_amplitudes.items():
            with self.subTest(variant=variant):
                v1 = module.build_descriptor(variant)
                v2 = module.build_v2_descriptor(variant)
                self.assertEqual(
                    v2["schema_version"],
                    "p4_v2_inverse_corridor_fixture_v2")
                self.assertEqual(
                    v2["design_record"],
                    "ICRA_P4_V2_INVERSE_CORRIDOR_FIXTURE_V2")
                self.assertEqual(
                    v2["centre_lines"]["risky"]["amplitude_y_m"],
                    expected_amplitude)
                self.assertEqual(
                    v2["centre_lines"]["safe"]["amplitude_y_m"],
                    -3.5 if variant == "EXACT_MIRROR" else 3.5)
                self.assertTrue(v2["scene_identity"].startswith("icra074-"))
                self.assertEqual(
                    v2["descriptor_sha256"], expected_hashes[variant])

                normalized_v1 = copy.deepcopy(v1)
                normalized_v2 = copy.deepcopy(v2)
                for descriptor in (normalized_v1, normalized_v2):
                    descriptor.pop("descriptor_sha256")
                    descriptor.pop("descriptor_hash_input_fields")
                    descriptor["schema_version"] = "normalized"
                    descriptor["design_record"] = "normalized"
                    descriptor["scene_identity"] = "normalized"
                    descriptor["centre_lines"]["risky"][
                        "amplitude_y_m"] = "normalized"
                self.assertEqual(normalized_v2, normalized_v1)

    def test_dense_analytic_risky_clearance_exceeds_frozen_guard_requirement(self):
        module = self._load_tool()
        expected_amplitudes = {
            "PRIMARY": -2.20,
            "EXACT_MIRROR": 2.20,
            "FLAT_NULL": -2.20,
        }

        for variant, expected_amplitude in expected_amplitudes.items():
            with self.subTest(variant=variant):
                descriptor = module.build_v2_descriptor(variant)
                observed = self._dense_curve_to_cuboid_clearance(
                    descriptor["centre_lines"]["risky"]["amplitude_y_m"])
                self.assertEqual(
                    descriptor["centre_lines"]["risky"]["amplitude_y_m"],
                    expected_amplitude)
                self.assertAlmostEqual(observed, 1.371035, places=5)
                self.assertGreaterEqual(observed, 1.349)


if __name__ == "__main__":
    unittest.main()
