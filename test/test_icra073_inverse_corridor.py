import importlib.util
import copy
import os
import subprocess
import sys
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
FIXTURE_TOOL = (
    REPO / "scripts/dev_planner/icra073_inverse_corridor_fixture.py")


class Icra073InverseCorridorTest(unittest.TestCase):
    @staticmethod
    def _load_fixture_tool():
        spec = importlib.util.spec_from_file_location(
            "icra073_inverse_corridor_fixture", FIXTURE_TOOL)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module

    def test_descriptors_freeze_primary_mirror_and_flat_null_geometry(self):
        module = self._load_fixture_tool()

        descriptors = {
            variant: module.build_descriptor(variant)
            for variant in ("PRIMARY", "EXACT_MIRROR", "FLAT_NULL")
        }

        for variant, descriptor in descriptors.items():
            with self.subTest(variant=variant):
                self.assertEqual(
                    descriptor["schema_version"],
                    "p4_v2_inverse_corridor_fixture_v1")
                self.assertEqual(descriptor["scene_variant"], variant)
                self.assertEqual(descriptor["frame_id"], "map")
                self.assertEqual(descriptor["units"], "m")
                self.assertEqual(descriptor["start_m"], [-12.0, 0.0, 1.5])
                self.assertEqual(descriptor["goal_m"], [12.0, 0.0, 1.5])
                self.assertEqual(descriptor["straight_seed"], {
                    "kind": "closed_line_segment",
                    "start_reference": "start_m",
                    "goal_reference": "goal_m",
                    "equation": "(-12+24*u, 0, 1.5)",
                    "u_interval_inclusive": [0.0, 1.0],
                })
                self.assertEqual(descriptor["tube_radius_m"], 0.75)
                self.assertEqual(descriptor["guard_band_width_m"], 0.5)
                self.assertEqual(
                    descriptor["central_cuboid"]["bounds_m"], {
                        "x": [-2.0, 2.0],
                        "y": [-0.75, 0.75],
                        "z": [0.0, 2.8],
                    })
                self.assertEqual(len(descriptor["descriptor_sha256"]), 64)
                self.assertNotIn(
                    "descriptor_sha256",
                    descriptor["descriptor_hash_input_fields"])
                self.assertEqual(
                    set(descriptor["descriptor_hash_input_fields"]),
                    set(descriptor) - {"descriptor_sha256"})

        primary = descriptors["PRIMARY"]
        mirror = descriptors["EXACT_MIRROR"]
        null = descriptors["FLAT_NULL"]
        self.assertEqual(primary["centre_lines"]["safe"]["amplitude_y_m"],
                         3.5)
        self.assertEqual(primary["centre_lines"]["risky"]["amplitude_y_m"],
                         -2.1)
        self.assertEqual(mirror["centre_lines"]["safe"]["amplitude_y_m"],
                         -3.5)
        self.assertEqual(mirror["centre_lines"]["risky"]["amplitude_y_m"],
                         2.1)
        self.assertEqual(null["centre_lines"], primary["centre_lines"])
        self.assertEqual(
            primary["provider_truth"]["route_interior_values"],
            {"safe": 1.0, "risky": 4.0})
        self.assertEqual(
            mirror["provider_truth"]["route_interior_values"],
            {"safe": 1.0, "risky": 4.0})
        self.assertEqual(
            null["provider_truth"]["route_interior_values"],
            {"safe": 1.0, "risky": 1.0})
        self.assertEqual(
            module.build_descriptor("PRIMARY")["descriptor_sha256"],
            primary["descriptor_sha256"])
        self.assertEqual(
            {variant: descriptor["descriptor_sha256"]
            for variant, descriptor in descriptors.items()},
            {
                "PRIMARY": (
                    "525b29ea7ef5014d653ccf3079c09ba1906f39cce9d9d305493538486abfbb97"),
                "EXACT_MIRROR": (
                    "e7e33fa4dc5419f8ef17a2f81a33d074c0b6c676c34aa796ad8486de3dcb2f69"),
                "FLAT_NULL": (
                    "cd66f46149c7ab858a6223b0908b80ca7c00dad864d05adc030e01ae0a54443e"),
            })

    def test_preflight_fails_closed_on_frozen_guard_inflation_conflict(self):
        module = self._load_fixture_tool()
        descriptors = {
            variant: module.build_descriptor(variant)
            for variant in module.VARIANTS
        }

        preflight = module.preflight_descriptors(descriptors)

        self.assertEqual(
            preflight["schema_version"], "icra073_fixture_preflight_v1")
        self.assertEqual(preflight["result"], "FAIL")
        self.assertEqual(preflight["failure_reasons"], [
            "curved_tubes_and_guards_occupancy_clear",
        ])
        required_checks = {
            "descriptor_hash_exact",
            "common_endpoints_and_nonstraight",
            "straight_seed_closed_collision_with_free_entry_exit",
            "curved_tubes_and_guards_occupancy_clear",
            "curved_routes_reachable",
            "polyline_hausdorff_within_0_01_m",
            "lidar_landmarks_symmetric",
            "outer_trees_close_only_third_homotopy",
            "primary_provider_support_finite_complete_and_ordered",
            "exact_mirror_geometric_y_negation",
            "flat_null_truth_finite_complete_identical",
            "decision_and_oracle_data_planes_isolated",
        }
        self.assertEqual(set(preflight["checks"]), required_checks)
        completed_before_blocker = {
            "descriptor_hash_exact",
            "common_endpoints_and_nonstraight",
            "straight_seed_closed_collision_with_free_entry_exit",
        }
        self.assertTrue(all(
            preflight["checks"][name]["accepted"]
            for name in completed_before_blocker))
        not_evaluated = required_checks - completed_before_blocker - set(
            preflight["failure_reasons"])
        self.assertEqual(set(preflight["incomplete_checks"]), not_evaluated)
        self.assertTrue(all(
            preflight["checks"][name]["status"] ==
            "NOT_EVALUATED_BLOCKED_BY_GEOMETRY"
            for name in not_evaluated))
        risky_clearance = preflight["checks"][
            "curved_tubes_and_guards_occupancy_clear"]["variants"][
                "PRIMARY"]["risky"]
        self.assertAlmostEqual(
            risky_clearance["minimum_raw_occupancy_clearance_m"],
            1.275072535, places=7)
        self.assertEqual(
            risky_clearance["required_guard_plus_inflation_m"], 1.349)
        self.assertGreater(
            risky_clearance["required_guard_plus_inflation_m"],
            risky_clearance["minimum_raw_occupancy_clearance_m"])
        self.assertEqual(
            preflight["checks"]["curved_routes_reachable"]["status"],
            "NOT_EVALUATED_BLOCKED_BY_GEOMETRY")
        mutations = {}
        blocked_corridor = copy.deepcopy(descriptors)
        blocked_corridor["PRIMARY"]["central_cuboid"]["bounds_m"]["y"] = [
            -2.0, 2.0]
        mutations["corridor occupancy"] = blocked_corridor
        provider_reversed = copy.deepcopy(descriptors)
        provider_reversed["PRIMARY"]["provider_truth"][
            "route_interior_values"] = {"safe": 5.0, "risky": 1.0}
        mutations["provider ordering"] = provider_reversed
        mirror_not_geometric = copy.deepcopy(descriptors)
        mirror_not_geometric["EXACT_MIRROR"]["centre_lines"]["safe"][
            "amplitude_y_m"] = 3.5
        mutations["mirror geometry"] = mirror_not_geometric
        null_not_flat = copy.deepcopy(descriptors)
        null_not_flat["FLAT_NULL"]["provider_truth"][
            "route_interior_values"]["risky"] = 1.01
        mutations["flat null"] = null_not_flat
        coarse_polyline = copy.deepcopy(descriptors)
        coarse_polyline["PRIMARY"]["polyline"]["segment_count"] = 4
        mutations["hausdorff"] = coarse_polyline

        for name, mutated in mutations.items():
            with self.subTest(name=name):
                rejected = module.preflight_descriptors(mutated)
                self.assertEqual(rejected["result"], "FAIL")
                self.assertTrue(rejected["failure_reasons"])

    def test_preflight_cli_rejects_forged_source_before_writing_evidence(self):
        output = (
            REPO / "results/icra27/icra073" /
            f"forged-source-{os.getpid()}.json")
        self.assertFalse(output.exists())

        completed = subprocess.run([
            sys.executable, str(FIXTURE_TOOL),
            "--preflight-all",
            "--source-head", "0" * 40,
            "--output", str(output),
        ], cwd=REPO, capture_output=True, text=True, check=False)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("SOURCE_BINDING_NOT_READY",
                      completed.stdout + completed.stderr)
        self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
