import importlib.util
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from launch import LaunchContext


REPO = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO / "launch" / "test_planner.launch.py"
SPEC = importlib.util.spec_from_file_location("test_planner_launch_g0c", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class P4G0CLaunchContractTest(unittest.TestCase):
    @staticmethod
    def _context():
        context = LaunchContext()
        for name, default in MODULE.ARG_DEFAULTS:
            context.launch_configurations[name] = str(default)
        return context

    def test_general_metrics_only_default_is_false_and_profile_is_exact(self):
        defaults = dict(MODULE.ARG_DEFAULTS)
        self.assertEqual(defaults["p4.metrics_only"], "false")
        profile = MODULE.EXPERIMENT_PRESETS["p4_g0c_metrics_calibration_v1"]
        scenario = MODULE.SCENARIO_PRESETS["p4_g0c_free_corridor_v1"]
        self.assertEqual(
            profile,
            {
                **MODULE.P4_G0C_FROZEN_LAUNCH_VALUES,
                **MODULE.P4_G0C_ARTIFACT_PRESET,
                "scenario": "p4_g0c_free_corridor_v1",
            },
        )
        self.assertEqual(scenario["p1_map_fixture"], "p1_fork_fused_v1")
        self.assertTrue(scenario["p1_fixture_central_obstacle_enabled"] == "true")
        self.assertGreater(
            float(scenario["p1_fixture_risky_canopy_probability"]),
            float(scenario["p1_fixture_safe_canopy_probability"]),
        )

    def test_conflicting_g0c_override_is_rejected_not_normalized(self):
        effective = dict(MODULE.P4_G0C_FROZEN_LAUNCH_VALUES)
        effective["p4.metrics_only"] = "false"
        with self.assertRaisesRegex(RuntimeError, "conflicting.*p4.metrics_only"):
            MODULE._validate_p4_g0c_profile_values(
                "p4_g0c_metrics_calibration_v1",
                effective,
                {"p4.metrics_only"},
            )

    def test_g0c_disabled_p1_p2_metrics_stay_disabled(self):
        context = self._context()
        context.launch_configurations["experiment"] = MODULE.P4_G0C_EXPERIMENT
        context.launch_configurations["p1.metrics_only"] = "false"
        context.launch_configurations["p2.metrics_only"] = "false"
        self.assertFalse(MODULE._effective_metrics_only(
            context, "p1.metrics_only", False, set()
        ))
        self.assertFalse(MODULE._effective_metrics_only(
            context, "p2.metrics_only", False, set()
        ))

    def test_binding_requires_exact_hashes_run_identity_and_run_local_paths(self):
        protocol = REPO / "config/icra27/p4_g0c_protocol_v1.json"
        registry = REPO / "config/icra27/p4_threshold_registry_v1.json"
        fixture = REPO / "config/icra27/p4_g0c_live_fixture_v1.json"
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "p4-g0c-seed211-rep01"
            binding = MODULE._p4_g0c_binding(
                experiment="p4_g0c_metrics_calibration_v1",
                protocol_path=protocol,
                registry_path=registry,
                fixture_path=fixture,
                declared_protocol_sha256=MODULE._sha256_file(protocol),
                declared_registry_sha256=MODULE._sha256_file(registry),
                declared_fixture_sha256=MODULE._sha256_file(fixture),
                run_id="p4-g0c-seed211-rep01",
                seed=211,
                repetition=1,
                run_manifest_path=run_dir / "p4_g0c_run_manifest.json",
                csv_path=run_dir / "p4_decisions.csv",
                effective_values=MODULE.P4_G0C_FROZEN_LAUNCH_VALUES,
            )
        self.assertEqual(binding["gate"], "G0C")
        self.assertEqual(binding["run_id"], "p4-g0c-seed211-rep01")
        self.assertEqual(binding["effective_values"]["p4.metrics_only"], True)
        self.assertEqual(
            binding["effective_values"]["p4.per_search_timeout_s"], 0.2
        )
        self.assertEqual(binding["required_process_set"], [
            "iap_rosnode", "ego_planner_node"
        ])
        self.assertFalse(binding["record_bag"])
        self.assertFalse(binding["start_rviz"])

        with self.assertRaisesRegex(RuntimeError, "protocol hash"):
            MODULE._p4_g0c_binding(
                experiment="p4_g0c_metrics_calibration_v1",
                protocol_path=protocol,
                registry_path=registry,
                fixture_path=fixture,
                declared_protocol_sha256="0" * 64,
                declared_registry_sha256=MODULE._sha256_file(registry),
                declared_fixture_sha256=MODULE._sha256_file(fixture),
                run_id="p4-g0c-seed211-rep01",
                seed=211,
                repetition=1,
                run_manifest_path=Path("/tmp/p4-g0c-seed211-rep01/manifest.json"),
                csv_path=Path("/tmp/p4-g0c-seed211-rep01/p4.csv"),
                effective_values=MODULE.P4_G0C_FROZEN_LAUNCH_VALUES,
            )

    def test_registered_profile_materializes_seed_csv_and_bound_manifest(self):
        protocol = REPO / "config/icra27/p4_g0c_protocol_v1.json"
        registry = REPO / "config/icra27/p4_threshold_registry_v1.json"
        fixture = REPO / "config/icra27/p4_g0c_live_fixture_v1.json"
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "p4-g0c-seed223-rep02"
            context = self._context()
            values = {
                "experiment": "p4_g0c_metrics_calibration_v1",
                "p4.g0c.protocol_path": str(protocol),
                "p4.g0c.protocol_sha256": MODULE._sha256_file(protocol),
                "p4.g0c.registry_path": str(registry),
                "p4.g0c.registry_sha256": MODULE._sha256_file(registry),
                "p4.g0c.fixture_path": str(fixture),
                "p4.g0c.fixture_sha256": MODULE._sha256_file(fixture),
                "p4.g0c.run_id": "p4-g0c-seed223-rep02",
                "p4.g0c.seed": "223",
                "p4.g0c.repetition": "2",
                "p4.g0c.run_manifest_path": str(
                    run_dir / "p4_g0c_run_manifest.json"
                ),
                "p4.g0c.csv_path": str(run_dir / "p4_decisions.csv"),
            }
            context.launch_configurations.update(values)
            argv = ["test_planner.launch.py"] + [
                f"{key}:={value}" for key, value in values.items()
            ]
            with mock.patch.object(MODULE.sys, "argv", argv):
                scenario, experiment, _ = MODULE._apply_presets(
                    context, "/iap"
                )
                binding = MODULE._prepare_p4_g0c_context(
                    context, experiment, "/iap"
                )

        self.assertEqual(scenario, "p4_g0c_free_corridor_v1")
        self.assertEqual(binding["seed"], 223)
        self.assertEqual(context.launch_configurations["forest_random_seed"], "223")
        self.assertEqual(context.launch_configurations["gnss_random_seed"], "223")
        self.assertEqual(
            context.launch_configurations["p4.debug_csv_path"],
            str(run_dir / "p4_decisions.csv"),
        )

    def test_conflicting_live_fixture_override_is_rejected(self):
        protocol = REPO / "config/icra27/p4_g0c_protocol_v1.json"
        registry = REPO / "config/icra27/p4_threshold_registry_v1.json"
        fixture = REPO / "config/icra27/p4_g0c_live_fixture_v1.json"
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "p4-g0c-seed211-rep01"
            context = self._context()
            values = {
                "experiment": MODULE.P4_G0C_EXPERIMENT,
                "p4.g0c.protocol_path": str(protocol),
                "p4.g0c.protocol_sha256": MODULE.P4_G0C_PROTOCOL_SHA256,
                "p4.g0c.registry_path": str(registry),
                "p4.g0c.registry_sha256": MODULE.P4_G0C_REGISTRY_SHA256,
                "p4.g0c.fixture_path": str(fixture),
                "p4.g0c.fixture_sha256": MODULE.P4_G0C_FIXTURE_SHA256,
                "p4.g0c.run_id": "p4-g0c-seed211-rep01",
                "p4.g0c.seed": "211",
                "p4.g0c.repetition": "1",
                "p4.g0c.run_manifest_path": str(
                    run_dir / "p4_g0c_run_manifest.json"
                ),
                "p4.g0c.csv_path": str(run_dir / "p4_decisions.csv"),
                "p1_fixture_central_obstacle_enabled": "false",
            }
            context.launch_configurations.update(values)
            argv = ["test_planner.launch.py"] + [
                f"{key}:={value}" for key, value in values.items()
            ]
            with mock.patch.object(MODULE.sys, "argv", argv):
                _, experiment, _ = MODULE._apply_presets(context, "/iap")
                with self.assertRaisesRegex(
                    RuntimeError, "conflicting.*central_obstacle"
                ):
                    MODULE._prepare_p4_g0c_context(
                        context, experiment, "/iap"
                    )


if __name__ == "__main__":
    unittest.main()
