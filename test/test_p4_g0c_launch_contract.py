import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

REPO = Path(__file__).resolve().parents[1]
SCRIPT_DIR = REPO / "scripts" / "dev_planner"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from run_p4_g0c_tests import require_hermetic_test_environment  # noqa: E402
from p4_g0c_surface_classifier import (  # noqa: E402
    production_surface_inventory,
)

require_hermetic_test_environment()

from launch import LaunchContext  # noqa: E402


MODULE_PATH = REPO / "launch" / "test_planner.launch.py"
SPEC = importlib.util.spec_from_file_location("test_planner_launch_g0c", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class P4G0CLaunchContractTest(unittest.TestCase):
    def test_v5_preset_materializes_only_the_versioned_obstacle_interval(self):
        v4 = MODULE.EXPERIMENT_PRESETS[MODULE.P4_G0C_EXPERIMENT_V4]
        v5 = MODULE.EXPERIMENT_PRESETS[MODULE.P4_G0C_EXPERIMENT_V5]
        differences = {
            key for key in set(v4) | set(v5) if v4.get(key) != v5.get(key)
        }
        self.assertEqual(differences, {
            "p4.g0c.protocol_path", "p4.g0c.registry_path",
            "p4.g0c.fixture_path", "p4.g0c.fixture_sha256",
            "p1_fixture_central_x_min_m",
            "p1_fixture_central_x_max_m",
        })
        self.assertEqual(v5["p1_fixture_central_x_min_m"], "-9.0")
        self.assertEqual(v5["p1_fixture_central_x_max_m"], "-7.0")
        self.assertEqual(dict(MODULE.ARG_DEFAULTS)["init_x"], "-12.0")
        self.assertEqual(
            dict(MODULE.ARG_DEFAULTS)["manager/planning_horizon"], "7.5"
        )
        self.assertEqual(
            dict(MODULE.ARG_DEFAULTS)["manager/control_points_distance"],
            "0.4",
        )

    def test_v4_preset_materializes_exact_p0_profile(self):
        profile = MODULE.EXPERIMENT_PRESETS[MODULE.P4_G0C_EXPERIMENT_V4]
        self.assertEqual(
            profile["p0.predictor.sigma_grow_m_sqrt_s"], "0.01"
        )
        self.assertEqual(
            profile["p0.predictor.sigma_growth_profile"],
            "legacy_iap_rq320_baseline_v1",
        )
        self.assertEqual(
            dict(MODULE.ARG_DEFAULTS)[
                "p4.require_risk_grid_ready_before_planning"
            ],
            "false",
        )
        self.assertEqual(
            profile["p4.require_risk_grid_ready_before_planning"], "true"
        )
        MODULE._validate_p4_g0c_profile_values(
            MODULE.P4_G0C_EXPERIMENT_V4, profile, set()
        )
        for key, value in MODULE.P4_G0C_V4_P0_PROFILE_VALUES.items():
            with self.subTest(key=key), self.assertRaisesRegex(
                RuntimeError, "profiled P0 mismatch"
            ):
                MODULE._validate_p4_g0c_profile_values(
                    MODULE.P4_G0C_EXPERIMENT_V4,
                    {**profile, key: "drift" if value != "0.01" else "0.02"},
                    {key},
                )

    def test_v4_readiness_mode_never_accepts_a_registered_r4_identity(self):
        protocol = REPO / "config/icra27/p4_g0c_protocol_v4.json"
        registry = REPO / "config/icra27/p4_threshold_registry_v4.json"
        fixture = REPO / "config/icra27/p4_g0c_live_fixture_v1.json"
        run_id = "p4-g0c-r4-seed211-rep01"
        with tempfile.TemporaryDirectory() as tmp, self.assertRaisesRegex(
            RuntimeError, "readiness identity is not isolated"
        ):
            run_dir = Path(tmp) / run_id
            MODULE._p4_g0c_binding(
                experiment=MODULE.P4_G0C_EXPERIMENT_V4,
                protocol_path=protocol,
                registry_path=registry,
                fixture_path=fixture,
                declared_protocol_sha256=MODULE._sha256_file(protocol),
                declared_registry_sha256=MODULE._sha256_file(registry),
                declared_fixture_sha256=MODULE._sha256_file(fixture),
                run_id=run_id, seed=211, repetition=1,
                run_manifest_path=run_dir / "p4_g0c_run_manifest.json",
                csv_path=run_dir / "p4_decisions.csv",
                effective_values={
                    **MODULE.P4_G0C_FROZEN_LAUNCH_VALUES,
                    **MODULE.P4_G0C_V4_P0_PROFILE_VALUES,
                },
                readiness_mode=True,
            )

    @staticmethod
    def _context():
        context = LaunchContext()
        for name, default in MODULE.ARG_DEFAULTS:
            context.launch_configurations[name] = str(default)
        return context

    @staticmethod
    def _node_parameters(node, context):
        result = {}
        for item in node._Node__parameters:
            for substitutions, value in item.items():
                key = "".join(part.perform(context) for part in substitutions)
                result[key] = value
        return result

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

    def test_r3_registered_paths_equal_production_launch_surface(self):
        surface = production_surface_inventory(REPO)
        self.assertEqual(
            sorted(item["name"] for item in surface["environment_actions"]),
            [
                "FASTRTPS_DEFAULT_PROFILES_FILE", "QT_X11_NO_MITSHM",
                "XDG_RUNTIME_DIR", "XDG_RUNTIME_DIR", "XDG_RUNTIME_DIR",
                "XDG_RUNTIME_DIR",
            ],
        )
        bindings = surface["runner_launch_bindings"]
        self.assertEqual(set(bindings["child_environment"].values()), {
            "HOME", "ROS_HOME", "ROS_LOG_DIR", "TMPDIR", "XDG_RUNTIME_DIR",
        })
        expected_outputs = {
            "bag_output_dir", "decision_csv_path", "export_root_dir",
            "iap_log_root", "launch_command_path", "run_manifest_path",
            "runtime_root_dir", "stdout_log_path",
        }
        classified_outputs = {
            item["classification"].split(":", 1)[1]
            for item in surface["mutations"]
            if ":" in item["classification"]
            and item["classification"].split(":", 1)[1] in expected_outputs
        }
        self.assertEqual(classified_outputs, expected_outputs)

    def test_v2_profile_and_binding_use_only_r2_identity(self):
        profile = MODULE.EXPERIMENT_PRESETS[
            "p4_g0c_metrics_calibration_v2"
        ]
        self.assertEqual(profile, {
            **MODULE.P4_G0C_FROZEN_LAUNCH_VALUES,
            **MODULE.P4_G0C_ARTIFACT_PRESET_V2,
            "scenario": "p4_g0c_free_corridor_v1",
        })
        protocol = REPO / "config/icra27/p4_g0c_protocol_v2.json"
        registry = REPO / "config/icra27/p4_threshold_registry_v2.json"
        fixture = REPO / "config/icra27/p4_g0c_live_fixture_v1.json"
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "p4-g0c-r2-seed211-rep01"
            binding = MODULE._p4_g0c_binding(
                experiment="p4_g0c_metrics_calibration_v2",
                protocol_path=protocol,
                registry_path=registry,
                fixture_path=fixture,
                declared_protocol_sha256=MODULE._sha256_file(protocol),
                declared_registry_sha256=MODULE._sha256_file(registry),
                declared_fixture_sha256=MODULE._sha256_file(fixture),
                run_id=run_dir.name,
                seed=211,
                repetition=1,
                run_manifest_path=run_dir / "p4_g0c_run_manifest.json",
                csv_path=run_dir / "p4_decisions.csv",
                effective_values=MODULE.P4_G0C_FROZEN_LAUNCH_VALUES,
            )
        self.assertEqual(binding["schema_version"], "p4_g0c_run_manifest_v2")
        self.assertEqual(binding["run_id"], "p4-g0c-r2-seed211-rep01")
        protocol_payload = json.loads(protocol.read_text())
        self.assertEqual(
            binding["dependency_manifest_sha256"],
            protocol_payload["runtime_dependency_manifest"]["sha256"],
        )
        self.assertEqual(
            binding["replacement_lineage_sha256"],
            protocol_payload["replacement_lineage"]["sha256"],
        )
        with self.assertRaisesRegex(RuntimeError, "run identity"):
            MODULE._p4_g0c_binding(
                experiment="p4_g0c_metrics_calibration_v2",
                protocol_path=protocol,
                registry_path=registry,
                fixture_path=fixture,
                declared_protocol_sha256=MODULE._sha256_file(protocol),
                declared_registry_sha256=MODULE._sha256_file(registry),
                declared_fixture_sha256=MODULE._sha256_file(fixture),
                run_id="p4-g0c-seed211-rep01",
                seed=211,
                repetition=1,
                run_manifest_path=(
                    Path(tmp) / "p4-g0c-seed211-rep01/p4_g0c_run_manifest.json"
                ),
                csv_path=Path(tmp) / "p4-g0c-seed211-rep01/p4_decisions.csv",
                effective_values=MODULE.P4_G0C_FROZEN_LAUNCH_VALUES,
            )

    def test_v3_binding_requires_exact_propagated_environment_and_outputs(self):
        protocol = REPO / "config/icra27/p4_g0c_protocol_v3.json"
        registry = REPO / "config/icra27/p4_threshold_registry_v3.json"
        fixture = REPO / "config/icra27/p4_g0c_live_fixture_v1.json"
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp).resolve()
            run_dir = root / "p4-g0c-r3-seed211-rep01"
            environment_root = root / "launch_environment"
            child = {
                "HOME": str(environment_root / "home"),
                "ROS_HOME": str(environment_root / "ros_home"),
                "ROS_LOG_DIR": str(environment_root / "ros_logs"),
                "TMPDIR": str(environment_root / "tmp"),
                "XDG_RUNTIME_DIR": str(environment_root / "xdg_runtime"),
            }
            outputs = {
                "bag_output_dir": str(run_dir / "bags"),
                "decision_csv_path": str(run_dir / "p4_decisions.csv"),
                "export_root_dir": str(run_dir / "exports"),
                "iap_log_root": str(run_dir / "runtime/iap_logs"),
                "launch_command_path": str(run_dir / "launch_command.json"),
                "run_manifest_path": str(run_dir / "p4_g0c_run_manifest.json"),
                "runtime_root_dir": str(run_dir / "runtime"),
                "stdout_log_path": str(run_dir / "stdout.log"),
            }
            arguments = {
                "experiment": MODULE.P4_G0C_EXPERIMENT_V3,
                "protocol_path": protocol,
                "registry_path": registry,
                "fixture_path": fixture,
                "declared_protocol_sha256": MODULE._sha256_file(protocol),
                "declared_registry_sha256": MODULE._sha256_file(registry),
                "declared_fixture_sha256": MODULE._sha256_file(fixture),
                "run_id": run_dir.name,
                "seed": 211,
                "repetition": 1,
                "run_manifest_path": run_dir / "p4_g0c_run_manifest.json",
                "csv_path": run_dir / "p4_decisions.csv",
                "effective_values": MODULE.P4_G0C_FROZEN_LAUNCH_VALUES,
                "child_environment": child,
                "mutable_output_paths": outputs,
            }
            with mock.patch.dict(MODULE.os.environ, child, clear=False):
                binding = MODULE._p4_g0c_binding(**arguments)
            self.assertEqual(binding["schema_version"], "p4_g0c_run_manifest_v3")
            self.assertEqual(binding["child_environment"], child)
            self.assertEqual(binding["mutable_output_paths"], outputs)

            for key in child:
                with self.subTest(key=key):
                    bad_environment = dict(child)
                    bad_environment[key] = f"{child[key]}-drift"
                    with mock.patch.dict(MODULE.os.environ, child, clear=False):
                        with self.assertRaisesRegex(
                            RuntimeError, "child environment is not canonical"
                        ):
                            MODULE._p4_g0c_binding(
                                **{
                                    **arguments,
                                    "child_environment": bad_environment,
                                }
                            )

            with mock.patch.dict(
                MODULE.os.environ,
                {**child, "ROS_LOG_DIR": f"{child['ROS_LOG_DIR']}-drift"},
                clear=False,
            ):
                with self.assertRaisesRegex(
                    RuntimeError, "propagated child environment mismatch"
                ):
                    MODULE._p4_g0c_binding(**arguments)

    def test_v2_launch_trust_split_requires_hashes_and_freezes_science(self):
        self.assertEqual(
            MODULE.P4_G0C_ARTIFACT_PRESET_V2["p4.g0c.protocol_sha256"],
            MODULE.P4_G0C_RUNTIME_HASH_REQUIRED,
        )
        self.assertEqual(
            MODULE.P4_G0C_ARTIFACT_PRESET_V2["p4.g0c.registry_sha256"],
            MODULE.P4_G0C_RUNTIME_HASH_REQUIRED,
        )
        fixture = REPO / "config/icra27/p4_g0c_live_fixture_v1.json"
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            protocol = root / "protocol.json"
            registry = root / "registry.json"
            protocol_payload = json.loads(
                (REPO / "config/icra27/p4_g0c_protocol_v2.json").read_text()
            )
            registry_payload = json.loads(
                (REPO / "config/icra27/p4_threshold_registry_v2.json").read_text()
            )
            protocol_payload["threshold_formulas"][
                "mean_improvement_min"
            ] = "Q50(original_mean-risk_mean)"
            protocol.write_bytes(MODULE._canonical_json_bytes(protocol_payload))
            registry_payload["protocol_sha256"] = MODULE._sha256_file(protocol)
            registry.write_bytes(MODULE._canonical_json_bytes(registry_payload))
            run_dir = root / "p4-g0c-r2-seed211-rep01"
            arguments = {
                "experiment": MODULE.P4_G0C_EXPERIMENT_V2,
                "protocol_path": protocol,
                "registry_path": registry,
                "fixture_path": fixture,
                "declared_protocol_sha256": MODULE._sha256_file(protocol),
                "declared_registry_sha256": MODULE._sha256_file(registry),
                "declared_fixture_sha256": MODULE._sha256_file(fixture),
                "run_id": run_dir.name,
                "seed": 211,
                "repetition": 1,
                "run_manifest_path": run_dir / "p4_g0c_run_manifest.json",
                "csv_path": run_dir / "p4_decisions.csv",
                "effective_values": MODULE.P4_G0C_FROZEN_LAUNCH_VALUES,
            }
            with self.assertRaisesRegex(RuntimeError, "scientific identity"):
                MODULE._p4_g0c_binding(**arguments)
            protocol_payload = json.loads(
                (REPO / "config/icra27/p4_g0c_protocol_v2.json").read_text()
            )
            protocol_payload["no_retry"] = 1
            protocol.write_bytes(MODULE._canonical_json_bytes(protocol_payload))
            registry_payload["protocol_sha256"] = MODULE._sha256_file(protocol)
            registry.write_bytes(MODULE._canonical_json_bytes(registry_payload))
            arguments.update({
                "declared_protocol_sha256": MODULE._sha256_file(protocol),
                "declared_registry_sha256": MODULE._sha256_file(registry),
            })
            with self.assertRaisesRegex(RuntimeError, "scientific identity"):
                MODULE._p4_g0c_binding(**arguments)
            arguments.update({
                "protocol_path": REPO / "config/icra27/p4_g0c_protocol_v2.json",
                "registry_path": REPO / "config/icra27/p4_threshold_registry_v2.json",
                "declared_protocol_sha256": MODULE._sha256_file(
                    REPO / "config/icra27/p4_g0c_protocol_v2.json"
                ),
                "declared_registry_sha256": MODULE._sha256_file(
                    REPO / "config/icra27/p4_threshold_registry_v2.json"
                ),
                "effective_values": {
                    **MODULE.P4_G0C_FROZEN_LAUNCH_VALUES,
                    "p1.metrics_only": 0,
                },
            })
            with self.assertRaisesRegex(RuntimeError, "effective config"):
                MODULE._p4_g0c_binding(**arguments)
            arguments.update({
                "protocol_path": REPO / "config/icra27/p4_g0c_protocol_v2.json",
                "registry_path": REPO / "config/icra27/p4_threshold_registry_v2.json",
                "effective_values": MODULE.P4_G0C_FROZEN_LAUNCH_VALUES,
                "declared_protocol_sha256": MODULE.P4_G0C_RUNTIME_HASH_REQUIRED,
                "declared_registry_sha256": MODULE.P4_G0C_RUNTIME_HASH_REQUIRED,
            })
            with self.assertRaisesRegex(RuntimeError, "protocol hash mismatch"):
                MODULE._p4_g0c_binding(**arguments)

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

    def test_v2_real_launch_path_keeps_node_and_both_manifests_false(self):
        protocol = REPO / "config/icra27/p4_g0c_protocol_v2.json"
        registry = REPO / "config/icra27/p4_threshold_registry_v2.json"
        fixture = REPO / "config/icra27/p4_g0c_live_fixture_v1.json"
        package_shares = {
            "iap": REPO,
            "local_sensing": REPO / "src/uav_simulator/local_sensing",
            "so3_control": REPO / "src/uav_simulator/so3_control",
        }
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            runtime_root = root / "runtime"
            export_root = root / "exports"
            run_dir = root / "p4-g0c-r2-seed211-rep01"
            values = {
                "experiment": MODULE.P4_G0C_EXPERIMENT_V2,
                "runtime_root_dir": str(runtime_root),
                "export_root_dir": str(export_root),
                "iap_log_root": str(runtime_root / "iap_logs"),
                "p4.g0c.protocol_path": str(protocol),
                "p4.g0c.protocol_sha256": MODULE._sha256_file(protocol),
                "p4.g0c.registry_path": str(registry),
                "p4.g0c.registry_sha256": MODULE._sha256_file(registry),
                "p4.g0c.fixture_path": str(fixture),
                "p4.g0c.fixture_sha256": MODULE._sha256_file(fixture),
                "p4.g0c.run_id": run_dir.name,
                "p4.g0c.seed": "211",
                "p4.g0c.repetition": "1",
                "p4.g0c.run_manifest_path": str(
                    run_dir / "p4_g0c_run_manifest.json"
                ),
                "p4.g0c.csv_path": str(run_dir / "p4_decisions.csv"),
            }
            context = self._context()
            context.launch_configurations.update(values)
            argv = ["test_planner.launch.py"] + [
                f"{key}:={value}" for key, value in values.items()
            ]

            def share(package):
                return str(package_shares[package])

            with (
                mock.patch.object(MODULE.sys, "argv", argv),
                mock.patch.object(
                    MODULE, "get_package_share_directory", side_effect=share
                ),
                mock.patch.object(
                    MODULE,
                    "get_package_prefix",
                    side_effect=lambda package: str(root / "prefix" / package),
                ),
            ):
                actions = MODULE._launch_setup(context)

            planner_nodes = [
                action for action in actions
                if getattr(action, "_Node__package", "") == "ego_planner"
                and getattr(action, "_Node__node_executable", "")
                == "ego_planner_node"
            ]
            self.assertEqual(len(planner_nodes), 1)
            planner_values = self._node_parameters(planner_nodes[0], context)
            test_manifest_paths = list(
                export_root.rglob("test_planner_manifest.json")
            )
            self.assertEqual(len(test_manifest_paths), 1)
            test_manifest = json.loads(test_manifest_paths[0].read_text())
            run_manifest = json.loads(
                (run_dir / "p4_g0c_run_manifest.json").read_text()
            )
            protocol_values = json.loads(protocol.read_text())["effective_values"]

        for key in ("p1.metrics_only", "p2.metrics_only"):
            self.assertFalse(planner_values[key])
            self.assertFalse(test_manifest[key])
            self.assertFalse(run_manifest["effective_values"][key])
            self.assertFalse(protocol_values[key])
            self.assertEqual(test_manifest[key], planner_values[key])
            self.assertEqual(
                run_manifest["effective_values"][key], planner_values[key]
            )

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
