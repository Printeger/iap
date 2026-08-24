import ast
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from launch import LaunchContext


REPO = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO / "launch" / "test_planner.launch.py"
RUNNER_PATH = REPO / "scripts" / "dev_planner" / "run_p4_g0c_calibration.py"
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
        source = MODULE_PATH.read_text()
        tree = ast.parse(source)
        runner_tree = ast.parse(RUNNER_PATH.read_text())
        functions = {
            node.name: node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
        }
        runner_functions = {
            node.name: node
            for node in runner_tree.body
            if isinstance(node, ast.FunctionDef)
        }

        def launch_configurations(node):
            return {
                item.args[0].value
                for item in ast.walk(node)
                if (
                    isinstance(item, ast.Call)
                    and isinstance(item.func, ast.Name)
                    and item.func.id == "LaunchConfiguration"
                    and item.args
                    and isinstance(item.args[0], ast.Constant)
                    and isinstance(item.args[0].value, str)
                )
            }

        generate = functions["generate_launch_description"]
        environment_actions = []
        for node in ast.walk(generate):
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "SetEnvironmentVariable"
                and len(node.args) >= 2
                and isinstance(node.args[0], ast.Constant)
            ):
                environment_actions.append(node)
        path_environment_actions = [
            node for node in environment_actions
            if (
                isinstance(node.args[1], ast.Call)
                and isinstance(node.args[1].func, ast.Name)
                and node.args[1].func.id == "LaunchConfiguration"
            ) or (
                isinstance(node.args[1], ast.Constant)
                and isinstance(node.args[1].value, str)
                and node.args[1].value.startswith("/")
            )
        ]
        self.assertEqual(
            {node.args[0].value for node in path_environment_actions},
            {"XDG_RUNTIME_DIR"},
        )
        xdg_actions = path_environment_actions
        self.assertEqual(len(xdg_actions), 2)
        self.assertTrue(all(
            any(keyword.arg == "condition" for keyword in node.keywords)
            for node in xdg_actions
        ))
        registered = [
            node for node in xdg_actions
            if isinstance(node.args[1], ast.Call)
        ]
        legacy = [
            node for node in xdg_actions
            if isinstance(node.args[1], ast.Constant)
        ]
        self.assertEqual(len(registered), 1)
        self.assertEqual(len(legacy), 1)
        self.assertEqual(legacy[0].args[1].value, "/tmp/runtime-root")
        registered_condition = {
            keyword.arg: keyword.value for keyword in registered[0].keywords
        }["condition"]
        legacy_condition = {
            keyword.arg: keyword.value for keyword in legacy[0].keywords
        }["condition"]
        self.assertEqual(registered_condition.func.id, "IfCondition")
        self.assertEqual(
            registered_condition.args[0].func.id, "EqualsSubstitution"
        )
        self.assertEqual(legacy_condition.func.id, "IfCondition")
        self.assertEqual(
            legacy_condition.args[0].func.id, "NotEqualsSubstitution"
        )
        for condition in (registered_condition, legacy_condition):
            predicate = condition.args[0]
            self.assertEqual(
                predicate.args[0].func.id, "LaunchConfiguration"
            )
            self.assertEqual(predicate.args[0].args[0].value, "experiment")
            self.assertEqual(predicate.args[1].id, "P4_G0C_EXPERIMENT_V3")
        xdg_value = registered[0].args[1]
        self.assertIsInstance(xdg_value, ast.Call)
        self.assertIsInstance(xdg_value.func, ast.Name)
        self.assertEqual(xdg_value.func.id, "LaunchConfiguration")
        self.assertEqual(xdg_value.args[0].value, "p4.g0c.child_xdg_runtime_dir")

        prepare = functions["_prepare_p4_g0c_context"]
        bindings = [
            node for node in ast.walk(prepare)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "_p4_g0c_binding"
        ]
        self.assertEqual(len(bindings), 1)
        keywords = {item.arg: item.value for item in bindings[0].keywords}
        child_keys = {
            key.value for key in keywords["child_environment"].keys
        }
        output_keys = {
            key.value for key in keywords["mutable_output_paths"].keys
        }
        self.assertEqual(child_keys, {
            "HOME", "ROS_HOME", "ROS_LOG_DIR", "TMPDIR", "XDG_RUNTIME_DIR",
        })
        self.assertEqual(output_keys, {
            "bag_output_dir", "decision_csv_path", "export_root_dir",
            "iap_log_root", "launch_command_path", "run_manifest_path",
            "runtime_root_dir", "stdout_log_path",
        })

        launch_command = runner_functions["launch_command"]
        runner_child_arguments = {}
        runner_output_arguments = {}
        for node in ast.walk(launch_command):
            if not isinstance(node, ast.Dict):
                continue
            for key, value in zip(node.keys, node.values):
                if not isinstance(key, ast.Constant) or not isinstance(key.value, str):
                    continue
                if (
                    isinstance(value, ast.Subscript)
                    and isinstance(value.value, ast.Name)
                    and isinstance(value.slice, ast.Constant)
                ):
                    if value.value.id == "child":
                        runner_child_arguments[key.value] = value.slice.value
                    elif value.value.id == "outputs":
                        runner_output_arguments[key.value] = value.slice.value
        self.assertEqual(runner_child_arguments, {
            "p4.g0c.child_home": "HOME",
            "p4.g0c.child_ros_home": "ROS_HOME",
            "p4.g0c.child_ros_log_dir": "ROS_LOG_DIR",
            "p4.g0c.child_tmpdir": "TMPDIR",
            "p4.g0c.child_xdg_runtime_dir": "XDG_RUNTIME_DIR",
        })
        self.assertEqual(set(runner_child_arguments.values()), child_keys)

        run_local_paths = {}
        launch_values = None
        for node in launch_command.body:
            if not isinstance(node, ast.Assign) or len(node.targets) != 1:
                continue
            target = node.targets[0]
            if (
                isinstance(target, ast.Name)
                and isinstance(node.value, ast.BinOp)
                and isinstance(node.value.op, ast.Div)
                and isinstance(node.value.left, ast.Name)
                and node.value.left.id == "run_dir"
                and isinstance(node.value.right, ast.Constant)
            ):
                run_local_paths[target.id] = node.value.right.value
            if (
                isinstance(target, ast.Name)
                and target.id == "values"
                and isinstance(node.value, ast.Dict)
            ):
                launch_values = node.value
        self.assertIsNotNone(launch_values)
        local_path_semantics = {
            "p4_g0c_run_manifest.json": "run_manifest_path",
            "p4_decisions.csv": "decision_csv_path",
        }
        for key, value in zip(launch_values.keys, launch_values.values):
            if not isinstance(key, ast.Constant) or not isinstance(key.value, str):
                continue
            local_names = {
                item.id for item in ast.walk(value)
                if isinstance(item, ast.Name) and item.id in run_local_paths
            }
            if local_names:
                self.assertEqual(len(local_names), 1)
                filename = run_local_paths[local_names.pop()]
                self.assertIn(filename, local_path_semantics)
                runner_output_arguments[key.value] = local_path_semantics[filename]
        self.assertEqual(runner_output_arguments, {
            "bag_output_dir": "bag_output_dir",
            "export_root_dir": "export_root_dir",
            "iap_log_root": "iap_log_root",
            "p4.g0c.csv_path": "decision_csv_path",
            "p4.g0c.run_manifest_path": "run_manifest_path",
            "runtime_root_dir": "runtime_root_dir",
        })

        direct_run_writes = set()
        for function_name in ("_execute_launch", "run"):
            for node in ast.walk(runner_functions[function_name]):
                if (
                    not isinstance(node, ast.Call)
                    or not isinstance(node.func, ast.Attribute)
                    or node.func.attr not in {"open", "write_text"}
                    or not isinstance(node.func.value, ast.BinOp)
                    or not isinstance(node.func.value.op, ast.Div)
                    or not isinstance(node.func.value.left, ast.Name)
                    or node.func.value.left.id != "run_dir"
                    or not isinstance(node.func.value.right, ast.Constant)
                ):
                    continue
                if node.func.attr == "open" and not (
                    node.args
                    and isinstance(node.args[0], ast.Constant)
                    and str(node.args[0].value).startswith(("w", "a", "x"))
                ):
                    continue
                direct_run_writes.add(node.func.value.right.value)
        self.assertEqual(
            direct_run_writes, {"launch_command.json", "stdout.log"}
        )
        direct_write_semantics = {
            "launch_command.json": "launch_command_path",
            "stdout.log": "stdout_log_path",
        }

        runtime_config = functions["_runtime_config"]
        launch_mutable_arguments = set()
        for node in ast.walk(runtime_config):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
                continue
            if node.func.id in {
                "_resolve_run_roots", "_materialize_iap_logging_config",
            }:
                launch_mutable_arguments.update(launch_configurations(node))

        launch_setup = functions["_launch_setup"]
        bag_root_assignments = [
            node for node in ast.walk(launch_setup)
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == "bag_root_dir"
                for target in node.targets
            )
            and launch_configurations(node)
        ]
        self.assertEqual(len(bag_root_assignments), 1)
        self.assertEqual(
            launch_configurations(bag_root_assignments[0]), {"bag_output_dir"}
        )
        self.assertTrue(any(
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "os"
            and node.func.attr == "makedirs"
            and node.args
            and isinstance(node.args[0], ast.Name)
            and node.args[0].id == "bag_root_dir"
            for node in ast.walk(launch_setup)
        ))
        launch_mutable_arguments.add("bag_output_dir")

        g0c_path_assignments = [
            node for node in ast.walk(launch_setup)
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == "g0c_path"
                for target in node.targets
            )
        ]
        self.assertEqual(len(g0c_path_assignments), 1)
        self.assertEqual(
            launch_configurations(g0c_path_assignments[0]),
            {"p4.g0c.run_manifest_path"},
        )
        self.assertTrue(any(
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "g0c_path"
            and node.func.attr == "write_text"
            for node in ast.walk(launch_setup)
        ))
        launch_mutable_arguments.add("p4.g0c.run_manifest_path")

        csv_aliases = [
            node for node in ast.walk(prepare)
            if isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
            and node.targets[0].id == "csv_path"
        ]
        self.assertEqual(len(csv_aliases), 1)
        self.assertEqual(
            launch_configurations(csv_aliases[0]), {"p4.g0c.csv_path"}
        )
        self.assertTrue(any(
            isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Subscript)
            and isinstance(node.targets[0].value, ast.Attribute)
            and node.targets[0].value.attr == "launch_configurations"
            and isinstance(node.targets[0].slice, ast.Constant)
            and node.targets[0].slice.value == "p4.debug_csv_path"
            and isinstance(node.value, ast.Name)
            and node.value.id == "csv_path"
            for node in ast.walk(prepare)
        ))
        ego_planner = functions["_ego_planner_node"]
        self.assertTrue(any(
            isinstance(node, ast.Dict)
            and any(
                isinstance(key, ast.Constant)
                and key.value == "p4.debug_csv_path"
                and isinstance(value, ast.Name)
                and value.id == "p4_debug_path"
                for key, value in zip(node.keys, node.values)
            )
            for node in ast.walk(ego_planner)
        ))
        launch_mutable_arguments.add("p4.g0c.csv_path")

        self.assertEqual(launch_mutable_arguments, {
            "bag_output_dir", "export_root_dir", "iap_log_root",
            "p4.g0c.csv_path", "p4.g0c.run_manifest_path",
            "runtime_root_dir",
        })
        launch_argument_semantics = {
            key: runner_output_arguments[key]
            for key in launch_mutable_arguments
        }
        actual_output_keys = set(launch_argument_semantics.values()) | {
            direct_write_semantics[path] for path in direct_run_writes
        }
        self.assertEqual(actual_output_keys, output_keys)
        self.assertIn(
            "p4.g0c.child_xdg_runtime_dir", dict(MODULE.ARG_DEFAULTS)
        )

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
