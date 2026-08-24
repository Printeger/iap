import textwrap
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]

from scripts.dev_planner.p4_g0c_surface_classifier import (  # noqa: E402
    SurfaceClassificationError,
    classify_environment_actions,
    classify_mutations,
    classify_process_output_arguments,
    production_surface,
)


class P4G0CSurfaceClassifierTest(unittest.TestCase):
    maxDiff = None

    def test_production_environment_actions_are_all_classified(self):
        surface = production_surface(REPO)
        actual = {
            (
                item["name"],
                item["classification"],
                item["r3_reachable"],
                item["value_kind"],
            )
            for item in surface["environment_actions"]
        }
        self.assertEqual(actual, {
            (
                "FASTRTPS_DEFAULT_PROFILES_FILE",
                "immutable_read_only_path",
                True,
                "trusted_join",
            ),
            ("QT_X11_NO_MITSHM", "non_path_scalar", True, "literal"),
            (
                "XDG_RUNTIME_DIR",
                "registered_mutable_path",
                True,
                "launch_configuration",
            ),
            (
                "XDG_RUNTIME_DIR",
                "registered_mutable_path",
                False,
                "absolute_literal",
            ),
        })

    def test_unregistered_joined_substitution_and_unresolved_environment_fail(
        self,
    ):
        cases = {
            "variable_bound": """
                def generate_launch_description():
                    value = LaunchConfiguration('new.runtime_dir')
                    return SetEnvironmentVariable('NEW_RUNTIME_DIR', value)
            """,
            "joined_path": """
                def generate_launch_description():
                    value = os.path.join('/tmp', 'runtime')
                    return SetEnvironmentVariable('XDG_RUNTIME_DIR', value)
            """,
            "substitution_list": """
                def generate_launch_description():
                    return SetEnvironmentVariable(
                        'XDG_RUNTIME_DIR',
                        [LaunchConfiguration('xdg.runtime_dir'), '/child'],
                    )
            """,
            "unresolved": """
                def generate_launch_description():
                    return SetEnvironmentVariable(
                        'FASTRTPS_DEFAULT_PROFILES_FILE', make_path())
            """,
        }
        for case, source in cases.items():
            with self.subTest(case=case), self.assertRaises(
                SurfaceClassificationError
            ):
                classify_environment_actions(textwrap.dedent(source))

    def test_each_supported_write_family_and_joined_target_is_classified(self):
        source = textwrap.dedent("""
            import os
            import shutil

            def mutate(root):
                open(root / 'builtin', 'w').close()
                open(root / 'builtin-update', 'r+').close()
                (root / 'path-open').open('a').close()
                (root / 'path-create').open('x').close()
                os.open(root / 'descriptor', os.O_WRONLY | os.O_CREAT)
                (root / 'text').write_text('x')
                (root / 'bytes').write_bytes(b'x')
                (root / 'touch').touch()
                (root / 'directory').mkdir()
                os.mkdir(root / 'os-directory')
                os.makedirs(root / 'parents')
                (root / 'old').rename(root / 'renamed')
                (root / 'old2').replace(root / 'replaced')
                (root / 'unlink').unlink()
                os.rename(root / 'os-old', root / 'os-renamed')
                os.replace(root / 'os-old2', root / 'os-replaced')
                os.unlink(root / 'os-unlink')
                shutil.copy(root / 'source', root / 'copy')
                shutil.copy2(root / 'source2', root / 'copy2')
                shutil.copyfile(root / 'source3', root / 'copyfile')
                shutil.copytree(root / 'tree', root / 'tree-copy')
                shutil.move(root / 'move-source', root / 'move-target')
                shutil.rmtree(root / 'remove-tree')
                (root / 'joined' / 'output').write_text('joined')
        """)
        records = classify_mutations(
            source,
            source_name="meta",
            root_bindings={("mutate", "root"): "registered:meta_root"},
        )
        self.assertEqual({item["api"] for item in records}, {
            "builtin.open", "Path.mkdir", "Path.open", "Path.rename",
            "Path.replace", "Path.touch", "Path.unlink", "Path.write_bytes",
            "Path.write_text", "os.makedirs", "os.mkdir", "os.open",
            "os.rename", "os.replace", "os.unlink", "shutil.copy",
            "shutil.copy2", "shutil.copyfile", "shutil.copytree",
            "shutil.move", "shutil.rmtree",
        })
        self.assertEqual(len(records), 24)
        self.assertTrue(all(
            item["classification"] == "derived:meta_root" for item in records
        ))

    def test_unknown_write_primitive_and_unresolved_target_fail(self):
        cases = {
            "unknown_primitive": """
                def mutate(root):
                    (root / 'output').custom_write('x')
            """,
            "unresolved_target": """
                def mutate(root):
                    mystery().write_text('x')
            """,
        }
        for case, source in cases.items():
            with self.subTest(case=case), self.assertRaises(
                SurfaceClassificationError
            ):
                classify_mutations(
                    textwrap.dedent(source),
                    source_name="meta",
                    root_bindings={("mutate", "root"): "registered:meta_root"},
                )

    def test_subprocess_outputs_fail_closed_on_unknown_or_unresolved_paths(self):
        supported = textwrap.dedent("""
            def launch(root):
                subprocess.Popen(command, stdout=root / 'stdout')
                ExecuteProcess(cmd=['tool', '--bag-output', root / 'bags'])
                Node(parameters=[{'summary_path': root / 'summary'}])
        """)
        records = classify_process_output_arguments(
            supported,
            target_policies={},
            root_bindings={("launch", "root"): "registered:meta_root"},
        )
        self.assertEqual({item["api"] for item in records}, {
            "ExecuteProcess.--bag-output", "Node.parameter.summary_path",
            "subprocess.Popen.stdout",
        })
        self.assertTrue(all(
            item["classification"] == "derived:meta_root" for item in records
        ))
        cases = {
            "unknown_flag": """
                def launch(root):
                    ExecuteProcess(cmd=['tool', '--new-output-dir', root])
            """,
            "unresolved_stdout": """
                def launch(root):
                    subprocess.Popen(command, stdout=mystery())
            """,
        }
        for case, source in cases.items():
            with self.subTest(case=case), self.assertRaises(
                SurfaceClassificationError
            ):
                classify_process_output_arguments(
                    textwrap.dedent(source),
                    target_policies={},
                    root_bindings={
                        ("launch", "root"): "registered:meta_root"
                    },
                )

    def test_production_path_and_mutation_surface_is_exact_and_complete(self):
        surface = production_surface(REPO)
        actual = sorted(
            (
                item["function"], item["api"], item["target"],
                item["classification"],
            )
            for item in surface["mutations"]
        )
        self.assertEqual(actual, sorted([
            (
                "_ego_planner_node", "Node.parameter.p1.debug_csv_path",
                "p1_debug_path", "derived:export_root_dir",
            ),
            (
                "_ego_planner_node", "Node.parameter.p2.debug_csv_path",
                "p2_debug_path", "derived:export_root_dir",
            ),
            (
                "_ego_planner_node", "Node.parameter.p3.debug_csv_path",
                "p3_debug_path", "derived:export_root_dir",
            ),
            (
                "_ego_planner_node", "Node.parameter.p4.debug_csv_path",
                "p4_debug_path", "registered:decision_csv_path",
            ),
            (
                "_ego_planner_node", "Node.input.gate0.candidate_events_path",
                "LaunchConfiguration('gate0.candidate_events_path').perform(context)",
                "immutable_read_only_path",
            ),
            (
                "_ego_planner_node", "Node.input.gate0.control_points_path",
                "LaunchConfiguration('gate0.control_points_path').perform(context)",
                "immutable_read_only_path",
            ),
            (
                "_ego_planner_node", "Node.input.gate0.evidence_manifest_path",
                "LaunchConfiguration('gate0.evidence_manifest_path').perform(context)",
                "immutable_read_only_path",
            ),
            (
                "_ego_planner_node", "Node.input.p1.evidence_manifest_path",
                "evidence['manifest_path']", "immutable_read_only_path",
            ),
            (
                "_execute_launch", "Path.open", "run_dir / 'stdout.log'",
                "registered:stdout_log_path",
            ),
            (
                "_execute_launch", "subprocess.Popen.stdout", "output",
                "registered:stdout_log_path",
            ),
            (
                "_launch_setup", "ExecuteProcess.--bag-output",
                "bag_output_dir", "registered:bag_output_dir",
            ),
            (
                "_launch_setup", "ExecuteProcess.--manifest",
                "evidence['manifest_path']", "derived:export_root_dir",
            ),
            (
                "_launch_setup", "Node.parameter.csv_path",
                "str(Path(export_dir) / 'test_planner_integrity_validation.csv')",
                "derived:export_root_dir",
            ),
            (
                "_launch_setup", "Node.parameter.summary_path",
                "str(Path(export_dir) / 'test_planner_validation_summary.json')",
                "derived:export_root_dir",
            ),
            (
                "_launch_setup", "Node.input.manifest_path",
                "evidence['manifest_path']", "immutable_read_only_path",
            ),
            (
                "_launch_setup", "Node.input.config_path",
                "runtime_config_path", "derived:runtime_root_dir",
            ),
            (
                "_launch_setup", "Node.input.manifest_path",
                "evidence['manifest_path']", "immutable_read_only_path",
            ),
            (
                "_launch_setup", "Path.mkdir", "g0c_path.parent",
                "derived:runs_root",
            ),
            (
                "_launch_setup", "Path.write_text", "g0c_path",
                "registered:run_manifest_path",
            ),
            (
                "_launch_setup", "Path.write_text", "manifest_path",
                "derived:export_root_dir",
            ),
            (
                "_launch_setup", "os.makedirs", "bag_root_dir",
                "registered:bag_output_dir",
            ),
            (
                "_materialize_gnss_scenario", "Path.write_text", "path",
                "derived:export_root_dir",
            ),
            (
                "_materialize_iap_logging_config", "Path.write_text",
                "config_path", "derived:runtime_root_dir",
            ),
            (
                "_materialize_iap_logging_config", "config.assignment.log_dir",
                "str(requested)", "registered:iap_log_root",
            ),
            (
                "_materialize_iap_logging_config", "config.assignment.log_dir",
                "str(requested)", "registered:iap_log_root",
            ),
            (
                "_materialize_iap_logging_config",
                "config.assignment.timing_csv_path", "str(timing_path)",
                "derived:iap_log_root",
            ),
            (
                "_materialize_iap_logging_config", "Path.write_text",
                "referenced_path", "derived:runtime_root_dir",
            ),
            (
                "_override_odometry_initialization_mode", "Path.write_text",
                "path", "derived:runtime_root_dir",
            ),
            (
                "_persist_result", "Path.mkdir", "runs_root",
                "registered:runs_root",
            ),
            (
                "_persist_result", "Path.write_text",
                "runs_root / 'p4_g0c_runner_state.json'", "derived:runs_root",
            ),
            (
                "_runtime_config", "Path.mkdir", "export_dir",
                "registered:export_root_dir",
            ),
            (
                "_runtime_config", "Path.open", "config_gnss_path",
                "derived:runtime_root_dir",
            ),
            (
                "_runtime_config", "Path.open", "config_path",
                "derived:runtime_root_dir",
            ),
            (
                "_runtime_config", "Path.open", "config_ros_path",
                "derived:runtime_root_dir",
            ),
            (
                "_runtime_config", "shutil.copy2", "config_odometry_path",
                "derived:runtime_root_dir",
            ),
            (
                "_runtime_config", "shutil.copytree", "runtime_config_dir",
                "derived:runtime_root_dir",
            ),
            (
                "_runtime_config", "shutil.copytree",
                "runtime_root / 'sim_ego'", "derived:runtime_root_dir",
            ),
            (
                "_validate_and_finalize_run", "Path.open", "inventory_path",
                "derived:runs_root",
            ),
            (
                "_validate_and_finalize_run", "Path.write_text",
                "manifest_path", "registered:run_manifest_path",
            ),
            (
                "prepare_launch_environment", "Path.mkdir",
                "environment_root", "derived:runs_root",
            ),
            (
                "prepare_launch_environment", "os.fchmod", "directory_fd",
                "registered:XDG_RUNTIME_DIR",
            ),
            (
                "prepare_launch_environment", "os.mkdir", "path.name",
                "derived:runs_root",
            ),
            (
                "launch_command", "ros2.launch.argument.bag_output_dir",
                "outputs['bag_output_dir']", "registered:bag_output_dir",
            ),
            (
                "launch_command", "ros2.launch.argument.export_root_dir",
                "outputs['export_root_dir']", "registered:export_root_dir",
            ),
            (
                "launch_command", "ros2.launch.argument.iap_log_root",
                "outputs['iap_log_root']", "registered:iap_log_root",
            ),
            (
                "launch_command", "ros2.launch.argument.p4.g0c.csv_path",
                "str(csv_path)", "registered:decision_csv_path",
            ),
            (
                "launch_command",
                "ros2.launch.argument.p4.g0c.run_manifest_path",
                "str(manifest_path)", "registered:run_manifest_path",
            ),
            (
                "launch_command", "ros2.launch.argument.runtime_root_dir",
                "outputs['runtime_root_dir']", "registered:runtime_root_dir",
            ),
            ("run", "Path.mkdir", "run_dir", "derived:runs_root"),
            (
                "run", "Path.write_text", "run_dir / 'launch_command.json'",
                "registered:launch_command_path",
            ),
        ]))

    def test_production_surface_reaches_all_eight_registered_outputs(self):
        expected = {
            "bag_output_dir", "decision_csv_path", "export_root_dir",
            "iap_log_root", "launch_command_path", "run_manifest_path",
            "runtime_root_dir", "stdout_log_path",
        }
        surface = production_surface(REPO)
        actual = {
            item["classification"].split(":", 1)[1]
            for item in surface["mutations"]
            if ":" in item["classification"]
            and item["classification"].split(":", 1)[1] in expected
        }
        self.assertEqual(actual, expected)

    def test_runner_launch_path_arguments_are_complete_and_classified(self):
        bindings = production_surface(REPO)["runner_launch_bindings"]
        self.assertEqual(bindings["child_environment"], {
            "p4.g0c.child_home": "HOME",
            "p4.g0c.child_ros_home": "ROS_HOME",
            "p4.g0c.child_ros_log_dir": "ROS_LOG_DIR",
            "p4.g0c.child_tmpdir": "TMPDIR",
            "p4.g0c.child_xdg_runtime_dir": "XDG_RUNTIME_DIR",
        })
        self.assertEqual(bindings["mutable_output_arguments"], {
            "bag_output_dir": "bag_output_dir",
            "export_root_dir": "export_root_dir",
            "iap_log_root": "iap_log_root",
            "p4.g0c.csv_path": "decision_csv_path",
            "p4.g0c.run_manifest_path": "run_manifest_path",
            "runtime_root_dir": "runtime_root_dir",
        })
        self.assertEqual(bindings["immutable_input_arguments"], {
            "p4.g0c.fixture_path", "p4.g0c.protocol_path",
            "p4.g0c.registry_path",
        })


if __name__ == "__main__":
    unittest.main()
