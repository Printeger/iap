import importlib.util
import json
import os
import stat
import struct
import subprocess
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
RUNNER_PATH = REPO / "scripts/dev_planner/run_p4_g0c_calibration.py"
SPEC = importlib.util.spec_from_file_location(
    "run_p4_g0c_calibration_dependency_test", RUNNER_PATH
)
RUNNER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(RUNNER)


class P4G0CDependencyPreflightTest(unittest.TestCase):
    def setUp(self):
        self.protocol = REPO / "config/icra27/p4_g0c_protocol_v2.json"
        self.registry = REPO / "config/icra27/p4_threshold_registry_v2.json"
        self.fixture = REPO / "config/icra27/p4_g0c_live_fixture_v1.json"
        self.bundle = RUNNER.load_bundle(
            self.protocol, self.registry, self.fixture
        )
        self.manifest_path = (
            REPO / "config/icra27/p4_g0c_runtime_dependencies_v2.json"
        )
        self.manifest = RUNNER.load_runtime_dependency_manifest(
            self.manifest_path,
            self.bundle.protocol["runtime_dependency_manifest"]["sha256"],
        )

    @staticmethod
    def _write_executable(path):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("#!/bin/sh\nexit 0\n")
        path.chmod(path.stat().st_mode | stat.S_IXUSR)

    @staticmethod
    def _write_shared_object(path):
        path.parent.mkdir(parents=True, exist_ok=True)
        source = Path("/lib") / f"{os.uname().machine}-linux-gnu/libdl.so.2"
        path.write_bytes(source.read_bytes())

    @staticmethod
    def _write_elf_executable(path):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(Path("/bin/true").read_bytes())
        path.chmod(path.stat().st_mode | stat.S_IXUSR)

    @staticmethod
    def _config_source(package, relative):
        roots = {
            "iap": REPO,
            "local_sensing": REPO / "src/uav_simulator/local_sensing",
            "so3_control": REPO / "src/uav_simulator/so3_control",
        }
        return roots[package] / relative

    def _complete_prefix(self, root):
        prefix = root / "fresh_prefix"
        for package in self.manifest["packages"]:
            marker = (
                prefix / "share/ament_index/resource_index/packages"
                / package["name"]
            )
            marker.parent.mkdir(parents=True, exist_ok=True)
            marker.write_text("")
            for executable in package["executables"]:
                self._write_executable(
                    prefix / "lib" / package["name"] / executable
                )
            for config in package["config_files"]:
                path = prefix / "share" / package["name"] / config
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(
                    self._config_source(package["name"], config).read_bytes()
                )
        launch_contract = self.manifest["launch_contract"]
        launch_path = (
            prefix / "share" / launch_contract["package"]
            / launch_contract["relative_path"]
        )
        launch_path.parent.mkdir(parents=True, exist_ok=True)
        launch_source = REPO / launch_contract["source_path"]
        if self.manifest["schema_version"] == RUNNER.DEPENDENCY_SCHEMA_V2:
            launch_bytes = subprocess.check_output(
                [
                    "git", "show",
                    "cddfa2197bb1d4ee8f68fd105596174c3db53c45:"
                    + launch_contract["source_path"],
                ],
                cwd=REPO,
            )
        else:
            launch_bytes = launch_source.read_bytes()
        launch_path.write_bytes(launch_bytes)
        for component in self.manifest["components"]:
            resource = (
                prefix / "share/ament_index/resource_index/rclcpp_components"
                / component["package"]
            )
            resource.parent.mkdir(parents=True, exist_ok=True)
            resource.write_text(
                f"{component['plugin']};{component['library']}\n"
            )
            library = prefix / component["library"]
            self._write_shared_object(library)
        for library in self.manifest["runtime_libraries"]:
            self._write_shared_object(prefix / library["relative_path"])
        return prefix

    @staticmethod
    def _environment(prefixes, allowed=None):
        values = [str(Path(path).resolve()) for path in prefixes]
        allowed_values = values if allowed is None else [
            str(Path(path).resolve()) for path in allowed
        ]
        return {
            "AMENT_PREFIX_PATH": os.pathsep.join(values),
            "P4_G0C_ALLOWED_PREFIXES": os.pathsep.join(allowed_values),
        }

    def test_icra046_reproduction_and_green_stop_before_gpu_and_launch(self):
        legacy_calls = []

        def legacy_runner(show_args_ok):
            if show_args_ok:
                legacy_calls.extend(["gpu", "launch"])

        legacy_runner(show_args_ok=True)
        self.assertEqual(legacy_calls, ["gpu", "launch"])

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            prefix = self._complete_prefix(root)
            so3_marker = (
                prefix / "share/ament_index/resource_index/packages/so3_control"
            )
            self.assertTrue(
                (prefix / "share/ament_index/resource_index/packages/iap").is_file()
            )
            self.assertTrue(
                (prefix / "share/ament_index/resource_index/packages/ego_planner").is_file()
            )
            so3_marker.unlink()
            calls = []
            result = RUNNER.run(
                self.bundle,
                root / "replacement-runs",
                dependency_environment=self._environment([prefix]),
                gpu_preflight=lambda _: calls.append("gpu"),
                launch_executor=lambda *_: calls.append("launch"),
            )
        self.assertEqual(calls, [])
        self.assertEqual(result["runner_state"], "FAILED")
        self.assertEqual(
            result["failure_reason"], "DEPENDENCY_PACKAGE_MISSING:so3_control"
        )
        self.assertEqual(result["gpu_preflight_invocations"], 0)
        self.assertEqual(result["launch_invocations"], 0)

    def test_manifest_declares_the_exact_active_launch_surface(self):
        packages = {item["name"]: item for item in self.manifest["packages"]}
        self.assertTrue(RUNNER.EXPECTED_ACTIVE_PACKAGES.issubset(packages))
        self.assertTrue(RUNNER.EXPECTED_BUILD_CLOSURE.issubset(packages))
        self.assertEqual(packages["ego_planner"]["executables"], [
            "ego_planner_node", "traj_server"
        ])
        self.assertEqual(packages["so3_control"]["config_files"], [
            "config/corrections_hummingbird.yaml",
            "config/gains_hummingbird.yaml",
        ])
        self.assertEqual(packages["iap"]["config_files"], [
            "config/config_odometry_gpu.json",
            "config/gnss_sim/demo7_skymask_nlos.yaml",
            "config/sim_demo11/config.json",
            "config/sim_demo11/config_gnss.json",
            "config/sim_demo11/config_ros.json",
            "config/sim_ego/config_global_mapping_gpu.json",
            "config/sim_ego/config_logging.json",
            "config/sim_ego/config_sensors.json",
            "config/sim_ego/config_sub_mapping_gpu.json",
            "config/sim_ego/config_viewer.json",
            "config/sim_ego/fastdds_udp_only.xml",
        ])
        self.assertEqual(
            {item["relative_path"] for item in self.manifest["runtime_libraries"]},
            RUNNER.EXPECTED_RUNTIME_LIBRARIES,
        )
        self.assertEqual(self.manifest["components"], [{
            "library": "lib/libso3_control_component.so",
            "package": "so3_control",
            "plugin": "SO3ControlComponent",
        }])
        self.assertEqual(
            self.manifest["inactive_packages"],
            ["rosbag2_transport", "rviz2"],
        )

    def test_v3_complete_closure_has_distinct_dependency_result_schema(self):
        self.protocol = REPO / "config/icra27/p4_g0c_protocol_v3.json"
        self.registry = REPO / "config/icra27/p4_threshold_registry_v3.json"
        self.bundle = RUNNER.load_bundle(
            self.protocol,
            self.registry,
            self.fixture,
            expected_protocol_schema=RUNNER.HARDENED_PROTOCOL_SCHEMA,
        )
        self.manifest_path = (
            REPO / "config/icra27/p4_g0c_runtime_dependencies_v3.json"
        )
        self.manifest = RUNNER.load_runtime_dependency_manifest(
            self.manifest_path,
            self.bundle.protocol["runtime_dependency_manifest"]["sha256"],
            expected_schema=RUNNER.DEPENDENCY_SCHEMA_V3,
            expected_experiment="p4_g0c_metrics_calibration_v3",
        )
        with tempfile.TemporaryDirectory() as tmp:
            prefix = self._complete_prefix(Path(tmp))
            result = RUNNER.validate_runtime_dependencies(
                self.bundle,
                self.manifest_path,
                self._environment([prefix]),
            )
        self.assertTrue(result["dependency_ready"])
        self.assertEqual(
            result["schema_version"],
            "p4_g0c_dependency_preflight_result_v3",
        )
        self.assertEqual(
            result["manifest_path"], str(self.manifest_path.resolve())
        )
        self.assertEqual(
            result["manifest_sha256"],
            "ff7c66f182296a1f057acafee5306d7d81aa49be8a40c14acd8e832d98cb5fc6",
        )
        self.assertEqual(result["validated_prefixes"], [str(prefix.resolve())])
        self.assertEqual(
            {
                "packages": result["package_count"],
                "executables": result["executable_count"],
                "components": result["component_count"],
                "configs": result["config_count"],
                "runtime_libraries": result["runtime_library_count"],
            },
            {
                "packages": 18,
                "executables": 13,
                "components": 1,
                "configs": 14,
                "runtime_libraries": 6,
            },
        )

    def test_manifest_path_is_stable_after_all_artifact_validation(self):
        protocol = REPO / "config/icra27/p4_g0c_protocol_v3.json"
        registry = REPO / "config/icra27/p4_threshold_registry_v3.json"
        bundle = RUNNER.load_bundle(
            protocol,
            registry,
            self.fixture,
            expected_protocol_schema=RUNNER.HARDENED_PROTOCOL_SCHEMA,
        )
        source_manifest = (
            REPO / "config/icra27/p4_g0c_runtime_dependencies_v3.json"
        )
        payload = json.loads(source_manifest.read_text())
        iap_package = next(
            package for package in payload["packages"]
            if package["name"] == "iap"
        )
        iap_package["config_files"].reverse()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            requested_manifest = root / "reordered-config-manifest.json"
            requested_manifest.write_bytes(RUNNER.canonical_bytes(payload))
            bundle.protocol["runtime_dependency_manifest"]["sha256"] = (
                RUNNER.sha256_file(requested_manifest)
            )
            self.manifest = payload
            prefix = self._complete_prefix(root)
            last_runtime_library = payload["runtime_libraries"][-1]
            (prefix / last_runtime_library["relative_path"]).write_bytes(
                Path("/lib")
                .joinpath(f"{os.uname().machine}-linux-gnu/libm.so.6")
                .read_bytes()
            )
            component_library = prefix / payload["components"][-1]["library"]
            component_library.write_bytes(
                Path("/lib")
                .joinpath(f"{os.uname().machine}-linux-gnu/libz.so.1")
                .read_bytes()
            )
            result = RUNNER.validate_runtime_dependencies(
                bundle,
                requested_manifest,
                self._environment([prefix]),
            )
        self.assertTrue(result["dependency_ready"])
        self.assertEqual(
            result["manifest_path"], str(requested_manifest.resolve())
        )
        self.assertEqual(result["config_count"], 14)
        self.assertEqual(result["runtime_library_count"], 6)
        self.assertEqual(result["component_count"], 1)

    def test_every_declared_package_executable_component_and_config_is_required(self):
        cases = []
        for package in self.manifest["packages"]:
            cases.append((
                f"package:{package['name']}",
                lambda prefix, package=package: (
                    prefix / "share/ament_index/resource_index/packages"
                    / package["name"]
                ),
                f"DEPENDENCY_PACKAGE_MISSING:{package['name']}",
            ))
            for executable in package["executables"]:
                cases.append((
                    f"executable:{package['name']}:{executable}",
                    lambda prefix, package=package, executable=executable: (
                        prefix / "lib" / package["name"] / executable
                    ),
                    f"DEPENDENCY_EXECUTABLE_MISSING:{package['name']}:{executable}",
                ))
            for config in package["config_files"]:
                cases.append((
                    f"config:{package['name']}:{config}",
                    lambda prefix, package=package, config=config: (
                        prefix / "share" / package["name"] / config
                    ),
                    f"DEPENDENCY_CONFIG_MISSING:{package['name']}:{config}",
                ))
        for component in self.manifest["components"]:
            cases.append((
                f"component:{component['package']}:{component['plugin']}",
                lambda prefix, component=component: (
                    prefix
                    / "share/ament_index/resource_index/rclcpp_components"
                    / component["package"]
                ),
                f"DEPENDENCY_COMPONENT_MISSING:{component['package']}:{component['plugin']}",
            ))
        for library in self.manifest["runtime_libraries"]:
            cases.append((
                f"runtime-library:{library['relative_path']}",
                lambda prefix, library=library: prefix / library["relative_path"],
                "DEPENDENCY_RUNTIME_LIBRARY_MISSING:"
                f"{library['package']}:{library['relative_path']}",
            ))

        for label, target, expected in cases:
            with self.subTest(case=label), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                prefix = self._complete_prefix(root)
                target(prefix).unlink()
                result = RUNNER.validate_runtime_dependencies(
                    self.bundle,
                    self.manifest_path,
                    self._environment([prefix]),
                )
                self.assertFalse(result["dependency_ready"])
                self.assertEqual(result["failure_reason"], expected)

    def test_complete_closure_passes_standalone_and_full_ordering(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            prefix = self._complete_prefix(root)
            environment = self._environment([prefix])
            standalone_calls = []
            standalone = RUNNER.run(
                self.bundle,
                root / "standalone-fresh-root",
                dependency_preflight_only=True,
                dependency_environment=environment,
                gpu_preflight=lambda _: standalone_calls.append("gpu"),
                launch_executor=lambda *_: standalone_calls.append("launch"),
            )
            full_calls = []
            full = RUNNER.run(
                self.bundle,
                root / "full-fresh-root",
                dependency_environment=environment,
                gpu_preflight=lambda _: (
                    full_calls.append("gpu") or {"gpu_ready": True}
                ),
                launch_executor=lambda *_: (
                    full_calls.append("launch") or (
                        1, {"required_processes_ok": False}
                    )
                ),
            )
        self.assertEqual(standalone_calls, [])
        self.assertEqual(standalone["runner_state"], "DEPENDENCY_PREFLIGHT_PASS")
        self.assertTrue(standalone["dependency_preflight"]["dependency_ready"])
        self.assertEqual(full_calls, ["gpu", "launch"])
        self.assertTrue(full["dependency_preflight"]["dependency_ready"])
        self.assertEqual(full["gpu_preflight_invocations"], 1)
        self.assertEqual(full["launch_invocations"], 1)

    def test_undeclared_duplicate_and_manifest_drift_reject(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            prefix = self._complete_prefix(root)
            undeclared = RUNNER.validate_runtime_dependencies(
                self.bundle,
                self.manifest_path,
                self._environment([prefix], allowed=[]),
            )
            self.assertEqual(
                undeclared["failure_reason"], "DEPENDENCY_PREFIX_UNDECLARED"
            )
            historical = RUNNER.validate_runtime_dependencies(
                self.bundle,
                self.manifest_path,
                self._environment([Path("/home/dev/ws_iap/install")]),
            )
            self.assertEqual(
                historical["failure_reason"], "DEPENDENCY_PREFIX_HISTORICAL"
            )

            alias = root / "prefix-alias"
            alias.symlink_to(prefix, target_is_directory=True)
            alias_result = RUNNER.validate_runtime_dependencies(
                self.bundle,
                self.manifest_path,
                {
                    "AMENT_PREFIX_PATH": str(alias),
                    "P4_G0C_ALLOWED_PREFIXES": str(alias),
                },
            )
            self.assertEqual(
                alias_result["failure_reason"],
                "DEPENDENCY_PREFIX_SYMLINK_OR_ALIAS",
            )

            duplicate = root / "duplicate_prefix"
            marker = (
                duplicate / "share/ament_index/resource_index/packages/iap"
            )
            marker.parent.mkdir(parents=True)
            marker.write_text("")
            duplicate_result = RUNNER.validate_runtime_dependencies(
                self.bundle,
                self.manifest_path,
                self._environment([prefix, duplicate]),
            )
            self.assertEqual(
                duplicate_result["failure_reason"],
                "DEPENDENCY_PACKAGE_DUPLICATE:iap",
            )

            drifted = root / "drifted.json"
            payload = json.loads(self.manifest_path.read_text())
            payload["inactive_packages"].append("unexpected")
            drifted.write_bytes(RUNNER.canonical_bytes(payload))
            drift_result = RUNNER.validate_runtime_dependencies(
                self.bundle,
                drifted,
                self._environment([prefix]),
            )
            self.assertEqual(
                drift_result["failure_reason"],
                "DEPENDENCY_MANIFEST_HASH_MISMATCH",
            )

    def test_runtime_library_symlink_escape_remains_fail_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            prefix = self._complete_prefix(root)
            library = self.manifest["runtime_libraries"][0]
            library_path = prefix / library["relative_path"]
            escaped_library = root / "escaped-library.so"
            escaped_library.write_bytes(library_path.read_bytes())
            library_path.unlink()
            library_path.symlink_to(escaped_library)
            result = RUNNER.validate_runtime_dependencies(
                self.bundle,
                self.manifest_path,
                self._environment([prefix]),
            )
        self.assertEqual(
            result["failure_reason"],
            "DEPENDENCY_RUNTIME_LIBRARY_MISSING:"
            f"{library['package']}:{library['relative_path']}",
        )

    def test_manifest_symlink_loop_returns_typed_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            first = root / "manifest-loop-a.json"
            second = root / "manifest-loop-b.json"
            first.symlink_to(second)
            second.symlink_to(first)
            result = RUNNER.validate_runtime_dependencies(
                self.bundle,
                first,
                {},
            )
        self.assertFalse(result["dependency_ready"])
        self.assertTrue(
            result["failure_reason"].startswith("DEPENDENCY_MANIFEST_INVALID:")
        )

    def test_component_library_registration_and_launch_hash_drift_reject(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            prefix = self._complete_prefix(root)
            environment = self._environment([prefix])
            component = self.manifest["components"][0]
            resource = (
                prefix / "share/ament_index/resource_index/rclcpp_components"
                / component["package"]
            )
            resource.write_text("WrongPlugin;lib/libwrong.so\n")
            mismatch = RUNNER.validate_runtime_dependencies(
                self.bundle, self.manifest_path, environment
            )
            self.assertEqual(
                mismatch["failure_reason"],
                "DEPENDENCY_COMPONENT_MISMATCH:so3_control:SO3ControlComponent",
            )

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            prefix = self._complete_prefix(root)
            component = self.manifest["components"][0]
            (prefix / component["library"]).unlink()
            missing_library = RUNNER.validate_runtime_dependencies(
                self.bundle, self.manifest_path, self._environment([prefix])
            )
            self.assertEqual(
                missing_library["failure_reason"],
                "DEPENDENCY_COMPONENT_LIBRARY_MISSING:so3_control:SO3ControlComponent",
            )

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            prefix = self._complete_prefix(root)
            component = self.manifest["components"][0]
            (prefix / component["library"]).write_bytes(b"not an ELF object\n")
            invalid_library = RUNNER.validate_runtime_dependencies(
                self.bundle, self.manifest_path, self._environment([prefix])
            )
            self.assertEqual(
                invalid_library["failure_reason"],
                "DEPENDENCY_COMPONENT_LIBRARY_INVALID:so3_control:SO3ControlComponent",
            )

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            prefix = self._complete_prefix(root)
            launch = self.manifest["launch_contract"]
            path = (
                prefix / "share" / launch["package"]
                / launch["relative_path"]
            )
            path.write_text("drift\n")
            launch_drift = RUNNER.validate_runtime_dependencies(
                self.bundle, self.manifest_path, self._environment([prefix])
            )
            self.assertEqual(
                launch_drift["failure_reason"],
                "DEPENDENCY_LAUNCH_CONTRACT_MISMATCH",
            )

    def test_config_executable_and_runtime_library_content_drift_reject(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            prefix = self._complete_prefix(root)
            relative = "config/sim_ego/config_viewer.json"
            (prefix / "share/iap" / relative).write_text("{}\n")
            result = RUNNER.validate_runtime_dependencies(
                self.bundle, self.manifest_path, self._environment([prefix])
            )
            self.assertEqual(
                result["failure_reason"],
                f"DEPENDENCY_CONFIG_HASH_MISMATCH:iap:{relative}",
            )

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            prefix = self._complete_prefix(root)
            executable = prefix / "lib/iap/iap_rosnode"
            executable.write_bytes(b"not loadable\n")
            executable.chmod(executable.stat().st_mode | stat.S_IXUSR)
            result = RUNNER.validate_runtime_dependencies(
                self.bundle, self.manifest_path, self._environment([prefix])
            )
            self.assertEqual(
                result["failure_reason"],
                "DEPENDENCY_EXECUTABLE_INVALID:iap:iap_rosnode",
            )

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            prefix = self._complete_prefix(root)
            executable = prefix / "lib/iap/iap_rosnode"
            self._write_elf_executable(executable)
            unresolved = executable.read_bytes().replace(
                b"libc.so.6", b"libQ.so.6"
            )
            self.assertNotEqual(unresolved, executable.read_bytes())
            executable.write_bytes(unresolved)
            result = RUNNER.validate_runtime_dependencies(
                self.bundle, self.manifest_path, self._environment([prefix])
            )
            self.assertEqual(
                result["failure_reason"],
                "DEPENDENCY_EXECUTABLE_INVALID:iap:iap_rosnode",
            )

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            prefix = self._complete_prefix(root)
            library = self.manifest["runtime_libraries"][0]
            path = prefix / library["relative_path"]
            payload = bytearray(path.read_bytes())
            byte_order = "<" if payload[5] == 1 else ">"
            current_machine = struct.unpack(f"{byte_order}H", payload[18:20])[0]
            wrong_machine = 183 if current_machine != 183 else 62
            payload[18:20] = struct.pack(f"{byte_order}H", wrong_machine)
            path.write_bytes(payload)
            result = RUNNER.validate_runtime_dependencies(
                self.bundle, self.manifest_path, self._environment([prefix])
            )
            self.assertEqual(
                result["failure_reason"],
                "DEPENDENCY_RUNTIME_LIBRARY_INVALID:"
                f"{library['package']}:{library['relative_path']}",
            )

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            prefix = self._complete_prefix(root)
            library = self.manifest["runtime_libraries"][0]
            path = prefix / library["relative_path"]
            unresolved = path.read_bytes().replace(b"libc.so.6", b"libQ.so.6")
            self.assertNotEqual(unresolved, path.read_bytes())
            path.write_bytes(unresolved)
            result = RUNNER.validate_runtime_dependencies(
                self.bundle, self.manifest_path, self._environment([prefix])
            )
            self.assertEqual(
                result["failure_reason"],
                "DEPENDENCY_RUNTIME_LIBRARY_INVALID:"
                f"{library['package']}:{library['relative_path']}",
            )

    def test_dependency_only_root_cannot_be_reused_for_full_execution(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            prefix = self._complete_prefix(root)
            runs = root / "consumed-root"
            environment = self._environment([prefix])
            first = RUNNER.run(
                self.bundle,
                runs,
                dependency_preflight_only=True,
                dependency_environment=environment,
            )
            self.assertEqual(first["runner_state"], "DEPENDENCY_PREFLIGHT_PASS")
            with self.assertRaisesRegex(RUNNER.RunnerError, "existing runner state"):
                RUNNER.run(
                    self.bundle,
                    runs,
                    dependency_environment=environment,
                    gpu_preflight=lambda _: self.fail("GPU must not run"),
                )


if __name__ == "__main__":
    unittest.main()
