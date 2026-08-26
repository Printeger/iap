import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest import mock


REPO = Path(__file__).resolve().parents[1]
RUNNER_PATH = REPO / "scripts/dev_planner/run_icra_p0_p5_qualification.py"
SPEC = importlib.util.spec_from_file_location("icra_p0_p5_live_runner", RUNNER_PATH)
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)
EMPTY_DEFAULT_KEYS = {
    "p1.debug_csv_path", "p2.debug_csv_path", "p3.debug_csv_path",
    "p4.debug_csv_path", "p4.profile_trace_path", "p4.g0c.protocol_path",
    "p4.g0c.protocol_sha256", "p4.g0c.registry_path",
    "p4.g0c.registry_sha256", "p4.g0c.fixture_path",
    "p4.g0c.fixture_sha256", "p4.g0c.run_id",
    "p4.g0c.run_manifest_path", "p4.g0c.csv_path", "p4.g0c.child_home",
    "p4.g0c.child_ros_home", "p4.g0c.child_ros_log_dir",
    "p4.g0c.child_tmpdir", "p4.g0c.child_xdg_runtime_dir",
}


class IcraP0P5LiveRunnerTest(unittest.TestCase):
    def setUp(self):
        self.contract_path = REPO / "config/icra27/icra_p0_p5_qualification_v1.json"
        self.contract = RUNNER.QUALIFICATION.load_contract(self.contract_path)

    def test_product_install_boundary_excludes_all_python_caches(self):
        cmake = (REPO / "CMakeLists.txt").read_text()
        for directory in ("launch", "config"):
            self.assertRegex(
                cmake,
                rf"install\(DIRECTORY {directory} DESTINATION share/iap\s+"
                r"PATTERN \"__pycache__\" EXCLUDE\s+"
                r"PATTERN \"\*\.pyc\" EXCLUDE\s+"
                r"PATTERN \"\*\.pyo\" EXCLUDE\s+"
                r"PATTERN \"\*\.pyd\" EXCLUDE\s*\)",
            )

    def test_cache_repair_removes_every_cache_and_preserves_non_cache_files(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            root = Path(tmp)
            base, overlay, source = root / "base", root / "overlay", root / "source"
            aliases = {
                "share/iap/launch/test_planner.launch.py": (
                    "launch/test_planner.launch.py"
                ),
                "share/iap/launch/icra_p0_p5_qualification.py": (
                    "launch/icra_p0_p5_qualification.py"
                ),
            }
            for relative, source_relative in aliases.items():
                (base / relative).parent.mkdir(parents=True, exist_ok=True)
                (overlay / relative).parent.mkdir(parents=True, exist_ok=True)
                (source / source_relative).parent.mkdir(parents=True, exist_ok=True)
                (base / relative).write_text("frozen\n")
                (overlay / relative).write_text("current\n")
                (source / source_relative).write_text("current\n")
            for parent in (base, overlay):
                (parent / "lib").mkdir(parents=True)
                (parent / "lib/libiap.so").write_bytes(b"binary")
            cache_rows = {
                "share/iap/launch/__pycache__/icra_p0_p5_qualification.cpython-312.pyc": b"a",
                "share/iap/launch/__pycache__/test_planner.launch.cpython-312.pyc": b"b",
                "share/iap/config/nested/__pycache__/unrelated.cpython-312.pyc": b"c",
                "share/iap/config/legacy.pyo": b"d",
                "share/iap/config/native.pyd": b"e",
            }
            for relative, data in cache_rows.items():
                path = overlay / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(data)
            before = RUNNER.tree_byte_inventory(overlay)
            journal = []
            proof = RUNNER.repair_overlay_cache_boundary(
                overlay, base, source, aliases, before,
                pre_mutation_recorder=journal.append,
            )
            self.assertEqual(journal[0]["phase_status"], "READY")
            self.assertEqual(
                set(proof["removed_cache_files"]), set(cache_rows)
            )
            self.assertFalse(any(
                RUNNER.is_python_cache_path(path.relative_to(overlay))
                for path in overlay.rglob("*")
            ))
            self.assertEqual((overlay / "lib/libiap.so").read_bytes(), b"binary")
            self.assertTrue(proof["binary_library_bytes_equal"])
            self.assertTrue(proof["non_cache_file_set_complete"])

    def test_cache_repair_rejects_missing_or_extra_non_cache_file(self):
        for mutation, error in (
            ("missing", "failed_overlay_pre_repair_inventory_mismatch"),
            ("extra", "failed_overlay_pre_repair_inventory_mismatch"),
        ):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory(
                dir=REPO / "results/icra27"
            ) as tmp:
                root = Path(tmp)
                base, overlay, source = (
                    root / "base", root / "overlay", root / "source"
                )
                for parent in (base, overlay):
                    (parent / "lib").mkdir(parents=True)
                    (parent / "lib/libiap.so").write_bytes(b"binary")
                cache = overlay / "share/iap/launch/__pycache__/stale.pyc"
                cache.parent.mkdir(parents=True)
                cache.write_bytes(b"cache")
                expected = RUNNER.tree_byte_inventory(overlay)
                if mutation == "missing":
                    (overlay / "lib/libiap.so").unlink()
                else:
                    (overlay / "extra.txt").write_text("extra")
                with self.assertRaisesRegex(RUNNER.LiveRunnerError, error):
                    RUNNER.repair_overlay_cache_boundary(
                        overlay, base, source, {}, expected
                    )

    def test_cache_repair_journals_initially_missing_base_file_before_unlink(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            root = Path(tmp)
            base, overlay, source = root / "base", root / "overlay", root / "source"
            for parent in (base, overlay):
                (parent / "lib").mkdir(parents=True)
                (parent / "lib/libiap.so").write_bytes(b"binary")
            (base / "include").mkdir()
            (base / "include/missing.h").write_text("required\n")
            cache = overlay / "share/iap/launch/__pycache__/stale.pyc"
            cache.parent.mkdir(parents=True)
            cache.write_bytes(b"cache")
            expected = RUNNER.tree_byte_inventory(overlay)
            journal = []
            with self.assertRaisesRegex(
                RUNNER.LiveRunnerError,
                "full_base_file_set_missing:include/missing.h",
            ):
                RUNNER.repair_overlay_cache_boundary(
                    overlay, base, source, {}, expected,
                    pre_mutation_recorder=journal.append,
                )
            self.assertTrue(cache.is_file())
            self.assertEqual(len(journal), 1)
            self.assertEqual(journal[0]["phase_status"], "BLOCKED")
            self.assertEqual(
                journal[0]["full_file_set_preflight"]["missing_base_files"],
                ["include/missing.h"],
            )
            self.assertEqual(
                journal[0]["cache_inventory"]["files"][0]["path"],
                "share/iap/launch/__pycache__/stale.pyc",
            )

    def test_repair_phase_command_never_predeclares_outer_exit(self):
        binding = RUNNER.repair_command_binding()
        self.assertNotIn("exit_code", binding)
        self.assertEqual(binding["outer_exit_recorded_externally"], True)

    def test_command_local_git_trust_works_with_real_isolated_home(self):
        isolated_home = (
            REPO / "results/icra27/icra070/live_environment/home"
        )
        self.assertTrue(isolated_home.is_dir())
        self.assertFalse((isolated_home / ".gitconfig").exists())
        environment = {
            key: value for key, value in RUNNER.os.environ.items()
            if not key.startswith("GIT_CONFIG_")
        }
        environment["HOME"] = str(isolated_home)
        untrusted = RUNNER.subprocess.run(
            ["git", "-C", str(REPO), "status", "--porcelain"],
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(untrusted.returncode, 128)
        self.assertIn("dubious ownership", untrusted.stderr)
        mismatched = RUNNER.subprocess.run(
            [
                "git", "-c", "safe.directory=/tmp/not-the-iap-repository",
                "-C", str(REPO), "status", "--porcelain",
            ],
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(mismatched.returncode, 128)
        self.assertIn("dubious ownership", mismatched.stderr)
        result = RUNNER.trusted_git(
            ["rev-parse", "--show-toplevel"],
            repository=REPO,
            environment=environment,
        )
        self.assertEqual(result.stdout.strip(), str(REPO.resolve()))
        self.assertEqual(result.returncode, 0)
        self.assertFalse((isolated_home / ".gitconfig").exists())

    def test_command_local_git_trust_rejects_aliased_repository(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            alias = Path(tmp) / "repository_alias"
            alias.symlink_to(REPO, target_is_directory=True)
            with self.assertRaisesRegex(
                RUNNER.LiveRunnerError, "git_repository_not_canonical"
            ):
                RUNNER.trusted_git(["status"], repository=alias)
            for invalid in (Path("."), Path(tmp)):
                with self.assertRaisesRegex(
                    RUNNER.LiveRunnerError, "git_repository_not_canonical"
                ):
                    RUNNER.trusted_git(["status"], repository=invalid)

    def test_complete_replacement_preflight_rejects_dirty_tracked_status(self):
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "tracked_worktree_not_clean"
        ):
            RUNNER.validate_trusted_worktree(
                "a" * 40, "a" * 40, " M tracked.py"
            )

    def test_complete_replacement_rejects_source_cache_mutation(self):
        changed = {
            "directories": ["__pycache__"],
            "files": [dict(RUNNER.EXPECTED_SOURCE_CACHE_INVENTORY["files"][0])],
        }
        changed["files"][0]["sha256"] = "0" * 64
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "source_cache_inventory_mismatch"
        ):
            RUNNER.validate_source_cache_inventory(changed)

    def test_complete_replacement_rejects_preexisting_v3_evidence(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            existing = Path(tmp) / "icra070_overlay_manifest_v3.json"
            existing.write_text("{}\n")
            with self.assertRaisesRegex(
                RUNNER.LiveRunnerError, "overlay_manifest_v3_already_exists"
            ):
                RUNNER.require_complete_replacement_outputs_absent({
                    "overlay_manifest_v3": existing,
                })

    def test_complete_overlay_copies_full_non_cache_set_modes_and_aliases(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            root = Path(tmp)
            base, replacement, source = (
                root / "base", root / "install_v2", root / "source"
            )
            binary = base / "lib/libiap.so"
            binary.parent.mkdir(parents=True)
            binary.write_bytes(b"binary")
            binary.chmod(0o755)
            relative = "share/iap/launch/test_planner.launch.py"
            source_relative = "launch/test_planner.launch.py"
            (base / relative).parent.mkdir(parents=True)
            (base / relative).write_text("old\n")
            (source / source_relative).parent.mkdir(parents=True)
            (source / source_relative).write_text("current\n")
            cache = base / "share/iap/launch/__pycache__/stale.pyc"
            cache.parent.mkdir(parents=True)
            cache.write_bytes(b"cache")

            proof = RUNNER.construct_complete_overlay(
                base, replacement, source,
                {relative: source_relative},
            )

            self.assertEqual((replacement / relative).read_text(), "current\n")
            self.assertEqual((replacement / "lib/libiap.so").read_bytes(), b"binary")
            self.assertEqual(
                (replacement / "lib/libiap.so").stat().st_mode & 0o777,
                0o755,
            )
            self.assertNotEqual(
                (replacement / "lib/libiap.so").stat().st_ino,
                binary.stat().st_ino,
            )
            self.assertFalse((replacement / cache.relative_to(base)).exists())
            self.assertTrue(proof["full_file_set_complete"])
            self.assertEqual(proof["cache_file_count"], 0)
            self.assertEqual(proof["symlink_count"], 0)
            self.assertEqual(proof["hard_link_count"], 0)

    def test_complete_overlay_audit_rejects_post_copy_file_and_mode_drift(self):
        for mutation, error in (
            ("missing", "replacement_file_set_missing"),
            ("extra", "replacement_file_set_extra"),
            ("mode", "replacement_base_bytes_or_mode_drift"),
            ("binary", "replacement_base_bytes_or_mode_drift"),
            ("cache", "replacement_file_set_extra"),
            ("symlink", "replacement_file_set_extra"),
            ("hard_link", "replacement_contains_hard_link"),
        ):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory(
                dir=REPO / "results/icra27"
            ) as tmp:
                root = Path(tmp)
                base, replacement, source = (
                    root / "base", root / "install_v2", root / "source"
                )
                (base / "lib").mkdir(parents=True)
                (base / "lib/libiap.so").write_bytes(b"binary")
                source.mkdir()
                RUNNER.construct_complete_overlay(
                    base, replacement, source, {}
                )
                installed = replacement / "lib/libiap.so"
                if mutation == "missing":
                    installed.unlink()
                elif mutation == "extra":
                    (replacement / "extra.txt").write_text("extra")
                elif mutation == "mode":
                    installed.chmod(0o755)
                elif mutation == "binary":
                    installed.write_bytes(b"changed")
                elif mutation == "cache":
                    cache = replacement / "lib/__pycache__/stale.pyc"
                    cache.parent.mkdir()
                    cache.write_bytes(b"cache")
                elif mutation == "symlink":
                    (replacement / "link").symlink_to("lib/libiap.so")
                else:
                    installed.unlink()
                    RUNNER.os.link(base / "lib/libiap.so", installed)
                with self.assertRaisesRegex(RUNNER.LiveRunnerError, error):
                    RUNNER.audit_complete_overlay(
                        base, replacement, source, {}
                    )

    def test_complete_overlay_cli_requires_bound_reviewed_commit(self):
        with mock.patch.object(
            RUNNER.sys,
            "argv",
            [
                "run_icra_p0_p5_qualification.py",
                "--prepare-complete-overlay",
                "--expected-replacement-commit",
                "not-a-commit",
            ],
        ):
            with self.assertRaisesRegex(
                RUNNER.LiveRunnerError,
                "expected_replacement_commit_malformed",
            ):
                RUNNER.main()

    def test_complete_overlay_v3_payloads_bind_new_root_and_no_claim(self):
        commit = "a" * 40
        proof = {
            "full_file_set_complete": True,
            "binary_library_bytes_equal": True,
        }
        manifest = RUNNER.build_complete_overlay_manifest_v3(
            commit, proof, "b" * 64, {"dependency_ready": True},
            {"alias": {"sha256": "c" * 64}},
            {"iap_prefix": str(RUNNER.COMPLETE_OVERLAY_INSTALL_ROOT)},
            {"exit_code": 0},
        )
        adoption = RUNNER.build_complete_adoption_payload_v3(
            commit, [], "d" * 64
        )
        self.assertEqual(
            manifest["install_root"],
            str(RUNNER.COMPLETE_OVERLAY_INSTALL_ROOT),
        )
        self.assertEqual(
            manifest["schema_version"],
            "icra070_complete_overlay_manifest_v3",
        )
        self.assertEqual(adoption["qualification_claim"], False)
        self.assertEqual(adoption["install_root"], manifest["install_root"])

    def test_complete_overlay_rejects_partial_or_second_construction(self):
        for state, error in (
            ("partial", "replacement_partial_root_already_exists"),
            ("second", "replacement_root_already_exists"),
        ):
            with self.subTest(state=state), tempfile.TemporaryDirectory(
                dir=REPO / "results/icra27"
            ) as tmp:
                root = Path(tmp)
                base, replacement, source = (
                    root / "base", root / "install_v2", root / "source"
                )
                (base / "lib").mkdir(parents=True)
                (base / "lib/libiap.so").write_bytes(b"binary")
                source.mkdir()
                if state == "partial":
                    replacement.with_name("install_v2.partial").mkdir()
                else:
                    RUNNER.construct_complete_overlay(
                        base, replacement, source, {}
                    )
                with self.assertRaisesRegex(RUNNER.LiveRunnerError, error):
                    RUNNER.construct_complete_overlay(
                        base, replacement, source, {}
                    )

    def test_complete_overlay_audit_rejects_alias_drift(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            root = Path(tmp)
            base, replacement, source = (
                root / "base", root / "install_v2", root / "source"
            )
            relative = "share/iap/launch/test_planner.launch.py"
            source_relative = "launch/test_planner.launch.py"
            (base / relative).parent.mkdir(parents=True)
            (base / relative).write_text("old\n")
            (source / source_relative).parent.mkdir(parents=True)
            (source / source_relative).write_text("current\n")
            aliases = {relative: source_relative}
            RUNNER.construct_complete_overlay(
                base, replacement, source, aliases
            )
            (replacement / relative).write_text("drift\n")
            with self.assertRaisesRegex(
                RUNNER.LiveRunnerError,
                "replacement_alias_bytes_or_mode_drift",
            ):
                RUNNER.audit_complete_overlay(
                    base, replacement, source, aliases
                )

    def test_complete_overlay_wrong_head_fails_before_root_creation(self):
        environment = RUNNER.expected_live_environment()
        with mock.patch.dict(RUNNER.os.environ, environment, clear=False):
            with self.assertRaisesRegex(
                RUNNER.LiveRunnerError, "replacement_head_mismatch"
            ):
                RUNNER.prepare_complete_overlay("0" * 40)
        self.assertFalse(RUNNER.COMPLETE_OVERLAY_INSTALL_ROOT.exists())
        self.assertFalse(RUNNER.COMPLETE_OVERLAY_PARTIAL_ROOT.exists())

    def test_cache_repair_rejects_binary_and_alias_drift(self):
        for mutation, error in (
            ("binary", "unauthorized_overlay_difference"),
            ("alias", "overlay_alias_source_mismatch"),
        ):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory(
                dir=REPO / "results/icra27"
            ) as tmp:
                root = Path(tmp)
                base, overlay, source = (
                    root / "base", root / "overlay", root / "source"
                )
                relative = "share/iap/launch/test_planner.launch.py"
                source_relative = "launch/test_planner.launch.py"
                for parent in (base, overlay):
                    (parent / "lib").mkdir(parents=True)
                    (parent / relative).parent.mkdir(parents=True)
                    (parent / "lib/libiap.so").write_bytes(b"binary")
                (source / source_relative).parent.mkdir(parents=True)
                (base / relative).write_text("old")
                (overlay / relative).write_text("current")
                (source / source_relative).write_text("current")
                if mutation == "binary":
                    (overlay / "lib/libiap.so").write_bytes(b"changed")
                else:
                    (source / source_relative).write_text("different")
                cache = overlay / "share/iap/launch/__pycache__/stale.pyc"
                cache.parent.mkdir(parents=True)
                cache.write_bytes(b"cache")
                expected = RUNNER.tree_byte_inventory(overlay)
                with self.assertRaisesRegex(RUNNER.LiveRunnerError, error):
                    RUNNER.repair_overlay_cache_boundary(
                        overlay, base, source, {relative: source_relative},
                        expected, pre_mutation_recorder=lambda record: None,
                    )

    def test_cache_repair_rejects_source_cache_mutation(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            root = Path(tmp)
            base, overlay, source = root / "base", root / "overlay", root / "source"
            for parent in (base, overlay):
                (parent / "lib").mkdir(parents=True)
                (parent / "lib/libiap.so").write_bytes(b"binary")
            overlay_cache = overlay / "share/iap/launch/__pycache__/stale.pyc"
            overlay_cache.parent.mkdir(parents=True)
            overlay_cache.write_bytes(b"overlay")
            source_cache = source / "launch/__pycache__/source.pyc"
            source_cache.parent.mkdir(parents=True)
            source_cache.write_bytes(b"before")
            expected_overlay = RUNNER.tree_byte_inventory(overlay)
            expected_source = RUNNER.python_cache_inventory(source)
            source_cache.write_bytes(b"after!")
            with self.assertRaisesRegex(
                RUNNER.LiveRunnerError, "source_cache_inventory_mismatch"
            ):
                RUNNER.repair_overlay_cache_boundary(
                    overlay, base, source, {}, expected_overlay,
                    expected_source_cache=expected_source,
                )

    def test_installed_import_probe_cannot_recreate_python_cache(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            install = Path(tmp) / "install"
            helper = install / "share/iap/launch/helper.py"
            planner = install / "share/iap/launch/planner.py"
            helper.parent.mkdir(parents=True)
            helper.write_text("VALUE = 1\n")
            planner.write_text("VALUE = 2\n")
            proof = RUNNER.run_no_bytecode_import_probe(
                install, [helper, planner],
                {"PATH": "/usr/bin:/bin", "PYTHONDONTWRITEBYTECODE": "1"},
            )
            self.assertEqual(proof["exit_code"], 0)
            self.assertTrue(proof["install_inventory_unchanged"])
            self.assertEqual(RUNNER.python_cache_inventory(install), {
                "files": [], "directories": [],
            })

    def test_original_blocker_is_frozen_and_v2_outputs_are_non_overwriting(self):
        frozen = RUNNER.verify_original_blocker_evidence()
        self.assertEqual(frozen["file_sha256"], {
            "results/icra27/icra070/compact/final_result.json": (
                "ebb95f8f6dc05bf72d7ed3ee3e65af1a8d7279a27e02a6c8efa691e5e374b26b"
            ),
            "results/icra27/icra070/compact/command_ledger.json": (
                "6bc56ea8d2d4a3e61c8a572a380a35da86e6d1035c2791e3848b917db48e6898"
            ),
            "results/icra27/icra070/compact/gnss_dependency_preflight.json": (
                "b35afeb83fb119efacc5ea40ef93ce24360ba65e2f57ec9a002d2f4824426ece"
            ),
            "results/icra27/icra070/compact/overlay_install_command.json": (
                "f9c48b12c4170122f58eb49c6fced267d5d4b19fd8633495b8ef1962b389e16f"
            ),
            "results/icra27/icra070/overlay_install_driver.cmake": (
                "cac3da758800fa42172b43caef6551f39f45fcc350c2a7270a2191167ef45581"
            ),
        })
        self.assertEqual(
            RUNNER.OVERLAY_MANIFEST_PATH.name,
            "icra070_overlay_manifest_v2.json",
        )
        self.assertEqual(
            RUNNER.REPAIR_EVIDENCE_PATH.name, "overlay_cache_repair_v1.json"
        )

    def test_complete_replacement_inputs_freeze_old_terminal_state(self):
        proof = RUNNER.verify_reviewed_pre_replacement_inputs()
        self.assertEqual(
            proof["preexisting_compact_summary"],
            {
                "entry_count": 8,
                "inventory_sha256": (
                    "33ae4cd9c12045d54d507d878915f085"
                    "fdf04836c7da0b21f9235c8c3df203d1"
                ),
            },
        )
        self.assertEqual(proof["retained_icra068"]["entry_count"], 7364)
        self.assertEqual(proof["failed_overlay"]["entry_count"], 474)
        self.assertEqual(
            proof["protected_pdf_sha256"],
            "1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6",
        )

    def test_existing_repair_evidence_rejects_second_invocation(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            existing = Path(tmp) / "repair.json"
            existing.write_text("{}\n")
            with self.assertRaisesRegex(
                RUNNER.LiveRunnerError, "repair_evidence_already_exists"
            ):
                RUNNER.require_repair_outputs_absent([existing])

    def test_python_cache_can_never_be_an_authorized_alias(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            root = Path(tmp)
            overlay, base, source = root / "overlay", root / "base", root / "source"
            for parent in (overlay, base, source):
                parent.mkdir()
            with self.assertRaisesRegex(
                RUNNER.LiveRunnerError, "python_cache_cannot_be_authorized"
            ):
                RUNNER.inventory_overlay_v2(
                    overlay, base, source,
                    {"share/iap/launch/__pycache__/bad.pyc": "launch/bad.pyc"},
                    {},
                )

    def test_fixed_identities_and_full_process_contract(self):
        self.assertEqual(
            RUNNER.LIVE_IDENTITIES,
            (
                ("SAFE_NORMAL", "icra-p0-p5-live-safe-normal-003"),
                ("FINAL_REJECT", "icra-p0-p5-live-final-reject-003"),
                ("RUNTIME_FAIL", "icra-p0-p5-live-runtime-fail-003"),
            ),
        )
        required = set(self.contract["required_processes"])
        self.assertEqual(required, set(RUNNER.REQUIRED_PROCESSES))
        self.assertGreaterEqual(len(required), 15)
        self.assertIn("test_planner_corridor_map_publisher", required)
        self.assertIn("test_planner_so3_control_container", required)
        self.assertIn("test_planner_bag_recorder", required)

    def test_cross_layer_route_remains_exact_full_sensor_target(self):
        self.assertEqual(len(self.contract["required_processes"]), 16)
        self.assertEqual(len(self.contract["required_topics"]), 10)
        self.assertIn(
            "test_planner_gnss_sim_node", self.contract["required_processes"]
        )
        self.assertIn("/sim/drone_0/imu", self.contract["required_topics"])
        self.assertIn("/sim/drone_0/lidar", self.contract["required_topics"])
        self.assertIn("/ublox_driver/range_meas", self.contract["required_topics"])
        for case_id, _ in RUNNER.LIVE_IDENTITIES:
            values = RUNNER.QUALIFICATION.resolve_launch_values(
                self.contract, case_id, {}
            )
            self.assertTrue(values["use_gnss"])
            self.assertTrue(values["use_araim"])
            self.assertEqual(values["integrity_fusion_mode"], "max_pl")
            self.assertEqual(values["p0.predictor.worker_count"], 4)
            self.assertEqual(values["p0.predictor.sigma_grow_m_sqrt_s"], 0.01)
            self.assertEqual(
                values["p0.predictor.sigma_growth_profile"],
                "legacy_iap_rq320_baseline_v1",
            )

    def test_live_config_is_repository_local_and_frozen(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            run_dir = Path(tmp) / "run"
            config = RUNNER.live_config(
                self.contract, "SAFE_NORMAL", "fixed-run", run_dir
            )
        expected = RUNNER.QUALIFICATION.resolve_launch_values(
            self.contract, "SAFE_NORMAL", {}
        )
        for key, value in expected.items():
            if key == "gnss_scenario_file":
                self.assertEqual(
                    config[key],
                    str(RUNNER.COMPLETE_OVERLAY_INSTALL_ROOT / "share/iap" / value),
                )
            else:
                self.assertEqual(config[key], value)
        self.assertTrue(config["record_bag"])
        self.assertFalse(config["start_rviz"])
        self.assertEqual(config["run_duration_s"], 90)
        self.assertEqual(config["gate0.evidence_run_id"], "fixed-run")
        self.assertTrue(config["runtime_root_dir"].startswith(str(REPO)))

    def test_exact_commands_omit_only_registered_empty_defaults(self):
        for case_id, run_id in RUNNER.LIVE_IDENTITIES:
            with self.subTest(case_id=case_id):
                run_dir = REPO / "results/icra27/icra070/live" / run_id
                config = RUNNER.live_config(
                    self.contract, case_id, run_id, run_dir
                )
                command, omitted = RUNNER.render_live_launch_command(
                    config, EMPTY_DEFAULT_KEYS
                )
                self.assertEqual(set(omitted), EMPTY_DEFAULT_KEYS)
                self.assertFalse(any(token.endswith(":=") for token in command))
                rendered_names = [
                    token.split(":=", 1)[0] for token in command[4:]
                ]
                expected_nonempty = {
                    key for key, value in config.items() if value != ""
                }
                self.assertEqual(set(rendered_names), expected_nonempty)
                self.assertEqual(len(rendered_names), len(set(rendered_names)))
                self.assertIn("planner_enable_all_safety:=false", command)
                self.assertIn("p1.lambda_integrity:=0.0", command)
                self.assertIn(
                    "gnss_scenario_file:="
                    + str(
                        RUNNER.COMPLETE_OVERLAY_INSTALL_ROOT
                        / "share/iap/config/gnss_sim/demo7_skymask_nlos.yaml"
                    ),
                    command,
                )
                self.assertIn(
                    "gnss_rinex_nav_file:=" + str(RUNNER.RINEX_EPHEMERIS_PATH),
                    command,
                )
                self.assertIn("gnss_trigger_topic:=/sim/drone_0/lidar", command)
                self.assertIn(
                    "gnss_fallback_to_synthetic_on_rinex_error:=false", command
                )

    def test_command_renderer_rejects_unregistered_empty_and_duplicates(self):
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "unregistered_empty_override"
        ):
            RUNNER.render_live_launch_command(
                [("experiment", "")], EMPTY_DEFAULT_KEYS
            )
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "duplicate_override"
        ):
            RUNNER.render_live_launch_command(
                [("record_bag", True), ("record_bag", False)],
                EMPTY_DEFAULT_KEYS,
            )

    def test_command_renderer_rejects_malformed_names_and_values(self):
        malformed = (
            ("bad name", "value"),
            ("record_bag", None),
            ("record_bag", []),
            ("record_bag", {}),
            ("run_duration_s", math.nan),
            ("run_duration_s", math.inf),
            ("runtime_root_dir", "line\nbreak"),
            ("runtime_root_dir", "nul\x00byte"),
            ("runtime_root_dir", "embedded:=override"),
        )
        for name, value in malformed:
            with self.subTest(name=name, value=repr(value)):
                with self.assertRaisesRegex(
                    RUNNER.LiveRunnerError, "malformed_override_(name|value|token)"
                ):
                    RUNNER.render_live_launch_command(
                        [(name, value)], EMPTY_DEFAULT_KEYS
                    )

    def test_adoption_payload_separates_product_and_runner_provenance(self):
        current_commit = "b" * 40
        changed = [
            "scripts/dev_planner/run_icra_p0_p5_qualification.py",
            "test/test_run_icra_p0_p5_qualification.py",
        ]
        overlay = {
            "schema_version": "icra070_isolated_overlay_manifest_v2",
            "runner": {"git_commit": current_commit},
            "overlay_inventory": {"authorized_differences": []},
            "binary_library_bytes_equal": True,
        }
        payload = RUNNER.build_adoption_payload(
            current_commit, changed, overlay, "c" * 64
        )
        self.assertEqual(
            payload["product_install"]["git_commit"], RUNNER.PRODUCT_COMMIT
        )
        self.assertEqual(
            payload["product_install"]["manifest_sha256"],
            RUNNER.PRODUCT_MANIFEST_SHA256,
        )
        self.assertEqual(payload["runner_analyzer"]["git_commit"], current_commit)
        self.assertEqual(payload["post_product_changed_files"], changed)
        self.assertEqual(payload["installed_runtime_source_overlap"], [])
        self.assertTrue(payload["product_binary_runtime_unchanged"])

    def test_adoption_rejects_wrong_product_manifest_hash(self):
        with mock.patch.object(RUNNER, "_sha256", return_value="0" * 64):
            with self.assertRaisesRegex(
                RUNNER.LiveRunnerError, "product_manifest_sha256_mismatch"
            ):
                RUNNER.build_adoption_payload("b" * 40, [])

    def test_gnss_dependency_preflight_resolves_exact_frozen_inputs(self):
        result = RUNNER.resolve_gnss_dependencies(
            self.contract, RUNNER.INSTALL_ROOT
        )
        self.assertTrue(result["dependency_ready"])
        self.assertEqual(
            result["gnss_simulator"]["path"],
            str(RUNNER.INSTALL_ROOT / "lib/gnss_sim/gnss_sim_node"),
        )
        self.assertTrue(result["gnss_simulator"]["executable"])
        self.assertEqual(
            result["scenario"]["path"],
            str(RUNNER.INSTALL_ROOT / "share/iap/config/gnss_sim/demo7_skymask_nlos.yaml"),
        )
        self.assertEqual(
            result["rinex_ephemeris"]["path"],
            "/home/dev/ws_iap/src/LIGO./Data/BRDM00DLR_S_20221870000_01D_MN.rnx",
        )
        self.assertEqual(result["sensor_model"], {
            "constellations": ["GPS", "BDS", "GAL", "GLO"],
            "pseudorange_noise_std_m": 5.0,
            "doppler_noise_std_mps": 0.5,
            "map_occlusion": True,
            "skymask": True,
            "nlos": True,
            "multipath": True,
            "time_source": "trigger_topic",
            "trigger_topic": "/sim/drone_0/lidar",
            "fallback_to_synthetic_on_rinex_error": False,
        })

    def test_dependency_preflight_rejects_case_or_path_drift(self):
        drift = json.loads(json.dumps(self.contract))
        drift["qualification_values"][
            "gnss_fallback_to_synthetic_on_rinex_error"
        ] = True
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "full_sensor_dependency_contract_mismatch"
        ):
            RUNNER.resolve_gnss_dependencies(drift, RUNNER.INSTALL_ROOT)

    def test_byte_inventory_detects_same_size_and_mtime_content_change(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            path = Path(tmp) / "artifact"
            path.write_bytes(b"before")
            before = RUNNER.tree_byte_inventory(Path(tmp))
            stat = path.stat()
            path.write_bytes(b"after!")
            path.touch()
            import os
            os.utime(path, ns=(stat.st_atime_ns, stat.st_mtime_ns))
            after = RUNNER.tree_byte_inventory(Path(tmp))
            self.assertNotEqual(before, after)

    def test_install_environment_removes_external_write_controls(self):
        with mock.patch.dict(
            "os.environ",
            {"DESTDIR": "/tmp/escape", "CMAKE_INSTALL_COMPONENT": "bad"},
            clear=False,
        ):
            environment = RUNNER.overlay_install_environment()
        self.assertNotIn("DESTDIR", environment)
        self.assertNotIn("CMAKE_INSTALL_PREFIX", environment)
        self.assertNotIn("CMAKE_INSTALL_COMPONENT", environment)
        self.assertEqual(environment["PATH"], "/usr/bin:/bin")

    def test_install_driver_sanitizer_removes_compile_and_manifest_writes(self):
        source = '''file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE FILE FILES "/base/lib.so")
if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
    COMMAND "/usr/bin/python3" "-m" "compileall" "/base/install/python"
  )
endif()
if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()
file(WRITE "/base/build/${CMAKE_INSTALL_MANIFEST}" "x")
'''
        sanitized = RUNNER.sanitize_install_driver_text(source)
        self.assertIn("file(INSTALL", sanitized)
        self.assertNotIn("compileall", sanitized)
        self.assertNotIn("file(WRITE", sanitized)
        self.assertNotIn("/base/install/python", sanitized)

    def test_package_resolution_rejects_duplicate_identity(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            root = Path(tmp)
            first, second = root / "first", root / "second"
            marker = "share/ament_index/resource_index/packages/iap"
            (first / marker).parent.mkdir(parents=True)
            (second / marker).parent.mkdir(parents=True)
            (first / marker).write_text("")
            resolution = RUNNER.verify_package_identities(
                [first, second], {"iap": first}, lambda _: str(first)
            )
            self.assertEqual(resolution["iap"]["resolved_prefix"], str(first))
            (second / marker).write_text("")
            with self.assertRaisesRegex(
                RUNNER.LiveRunnerError, "duplicate_or_stale_package_identity"
            ):
                RUNNER.verify_package_identities(
                    [first, second], {"iap": first}, lambda _: str(first)
                )

    def test_overlay_install_command_is_no_compile_and_repository_local(self):
        command = RUNNER.overlay_install_command()
        self.assertEqual(command, [
            "cmake",
            f"-DCMAKE_INSTALL_PREFIX={RUNNER.OVERLAY_INSTALL_ROOT}",
            "-P", str(RUNNER.OVERLAY_INSTALL_DRIVER_PATH),
        ])
        self.assertNotIn("--build", command)
        self.assertTrue(str(RUNNER.OVERLAY_INSTALL_ROOT).startswith(str(REPO)))

    def test_overlay_inventory_allows_only_registered_current_alias_changes(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            root = Path(tmp)
            base = root / "base"
            overlay = root / "overlay"
            source = root / "source"
            for parent in (base, overlay, source):
                (parent / "lib").mkdir(parents=True)
                (parent / "share/iap/launch").mkdir(parents=True)
            (source / "launch").mkdir(parents=True)
            (base / "lib/libiap.so").write_bytes(b"binary")
            (overlay / "lib/libiap.so").write_bytes(b"binary")
            relative = "share/iap/launch/test_planner.launch.py"
            (base / relative).write_text("old\n")
            (overlay / relative).write_text("current\n")
            (source / "launch/test_planner.launch.py").write_text("current\n")
            inventory = RUNNER.inventory_overlay(
                overlay, base,
                {relative: "launch/test_planner.launch.py"},
                source_root=source,
            )
            self.assertEqual(inventory["authorized_differences"], [relative])
            self.assertTrue(inventory["binary_library_bytes_equal"])
            (overlay / "lib/libiap.so").write_bytes(b"changed")
            with self.assertRaisesRegex(
                RUNNER.LiveRunnerError, "unauthorized_overlay_difference"
            ):
                RUNNER.inventory_overlay(
                    overlay, base,
                    {relative: "launch/test_planner.launch.py"},
                    source_root=source,
                )

    def test_live_environment_prefers_overlay_then_immutable_base(self):
        environment = RUNNER.expected_live_environment()
        self.assertEqual(environment["PYTHONDONTWRITEBYTECODE"], "1")
        self.assertEqual(environment["AMENT_PREFIX_PATH"].split(":"), [
            str(RUNNER.COMPLETE_OVERLAY_INSTALL_ROOT), str(RUNNER.INSTALL_ROOT),
            "/root/ros2_ws/install", "/opt/ros/jazzy",
        ])
        self.assertEqual(environment["LD_LIBRARY_PATH"].split(":")[:2], [
            str(RUNNER.COMPLETE_OVERLAY_INSTALL_ROOT / "lib"),
            str(RUNNER.INSTALL_ROOT / "lib"),
        ])

    def test_overlay_manifest_separates_three_provenance_layers(self):
        dependency = {"dependency_ready": True, "schema_version": "dep-v1"}
        inventory = {
            "file_count": 3,
            "file_sha256": {"lib/libiap.so": "a" * 64},
            "base_file_sha256": {"lib/libiap.so": "a" * 64},
            "authorized_differences": [
                "share/iap/launch/test_planner.launch.py"
            ],
            "binary_library_bytes_equal": True,
        }
        payload = RUNNER.build_overlay_manifest_payload(
            "b" * 40, dependency, inventory,
            evidence_bindings={
                "overlay_install_command_sha256": "c" * 64,
                "dependency_preflight_sha256": "d" * 64,
            },
            retained_artifacts={"byte_inventory_equal": True},
            repair_binding={
                "path": "results/icra27/icra070/compact/overlay_cache_repair_v1.json",
                "sha256": "e" * 64,
            },
            import_probe={"install_inventory_unchanged": True},
        )
        self.assertEqual(
            payload["product_build"]["git_commit"], RUNNER.PRODUCT_COMMIT
        )
        self.assertEqual(
            payload["corrected_full_sensor_contract"]["git_commit"], "b" * 40
        )
        self.assertEqual(
            payload["schema_version"], "icra070_isolated_overlay_manifest_v2"
        )
        self.assertEqual(payload["runner"]["git_commit"], "b" * 40)
        self.assertTrue(payload["binary_library_bytes_equal"])
        self.assertEqual(
            payload["evidence_bindings"]["overlay_install_command_sha256"],
            "c" * 64,
        )
        self.assertTrue(payload["retained_artifacts"]["byte_inventory_equal"])
        self.assertEqual(payload["cache_boundary"], {
            "all_python_cache_excluded": True,
            "python_cache_allowlist": [],
            "pythondontwritebytecode": "1",
            "repair_binding": {
                "path": "results/icra27/icra070/compact/overlay_cache_repair_v1.json",
                "sha256": "e" * 64,
            },
            "installed_import_probe": {
                "install_inventory_unchanged": True
            },
        })
        self.assertEqual(payload["runtime_prefix_order"][:2], [
            str(RUNNER.OVERLAY_INSTALL_ROOT), str(RUNNER.INSTALL_ROOT),
        ])

    def test_parse_only_command_preserves_rendered_overrides(self):
        rendered = [
            "ros2", "launch", "iap", "test_planner.launch.py",
            "record_bag:=true", "run_duration_s:=90",
        ]
        self.assertEqual(
            RUNNER.parse_only_command(rendered),
            [
                "ros2", "launch", "--show-args", "iap",
                "test_planner.launch.py", "record_bag:=true",
                "run_duration_s:=90",
            ],
        )

    def test_replacement_analysis_reconciles_only_proven_commit_split(self):
        base = {
            "technical_failures": ["install manifest commit mismatch"],
            "behavioral_failures": [],
        }
        accepted = RUNNER.reconcile_replacement_analysis(base, [])
        self.assertEqual(
            accepted["status"], "P5_PROSPECTIVE_QUALIFICATION_PASS"
        )
        self.assertTrue(accepted["qualification_claim"])

        forged = {
            "technical_failures": [
                "install manifest commit mismatch", "raw artifact hash mismatch",
            ],
            "behavioral_failures": [],
        }
        rejected = RUNNER.reconcile_replacement_analysis(forged, [])
        self.assertEqual(
            rejected["status"],
            "P5_PROSPECTIVE_QUALIFICATION_TECHNICAL_BLOCKER",
        )
        self.assertIn("raw artifact hash mismatch", rejected["technical_failures"])

        missing_split = RUNNER.reconcile_replacement_analysis(
            {"technical_failures": [], "behavioral_failures": []}, []
        )
        self.assertEqual(
            missing_split["status"],
            "P5_PROSPECTIVE_QUALIFICATION_TECHNICAL_BLOCKER",
        )
        self.assertIn(
            "expected product/runner commit split marker missing",
            missing_split["technical_failures"],
        )

    def _valid_parser_proof(self, parse_root):
        cases = []
        for case_id, run_id in RUNNER.LIVE_IDENTITIES:
            case_root = parse_root / case_id.lower()
            case_root.mkdir(parents=True)
            stdout_path = case_root / "stdout.log"
            stderr_path = case_root / "stderr.log"
            stdout_path.write_text("args\n")
            stderr_path.write_text("")
            run_dir = RUNNER.TASK_ROOT / "live" / run_id
            config = RUNNER.live_config(
                self.contract, case_id, run_id, run_dir
            )
            rendered, omitted = RUNNER.render_live_launch_command(
                config, EMPTY_DEFAULT_KEYS
            )
            cases.append({
                "case_id": case_id,
                "replacement_run_id": run_id,
                "rendered_live_argv": rendered,
                "omitted_empty_defaults": omitted,
                "argv": RUNNER.parse_only_command(rendered),
                "exit_code": 0,
                "timed_out": False,
                "observed_process_group_pids": [123],
                "observed_required_processes": [],
                "remaining_process_group_pids": [],
                "task_owned_process_audit_passed": True,
                "stdout_path": str(stdout_path.relative_to(REPO)),
                "stdout_sha256": RUNNER._sha256(stdout_path),
                "stderr_path": str(stderr_path.relative_to(REPO)),
                "stderr_sha256": RUNNER._sha256(stderr_path),
                "parse_passed": True,
                "subprocess_environment": {
                    "PYTHONDONTWRITEBYTECODE": "1"
                },
                "full_sensor_overrides": RUNNER.full_sensor_launch_overrides(
                    self.contract
                ),
            })
        return {
            "schema_version": "icra070_ros_launch_parser_proof_v1",
            "case_order": [case_id for case_id, _ in RUNNER.LIVE_IDENTITIES],
            "cases": cases,
            "parse_invocations": 3,
            "main_flow_child_invocations": 0,
            "parse_ready": True,
            "full_sensor_resolution": RUNNER.full_sensor_resolution(
                self.contract
            ),
            "required_full_sensor_overrides": (
                RUNNER.full_sensor_launch_overrides(self.contract)
            ),
        }

    def test_parser_proof_validation_rejects_omission_and_tamper(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            parse_root = Path(tmp)
            proof = self._valid_parser_proof(parse_root)
            self.assertEqual(
                RUNNER.parse_proof_failures(
                    proof, self.contract, parse_root=parse_root
                ), []
            )

            omitted = json.loads(json.dumps(proof))
            del omitted["cases"][0]["stdout_sha256"]
            self.assertIn(
                "parse case[SAFE_NORMAL] stdout binding mismatch",
                RUNNER.parse_proof_failures(
                    omitted, self.contract, parse_root=parse_root
                ),
            )

            tampered = json.loads(json.dumps(proof))
            tampered["cases"][1]["argv"][-1] += "-tampered"
            self.assertIn(
                "parse case[FINAL_REJECT] argv mismatch",
                RUNNER.parse_proof_failures(
                    tampered, self.contract, parse_root=parse_root
                ),
            )

            sensor_drift = json.loads(json.dumps(proof))
            sensor_drift["cases"][2]["full_sensor_overrides"][
                "gnss_fallback_to_synthetic_on_rinex_error"
            ] = "true"
            self.assertIn(
                "parse case[RUNTIME_FAIL] full-sensor argv mismatch",
                RUNNER.parse_proof_failures(
                    sensor_drift, self.contract, parse_root=parse_root
                ),
            )

    def test_replacement_readiness_rejects_incomplete_runner_before_claim(self):
        bundle = {"manifest": {}}
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "replacement_evidence_not_ready"
        ):
            RUNNER.require_replacement_evidence_ready(bundle, self.contract)

    def test_replacement_analyzer_does_not_claim_incomplete_evidence(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            task_root = Path(tmp)
            input_path = task_root / "live/icra_p0_p5_evidence_v1.json"
            output_path = task_root / "compact/icra_p0_p5_analysis_v1.json"
            input_path.parent.mkdir(parents=True)
            input_path.write_text(json.dumps({"manifest": {}}) + "\n")
            with mock.patch.object(RUNNER, "TASK_ROOT", task_root), \
                    mock.patch.object(
                        RUNNER.QUALIFICATION, "_claim_live_analyzer_once"
                    ) as claim:
                with self.assertRaisesRegex(
                    RUNNER.LiveRunnerError, "replacement_evidence_not_ready"
                ):
                    RUNNER.analyze_replacement_live(input_path, output_path)
            claim.assert_not_called()

    def test_normalizer_binds_real_bag_and_p5_rows(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            run_dir = Path(tmp) / "run"
            export = run_dir / "exports/one"
            bag = run_dir / "bags/one"
            export.mkdir(parents=True)
            bag.mkdir(parents=True)
            manifest = RUNNER.QUALIFICATION.build_launch_binding(
                self.contract, self.contract_path, "RUNTIME_FAIL",
                "a" * 40, "fixed-run",
                RUNNER.QUALIFICATION.resolve_launch_values(
                    self.contract, "RUNTIME_FAIL", {}
                ),
            )
            (export / "test_planner_manifest.json").write_text(
                json.dumps({"icra_p0_p5_qualification": manifest}) + "\n"
            )
            (bag / "metadata.yaml").write_text("metadata\n")
            (bag / "evidence.db3").write_bytes(b"bag")
            for name in (
                "process_result.json", "capture_ready.json",
                "launch_command.json", "stdout.log",
            ):
                (run_dir / name).write_text("evidence\n")
            (run_dir / "integrity_report.jsonl").write_text(
                json.dumps({"valid": True, "n_sv_used": 8}) + "\n"
                + json.dumps({"valid": True, "n_sv_used": 9}) + "\n"
            )
            process_result = {
                "required_processes_ok": True,
                "controlled_shutdown": True,
                "orphan_check_passed": True,
                "forced_orphan_cleanup": False,
                "remaining_process_group_pids": [],
                "required_processes": {
                    name: {"seen": True, "runtime_failure": False}
                    for name in RUNNER.REQUIRED_PROCESSES
                },
                "process_failures": [],
            }
            metadata = {"missing": False, "topic_counts": {
                topic: 3 for topic in self.contract["required_topics"]
            }}
            p0_rows = [
                {"ready": True, "stale": False, "generation_id": 4,
                 "predictor_requested_worker_count": 4,
                 "predictor_effective_worker_count": 4,
                 "gnss_epoch_seen": True, "gnss_epoch_valid": True,
                 "gnss_epoch_fresh": True,
                 "predictor_gnss_used_count": 4,
                 "predictor_lidar_used_count": 4,
                 "predictor_horizon_fusion_count": 6,
                 "refresh_duration_ms": 12.0},
                {"ready": True, "stale": False, "generation_id": 5,
                 "predictor_requested_worker_count": 4,
                 "predictor_effective_worker_count": 4,
                 "gnss_epoch_seen": True, "gnss_epoch_valid": True,
                 "gnss_epoch_fresh": True,
                 "predictor_gnss_used_count": 5,
                 "predictor_lidar_used_count": 5,
                 "predictor_horizon_fusion_count": 6,
                 "refresh_duration_ms": 11.0},
            ]
            p5_rows = [
                {"bag_time_s": 1.0, "phase": "final_candidate", "action": "OK",
                 "reason": "ok", "final_candidate_traj_id": 7,
                 "final_candidate_rejected": False, "parse_error": ""},
                {"bag_time_s": 2.0, "phase": "runtime_committed", "action": "OK",
                 "reason": "ok", "final_candidate_traj_id": 7,
                 "parse_error": ""},
                {"bag_time_s": 4.5, "phase": "runtime_committed",
                 "action": "REQUEST_EMERGENCY_STOP_CANDIDATE",
                 "reason": "future_unknown_timeout", "final_candidate_traj_id": 7,
                 "future_unknown_duration_s": 2.5,
                 "samples": [{
                     "trajectory_sample_source": "runtime_committed",
                     "fixture_match": True,
                     "fixture_expected_reason": "future_unknown",
                     "reason": "future_unknown", "unknown": True,
                     "x": 1.0, "y": 0.0, "z": 0.0, "query_tau_s": 1.0,
                 }],
                 "parse_error": ""},
            ]
            bsplines = [{"bag_time_s": 1.5, "traj_id": 7}]
            with mock.patch.object(
                RUNNER.SAFETY_ANALYZER, "read_bag_metadata", return_value=metadata
            ), mock.patch.object(
                RUNNER.SAFETY_ANALYZER, "read_p0_bag_artifacts",
                return_value=({"health_rows": p0_rows}, ""),
            ), mock.patch.object(
                RUNNER.SAFETY_ANALYZER, "read_p5_status_messages",
                return_value=(p5_rows, ""),
            ), mock.patch.object(
                RUNNER.SAFETY_ANALYZER, "read_bspline_messages",
                return_value=(bsplines, ""),
            ):
                normalized = RUNNER.normalize_live_run(
                    self.contract, self.contract_path, "RUNTIME_FAIL",
                    "fixed-run", run_dir, process_result, "a" * 40,
                )
                p0_rows[0]["gnss_epoch_fresh"] = False
                with self.assertRaisesRegex(
                    RUNNER.LiveRunnerError, "full_sensor_p0_rows_missing"
                ):
                    RUNNER.normalize_live_run(
                        self.contract, self.contract_path, "RUNTIME_FAIL",
                        "fixed-run", run_dir, process_result, "a" * 40,
                    )
                p0_rows[0]["gnss_epoch_fresh"] = True
                process_result["required_processes"][
                    next(iter(RUNNER.REQUIRED_PROCESSES))
                ]["seen"] = False
                with self.assertRaisesRegex(
                    RUNNER.LiveRunnerError, "required_process_lifecycle_mismatch"
                ):
                    RUNNER.normalize_live_run(
                        self.contract, self.contract_path, "RUNTIME_FAIL",
                        "fixed-run", run_dir, process_result, "a" * 40,
                    )
                process_result["required_processes"][
                    next(iter(RUNNER.REQUIRED_PROCESSES))
                ]["seen"] = True
                process_result["process_failures"] = [{
                    "process_name": next(iter(RUNNER.REQUIRED_PROCESSES)),
                    "phase": "runtime", "reason": "required_process_died",
                }]
                with self.assertRaisesRegex(
                    RUNNER.LiveRunnerError, "required_process_lifecycle_mismatch"
                ):
                    RUNNER.normalize_live_run(
                        self.contract, self.contract_path, "RUNTIME_FAIL",
                        "fixed-run", run_dir, process_result, "a" * 40,
                    )
        self.assertFalse(normalized["validation_only"])
        self.assertEqual(
            [event["type"] for event in normalized["events"]],
            ["FINAL_ACCEPT", "NORMAL_PUBLISH", "RUNTIME_ACTION"],
        )
        self.assertEqual(normalized["events"][-1]["action"], "EMERGENCY_STOP")
        self.assertEqual(normalized["events"][-1]["reason"], "future_unknown_timeout")
        self.assertGreater(normalized["p0_samples"][0]["predictor_gnss_used_count"], 0)
        self.assertGreater(normalized["p0_samples"][0]["predictor_lidar_used_count"], 0)
        self.assertGreater(normalized["integrity_samples"][0]["n_sv_used"], 0)
        self.assertTrue(any(path.endswith("evidence.db3") for path in normalized["raw_sources"]))
        self.assertTrue(any(path.endswith("process_result.json") for path in normalized["raw_sources"]))

    def test_event_normalization_rejects_duplicates_and_early_runtime(self):
        accepted = {
            "bag_time_s": 2.0, "phase": "final_candidate", "action": "OK",
            "final_candidate_traj_id": 7, "final_candidate_rejected": False,
            "parse_error": "",
        }
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "final_accept_event_cardinality_mismatch"
        ):
            RUNNER._normalize_events(
                "SAFE_NORMAL", self.contract, [accepted, dict(accepted)],
                [{"bag_time_s": 3.0, "traj_id": 7}],
            )
        early = {
            "bag_time_s": 1.0, "phase": "runtime_committed",
            "action": "REQUEST_EMERGENCY_STOP_CANDIDATE",
            "reason": "future_unknown", "future_unknown_duration_s": 2.0,
            "samples": [{
                "trajectory_sample_source": "runtime_committed",
                "fixture_match": True,
                "fixture_expected_reason": "future_unknown",
                "reason": "future_unknown", "unknown": True,
                "x": 1.0, "y": 0.0, "z": 0.0, "query_tau_s": 1.0,
            }],
            "parse_error": "",
        }
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "future_unknown_emergency_before_threshold"
        ):
            RUNNER._normalize_events(
                "RUNTIME_FAIL", self.contract, [accepted, early],
                [{"bag_time_s": 3.0, "traj_id": 7}],
            )

    def test_event_normalization_rejects_unregistered_fixture_attribution(self):
        rejected = {
            "bag_time_s": 2.0, "phase": "final_candidate", "action": "REPLAN",
            "final_candidate_traj_id": 7, "final_candidate_rejected": True,
            "samples": [{
                "trajectory_sample_source": "final_candidate",
                "fixture_match": True, "fixture_expected_reason": "unrelated",
                "reason": "unrelated", "bad": True,
                "x": -10.0, "y": 0.0, "z": 1.1, "query_tau_s": 1.0,
            }],
        }
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "registered_fixture_evidence_mismatch"
        ):
            RUNNER._normalize_events(
                "FINAL_REJECT", self.contract, [rejected], []
            )
        fixture_only = dict(rejected)
        fixture_only.update(
            action="OK", final_candidate_rejected=False,
            final_candidate_traj_id=6,
        )
        unrelated_reject = dict(rejected)
        unrelated_reject["samples"] = []
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "registered_fixture_evidence_mismatch"
        ):
            RUNNER._normalize_events(
                "FINAL_REJECT", self.contract,
                [fixture_only, unrelated_reject], [],
            )
        accepted = {
            "bag_time_s": 1.0, "phase": "final_candidate", "action": "OK",
            "final_candidate_traj_id": 9, "final_candidate_rejected": False,
        }
        fixture_runtime = {
            "bag_time_s": 2.0, "phase": "runtime_committed", "action": "OK",
            "samples": [{
                "trajectory_sample_source": "runtime_committed",
                "fixture_match": True,
                "fixture_expected_reason": "future_unknown",
                "reason": "future_unknown", "unknown": True,
                "x": 1.0, "y": 0.0, "z": 0.0, "query_tau_s": 1.0,
            }],
        }
        unrelated_emergency = {
            "bag_time_s": 4.0, "phase": "runtime_committed",
            "action": "REQUEST_EMERGENCY_STOP_CANDIDATE",
            "reason": "future_unknown_timeout", "future_unknown_duration_s": 2.0,
            "samples": [],
        }
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "registered_fixture_evidence_mismatch"
        ):
            RUNNER._normalize_events(
                "RUNTIME_FAIL", self.contract,
                [accepted, fixture_runtime, unrelated_emergency],
                [{"bag_time_s": 1.5, "traj_id": 9}],
            )
        attributed_emergency = dict(unrelated_emergency)
        attributed_emergency["bag_time_s"] = 5.0
        attributed_emergency["samples"] = fixture_runtime["samples"]
        with self.assertRaisesRegex(
            RUNNER.LiveRunnerError, "registered_fixture_evidence_mismatch"
        ):
            RUNNER._normalize_events(
                "RUNTIME_FAIL", self.contract,
                [accepted, unrelated_emergency, attributed_emergency],
                [{"bag_time_s": 1.5, "traj_id": 9}],
            )

    def test_live_environment_rejects_caller_overlay(self):
        with mock.patch.dict(RUNNER.os.environ, {}, clear=True):
            with self.assertRaisesRegex(
                RUNNER.LiveRunnerError, "live_environment_mismatch"
            ):
                RUNNER.validate_live_environment()

    def test_top_level_early_exit_is_runtime_failure_not_success(self):
        launch = mock.Mock(pid=987654)
        launch.wait.return_value = 0
        monitor = mock.Mock()
        monitor.finish.return_value = {
            "required_processes_ok": True,
            "process_failures": [],
            "required_processes": {
                name: {"seen": True, "runtime_failure": False}
                for name in RUNNER.REQUIRED_PROCESSES
            },
        }
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            with mock.patch.object(
                RUNNER.subprocess, "Popen", return_value=launch
            ), mock.patch.object(
                RUNNER.GATE_RUNNER, "RequiredProcessMonitor", return_value=monitor
            ):
                exit_code, result = RUNNER._run_launch(
                    ["ros2", "launch"], Path(tmp) / "stdout.log"
                )
        self.assertEqual(exit_code, 0)
        self.assertFalse(result["required_processes_ok"])
        self.assertFalse(result["controlled_shutdown"])
        self.assertTrue(any(
            item["reason"] == "top_level_launch_exited_before_controlled_shutdown"
            for item in result["process_failures"]
        ))

    def test_run_matrix_stops_after_first_failure_and_never_retries(self):
        calls = []

        def execute(case_id, run_id):
            calls.append((case_id, run_id))
            return {"completed": case_id == "SAFE_NORMAL"}

        result = RUNNER.run_ordered_attempts(execute)
        self.assertEqual(calls, list(RUNNER.LIVE_IDENTITIES[:2]))
        self.assertEqual(result["attempted"], [item[1] for item in calls])
        self.assertEqual(result["completed"], [RUNNER.LIVE_IDENTITIES[0][1]])
        self.assertEqual(result["retries"], 0)

    def test_install_alias_audit_rejects_source_install_mismatch(self):
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            install = Path(tmp)
            for relative, source in RUNNER.INSTALLED_ALIASES.items():
                target = install / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes((REPO / source).read_bytes())
            mismatched = install / next(iter(RUNNER.INSTALLED_ALIASES))
            mismatched.write_text("stale\n")
            with self.assertRaisesRegex(
                RUNNER.LiveRunnerError, "installed_source_mismatch"
            ):
                RUNNER.verify_installed_aliases(install)

    def test_install_manifest_revalidation_rejects_reduced_inventory(self):
        retained = REPO / "results/icra27/icra068/icra068_install_manifest.json"
        manifest = json.loads(retained.read_text())
        manifest["git_commit"] = "a" * 40
        manifest["file_hashes"] = {}
        with tempfile.TemporaryDirectory(dir=REPO / "results/icra27") as tmp:
            path = Path(tmp) / "manifest.json"
            path.write_text(json.dumps(manifest) + "\n")
            with mock.patch.dict(
                RUNNER.os.environ,
                {"AMENT_PREFIX_PATH": ":".join(manifest["active_prefixes"])},
                clear=False,
            ), self.assertRaisesRegex(
                RUNNER.LiveRunnerError, "install_manifest_inventory_mismatch"
            ):
                RUNNER.validate_frozen_install_manifest(path, "a" * 40)

    def test_linkage_inventory_rejects_one_nibble_drift(self):
        libraries = ("lib/libiap.so",)
        with mock.patch.object(
            RUNNER, "_linkage_ready", return_value="a" * 64
        ):
            self.assertFalse(RUNNER.linkage_inventory_matches(
                {libraries[0]: "b" + "a" * 63}, libraries,
                RUNNER.INSTALL_ROOT, {},
            ))


if __name__ == "__main__":
    unittest.main()
