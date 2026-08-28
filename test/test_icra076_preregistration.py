#!/usr/bin/env python3
"""Focused public-contract tests for the ICRA-076 preregistration freeze."""

from __future__ import annotations

import importlib.util
import copy
import json
import math
import shutil
import subprocess
import tempfile
import unittest
import unittest.mock
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[1]
MODULE_PATH = REPOSITORY / "scripts/dev_planner/icra076_preregistration.py"
PROTOCOL_PATH = REPOSITORY / "config/icra27/icra076_preregistration_v1.json"
REGISTRY_PATH = REPOSITORY / "config/icra27/icra076_seed_registry_v1.json"
PROBE_PATH = Path("/home/dev/ws_iap/build/bspline_opt/icra076_repeatability_probe")
SNAPSHOT_INPUT_PATH = (
    REPOSITORY / "config/icra27/icra076_flat_null_snapshot_v1.json")
REPLAY_RUNNER_PATH = (
    REPOSITORY / "scripts/dev_planner/icra076_repeatability_replay.py")


def load_module():
    spec = importlib.util.spec_from_file_location("icra076_preregistration", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class Icra076PreregistrationTest(unittest.TestCase):
    @staticmethod
    def _verification(module, source_head="0" * 40):
        protocol = json.loads(PROTOCOL_PATH.read_text())
        environment = module.current_freeze_environment_binding(
            protocol, REPOSITORY)
        return {
            "schema_version": "icra077a_repository_local_verification_v2",
            "source_head": source_head,
            "commands": [
                {"category": category, "argv": argv, "enabled": True,
                 "skipped": False, "exit_code": 0}
                for category, argv in module.expected_verification_argv().items()],
            **environment,
        }

    @staticmethod
    def _measurement(index, d_peak):
        b_risk = 1.0
        b_original = b_risk + d_peak
        return {
            "schema_version": "icra076_production_replay_measurement_v1",
            "status": "PASS",
            "unit": "m",
            "domain": "controllable_interior_b_equals_2r",
            "measurement_source": "P4GuideDecision.risk_profile.max",
            "invocation_index": index,
            "snapshot_identity": {
                "serialized_input_sha256": "a" * 64,
                "snapshot_config_hash": "0123456789abcdef",
                "snapshot_generation": 1,
            },
            "sample_identity": {
                "equal_arc_lattice_count": 200,
                "original_lattice_hash": "1111111111111111",
                "risk_lattice_hash": "2222222222222222",
                "original_controllable_sample_count": 174,
                "risk_controllable_sample_count": 174,
                "original_valid_count": 174,
                "risk_valid_count": 174,
            },
            "B_original_m": b_original,
            "B_risk_m": b_risk,
            "D_peak_m": d_peak,
        }

    def test_measured_repeatability_uses_absolute_d_peak_not_reference_delta(self):
        module = load_module()
        values = [0.5] + [0.0] * 55 + [0.2, 0.3, 0.4, 0.5]
        measurements = [self._measurement(index, value)
                        for index, value in enumerate(values, start=1)]
        result = module.calculate_measured_repeatability(measurements)
        self.assertEqual(result["nearest_rank"], 57)
        self.assertEqual(result["u95_repeatability_m"], 0.3)
        self.assertEqual(result["absolute_D_peak_m"], [abs(v) for v in values])

    def test_probe_parser_requires_machine_measurement_not_pass_transcript(self):
        module = load_module()
        transcript = "[==========] 1 test ran\n[  PASSED  ] 1 test.\n"
        with self.assertRaises(module.Icra076Error) as transcript_only:
            module.parse_probe_output(transcript.encode("utf-8"), 1)
        self.assertEqual(transcript_only.exception.code,
                         "PROBE_OUTPUT_NOT_MEASUREMENT")
        raw = (json.dumps(self._measurement(1, 0.0), sort_keys=True) +
               "\n").encode("utf-8")
        parsed = module.parse_probe_output(raw, 1)
        self.assertEqual(parsed["B_original_m"], 1.0)
        self.assertEqual(parsed["B_risk_m"], 1.0)

    def test_production_probe_emits_profile_measurement_from_serialized_input(self):
        module = load_module()
        completed = subprocess.run(
            [str(PROBE_PATH), "--snapshot", str(SNAPSHOT_INPUT_PATH),
             "--invocation-index", "1"],
            cwd=REPOSITORY, capture_output=True, check=False)
        self.assertEqual(completed.returncode, 0, completed.stderr.decode())
        measurement = module.parse_probe_output(completed.stdout, 1)
        self.assertEqual(measurement["measurement_source"],
                         "P4GuideDecision.risk_profile.max")
        self.assertEqual(measurement["sample_identity"][
            "equal_arc_lattice_count"], 200)
        self.assertEqual(measurement["B_original_m"], 1.0)
        self.assertEqual(measurement["B_risk_m"], 1.0)
        self.assertEqual(measurement["D_peak_m"], 0.0)

    def test_replay_runner_retains_sixty_emissions_and_computes_u95(self):
        module = load_module()
        allowed = REPOSITORY / "results/icra27/icra076"
        allowed.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(
                prefix="icra076-runner-test-", dir=allowed) as temporary:
            output_root = Path(temporary) / "repeatability-replay-test"
            completed = subprocess.run(
                ["python3", str(REPLAY_RUNNER_PATH),
                 "--snapshot", str(SNAPSHOT_INPUT_PATH),
                 "--output-root", str(output_root)],
                cwd=REPOSITORY, capture_output=True, text=True, check=False)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            manifest = output_root / "manifest.json"
            result = module.load_measured_replay_manifest(manifest)
            self.assertEqual(result["measurement_count"], 60)
            self.assertEqual(result["u95_repeatability_m"], 0.0)
            self.assertEqual(len(list(output_root.glob("measurement-*.json"))),
                             60)
            original = json.loads(manifest.read_text())
            for section, code in (
                    ("executable", "REPLAY_EXECUTABLE_OR_INPUT_DRIFT"),
                    ("serialized_input", "REPLAY_EXECUTABLE_OR_INPUT_DRIFT")):
                mutated = copy.deepcopy(original)
                mutated[section]["sha256"] = "f" * 64
                manifest.write_text(json.dumps(mutated))
                with self.subTest(section=section), \
                        self.assertRaises(module.Icra076Error) as caught:
                    module.load_measured_replay_manifest(manifest)
                self.assertEqual(caught.exception.code, code)
            alternate_probe = Path(temporary) / "alternate-probe"
            shutil.copy2(PROBE_PATH, alternate_probe)
            mutated = copy.deepcopy(original)
            mutated["executable"] = module.inventory_path(
                alternate_probe, alternate_probe.parent)
            mutated["executable"]["path"] = str(alternate_probe)
            manifest.write_text(json.dumps(mutated))
            with self.assertRaises(module.Icra076Error) as executable_path:
                module.load_measured_replay_manifest(manifest)
            self.assertEqual(executable_path.exception.code,
                             "REPLAY_INVOCATION_IDENTITY_INVALID")
            alternate_input = Path(temporary) / "alternate-input.json"
            shutil.copy2(SNAPSHOT_INPUT_PATH, alternate_input)
            mutated = copy.deepcopy(original)
            mutated["serialized_input"] = module.inventory_path(
                alternate_input, REPOSITORY)
            manifest.write_text(json.dumps(mutated))
            with self.assertRaises(module.Icra076Error) as input_path:
                module.load_measured_replay_manifest(manifest)
            self.assertEqual(input_path.exception.code,
                             "REPLAY_INVOCATION_IDENTITY_INVALID")
            mutated = copy.deepcopy(original)
            mutated["accepted_debt"]["icra076_status"] = "PASS"
            manifest.write_text(json.dumps(mutated))
            with self.assertRaises(module.Icra076Error) as environment:
                module.load_measured_replay_manifest(manifest)
            self.assertEqual(environment.exception.code,
                             "REPLAY_ENVIRONMENT_DRIFT")
            manifest.write_text(json.dumps(original))

    def test_measured_replay_adversaries_fail_closed(self):
        module = load_module()
        valid = [self._measurement(index, 0.0)
                 for index in range(1, 61)]
        cases = []

        changed_b = copy.deepcopy(valid)
        changed_b[12]["B_original_m"] = 1.25
        cases.append((changed_b, "REPEATABILITY_B_D_INCONSISTENT"))

        wrong_unit = copy.deepcopy(valid)
        wrong_unit[2]["unit"] = "risk_cost"
        cases.append((wrong_unit, "REPEATABILITY_MEASUREMENT_INVALID"))

        wrong_domain = copy.deepcopy(valid)
        wrong_domain[3]["domain"] = "whole_path"
        cases.append((wrong_domain, "REPEATABILITY_MEASUREMENT_INVALID"))

        wrong_samples = copy.deepcopy(valid)
        wrong_samples[4]["sample_identity"]["equal_arc_lattice_count"] = 199
        cases.append((wrong_samples, "REPEATABILITY_SAMPLE_IDENTITY_INVALID"))

        incomplete = copy.deepcopy(valid)
        incomplete[5]["sample_identity"]["risk_valid_count"] = 173
        cases.append((incomplete, "REPEATABILITY_SAMPLE_IDENTITY_INVALID"))

        snapshot_drift = copy.deepcopy(valid)
        snapshot_drift[6]["snapshot_identity"]["serialized_input_sha256"] = "b" * 64
        cases.append((snapshot_drift, "REPEATABILITY_SNAPSHOT_IDENTITY_DRIFT"))

        for measurements, code in cases:
            with self.subTest(code=code):
                with self.assertRaises(module.Icra076Error) as caught:
                    module.calculate_measured_repeatability(measurements)
                self.assertEqual(caught.exception.code, code)

        with self.assertRaises(module.Icra076Error) as count:
            module.calculate_measured_repeatability(valid[:-1])
        self.assertEqual(count.exception.code,
                         "REPEATABILITY_MEASUREMENT_COUNT_INVALID")

    def _assert_mutation_rejected(self, target, mutate, code,
                                  bind_mutated_inputs=True):
        module = load_module()
        documents = {
            "protocol": json.loads(PROTOCOL_PATH.read_text()),
            "registry": json.loads(REGISTRY_PATH.read_text()),
            "schema": json.loads((REPOSITORY / "config/icra27/"
                                   "icra076_preregistration_schema_v1.json").read_text()),
            "replay": json.loads((REPOSITORY / "config/icra27/"
                                   "icra076_repeatability_replay_v1.json").read_text()),
        }
        mutate(documents[target])
        with tempfile.TemporaryDirectory(prefix="icra076-test-", dir=REPOSITORY) as temporary:
            root = Path(temporary)
            paths = {name: root / f"{name}.json" for name in documents}
            relative = {name: str(path.relative_to(REPOSITORY))
                        for name, path in paths.items()}
            documents["protocol"]["seed_registry"]["path"] = relative["registry"]
            documents["protocol"]["repeatability"]["input_path"] = relative["replay"]
            if bind_mutated_inputs:
                documents["protocol"]["seed_registry"]["canonical_sha256"] = (
                    module.canonical_sha256(documents["registry"]))
                documents["protocol"]["repeatability"]["input_canonical_sha256"] = (
                    module.canonical_sha256(documents["replay"]))
            for name, path in paths.items():
                path.write_text(json.dumps(documents[name], indent=2) + "\n")
            with self.assertRaises(module.Icra076Error) as caught:
                module.validate_preregistration(
                    paths["protocol"], paths["registry"], REPOSITORY,
                    paths["schema"], paths["replay"])
        self.assertEqual(caught.exception.code, code)

    def test_exact_binomial_rule_requires_59_of_60(self):
        module = load_module()
        rule = module.exact_binomial_passing_rule(60, 0.9, 0.05)
        self.assertEqual(rule["minimum_success_count"], 59)
        self.assertEqual(rule["failure_count_allowed"], 1)
        self.assertEqual(rule["tail_at_minimum"],
                         "0.013777078966010639279834378692906971971274646477790191815741")
        self.assertEqual(rule["tail_below_minimum"],
                         "0.053045081815992654618492752745153896913555039820283975251911")

    def test_protocol_freezes_estimand_threshold_seeds_and_order(self):
        module = load_module()
        frozen = module.validate_preregistration(PROTOCOL_PATH, REGISTRY_PATH)
        self.assertEqual(frozen["endpoint_buffer_m"], 1.5)
        replay = json.loads((REPOSITORY / "config/icra27/"
                             "icra076_repeatability_replay_v1.json").read_text())
        evidence_exists = (REPOSITORY /
                           replay["measured_replay_manifest_path"]).exists()
        self.assertEqual(frozen["repeatability_pending"], not evidence_exists)
        self.assertEqual(frozen["u95_repeatability_m"],
                         0.0 if evidence_exists else None)
        self.assertEqual(frozen["delta_peak_m"],
                         0.3 if evidence_exists else None)
        self.assertEqual(frozen["minimum_success_count"], 59)
        self.assertEqual(len(frozen["execution_order"]), 360)
        self.assertEqual(len({row["run_id"] for row in frozen["execution_order"]}),
                         360)
        for scene in ("PRIMARY", "EXACT_MIRROR", "FLAT_NULL"):
            rows = [row for row in frozen["execution_order"]
                    if row["scene"] == scene]
            self.assertEqual(len(rows), 120)
            self.assertEqual(len({row["seed"] for row in rows}), 60)
            for seed in {row["seed"] for row in rows}:
                self.assertEqual(
                    {row["arm"] for row in rows if row["seed"] == seed},
                    {"P0_P5_CONTROL", "P0_P4_V2_P5_TREATMENT"})

    def test_freeze_requires_fresh_measured_replay_manifest(self):
        module = load_module()
        replay = json.loads((REPOSITORY / "config/icra27/"
                             "icra076_repeatability_replay_v1.json").read_text())
        self.assertEqual(replay["schema_version"],
                         "icra076_production_measured_replay_binding_v2")
        self.assertEqual(replay["measured_replay_manifest_path"],
                         "results/icra27/icra076/"
                         "repeatability-replay-004/manifest.json")
        verification = self._verification(module)
        manifest = (REPOSITORY / replay["measured_replay_manifest_path"]).resolve()
        original_exists = Path.exists
        def exists_except_manifest(path):
            return False if path.resolve() == manifest else original_exists(path)
        with unittest.mock.patch.object(Path, "exists", exists_except_manifest), \
                tempfile.TemporaryDirectory(
                    prefix="icra076-freeze-pending-", dir=REPOSITORY) as temporary:
            with self.assertRaises(module.Icra076Error) as caught:
                module.create_freeze_record(
                    PROTOCOL_PATH, REGISTRY_PATH,
                    Path(temporary) / "freeze.json", verification)
        self.assertEqual(caught.exception.code, "MEASURED_REPLAY_REQUIRED")

    def test_runner_has_no_measurement_constant_injection_seam(self):
        source = REPLAY_RUNNER_PATH.read_text()
        for forbidden in ("B_original_m", "B_risk_m", "D_peak_m",
                          "fixture_source", "expected_u95"):
            self.assertNotIn(forbidden, source)

    def test_schema_downgrade_is_typed_and_fail_closed(self):
        self._assert_mutation_rejected(
            "schema", lambda value: value.update(
                schema_version="icra076_preregistration_schema_v0"),
            "SCHEMA_DOWNGRADE_OR_MISMATCH")

    def test_omitted_identity_seed_and_order_are_rejected(self):
        cases = (
            ("protocol", lambda value: value["seed_registry"].pop("canonical_sha256"),
             "SEED_REGISTRY_IDENTITY_MISMATCH", False),
            ("registry", lambda value: value["confirmatory_seeds"]["PRIMARY"].pop(),
             "CONFIRMATORY_SEED_CARDINALITY_INVALID", True),
            ("protocol", lambda value: value["execution_order"].pop(
                "expanded_order_sha256"), "EXECUTION_ORDER_INVALID", True),
        )
        for target, mutation, code, bind in cases:
            with self.subTest(code=code):
                self._assert_mutation_rejected(target, mutation, code, bind)

    def test_reused_seed_and_wrong_arm_or_scene_are_rejected(self):
        self._assert_mutation_rejected(
            "registry", lambda value: value["confirmatory_seeds"][
                "EXACT_MIRROR"].__setitem__(
                    0, value["confirmatory_seeds"]["PRIMARY"][0]),
            "CONFIRMATORY_SEED_REUSED_ACROSS_SCENES")
        self._assert_mutation_rejected(
            "protocol", lambda value: value["formal_arms"].__setitem__(
                1, "P0_P4_LEGACY"), "FORMAL_ARMS_INVALID")
        self._assert_mutation_rejected(
            "protocol", lambda value: value["scenes"].__setitem__(
                2, "UNKNOWN"), "FORMAL_SCENES_INVALID")

    def test_threshold_and_statistical_value_drift_are_rejected(self):
        self._assert_mutation_rejected(
            "protocol", lambda value: value["delta_peak"].update(value="0.2"),
            "DELTA_PEAK_CONTRACT_INVALID")
        self._assert_mutation_rejected(
            "protocol", lambda value: value["exact_binomial_rule"].update(
                minimum_success_count=58), "BINOMIAL_RULE_DRIFT")
        self._assert_mutation_rejected(
            "replay", lambda value: value.update(unit="risk_cost"),
            "STATISTICAL_UNIT_INVALID")
        self._assert_mutation_rejected(
            "protocol", lambda value: value["domain_sesoi"].update(
                value=math.inf), "DOMAIN_SESOI_INVALID")

    def test_held_out_reference_is_rejected_before_path_access(self):
        self._assert_mutation_rejected(
            "replay", lambda value: value.update(
                measured_replay_manifest_path="results/icra077/held_out.json"),
            "HELD_OUT_PATH_FORBIDDEN")

    def test_top_level_held_out_inputs_are_rejected_before_access(self):
        module = load_module()
        forbidden = REPOSITORY / "held_out-do-not-open.json"
        with self.assertRaises(module.Icra076Error) as protocol:
            module.validate_preregistration(forbidden, REGISTRY_PATH)
        self.assertEqual(protocol.exception.code, "HELD_OUT_PATH_FORBIDDEN")
        with self.assertRaises(module.Icra076Error) as verification:
            module.load_verification(Path("/tmp/held-out-do-not-open.json"))
        self.assertEqual(verification.exception.code, "HELD_OUT_PATH_FORBIDDEN")
        with tempfile.TemporaryDirectory(prefix="icra076-alias-", dir=REPOSITORY) as temporary:
            root = Path(temporary)
            held_out = root / "held_out"
            held_out.mkdir()
            (held_out / "input.json").write_text("{}")
            alias = root / "alias"
            alias.symlink_to(held_out, target_is_directory=True)
            with self.assertRaises(module.Icra076Error) as parent_alias:
                module.validate_preregistration(alias / "input.json", REGISTRY_PATH)
            self.assertEqual(parent_alias.exception.code,
                             "HELD_OUT_PATH_FORBIDDEN")
        with tempfile.TemporaryDirectory(prefix="icra076-external-alias-") as temporary:
            root = Path(temporary)
            held_out = root / "held-out"
            held_out.mkdir()
            (held_out / "verification.json").write_text("{}")
            alias = root / "alias"
            alias.symlink_to(held_out, target_is_directory=True)
            with self.assertRaises(module.Icra076Error) as external_alias:
                module.load_verification(alias / "verification.json")
            self.assertEqual(external_alias.exception.code,
                             "HELD_OUT_PATH_FORBIDDEN")

    def test_output_must_be_repository_local_fresh_and_non_symlinked(self):
        module = load_module()
        with tempfile.TemporaryDirectory(prefix="icra076-output-", dir=REPOSITORY) as temporary:
            allowed_root = Path(temporary)
            fresh = allowed_root / "freeze-001.json"
            self.assertEqual(
                module.validate_output_path(fresh, REPOSITORY, allowed_root),
                fresh.resolve())
            fresh.write_text("retained\n")
            with self.assertRaises(module.Icra076Error) as overwrite:
                module.validate_output_path(fresh, REPOSITORY, allowed_root)
            self.assertEqual(overwrite.exception.code, "OUTPUT_ALREADY_EXISTS")
            link = allowed_root / "link"
            link.symlink_to(REPOSITORY / "results", target_is_directory=True)
            with self.assertRaises(module.Icra076Error) as symlinked:
                module.validate_output_path(link / "freeze.json", REPOSITORY,
                                            allowed_root)
            self.assertEqual(symlinked.exception.code, "OUTPUT_SYMLINK_COMPONENT")
        with self.assertRaises(module.Icra076Error) as external:
            module.validate_output_path(Path("/tmp/icra076.json"), REPOSITORY,
                                        REPOSITORY / "results/icra27/icra076")
        self.assertEqual(external.exception.code, "OUTPUT_OUTSIDE_ALLOWED_ROOT")

    def test_source_and_install_inventory_drift_is_typed(self):
        module = load_module()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source.cpp"
            installed = root / "installed.so"
            source.write_bytes(b"source-v1")
            installed.write_bytes(b"installed-v1")
            source_record = module.inventory_path(source, root)
            install_record = module.inventory_path(installed, root)
            module.verify_inventory([source_record], root, "SOURCE")
            module.verify_inventory([install_record], root, "INSTALL")
            source.write_bytes(b"source-v2")
            with self.assertRaises(module.Icra076Error) as source_drift:
                module.verify_inventory([source_record], root, "SOURCE")
            self.assertEqual(source_drift.exception.code, "SOURCE_BYTE_DRIFT")
            source.write_bytes(b"source-v1")
            installed.write_bytes(b"installed-v2")
            with self.assertRaises(module.Icra076Error) as install_drift:
                module.verify_inventory([install_record], root, "INSTALL")
            self.assertEqual(install_drift.exception.code, "INSTALL_BYTE_DRIFT")

    def test_inventory_schema_and_source_coverage_are_complete(self):
        module = load_module()
        protocol = json.loads(PROTOCOL_PATH.read_text())
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            target = root / "target"
            target.write_bytes(b"bytes")
            link = root / "link"
            link.symlink_to(target)
            self.assertEqual(
                set(module.inventory_path(link, root)),
                set(protocol["byte_freeze"]["inventory_record_fields"]))
        roots = protocol["byte_freeze"]["source_inventory_roots"]
        for required in (
                "include/iap", "src/iap", "src/uav_simulator", "launch",
                "config", "scripts/dev_planner",
                "test/test_icra073_inverse_corridor.py",
                "test/test_icra074_geometry.py",
                "test/test_icra075_exploratory.py",
                "test/test_icra076_preregistration.py",
                "scripts/dev_planner/build_iap_dev.sh",
                "docs/icra27/dev/ICRA_P4_V2_INVERSE_CORRIDOR_FIXTURE.md",
                "docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md",
                "docs/icra27/ICRA_FOUR_LAYER_DEVELOPMENT_WORKFLOW.md",
                "docs/icra27/ICRA_CROSS_LAYER_GUARD_PLAN.md"):
            self.assertIn(required, roots)
        self.assertEqual(
            protocol["byte_freeze"]["rebuilt_packages"],
            ["iap", "plan_env", "traj_utils", "path_searching",
             "bspline_opt", "ego_planner"])
        self.assertTrue({"gnss_sim", "local_sensing", "map_generator"}.issubset(
            protocol["byte_freeze"]["runtime_install_packages"]))

    def test_skipped_disabled_or_failed_verification_is_rejected(self):
        module = load_module()
        valid = self._verification(module)
        module.validate_verification(valid)
        wrong_command = copy.deepcopy(valid)
        wrong_command["commands"][0]["argv"] = ["true"]
        with self.assertRaises(module.Icra076Error) as command:
            module.validate_verification(wrong_command)
        self.assertEqual(command.exception.code,
                         "REQUIRED_VERIFICATION_COMMAND_DRIFT")
        for field, value in (("skipped", True), ("enabled", False),
                             ("exit_code", 1)):
            mutated = copy.deepcopy(valid)
            mutated["commands"][0][field] = value
            with self.subTest(field=field), \
                    self.assertRaises(module.Icra076Error) as caught:
                module.validate_verification(mutated)
            self.assertEqual(caught.exception.code,
                             "REQUIRED_VERIFICATION_NOT_PASS")

    def test_verification_is_repository_local_and_source_bound(self):
        module = load_module()
        valid = self._verification(module)
        with tempfile.TemporaryDirectory(prefix="icra076-verification-") as root:
            external = Path(root) / "verification.json"
            external.write_text(json.dumps(valid))
            with self.assertRaises(module.Icra076Error) as caught:
                module.load_verification(external)
            self.assertEqual(caught.exception.code,
                             "EXTERNAL_INPUT_PATH_FORBIDDEN")
        module.validate_verification(valid, "0" * 40)
        with self.assertRaises(module.Icra076Error) as source:
            module.validate_verification(valid, "1" * 40)
        self.assertEqual(source.exception.code,
                         "REQUIRED_VERIFICATION_SOURCE_DRIFT")
        changed = copy.deepcopy(valid)
        changed["governance_snapshot"]["git_blobs"][0]["size_bytes"] += 1
        with self.assertRaises(module.Icra076Error) as environment:
            module.validate_verification(changed, "0" * 40)
        self.assertEqual(environment.exception.code,
                         "REQUIRED_VERIFICATION_ENVIRONMENT_DRIFT")


if __name__ == "__main__":
    unittest.main()
