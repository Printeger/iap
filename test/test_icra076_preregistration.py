#!/usr/bin/env python3
"""Focused public-contract tests for the ICRA-076 preregistration freeze."""

from __future__ import annotations

import importlib.util
import copy
import json
import math
import tempfile
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[1]
MODULE_PATH = REPOSITORY / "scripts/dev_planner/icra076_preregistration.py"
PROTOCOL_PATH = REPOSITORY / "config/icra27/icra076_preregistration_v1.json"
REGISTRY_PATH = REPOSITORY / "config/icra27/icra076_seed_registry_v1.json"


def load_module():
    spec = importlib.util.spec_from_file_location("icra076_preregistration", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class Icra076PreregistrationTest(unittest.TestCase):
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
        self.assertEqual(frozen["delta_peak_m"], 0.3)
        self.assertEqual(frozen["endpoint_buffer_m"], 1.5)
        self.assertEqual(frozen["u95_repeatability_m"], 0.0)
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
            "protocol", lambda value: value["delta_peak"].update(value=0.2),
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
            "replay", lambda value: value["admissibility"].update(
                retained_non_held_out_evidence_path="results/icra077/held_out.json"),
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
                "config", "scripts/dev_planner", "test/test_icra076_preregistration.py",
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

    def test_repeatability_consumes_sixty_serialized_observations(self):
        module = load_module()
        replay = json.loads((REPOSITORY / "config/icra27/"
                             "icra076_repeatability_replay_v1.json").read_text())
        observations = replay["serialized_snapshot_replays"]
        self.assertEqual(len(observations), 60)
        self.assertEqual([item["replay_index"] for item in observations],
                         list(range(1, 61)))
        mutated = copy.deepcopy(replay)
        mutated["serialized_snapshot_replays"][59][
            "risk_interior_provider_c_pi_m"] += 0.5
        with tempfile.TemporaryDirectory(prefix="icra076-replay-", dir=REPOSITORY) as temporary:
            path = Path(temporary) / "replay.json"
            path.write_text(json.dumps(mutated))
            with self.assertRaises(module.Icra076Error) as caught:
                module.validate_preregistration(
                    PROTOCOL_PATH, REGISTRY_PATH, replay_path=path)
        self.assertIn(caught.exception.code, {
            "REPEATABILITY_SNAPSHOT_NOT_BYTE_IDENTICAL",
            "REPEATABILITY_CONTRACT_MISMATCH"})

    def test_skipped_disabled_or_failed_verification_is_rejected(self):
        module = load_module()
        valid = {"commands": [
            {"category": category, "argv": argv, "enabled": True,
             "skipped": False, "exit_code": 0}
            for category, argv in module.expected_verification_argv().items()]}
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


if __name__ == "__main__":
    unittest.main()
