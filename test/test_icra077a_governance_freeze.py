#!/usr/bin/env python3
"""Public-contract tests for the ICRA-077A governance freeze boundary."""

from __future__ import annotations

import copy
import importlib.util
import json
import subprocess
import tempfile
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[1]
MODULE_PATH = REPOSITORY / "scripts/dev_planner/icra076_preregistration.py"
PROTOCOL_PATH = REPOSITORY / "config/icra27/icra076_preregistration_v1.json"
FREEZE005_PATH = (
    REPOSITORY / "results/icra27/icra076/preregistration-freeze-005.json")
GOVERNANCE_PATHS = (
    "docs/icra27/ICRA_IMPLEMENTATION_PLAN.md",
    "docs/icra27/ICRA_FOUR_LAYER_DEVELOPMENT_WORKFLOW.md",
    "docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md",
)


def load_module():
    spec = importlib.util.spec_from_file_location("icra076_preregistration", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class Icra077aGovernanceFreezeTest(unittest.TestCase):
    def setUp(self):
        self.module = load_module()
        self.protocol = json.loads(PROTOCOL_PATH.read_text())
        self.contract = self.protocol["byte_freeze"]["governance_snapshot"]

    def _coordinated_route_mutation(self, field, value):
        current = (REPOSITORY / GOVERNANCE_PATHS[2]).read_text()
        begin = current.index(self.module.ROUTE_BEGIN) + len(self.module.ROUTE_BEGIN)
        end = current.index(self.module.ROUTE_END)
        payload = current[begin:end].strip()
        document = json.loads(payload[len("```json\n"):-len("\n```")])
        document[field] = value
        for change in document["protected_transition"]["changes"]:
            if change["field"] == field:
                change["new"] = value
        rendered = "```json\n" + json.dumps(document, indent=2) + "\n```"
        return current[:begin] + "\n" + rendered + "\n" + current[end:], document

    def test_joint_current_transition_and_config_mutation_cannot_self_authorize(self):
        current, document = self._coordinated_route_mutation(
            "active_route", "P0_P5_CONTROL")
        mutated = copy.deepcopy(self.contract)
        protected = {
            field: document[field] for field in self.module.PROTECTED_ROUTE_FIELDS}
        mutated["protected_route"]["sha256"] = self.module.canonical_sha256(
            protected)
        with self.assertRaises(self.module.Icra076Error) as caught:
            self.module.validate_governance_snapshot(
                mutated, REPOSITORY, current_route_text=current)
        self.assertEqual(caught.exception.code,
                         "PROTECTED_ROUTE_FROZEN_FINGERPRINT_MISMATCH")

    def test_every_protected_field_is_cross_bound_to_frozen_blob(self):
        replacements = {
            "route_owner": "BUILDER",
            "active_route": "P0_P5_CONTROL",
            "required_modules": ["P0_ADVISORY_RISK_FIELD"],
            "research_question": "Changed research question",
            "primary_claim": "Changed primary claim",
            "secondary_claims": ["Changed secondary claim"],
            "formal_arms": ["P0_P5_CONTROL"],
            "qualification_scenes": ["PRIMARY"],
            "gate_sequence": ["ICRA-077_HELD_OUT_CONFIRMATION"],
            "fallback_policy": "AUTOMATIC",
            "scientific_no_go_transition": "CONTINUE",
            "campaign_activation": "BUILDER_APPROVAL",
        }
        for field, value in replacements.items():
            current, document = self._coordinated_route_mutation(field, value)
            with self.subTest(field=field, attack="current_plus_transition"), \
                    self.assertRaises(self.module.Icra076Error) as drift:
                self.module.validate_governance_snapshot(
                    self.contract, REPOSITORY, current_route_text=current)
            self.assertEqual(drift.exception.code, "PROTECTED_ROUTE_DRIFT")
            mutated = copy.deepcopy(self.contract)
            protected = {
                name: document[name]
                for name in self.module.PROTECTED_ROUTE_FIELDS}
            mutated["protected_route"]["sha256"] = \
                self.module.canonical_sha256(protected)
            with self.subTest(field=field, attack="plus_config_sha"), \
                    self.assertRaises(self.module.Icra076Error) as caught:
                self.module.validate_governance_snapshot(
                    mutated, REPOSITORY, current_route_text=current)
            self.assertEqual(
                caught.exception.code,
                "PROTECTED_ROUTE_FROZEN_FINGERPRINT_MISMATCH")

    def test_configured_fingerprint_must_match_frozen_blob(self):
        mutated = copy.deepcopy(self.contract)
        mutated["protected_route"]["sha256"] = "0" * 64
        with self.assertRaises(self.module.Icra076Error) as caught:
            self.module.validate_governance_snapshot(mutated, REPOSITORY)
        self.assertEqual(caught.exception.code,
                         "PROTECTED_ROUTE_FROZEN_FINGERPRINT_MISMATCH")

    def test_exact_three_current_drifts_pass_only_through_git_blob_snapshots(self):
        old = json.loads(FREEZE005_PATH.read_text())
        with self.assertRaises(self.module.Icra076Error) as legacy:
            self.module.verify_inventory(
                old["source_inventory"], REPOSITORY, "SOURCE")
        self.assertEqual(legacy.exception.code, "SOURCE_BYTE_DRIFT")
        result = self.module.validate_governance_snapshot(
            self.contract, REPOSITORY)
        self.assertEqual(tuple(item["path"] for item in result["git_blobs"]),
                         GOVERNANCE_PATHS)
        self.assertEqual(result["frozen_source_commit"],
                         "3a3486f793df1b8299a87bf3400d7e2c34979018")
        self.assertEqual(result["protected_route"]["frozen_derived_sha256"],
                         "1e05d2763e588f4d7c148984a4df9a9e6c3cdd52e5b5859810fc638ac633e25a")
        self.assertEqual(result["protected_route"]["current_matched_sha256"],
                         result["protected_route"]["frozen_derived_sha256"])
        self.assertTrue(result["protected_route"]["protected_fields_match"])
        for record in result["git_blobs"]:
            frozen = subprocess.check_output(
                ["git", "cat-file", "blob", record["git_blob_oid"]],
                cwd=REPOSITORY)
            current = (REPOSITORY / record["path"]).read_bytes()
            self.assertNotEqual(current, frozen, record["path"])

    def test_blob_identity_and_exact_path_set_fail_closed(self):
        cases = []
        for field, value in (
                ("path", "docs/icra27/ICRA_SCOPE.md"),
                ("git_blob_oid", "0" * 40),
                ("git_object_type", "tree"),
                ("frozen_source_commit", "0" * 40),
                ("size_bytes", 1),
                ("sha256", "0" * 64)):
            mutated = copy.deepcopy(self.contract)
            mutated["git_blobs"][0][field] = value
            cases.append((mutated, "GOVERNANCE_PATH_SET_INVALID" if
                          field == "path" else
                          "GOVERNANCE_BLOB_IDENTITY_INVALID"))
        fourth = copy.deepcopy(self.contract)
        fourth["git_blobs"].append(copy.deepcopy(fourth["git_blobs"][0]))
        fourth["git_blobs"][-1]["path"] = "docs/icra27/ICRA_SCOPE.md"
        cases.append((fourth, "GOVERNANCE_PATH_SET_INVALID"))
        for malformed in (None, "ignored", 7):
            fourth = copy.deepcopy(self.contract)
            fourth["git_blobs"].append(malformed)
            cases.append((fourth, "GOVERNANCE_PATH_SET_INVALID"))
        for value, code in cases:
            with self.subTest(code=code), \
                    self.assertRaises(self.module.Icra076Error) as caught:
                self.module.validate_governance_snapshot(value, REPOSITORY)
            self.assertEqual(caught.exception.code, code)

    def test_source_commit_must_exist_be_ancestor_and_fetch_confirmed(self):
        with tempfile.TemporaryDirectory(
                prefix="icra077a-lineage-", dir=REPOSITORY) as temporary:
            repo = Path(temporary)
            def git(*args, input_text=None):
                return subprocess.check_output(
                    ["git", *args], cwd=repo, text=True, input=input_text).strip()
            git("init", "-q")
            git("config", "user.email", "icra077a@example.invalid")
            git("config", "user.name", "ICRA077A Test")
            (repo / "tracked.txt").write_text("one\n")
            git("add", "tracked.txt")
            git("commit", "-q", "-m", "one")
            first = git("rev-parse", "HEAD")
            git("update-ref", "refs/remotes/origin/dev/icra", first)
            self.module.validate_governance_commit_lineage(repo, first)

            with self.assertRaises(self.module.Icra076Error) as absent:
                self.module.validate_governance_commit_lineage(repo, "0" * 40)
            self.assertEqual(absent.exception.code, "GOVERNANCE_COMMIT_MISSING")

            tree = git("rev-parse", "HEAD^{tree}")
            unrelated = git("commit-tree", tree, input_text="unrelated\n")
            with self.assertRaises(self.module.Icra076Error) as nonancestor:
                self.module.validate_governance_commit_lineage(repo, unrelated)
            self.assertEqual(nonancestor.exception.code,
                             "GOVERNANCE_COMMIT_NOT_ANCESTOR")

            (repo / "tracked.txt").write_text("two\n")
            git("add", "tracked.txt")
            git("commit", "-q", "-m", "two")
            with self.assertRaises(self.module.Icra076Error) as unconfirmed:
                self.module.validate_governance_commit_lineage(repo, first)
            self.assertEqual(unconfirmed.exception.code,
                             "GOVERNANCE_LINEAGE_NOT_FETCH_CONFIRMED")

    def test_protected_route_mutation_rejects_but_later_prose_passes(self):
        current = (REPOSITORY / GOVERNANCE_PATHS[2]).read_text()
        prose = current + "\nLater Supervisor review prose only.\n"
        result = self.module.validate_governance_snapshot(
            self.contract, REPOSITORY, current_route_text=prose)
        self.assertEqual(result["protected_route"]["current_matched_sha256"],
                         "1e05d2763e588f4d7c148984a4df9a9e6c3cdd52e5b5859810fc638ac633e25a")
        changed = current.replace(
            '"active_route": "P0_P4_V2_P5"',
            '"active_route": "P0_P5_CONTROL"', 1)
        with self.assertRaises(self.module.Icra076Error) as protected:
            self.module.validate_governance_snapshot(
                self.contract, REPOSITORY, current_route_text=changed)
        self.assertEqual(protected.exception.code, "PROTECTED_ROUTE_DRIFT")
        duplicate = current.replace(
            '"active_route": "P0_P4_V2_P5",',
            ('"active_route": "P0_P4_V2_P5",\n'
             '  "active_route": "P0_P4_V2_P5",'), 1)
        unknown = current.replace(
            '"schema_version": "icra_user_route_lock_v1",',
            ('"schema_version": "icra_user_route_lock_v1",\n'
             '  "unknown_route_field": "forbidden",'), 1)
        for malformed in (duplicate, unknown):
            with self.assertRaises(self.module.Icra076Error) as parsed:
                self.module.validate_governance_snapshot(
                    self.contract, REPOSITORY, current_route_text=malformed)
            self.assertEqual(parsed.exception.code, "PROTECTED_ROUTE_DRIFT")

    def test_non_governance_source_bytes_remain_strict(self):
        record = self.module.inventory_path(
            REPOSITORY / "scripts/dev_planner/icra076_repeatability_replay.py",
            REPOSITORY)
        changed = copy.deepcopy(record)
        changed["sha256"] = "0" * 64
        with self.assertRaises(self.module.Icra076Error) as strict:
            self.module.verify_inventory([changed], REPOSITORY, "SOURCE")
        self.assertEqual(strict.exception.code, "SOURCE_BYTE_DRIFT")


if __name__ == "__main__":
    unittest.main()
