#!/usr/bin/env python3
"""Behavioral tests for the ICRA-071 repository route guard."""

from __future__ import annotations

import importlib.util
import json
import shutil
import subprocess
import tempfile
import unittest
import os
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[1]
VERIFIER_PATH = REPOSITORY / "scripts/dev_planner/verify_icra_research_route.py"
ROUTE_LOCK_PATH = (
    REPOSITORY
    / "docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md"
)
REQUIRED_DEVELOPMENT_DOCS_FOR_TEST = (
    "DEV_LOG.md", "docs/CHANGES.md", "docs/TRACEABILITY.md",
)


def load_verifier():
    spec = importlib.util.spec_from_file_location("icra_route_guard", VERIFIER_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class RouteLockParserTest(unittest.TestCase):
    def test_current_route_lock_is_typed_immutable_and_history_bound(self):
        guard = load_verifier()
        route = guard.parse_route_lock(ROUTE_LOCK_PATH, REPOSITORY)

        self.assertEqual(route.route_owner, "USER")
        self.assertEqual(route.active_route, "P0_P4_V2_P5")
        self.assertEqual(route.user_decision_id, "USER-ICRA-ROUTE-20260826-001")
        self.assertEqual(
            route.approval_anchor,
            "48caa9ddf24990accb65e2ad230d12821487793c",
        )
        with self.assertRaises((AttributeError, TypeError)):
            route.active_route = "P0_P5_CONTINGENCY"

    def test_missing_duplicate_and_reversed_sentinels_fail_typed(self):
        guard = load_verifier()
        original = ROUTE_LOCK_PATH.read_text()
        begin = "<!-- ICRA_USER_ROUTE_LOCK_V1_BEGIN -->"
        end = "<!-- ICRA_USER_ROUTE_LOCK_V1_END -->"
        mutations = {
            "ROUTE_LOCK_SENTINEL_MISSING": original.replace(begin, "", 1),
            "ROUTE_LOCK_SENTINEL_DUPLICATE": original.replace(
                begin, begin + "\n" + begin, 1
            ),
            "ROUTE_LOCK_SENTINEL_REVERSED": original.replace(
                begin, "@@ROUTE_BEGIN@@", 1
            ).replace(end, begin, 1).replace("@@ROUTE_BEGIN@@", end, 1),
        }
        with tempfile.TemporaryDirectory(dir=REPOSITORY / "results/icra27") as temporary:
            for reason, text in mutations.items():
                path = Path(temporary) / f"{reason}.md"
                path.write_text(text)
                with self.subTest(reason=reason), self.assertRaisesRegex(
                    guard.RouteGuardError, reason
                ):
                    guard.parse_route_lock(path, REPOSITORY)

    def test_invalid_decision_types_empty_lists_and_unreachable_anchor_fail(self):
        guard = load_verifier()
        original = ROUTE_LOCK_PATH.read_text()
        mutations = {
            "ROUTE_DECISION_ID_INVALID": original.replace(
                '"user_decision_id": "USER-ICRA-ROUTE-20260826-001"',
                '"user_decision_id": "reused"', 1,
            ),
            "ROUTE_FORMAL_ARMS_INVALID": original.replace(
                '"formal_arms": [\n    "P0_P5_CONTROL",\n    "P0_P4_V2_P5_TREATMENT"\n  ]',
                '"formal_arms": []', 1,
            ),
            "ROUTE_SCENES_INVALID": original.replace(
                '"qualification_scenes": [\n    "PRIMARY",\n    "EXACT_MIRROR",\n    "FLAT_NULL"\n  ]',
                '"qualification_scenes": ["PRIMARY", "PRIMARY"]', 1,
            ),
            "ROUTE_OWNER_NOT_USER": original.replace(
                '"route_owner": "USER"', '"route_owner": 7', 1,
            ),
            "ROUTE_GUARD_STRENGTH_INVALID": original.replace(
                '"guard_strength": "ACCIDENT_PREVENTION_NOT_A_SECURITY_BOUNDARY"',
                '"guard_strength": "CRYPTOGRAPHIC_ENFORCEMENT"', 1,
            ),
            "ROUTE_APPROVAL_ANCHOR_NOT_REACHABLE": original.replace(
                "48caa9ddf24990accb65e2ad230d12821487793c",
                "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            ),
        }
        with tempfile.TemporaryDirectory(
            dir=REPOSITORY / "results/icra27"
        ) as temporary:
            for reason, text in mutations.items():
                path = Path(temporary) / f"{reason}.md"
                path.write_text(text)
                with self.subTest(reason=reason), self.assertRaisesRegex(
                    guard.RouteGuardError, reason
                ):
                    guard.parse_route_lock(path, REPOSITORY)

    def test_named_route_claim_arm_fallback_and_campaign_drifts_are_detected(self):
        guard = load_verifier()
        original = ROUTE_LOCK_PATH.read_text()
        begin = "<!-- ICRA_USER_ROUTE_LOCK_V1_BEGIN -->"
        end = "<!-- ICRA_USER_ROUTE_LOCK_V1_END -->"
        payload = original.split(begin, 1)[1].split(end, 1)[0].strip()
        document = json.loads(payload[7:-3])
        mutations = {
            "active_route": "P0_P5_CONTINGENCY",
            "required_modules": [
                value for value in document["required_modules"] if "P4" not in value
            ],
            "primary_claim": "Mean/CVaR numerical noise only.",
            "formal_arms": ["P0_P5_CONTROL"],
            "fallback_policy": "SUPERVISOR_CONTINGENCY_ACTIVATION",
            "campaign_activation": "RUNNER_AUTOMATIC_ACTIVATION",
        }
        for field, value in mutations.items():
            changed = json.loads(json.dumps(document))
            changed[field] = value
            after = original.replace(
                payload,
                "```json\n" + json.dumps(changed, indent=2) + "\n```",
                1,
            )
            with self.subTest(field=field):
                self.assertEqual(
                    guard.protected_route_fields_changed(original, after),
                    frozenset({field}),
                )


class RouteStateVerifierTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(
            dir=REPOSITORY / "results/icra27"
        )
        self.root = Path(self.temporary.name)
        self.paths = {}
        for name, source in {
            "route": ROUTE_LOCK_PATH,
            "state": REPOSITORY / "AGENT_STATE.md",
            "task": REPOSITORY / "NEXT_TASK.md",
            "scope": REPOSITORY / "docs/icra27/ICRA_SCOPE.md",
            "plan": REPOSITORY / "docs/icra27/ICRA_IMPLEMENTATION_PLAN.md",
            "review": REPOSITORY / "docs/icra27/ICRA_PLAN_REVIEW.md",
        }.items():
            target = self.root / source.name
            shutil.copy2(source, target)
            self.paths[name] = target

    def tearDown(self):
        self.temporary.cleanup()

    def run_guard(self):
        return subprocess.run(
            [
                "python3", str(VERIFIER_PATH),
                "--repository", str(REPOSITORY),
                "--route-lock", str(self.paths["route"]),
                "--state", str(self.paths["state"]),
                "--task", str(self.paths["task"]),
                "--scope", str(self.paths["scope"]),
                "--plan", str(self.paths["plan"]),
                "--plan-review", str(self.paths["review"]),
            ],
            text=True,
            capture_output=True,
            check=False,
            env={"PATH": str(Path("/usr/bin")), "PYTHONDONTWRITEBYTECODE": "1"},
        )

    def mutate(self, name, old, new):
        path = self.paths[name]
        text = path.read_text()
        self.assertIn(old, text)
        path.write_text(text.replace(old, new, 1))

    def test_current_authority_set_passes_as_consistency_not_authentication(self):
        result = self.run_guard()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            result.stdout.strip(),
            "ICRA_ROUTE_GUARD_PASS:REPOSITORY_CONSISTENT_NOT_USER_AUTHENTICATION",
        )

    def test_state_route_p4_verdict_and_campaign_drift_fail_typed(self):
        mutations = (
            (
                "conference_route: P0_P4_V2_P5",
                "conference_route: P0_P5_CONTINGENCY",
                "STATE_ACTIVE_ROUTE_MISMATCH",
            ),
            (
                "p4_v1_status: G0A_PASS_G0B_PASS_G0C_SCIENTIFIC_NO_GO_IMMUTABLE",
                "p4_v1_status: G0A_PASS_G0B_PASS_G0C_PASS",
                "P4_V1_NO_GO_RELABELLED",
            ),
            (
                "campaign_status: BLOCKED_UNTIL_ICRA079_REVIEW_PASS_AND_DISTINCT_USER_APPROVAL",
                "campaign_status: ACTIVE_BY_SUPERVISOR_VERDICT",
                "CAMPAIGN_BARRIER_INVALID",
            ),
        )
        original = self.paths["state"].read_text()
        for old, new, reason in mutations:
            self.paths["state"].write_text(original.replace(old, new, 1))
            result = self.run_guard()
            with self.subTest(reason=reason):
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(f"ICRA_ROUTE_GUARD_FAIL:{reason}", result.stderr)

    def test_task_and_active_scope_plan_disagreement_fail_typed(self):
        self.mutate(
            "task",
            "> Active gate: `USER_RESEARCH_ROUTE_AUTHORITY_GUARD`",
            "> Active gate: `P4_V2_PRODUCT_DEVELOPMENT`",
        )
        result = self.run_guard()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("ICRA_ROUTE_GUARD_FAIL:TASK_GATE_MISMATCH", result.stderr)

        shutil.copy2(REPOSITORY / "NEXT_TASK.md", self.paths["task"])
        self.mutate("scope", "`P0_P4_V2_P5`", "`P0_P5_CONTINGENCY`")
        result = self.run_guard()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(
            "ICRA_ROUTE_GUARD_FAIL:ACTIVE_DOCUMENT_ROUTE_MISMATCH",
            result.stderr,
        )

    def test_active_documents_must_reference_canonical_decision_source(self):
        self.mutate(
            "scope",
            "ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md",
            "UNBOUND_ROUTE_PROPOSAL.md",
        )
        result = self.run_guard()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(
            "ACTIVE_DOCUMENT_AUTHORITY_MISMATCH:scope", result.stderr
        )

    def test_documentation_only_historical_route_text_does_not_activate_it(self):
        with self.paths["scope"].open("a") as stream:
            stream.write(
                "\n## Historical appendix\nconference_route: P0_P5_CONTINGENCY\n"
                "campaign_status: ACTIVE\n"
            )
        result = self.run_guard()
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_required_scientific_no_go_blocked_state_is_valid(self):
        state = self.paths["state"].read_text()
        state = state.replace("active_role: DEEPSEEK", "active_role: SUPERVISOR", 1)
        state = state.replace("status: TASK_READY", "status: BLOCKED_AWAITING_USER_RESEARCH_DECISION", 1)
        state = state.replace("task_id: ICRA-071", "task_id: NONE", 1)
        state = state.replace("next_task: NEXT_TASK.md", "next_task: NONE", 1)
        self.paths["state"].write_text(state)
        self.paths["task"].write_text(
            "# NONE — No active ICRA task\n\n"
            "> Active gate: `USER_RESEARCH_ROUTE_AUTHORITY_GUARD`\n"
            "> Owner: `SUPERVISOR`\n"
            "> Activation: `BLOCKED_AWAITING_USER_RESEARCH_DECISION`\n"
            "> User decision: `USER-ICRA-ROUTE-20260826-001`\n"
        )
        result = self.run_guard()
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_duplicate_json_key_and_unknown_schema_field_fail_typed(self):
        guard = load_verifier()
        original = ROUTE_LOCK_PATH.read_text()
        mutations = {
            "ROUTE_LOCK_DUPLICATE_JSON_KEY": original.replace(
                '  "route_owner": "USER",',
                '  "route_owner": "USER",\n  "route_owner": "USER",',
                1,
            ),
            "ROUTE_LOCK_SCHEMA_FIELDS_INVALID": original.replace(
                '  "route_owner": "USER",',
                '  "route_owner": "USER",\n  "unexpected": true,',
                1,
            ),
        }
        with tempfile.TemporaryDirectory(dir=REPOSITORY / "results/icra27") as temporary:
            for reason, text in mutations.items():
                path = Path(temporary) / f"{reason}.md"
                path.write_text(text)
                with self.subTest(reason=reason), self.assertRaisesRegex(
                    guard.RouteGuardError, reason
                ):
                    guard.parse_route_lock(path, REPOSITORY)

    def test_owner_anchor_and_duplicate_lists_fail_typed(self):
        guard = load_verifier()
        original = ROUTE_LOCK_PATH.read_text()
        mutations = {
            "ROUTE_OWNER_NOT_USER": original.replace(
                '"route_owner": "USER"', '"route_owner": "SUPERVISOR"', 1
            ),
            "ROUTE_APPROVAL_ANCHOR_INVALID": original.replace(
                '"approval_anchor": "48caa9ddf24990accb65e2ad230d12821487793c"',
                '"approval_anchor": "not-a-commit"',
                1,
            ),
            "ROUTE_REQUIRED_MODULES_DUPLICATE": original.replace(
                '"P4_V2_COLLISION_GUIDE",',
                '"P4_V2_COLLISION_GUIDE",\n    "P4_V2_COLLISION_GUIDE",',
                1,
            ),
        }
        with tempfile.TemporaryDirectory(dir=REPOSITORY / "results/icra27") as temporary:
            for reason, text in mutations.items():
                path = Path(temporary) / f"{reason}.md"
                path.write_text(text)
                with self.subTest(reason=reason), self.assertRaisesRegex(
                    guard.RouteGuardError, reason
                ):
                    guard.parse_route_lock(path, REPOSITORY)


class RepositoryHookTest(unittest.TestCase):
    def test_all_code_roots_and_extension_fallback_require_three_docs(self):
        guard = load_verifier()
        roots = (
            "src", "include", "apps", "msg", "cmake", "launch", "config",
            "scripts", "test", "tests", "tools", "docker", "data",
            "thirdparty", ".githooks",
        )
        for root in roots:
            with self.subTest(root=root):
                self.assertTrue(guard.requires_development_docs(f"{root}/change.bin"))
        self.assertTrue(guard.requires_development_docs("new_source_root/module.py"))
        self.assertTrue(guard.requires_development_docs("CMakeLists.txt"))
        self.assertTrue(guard.requires_development_docs("package.xml"))
        self.assertFalse(guard.requires_development_docs("results/run/module.py"))
        self.assertFalse(guard.requires_development_docs("docs/notes.md"))

    def test_commit_message_requires_requirement_id(self):
        with tempfile.TemporaryDirectory(
            dir=REPOSITORY / "results/icra27"
        ) as temporary:
            message = Path(temporary) / "message.txt"
            message.write_text("feat(icra): guard route\n")
            rejected = subprocess.run(
                ["python3", str(VERIFIER_PATH), "--commit-message", str(message)],
                text=True, capture_output=True, check=False,
            )
            self.assertNotEqual(rejected.returncode, 0)
            self.assertIn("COMMIT_MESSAGE_REQUIREMENT_ID_MISSING", rejected.stderr)
            message.write_text("feat(icra): guard route IAP-RQ-424\n")
            accepted = subprocess.run(
                ["python3", str(VERIFIER_PATH), "--commit-message", str(message)],
                text=True, capture_output=True, check=False,
            )
            self.assertEqual(accepted.returncode, 0, accepted.stderr)

    def test_hook_wrappers_share_verifier_and_have_no_bypass(self):
        for name, mode in (
            ("pre-commit", "pre-commit"),
            ("pre-push", "pre-push"),
            ("commit-msg", "commit-message"),
        ):
            path = REPOSITORY / ".githooks" / name
            with self.subTest(name=name):
                text = path.read_text()
                self.assertIn("verify_icra_research_route.py", text)
                self.assertIn(mode, text)
                self.assertNotIn("IAP_SKIP_DOCS", text)
                self.assertTrue(os.access(path, os.X_OK))

    def test_hooks_path_install_check_reject_and_second_install(self):
        with tempfile.TemporaryDirectory(
            dir=REPOSITORY / "results/icra27"
        ) as temporary:
            repository = Path(temporary)
            subprocess.run(["git", "init", "-q", str(repository)], check=True)
            home = repository / "home"
            home.mkdir()
            environment = {"PATH": os.environ["PATH"], "HOME": str(home)}

            def invoke(mode):
                return subprocess.run(
                    ["python3", str(VERIFIER_PATH), "--repository", str(repository), mode],
                    text=True, capture_output=True, check=False, env=environment,
                )

            missing = invoke("--check-hooks")
            self.assertNotEqual(missing.returncode, 0)
            self.assertIn("HOOKS_PATH_MISSING", missing.stderr)
            subprocess.run(
                ["git", "-C", str(repository), "config", "--local",
                 "core.hooksPath", "/tmp/stale-hooks"],
                check=True, env=environment,
            )
            absolute = invoke("--check-hooks")
            self.assertNotEqual(absolute.returncode, 0)
            self.assertIn("HOOKS_PATH_ABSOLUTE", absolute.stderr)
            subprocess.run(
                ["git", "-C", str(repository), "config", "--local",
                 "core.hooksPath", "old-hooks"],
                check=True, env=environment,
            )
            stale = invoke("--check-hooks")
            self.assertNotEqual(stale.returncode, 0)
            self.assertIn("HOOKS_PATH_STALE", stale.stderr)
            first = invoke("--install-hooks")
            second = invoke("--install-hooks")
            checked = invoke("--check-hooks")
            self.assertEqual(first.returncode, 0, first.stderr)
            self.assertEqual(second.returncode, 0, second.stderr)
            self.assertEqual(checked.returncode, 0, checked.stderr)
            value = subprocess.check_output(
                ["git", "-C", str(repository), "config", "--local", "--get",
                 "core.hooksPath"],
                text=True, env=environment,
            ).strip()
            self.assertEqual(value, ".githooks")
            global_value = subprocess.run(
                ["git", "config", "--global", "--get", "core.hooksPath"],
                text=True, capture_output=True, check=False, env=environment,
            )
            self.assertNotEqual(global_value.returncode, 0)


class StagedRepositoryGuardTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(
            dir=REPOSITORY / "results/icra27"
        )
        self.repository = Path(self.temporary.name)
        self.environment = {
            "PATH": os.environ["PATH"],
            "HOME": str(self.repository / "home"),
            "PYTHONDONTWRITEBYTECODE": "1",
        }
        (self.repository / "home").mkdir()
        self.git("init", "-q")
        self.git("config", "user.email", "guard@example.invalid")
        self.git("config", "user.name", "Route Guard Test")
        (self.repository / "anchor.txt").write_text("pre-decision anchor\n")
        self.git("add", "anchor.txt")
        self.git("commit", "-q", "-m", "anchor IAP-RQ-424")
        self.anchor = self.git("rev-parse", "HEAD").stdout.strip()
        sources = {
            "AGENT_STATE.md": REPOSITORY / "AGENT_STATE.md",
            "NEXT_TASK.md": REPOSITORY / "NEXT_TASK.md",
            "SUPERVISOR_LOG.md": REPOSITORY / "SUPERVISOR_LOG.md",
            "DEV_LOG.md": REPOSITORY / "DEV_LOG.md",
            "docs/CHANGES.md": REPOSITORY / "docs/CHANGES.md",
            "docs/TRACEABILITY.md": REPOSITORY / "docs/TRACEABILITY.md",
            "docs/REQS.md": REPOSITORY / "docs/REQS.md",
            "docs/icra27/ICRA_SCOPE.md": REPOSITORY / "docs/icra27/ICRA_SCOPE.md",
            "docs/icra27/ICRA_IMPLEMENTATION_PLAN.md": (
                REPOSITORY / "docs/icra27/ICRA_IMPLEMENTATION_PLAN.md"
            ),
            "docs/icra27/ICRA_PLAN_REVIEW.md": (
                REPOSITORY / "docs/icra27/ICRA_PLAN_REVIEW.md"
            ),
            "docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md": (
                ROUTE_LOCK_PATH
            ),
            "scripts/dev_planner/verify_icra_research_route.py": VERIFIER_PATH,
            ".githooks/pre-commit": REPOSITORY / ".githooks/pre-commit",
            ".githooks/pre-push": REPOSITORY / ".githooks/pre-push",
            ".githooks/commit-msg": REPOSITORY / ".githooks/commit-msg",
        }
        for relative, source in sources.items():
            target = self.repository / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            text = source.read_text().replace(
                "48caa9ddf24990accb65e2ad230d12821487793c", self.anchor
            )
            target.write_text(text)
            target.chmod(source.stat().st_mode & 0o777)
        historical_product = self.repository / "src/p4/historical.cpp"
        historical_product.parent.mkdir(parents=True, exist_ok=True)
        historical_product.write_text("int historical = 1;\n")
        self.git("add", *sources, "src/p4/historical.cpp")
        self.git("commit", "-q", "-m", "fixture route decision IAP-RQ-424")
        installed = self.invoke_verifier("--install-hooks")
        self.assertEqual(installed.returncode, 0, installed.stderr)

    def tearDown(self):
        self.temporary.cleanup()

    def git(self, *arguments):
        return subprocess.run(
            ["git", "-C", str(self.repository), *arguments],
            text=True, capture_output=True, check=True, env=self.environment,
        )

    def invoke_verifier(self, *arguments):
        return subprocess.run(
            [
                "python3",
                str(self.repository / "scripts/dev_planner/verify_icra_research_route.py"),
                "--repository", str(self.repository), *arguments,
            ],
            text=True, capture_output=True, check=False, env=self.environment,
        )

    def append_and_stage(self, relative, text="\nICRA-071 test mutation\n"):
        path = self.repository / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a") as stream:
            stream.write(text)
        self.git("add", relative)

    def make_supervisor_blocked_head(self):
        state_path = self.repository / "AGENT_STATE.md"
        state = state_path.read_text()
        state = state.replace("active_role: DEEPSEEK", "active_role: SUPERVISOR", 1)
        state = state.replace("status: TASK_READY", "status: BLOCKED_AWAITING_USER_RESEARCH_DECISION", 1)
        state = state.replace("task_id: ICRA-071", "task_id: NONE", 1)
        state = state.replace("next_task: NEXT_TASK.md", "next_task: NONE", 1)
        state_path.write_text(state)
        (self.repository / "NEXT_TASK.md").write_text(
            "# NONE — No active ICRA task\n\n"
            "> Active gate: `USER_RESEARCH_ROUTE_AUTHORITY_GUARD`\n"
            "> Owner: `SUPERVISOR`\n"
            "> Activation: `BLOCKED_AWAITING_USER_RESEARCH_DECISION`\n"
            "> User decision: `USER-ICRA-ROUTE-20260826-001`\n"
        )
        self.git("add", "AGENT_STATE.md", "NEXT_TASK.md")
        self.git("config", "core.hooksPath", ".disabled-hooks")
        self.git("commit", "-q", "-m", "supervisor blocked IAP-RQ-424")
        self.git("config", "core.hooksPath", ".githooks")

    def stage_route_claim_decision(self, decision_id, approval_anchor):
        relative = (
            "docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md"
        )
        path = self.repository / relative
        text = path.read_text()
        begin = "<!-- ICRA_USER_ROUTE_LOCK_V1_BEGIN -->"
        end = "<!-- ICRA_USER_ROUTE_LOCK_V1_END -->"
        prefix, remainder = text.split(begin, 1)
        payload, suffix = remainder.split(end, 1)
        document = json.loads(payload.strip()[7:-3])
        old_claim = document["primary_claim"]
        new_claim = old_claim + " Synthetic proposal only."
        document["primary_claim"] = new_claim
        document["user_decision_id"] = decision_id
        document["approval_anchor"] = approval_anchor
        document["protected_transition"]["from_anchor"] = approval_anchor
        for row in document["protected_transition"]["changes"]:
            if row["field"] == "primary_claim":
                row["old"] = old_claim
                row["new"] = new_claim
        path.write_text(
            prefix + begin + "\n```json\n"
            + json.dumps(document, indent=2) + "\n```\n" + end + suffix
        )
        self.git("add", relative)

    def stage_all_route_transition_documents(self):
        for relative in (
            "AGENT_STATE.md", "NEXT_TASK.md", "SUPERVISOR_LOG.md",
            "docs/CHANGES.md", "docs/TRACEABILITY.md", "docs/REQS.md",
            "docs/icra27/ICRA_SCOPE.md",
            "docs/icra27/ICRA_IMPLEMENTATION_PLAN.md",
            "docs/icra27/ICRA_PLAN_REVIEW.md",
        ):
            self.append_and_stage(relative, "\nSynthetic synchronized proposal.\n")

    def test_pre_commit_requires_all_three_development_documents(self):
        self.append_and_stage(
            "scripts/dev_planner/verify_icra_research_route.py",
            "\n# focused guard fixture mutation\n",
        )
        rejected = self.invoke_verifier("--hook", "pre-commit")
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn("DEVELOPMENT_DOCS_NOT_STAGED", rejected.stderr)
        for document in REQUIRED_DEVELOPMENT_DOCS_FOR_TEST:
            self.append_and_stage(document)
        accepted = self.invoke_verifier("--hook", "pre-commit")
        self.assertEqual(accepted.returncode, 0, accepted.stderr)

    def test_icra071_rejects_product_change_even_with_all_documents(self):
        self.append_and_stage("src/p4/collision_guide.cpp", "int changed = 1;\n")
        for document in REQUIRED_DEVELOPMENT_DOCS_FOR_TEST:
            self.append_and_stage(document)
        result = self.invoke_verifier("--hook", "pre-commit")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("ICRA071_PRODUCT_CHANGE_FORBIDDEN", result.stderr)

    def test_icra071_rejects_product_deletion_even_with_all_documents(self):
        path = self.repository / "src/p4/historical.cpp"
        path.unlink()
        self.git("add", "-u", "src/p4/historical.cpp")
        for document in REQUIRED_DEVELOPMENT_DOCS_FOR_TEST:
            self.append_and_stage(document)
        result = self.invoke_verifier("--hook", "pre-commit")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("ICRA071_PRODUCT_CHANGE_FORBIDDEN", result.stderr)

    def test_icra071_rejects_hook_deletion_with_valid_worktree_copy(self):
        path = self.repository / ".githooks/pre-push"
        original = path.read_text()
        mode = path.stat().st_mode & 0o777
        path.unlink()
        self.git("add", "-u", ".githooks/pre-push")
        path.write_text(original)
        path.chmod(mode)
        for document in REQUIRED_DEVELOPMENT_DOCS_FOR_TEST:
            self.append_and_stage(document)
        result = self.invoke_verifier("--hook", "pre-commit")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("ICRA071_GUARD_FILE_DELETION_FORBIDDEN", result.stderr)

    def test_verifier_change_cannot_delete_required_development_documents(self):
        self.append_and_stage(
            "scripts/dev_planner/verify_icra_research_route.py",
            "\n# allowed verifier change\n",
        )
        for document in REQUIRED_DEVELOPMENT_DOCS_FOR_TEST:
            (self.repository / document).unlink()
            self.git("add", "-u", document)
        result = self.invoke_verifier("--hook", "pre-commit")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("ICRA071_GUARD_FILE_DELETION_FORBIDDEN", result.stderr)

    def test_hook_rename_cannot_hide_essential_source_deletion(self):
        source = self.repository / ".githooks/pre-push"
        destination = self.repository / "results/icra27/icra071/pre-push-copy"
        destination.parent.mkdir(parents=True, exist_ok=True)
        source.rename(destination)
        self.git("add", "-A", ".githooks/pre-push", "results/icra27/icra071")
        for document in REQUIRED_DEVELOPMENT_DOCS_FOR_TEST:
            self.append_and_stage(document)
        result = self.invoke_verifier("--hook", "pre-commit")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("ICRA071_GUARD_FILE_DELETION_FORBIDDEN", result.stderr)

    def test_product_rename_into_evidence_namespace_remains_forbidden(self):
        source = self.repository / "src/p4/historical.cpp"
        destination = self.repository / "results/icra27/icra071/historical.cpp"
        destination.parent.mkdir(parents=True, exist_ok=True)
        source.rename(destination)
        self.git("add", "-A", "src/p4/historical.cpp", "results/icra27/icra071")
        for document in REQUIRED_DEVELOPMENT_DOCS_FOR_TEST:
            self.append_and_stage(document)
        result = self.invoke_verifier("--hook", "pre-commit")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("ICRA071_PRODUCT_CHANGE_FORBIDDEN", result.stderr)

    def test_builder_cannot_stage_supervisor_authority(self):
        self.append_and_stage("AGENT_STATE.md")
        state = self.invoke_verifier("--hook", "pre-commit")
        self.assertNotEqual(state.returncode, 0)
        self.assertIn("BUILDER_SUPERVISOR_FILE_STAGED", state.stderr)

    def test_builder_cannot_stage_route_lock_authority(self):
        roadmap = self.repository / (
            "docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md"
        )
        text = roadmap.read_text().replace(
            '"primary_claim": "P4-v2 lowers',
            '"primary_claim": "P4-v2 weakens',
            1,
        )
        roadmap.write_text(text)
        self.git("add", str(roadmap.relative_to(self.repository)))
        route = self.invoke_verifier("--hook", "pre-commit")
        self.assertNotEqual(route.returncode, 0)
        self.assertIn("BUILDER_ROUTE_LOCK_STAGED", route.stderr)

    def test_pre_push_rejects_worktree_route_drift(self):
        state = self.repository / "AGENT_STATE.md"
        state.write_text(state.read_text().replace(
            "conference_route: P0_P4_V2_P5",
            "conference_route: P0_P5_CONTINGENCY",
            1,
        ))
        result = self.invoke_verifier("--hook", "pre-push")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("PUSH_TRACKED_WORKTREE_NOT_CLEAN", result.stderr)

    def test_pre_commit_rejects_index_worktree_authority_split(self):
        path = self.repository / "docs/icra27/ICRA_SCOPE.md"
        original = path.read_text()
        path.write_text(original.replace("`P0_P4_V2_P5`", "`P0_P5_CONTINGENCY`", 1))
        self.git("add", "docs/icra27/ICRA_SCOPE.md")
        path.write_text(original)
        result = self.invoke_verifier("--hook", "pre-commit")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("STAGED_AUTHORITY_WORKTREE_MISMATCH", result.stderr)

    def test_pre_push_binds_actual_outbound_ref_to_checked_out_head(self):
        result = subprocess.run(
            [
                "python3",
                str(self.repository / "scripts/dev_planner/verify_icra_research_route.py"),
                "--repository", str(self.repository), "--hook", "pre-push",
            ],
            input=(
                f"refs/heads/dev/icra {self.anchor} "
                f"refs/heads/dev/icra {'0' * 40}\n"
            ),
            text=True, capture_output=True, check=False, env=self.environment,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("PUSHED_COMMIT_NOT_CHECKED_OUT_HEAD", result.stderr)

    def test_supervisor_historical_proposal_does_not_activate_route(self):
        self.make_supervisor_blocked_head()
        self.append_and_stage(
            "SUPERVISOR_LOG.md",
            "\nSynthetic proposal: P0_P5_CONTINGENCY; not activated.\n",
        )
        result = self.invoke_verifier("--hook", "pre-commit")
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_supervisor_route_change_requires_full_document_sync(self):
        self.make_supervisor_blocked_head()
        head = self.git("rev-parse", "HEAD").stdout.strip()
        self.stage_route_claim_decision("USER-ICRA-ROUTE-20260826-999", head)
        result = self.invoke_verifier("--hook", "pre-commit")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("ROUTE_CHANGE_DOCUMENT_SYNC_MISSING", result.stderr)

    def test_supervisor_route_change_rejects_reused_decision_and_stale_anchor(self):
        self.make_supervisor_blocked_head()
        head = self.git("rev-parse", "HEAD").stdout.strip()
        self.stage_all_route_transition_documents()
        self.stage_route_claim_decision("USER-ICRA-ROUTE-20260826-001", head)
        reused = self.invoke_verifier("--hook", "pre-commit")
        self.assertNotEqual(reused.returncode, 0)
        self.assertIn("ROUTE_CHANGE_DECISION_ID_NOT_DISTINCT", reused.stderr)

    def test_supervisor_route_change_rejects_stale_approval_anchor(self):
        self.make_supervisor_blocked_head()
        self.stage_all_route_transition_documents()
        self.stage_route_claim_decision(
            "USER-ICRA-ROUTE-20260826-999", self.anchor
        )
        stale = self.invoke_verifier("--hook", "pre-commit")
        self.assertNotEqual(stale.returncode, 0)
        self.assertIn("ROUTE_CHANGE_APPROVAL_ANCHOR_STALE", stale.stderr)

    def test_no_go_blocked_state_cannot_activate_alternate_task_without_decision(self):
        state_path = self.repository / "AGENT_STATE.md"
        state = state_path.read_text()
        state = state.replace("active_role: DEEPSEEK", "active_role: SUPERVISOR", 1)
        state = state.replace("status: TASK_READY", "status: BLOCKED_AWAITING_USER_RESEARCH_DECISION", 1)
        state = state.replace("next_task: NEXT_TASK.md", "next_task: NONE", 1)
        state_path.write_text(state)
        self.git("add", "AGENT_STATE.md")
        self.git("config", "core.hooksPath", ".disabled-hooks")
        self.git("commit", "-q", "-m", "blocked no-go IAP-RQ-424")
        self.git("config", "core.hooksPath", ".githooks")
        state = state_path.read_text()
        state = state.replace("active_role: SUPERVISOR", "active_role: DEEPSEEK", 1)
        state = state.replace("status: BLOCKED_AWAITING_USER_RESEARCH_DECISION", "status: TASK_READY", 1)
        state = state.replace("next_task: NONE", "next_task: NEXT_TASK.md", 1)
        state_path.write_text(state)
        self.git("add", "AGENT_STATE.md")
        result = self.invoke_verifier("--hook", "pre-commit")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(
            "NO_GO_ALTERNATE_TASK_WITHOUT_USER_DECISION", result.stderr
        )


if __name__ == "__main__":
    unittest.main()
