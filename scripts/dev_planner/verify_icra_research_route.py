#!/usr/bin/env python3
"""Repository-local ICRA research-route consistency guard.

This guard prevents accidental repository drift. It is not a security boundary
and does not authenticate a same-permission process as the human route owner.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Any, NamedTuple


ROUTE_BEGIN = "<!-- ICRA_USER_ROUTE_LOCK_V1_BEGIN -->"
ROUTE_END = "<!-- ICRA_USER_ROUTE_LOCK_V1_END -->"
ROUTE_SCHEMA = "icra_user_route_lock_v1"
SUPPORTED_GUARD_STRENGTH = "ACCIDENT_PREVENTION_NOT_A_SECURITY_BOUNDARY"
ROUTE_FIELDS = {
    "schema_version", "route_owner", "active_route", "required_modules",
    "research_question", "primary_claim", "secondary_claims", "formal_arms",
    "qualification_scenes", "gate_sequence", "fallback_policy",
    "scientific_no_go_transition", "campaign_activation", "approval_anchor",
    "user_decision_id", "protected_transition", "user_decision",
    "guard_strength",
}
PROTECTED_ROUTE_FIELDS = {
    "route_owner", "active_route", "required_modules", "research_question",
    "primary_claim", "secondary_claims", "formal_arms",
    "qualification_scenes", "gate_sequence", "fallback_policy",
    "scientific_no_go_transition", "campaign_activation",
}
USER_DECISION_FIELDS = {
    "route_disposition", "p0_p5_disposition", "p4_primary_endpoint",
    "confirmatory_size", "enforcement",
}
CODE_ROOTS = {
    "src", "include", "apps", "msg", "cmake", "launch", "config",
    "scripts", "test", "tests", "tools", "docker", "data", "thirdparty",
    ".githooks",
}
SOURCE_FALLBACK_SUFFIXES = {
    ".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx", ".py",
    ".pyi", ".cmake", ".sh", ".bash", ".yaml", ".yml", ".json", ".xml",
    ".proto", ".msg", ".srv", ".action", ".toml",
}
GENERATED_OR_EVIDENCE_ROOTS = {
    ".git", "results", "build", "install", "log", "logs", "raw", "bags",
}
REQUIRED_DEVELOPMENT_DOCS = {
    "DEV_LOG.md", "docs/CHANGES.md", "docs/TRACEABILITY.md",
}
ROUTE_TRANSITION_DOCUMENTS = {
    "AGENT_STATE.md", "NEXT_TASK.md", "SUPERVISOR_LOG.md",
    "docs/CHANGES.md", "docs/TRACEABILITY.md", "docs/REQS.md",
    "docs/icra27/ICRA_SCOPE.md", "docs/icra27/ICRA_IMPLEMENTATION_PLAN.md",
    "docs/icra27/ICRA_PLAN_REVIEW.md",
    "docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md",
}
AUTHORITY_INPUT_PATHS = {
    "AGENT_STATE.md", "NEXT_TASK.md",
    "docs/icra27/ICRA_SCOPE.md", "docs/icra27/ICRA_IMPLEMENTATION_PLAN.md",
    "docs/icra27/ICRA_PLAN_REVIEW.md",
    "docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md",
}
ICRA071_ALLOWED_STAGED_FILES = {
    ".githooks/pre-commit", ".githooks/pre-push", ".githooks/commit-msg",
    "scripts/dev_planner/verify_icra_research_route.py",
    "test/test_verify_icra_research_route.py",
    "DEV_LOG.md", "docs/CHANGES.md", "docs/TRACEABILITY.md",
}


class RouteGuardError(RuntimeError):
    """A stable typed repository guard failure."""

    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


class RouteLock(NamedTuple):
    schema_version: str
    route_owner: str
    active_route: str
    required_modules: tuple[str, ...]
    research_question: str
    primary_claim: str
    secondary_claims: tuple[str, ...]
    formal_arms: tuple[str, ...]
    qualification_scenes: tuple[str, ...]
    gate_sequence: tuple[str, ...]
    fallback_policy: str
    scientific_no_go_transition: str
    campaign_activation: str
    approval_anchor: str
    user_decision_id: str
    protected_transition: tuple[tuple[str, Any, Any], ...]
    user_decision: tuple[tuple[str, str], ...]
    guard_strength: str


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise RouteGuardError("ROUTE_LOCK_DUPLICATE_JSON_KEY")
        result[key] = value
    return result


def _nonempty_string(value: Any, reason: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise RouteGuardError(reason)
    return value


def _unique_string_tuple(value: Any, reason: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not value:
        raise RouteGuardError(reason)
    observed = tuple(_nonempty_string(item, reason) for item in value)
    if len(set(observed)) != len(observed):
        raise RouteGuardError(reason)
    return observed


def _git(repository: Path, *arguments: str) -> subprocess.CompletedProcess[str]:
    canonical = repository.resolve(strict=True)
    return subprocess.run(
        ["git", "-c", f"safe.directory={canonical}", "-C", str(canonical),
         *arguments],
        text=True,
        capture_output=True,
        check=False,
    )


def requires_development_docs(relative_path: str) -> bool:
    """Return whether a staged path is code/interface/config-like."""
    path = Path(relative_path)
    if path.is_absolute() or not path.parts:
        return False
    if path.parts[0] in GENERATED_OR_EVIDENCE_ROOTS or path.parts[0] == "docs":
        return False
    if "__pycache__" in path.parts or path.suffix in {".pyc", ".pyo", ".pyd"}:
        return False
    if path.parts[0] in CODE_ROOTS:
        return True
    if len(path.parts) == 1 and (
        path.name in {"CMakeLists.txt", "package.xml"}
        or path.suffix.lower() in {".cmake", ".toolchain", ".toml", ".yaml", ".yml"}
    ):
        return True
    return path.suffix.lower() in SOURCE_FALLBACK_SUFFIXES


def _hooks_path_value(repository: Path) -> str | None:
    result = _git(repository, "config", "--local", "--get", "core.hooksPath")
    if result.returncode == 1:
        return None
    if result.returncode != 0:
        raise RouteGuardError("HOOKS_PATH_QUERY_FAILED")
    return result.stdout.strip()


def check_hooks_path(repository: Path) -> None:
    value = _hooks_path_value(repository)
    if value is None:
        raise RouteGuardError("HOOKS_PATH_MISSING")
    if Path(value).is_absolute():
        raise RouteGuardError("HOOKS_PATH_ABSOLUTE")
    if value != ".githooks":
        raise RouteGuardError("HOOKS_PATH_STALE")


def install_hooks_path(repository: Path) -> None:
    result = _git(
        repository, "config", "--local", "core.hooksPath", ".githooks"
    )
    if result.returncode != 0:
        raise RouteGuardError("HOOKS_PATH_INSTALL_FAILED")
    check_hooks_path(repository)


def validate_commit_message(path: Path) -> None:
    message = Path(path).read_text()
    if re.search(r"\bIAP-RQ-\d{3}\b", message) is None:
        raise RouteGuardError("COMMIT_MESSAGE_REQUIREMENT_ID_MISSING")


def _require_reachable_anchor(repository: Path, anchor: str) -> None:
    exists = _git(repository, "cat-file", "-e", f"{anchor}^{{commit}}")
    reachable = _git(repository, "merge-base", "--is-ancestor", anchor, "HEAD")
    if exists.returncode != 0 or reachable.returncode != 0:
        raise RouteGuardError("ROUTE_APPROVAL_ANCHOR_NOT_REACHABLE")


def parse_route_lock_text(text: str, repository: Path) -> RouteLock:
    """Parse canonical route-lock text and bind its approval history."""
    begin_positions = [match.start() for match in re.finditer(re.escape(ROUTE_BEGIN), text)]
    end_positions = [match.start() for match in re.finditer(re.escape(ROUTE_END), text)]
    if not begin_positions or not end_positions:
        raise RouteGuardError("ROUTE_LOCK_SENTINEL_MISSING")
    if len(begin_positions) != 1 or len(end_positions) != 1:
        raise RouteGuardError("ROUTE_LOCK_SENTINEL_DUPLICATE")
    if begin_positions[0] >= end_positions[0]:
        raise RouteGuardError("ROUTE_LOCK_SENTINEL_REVERSED")
    payload = text[
        begin_positions[0] + len(ROUTE_BEGIN):end_positions[0]
    ].strip()
    fence = re.fullmatch(r"```json\s*\n(?P<body>.*)\n```", payload, re.DOTALL)
    if fence is None:
        raise RouteGuardError("ROUTE_LOCK_JSON_ENVELOPE_INVALID")
    try:
        document = json.loads(fence.group("body"), object_pairs_hook=_unique_object)
    except RouteGuardError:
        raise
    except json.JSONDecodeError as exc:
        raise RouteGuardError("ROUTE_LOCK_JSON_INVALID") from exc
    if not isinstance(document, dict) or set(document) != ROUTE_FIELDS:
        raise RouteGuardError("ROUTE_LOCK_SCHEMA_FIELDS_INVALID")
    if document["schema_version"] != ROUTE_SCHEMA:
        raise RouteGuardError("ROUTE_LOCK_SCHEMA_VERSION_INVALID")
    if document["route_owner"] != "USER":
        raise RouteGuardError("ROUTE_OWNER_NOT_USER")
    active_route = _nonempty_string(document["active_route"], "ROUTE_ACTIVE_INVALID")
    research_question = _nonempty_string(
        document["research_question"], "ROUTE_RESEARCH_QUESTION_INVALID"
    )
    primary_claim = _nonempty_string(
        document["primary_claim"], "ROUTE_PRIMARY_CLAIM_INVALID"
    )
    required_modules = _unique_string_tuple(
        document["required_modules"], "ROUTE_REQUIRED_MODULES_DUPLICATE"
    )
    secondary_claims = _unique_string_tuple(
        document["secondary_claims"], "ROUTE_SECONDARY_CLAIMS_INVALID"
    )
    formal_arms = _unique_string_tuple(
        document["formal_arms"], "ROUTE_FORMAL_ARMS_INVALID"
    )
    qualification_scenes = _unique_string_tuple(
        document["qualification_scenes"], "ROUTE_SCENES_INVALID"
    )
    gate_sequence = _unique_string_tuple(
        document["gate_sequence"], "ROUTE_GATE_SEQUENCE_INVALID"
    )
    for field in (
        "fallback_policy", "scientific_no_go_transition", "campaign_activation",
    ):
        _nonempty_string(document[field], f"ROUTE_{field.upper()}_INVALID")
    anchor = document["approval_anchor"]
    if not isinstance(anchor, str) or re.fullmatch(r"[0-9a-f]{40}", anchor) is None:
        raise RouteGuardError("ROUTE_APPROVAL_ANCHOR_INVALID")
    decision_id = _nonempty_string(
        document["user_decision_id"], "ROUTE_DECISION_ID_INVALID"
    )
    if re.fullmatch(r"USER-ICRA-ROUTE-\d{8}-\d{3,}", decision_id) is None:
        raise RouteGuardError("ROUTE_DECISION_ID_INVALID")
    transition = document["protected_transition"]
    if not isinstance(transition, dict) or set(transition) != {"from_anchor", "changes"}:
        raise RouteGuardError("ROUTE_PROTECTED_TRANSITION_INVALID")
    if transition["from_anchor"] != anchor or not isinstance(transition["changes"], list):
        raise RouteGuardError("ROUTE_PROTECTED_TRANSITION_INVALID")
    changes: list[tuple[str, Any, Any]] = []
    for row in transition["changes"]:
        if not isinstance(row, dict) or set(row) != {"field", "old", "new"}:
            raise RouteGuardError("ROUTE_PROTECTED_TRANSITION_INVALID")
        field = row["field"]
        if field not in PROTECTED_ROUTE_FIELDS or any(
            prior[0] == field for prior in changes
        ):
            raise RouteGuardError("ROUTE_PROTECTED_TRANSITION_INVALID")
        if row["new"] != document[field]:
            raise RouteGuardError("ROUTE_PROTECTED_TRANSITION_INVALID")
        changes.append((field, row["old"], row["new"]))
    if {row[0] for row in changes} != PROTECTED_ROUTE_FIELDS:
        raise RouteGuardError("ROUTE_PROTECTED_TRANSITION_INVALID")
    user_decision = document["user_decision"]
    if not isinstance(user_decision, dict) or set(user_decision) != USER_DECISION_FIELDS:
        raise RouteGuardError("ROUTE_USER_DECISION_INVALID")
    decision_items = tuple(
        (key, _nonempty_string(user_decision[key], "ROUTE_USER_DECISION_INVALID"))
        for key in sorted(USER_DECISION_FIELDS)
    )
    if document["guard_strength"] != SUPPORTED_GUARD_STRENGTH:
        raise RouteGuardError("ROUTE_GUARD_STRENGTH_INVALID")
    _require_reachable_anchor(Path(repository).resolve(), anchor)
    return RouteLock(
        document["schema_version"], document["route_owner"], active_route,
        required_modules, research_question, primary_claim, secondary_claims,
        formal_arms, qualification_scenes, gate_sequence,
        document["fallback_policy"], document["scientific_no_go_transition"],
        document["campaign_activation"], anchor, decision_id, tuple(changes),
        decision_items, document["guard_strength"],
    )


def parse_route_lock(path: Path, repository: Path) -> RouteLock:
    """Parse exactly one canonical route lock file."""
    return parse_route_lock_text(Path(path).read_text(), repository)


def _parse_state_text(text: str) -> dict[str, str]:
    matches = re.findall(r"```yaml\s*\n(.*?)\n```", text, re.DOTALL)
    if len(matches) != 1:
        raise RouteGuardError("STATE_YAML_BLOCK_INVALID")
    state: dict[str, str] = {}
    for line in matches[0].splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        match = re.fullmatch(r"([a-z][a-z0-9_]*):\s*(.*?)\s*", line)
        if match is None or match.group(1) in state:
            raise RouteGuardError("STATE_YAML_SCHEMA_INVALID")
        value = match.group(2)
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "'\"":
            value = value[1:-1]
        state[match.group(1)] = value
    return state


def _parse_state(path: Path) -> dict[str, str]:
    return _parse_state_text(Path(path).read_text())


def _task_metadata(path: Path) -> dict[str, str]:
    return _task_metadata_text(Path(path).read_text())


def _task_metadata_text(text: str) -> dict[str, str]:
    title = re.search(r"^#\s+(ICRA-\d{3}|NONE)\b", text, re.MULTILINE)
    if title is None:
        raise RouteGuardError("TASK_ID_MISSING")
    metadata = {"task_id": title.group(1)}
    for label, key in (
        ("Active gate", "gate"), ("Owner", "owner"),
        ("Activation", "activation"), ("User decision", "user_decision_id"),
    ):
        match = re.search(
            rf"^>\s*{re.escape(label)}:\s*`([^`]+)`\s*$", text, re.MULTILINE
        )
        if match is None:
            raise RouteGuardError(f"TASK_{key.upper()}_MISSING")
        metadata[key] = match.group(1)
    return metadata


def _active_document_prelude(path: Path) -> str:
    return _active_document_prelude_text(Path(path).read_text())


def _active_document_prelude_text(text: str) -> str:
    marker = re.search(r"^##\s+Superseded\b", text, re.MULTILINE)
    return text[:marker.start()] if marker is not None else text


def _require_fresh_user_decision(
    route: RouteLock, repository: Path, route_lock_path: Path,
) -> None:
    try:
        relative = route_lock_path.resolve().relative_to(repository.resolve())
    except ValueError:
        return
    prior = _git(repository, "show", f"{route.approval_anchor}:{relative.as_posix()}")
    if prior.returncode == 0 and route.user_decision_id in prior.stdout:
        raise RouteGuardError("USER_DECISION_ID_REUSED")


def verify_repository_route(
    repository: Path, route_lock_path: Path, state_path: Path, task_path: Path,
    scope_path: Path, plan_path: Path, plan_review_path: Path,
) -> RouteLock:
    route = parse_route_lock(route_lock_path, repository)
    state = _parse_state(state_path)
    task = _task_metadata(task_path)
    required_state = {
        "conference_route", "route_owner", "route_lock", "user_decision_id",
        "user_approval_anchor", "active_role", "status", "gate", "task_id",
        "next_task", "p4_v1_status", "p4_v2_status", "campaign_status",
    }
    if not required_state.issubset(state):
        raise RouteGuardError("STATE_REQUIRED_FIELDS_MISSING")
    if state["conference_route"] != route.active_route:
        raise RouteGuardError("STATE_ACTIVE_ROUTE_MISMATCH")
    if state["route_owner"] != route.route_owner:
        raise RouteGuardError("STATE_ROUTE_OWNER_MISMATCH")
    expected_route_lock = (
        "docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md"
    )
    if state["route_lock"] != expected_route_lock:
        raise RouteGuardError("STATE_ROUTE_LOCK_PATH_MISMATCH")
    if "SCIENTIFIC_NO_GO" not in state["p4_v1_status"]:
        raise RouteGuardError("P4_V1_NO_GO_RELABELLED")
    if "NOT_STARTED" not in state["p4_v2_status"] \
            or "BLOCKED" not in state["p4_v2_status"]:
        raise RouteGuardError("P4_V2_PREMATURE_ACTIVATION")
    if "BLOCKED" not in state["campaign_status"] \
            or "ICRA079" not in state["campaign_status"] \
            or "USER" not in state["campaign_status"]:
        raise RouteGuardError("CAMPAIGN_BARRIER_INVALID")
    if state["user_decision_id"] != route.user_decision_id:
        raise RouteGuardError("STATE_DECISION_MISMATCH")
    if state["user_approval_anchor"] != route.approval_anchor:
        raise RouteGuardError("STATE_APPROVAL_ANCHOR_MISMATCH")
    _require_fresh_user_decision(route, repository, route_lock_path)
    if state["task_id"] != task["task_id"]:
        raise RouteGuardError("TASK_ID_MISMATCH")
    if state["gate"] != task["gate"]:
        raise RouteGuardError("TASK_GATE_MISMATCH")
    if state["active_role"] != task["owner"]:
        raise RouteGuardError("TASK_OWNER_MISMATCH")
    if state["status"] != task["activation"]:
        raise RouteGuardError("TASK_ACTIVATION_MISMATCH")
    if task["user_decision_id"] != route.user_decision_id:
        raise RouteGuardError("TASK_DECISION_MISMATCH")
    blocked_no_go = state["status"] == "BLOCKED_AWAITING_USER_RESEARCH_DECISION"
    if blocked_no_go:
        if state["active_role"] != "SUPERVISOR" \
                or state["next_task"] != "NONE" \
                or state["task_id"] != "NONE":
            raise RouteGuardError("NO_GO_BLOCKED_STATE_INVALID")
    elif state["status"] == "TASK_READY":
        if state["next_task"] != "NEXT_TASK.md":
            raise RouteGuardError("ACTIVE_TASK_POINTER_MISMATCH")
    else:
        raise RouteGuardError("STATE_ACTIVATION_INVALID")
    task_text = Path(task_path).read_text()
    if not blocked_no_go and route.active_route not in task_text:
        raise RouteGuardError("TASK_ROUTE_MISMATCH")
    if not blocked_no_go and "campaign" not in task_text.lower():
        raise RouteGuardError("TASK_CAMPAIGN_BARRIER_MISSING")
    active_documents = {
        "scope": _active_document_prelude(scope_path),
        "plan": _active_document_prelude(plan_path),
        "review": _active_document_prelude(plan_review_path),
    }
    route_authority_name = Path(expected_route_lock).name
    for label, text in active_documents.items():
        if route.user_decision_id not in text \
                or route_authority_name not in text \
                or "P4-v2" not in text \
                or "SCIENTIFIC_NO_GO" not in text \
                or "ICRA-071" not in text \
                or "campaign" not in text.lower():
            raise RouteGuardError(f"ACTIVE_DOCUMENT_AUTHORITY_MISMATCH:{label}")
    if route.active_route not in active_documents["scope"] \
            or route.active_route not in active_documents["plan"]:
        raise RouteGuardError("ACTIVE_DOCUMENT_ROUTE_MISMATCH")
    return route


def _default_paths(repository: Path) -> dict[str, Path]:
    return {
        "route_lock": repository
        / "docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md",
        "state": repository / "AGENT_STATE.md",
        "task": repository / "NEXT_TASK.md",
        "scope": repository / "docs/icra27/ICRA_SCOPE.md",
        "plan": repository / "docs/icra27/ICRA_IMPLEMENTATION_PLAN.md",
        "plan_review": repository / "docs/icra27/ICRA_PLAN_REVIEW.md",
    }


def _verify_defaults(repository: Path) -> RouteLock:
    paths = _default_paths(repository)
    return verify_repository_route(
        repository, paths["route_lock"], paths["state"], paths["task"],
        paths["scope"], paths["plan"], paths["plan_review"],
    )


def _staged_paths(repository: Path) -> set[str]:
    result = _git(
        repository, "diff", "--cached", "--no-renames", "--name-only",
        "--diff-filter=ACMRD"
    )
    if result.returncode != 0:
        raise RouteGuardError("STAGED_PATH_QUERY_FAILED")
    return {line for line in result.stdout.splitlines() if line}


def _staged_deletions(repository: Path) -> set[str]:
    result = _git(
        repository, "diff", "--cached", "--no-renames", "--name-only",
        "--diff-filter=D"
    )
    if result.returncode != 0:
        raise RouteGuardError("STAGED_DELETION_QUERY_FAILED")
    return {line for line in result.stdout.splitlines() if line}


def _git_file(repository: Path, revision: str, relative: str) -> str:
    result = _git(repository, "show", f"{revision}:{relative}")
    if result.returncode != 0:
        raise RouteGuardError(f"GIT_FILE_READ_FAILED:{revision}:{relative}")
    return result.stdout


def _route_block(text: str) -> str:
    begin = text.find(ROUTE_BEGIN)
    end = text.find(ROUTE_END)
    if begin < 0 or end < 0 or begin >= end:
        return "INVALID_ROUTE_BLOCK"
    return text[begin:end + len(ROUTE_END)]


def _route_json_document(text: str) -> dict[str, Any]:
    block = _route_block(text)
    if block == "INVALID_ROUTE_BLOCK":
        raise RouteGuardError("ROUTE_LOCK_SENTINEL_MISSING")
    payload = block[len(ROUTE_BEGIN):-len(ROUTE_END)].strip()
    fence = re.fullmatch(r"```json\s*\n(?P<body>.*)\n```", payload, re.DOTALL)
    if fence is None:
        raise RouteGuardError("ROUTE_LOCK_JSON_ENVELOPE_INVALID")
    try:
        document = json.loads(fence.group("body"), object_pairs_hook=_unique_object)
    except RouteGuardError:
        raise
    except json.JSONDecodeError as exc:
        raise RouteGuardError("ROUTE_LOCK_JSON_INVALID") from exc
    if not isinstance(document, dict):
        raise RouteGuardError("ROUTE_LOCK_SCHEMA_FIELDS_INVALID")
    return document


def protected_route_fields_changed(
    before_text: str, after_text: str,
) -> frozenset[str]:
    before = _route_json_document(before_text)
    after = _route_json_document(after_text)
    return frozenset(
        field for field in PROTECTED_ROUTE_FIELDS
        if before.get(field) != after.get(field)
    )


def _validate_staged_route_transition(
    repository: Path, staged: set[str], head_state: dict[str, str],
    staged_state: dict[str, str], roadmap_relative: str,
) -> None:
    if head_state.get("active_role") != "SUPERVISOR":
        raise RouteGuardError("BUILDER_ROUTE_LOCK_STAGED")
    missing = ROUTE_TRANSITION_DOCUMENTS - staged
    if missing:
        raise RouteGuardError(
            "ROUTE_CHANGE_DOCUMENT_SYNC_MISSING:" + ",".join(sorted(missing))
        )
    if any(requires_development_docs(path) for path in staged):
        raise RouteGuardError("ROUTE_CHANGE_PRODUCT_FILE_STAGED")
    if staged_state.get("active_role") != "SUPERVISOR":
        raise RouteGuardError("ROUTE_CHANGE_SUPERVISOR_NOT_ACTIVE")
    head_document = _route_json_document(
        _git_file(repository, "HEAD", roadmap_relative)
    )
    staged_document = _route_json_document(
        _git_file(repository, "", roadmap_relative)
    )
    old_decision = head_document.get("user_decision_id")
    new_decision = staged_document.get("user_decision_id")
    if not isinstance(new_decision, str) or new_decision == old_decision:
        raise RouteGuardError("ROUTE_CHANGE_DECISION_ID_NOT_DISTINCT")
    head = _git(repository, "rev-parse", "HEAD")
    if head.returncode != 0:
        raise RouteGuardError("ROUTE_CHANGE_HEAD_QUERY_FAILED")
    head_commit = head.stdout.strip()
    if staged_document.get("approval_anchor") != head_commit:
        raise RouteGuardError("ROUTE_CHANGE_APPROVAL_ANCHOR_STALE")
    transition = staged_document.get("protected_transition")
    if not isinstance(transition, dict) \
            or transition.get("from_anchor") != head_commit \
            or not isinstance(transition.get("changes"), list):
        raise RouteGuardError("ROUTE_CHANGE_TRANSITION_INVALID")
    rows = {
        row.get("field"): row for row in transition["changes"]
        if isinstance(row, dict)
    }
    changed_fields = {
        field for field in PROTECTED_ROUTE_FIELDS
        if head_document.get(field) != staged_document.get(field)
    }
    if not changed_fields:
        raise RouteGuardError("ROUTE_CHANGE_PROTECTED_FIELDS_UNCHANGED")
    for field in changed_fields:
        row = rows.get(field)
        if row is None or row.get("old") != head_document.get(field) \
                or row.get("new") != staged_document.get(field):
            raise RouteGuardError("ROUTE_CHANGE_OLD_NEW_MISMATCH")
    if staged_state.get("user_decision_id") != new_decision \
            or staged_state.get("user_approval_anchor") != head_commit:
        raise RouteGuardError("ROUTE_CHANGE_STATE_DECISION_MISMATCH")
    staged_task = _task_metadata_text(
        _git_file(repository, "", "NEXT_TASK.md")
    )
    if staged_task.get("user_decision_id") != new_decision \
            or staged_task.get("owner") != "SUPERVISOR":
        raise RouteGuardError("ROUTE_CHANGE_TASK_DECISION_MISMATCH")
    for relative in (
        "docs/icra27/ICRA_SCOPE.md",
        "docs/icra27/ICRA_IMPLEMENTATION_PLAN.md",
    ):
        active = _active_document_prelude_text(
            _git_file(repository, "", relative)
        )
        if new_decision not in active or head_commit not in active:
            raise RouteGuardError("ROUTE_CHANGE_ACTIVE_DOCUMENT_DECISION_MISMATCH")
        for field in changed_fields:
            value = staged_document[field]
            values = value if isinstance(value, list) else [value]
            if any(not isinstance(item, str) or item not in active for item in values):
                raise RouteGuardError(
                    f"ROUTE_CHANGE_ACTIVE_DOCUMENT_FIELD_MISMATCH:{field}"
                )


def _require_staged_authority_matches_worktree(
    repository: Path, staged: set[str],
) -> None:
    for relative in sorted(AUTHORITY_INPUT_PATHS.intersection(staged)):
        path = repository / relative
        if not path.is_file() or path.read_text() != _git_file(repository, "", relative):
            raise RouteGuardError(f"STAGED_AUTHORITY_WORKTREE_MISMATCH:{relative}")


def run_pre_commit_guard(repository: Path) -> None:
    staged = _staged_paths(repository)
    deletions = _staged_deletions(repository)
    paths = _default_paths(repository)
    head_state = _parse_state_text(_git_file(repository, "HEAD", "AGENT_STATE.md"))
    staged_state = (
        _parse_state_text(_git_file(repository, "", "AGENT_STATE.md"))
        if "AGENT_STATE.md" in staged else _parse_state(paths["state"])
    )
    roadmap_relative = (
        "docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md"
    )
    route_changed = roadmap_relative in staged and _route_block(
        _git_file(repository, "HEAD", roadmap_relative)
    ) != _route_block(_git_file(repository, "", roadmap_relative))
    if head_state.get("status") == "BLOCKED_AWAITING_USER_RESEARCH_DECISION" \
            and staged_state.get("status") == "TASK_READY" and not route_changed:
        raise RouteGuardError("NO_GO_ALTERNATE_TASK_WITHOUT_USER_DECISION")
    if any(requires_development_docs(path) for path in staged):
        missing = REQUIRED_DEVELOPMENT_DOCS - staged
        if missing:
            raise RouteGuardError(
                "DEVELOPMENT_DOCS_NOT_STAGED:" + ",".join(sorted(missing))
            )
    essential_guard_files = ICRA071_ALLOWED_STAGED_FILES
    forbidden_guard_deletions = sorted(deletions.intersection(essential_guard_files))
    if forbidden_guard_deletions:
        raise RouteGuardError(
            "ICRA071_GUARD_FILE_DELETION_FORBIDDEN:"
            + ",".join(forbidden_guard_deletions)
        )
    if head_state.get("active_role") == "DEEPSEEK" and route_changed:
        raise RouteGuardError("BUILDER_ROUTE_LOCK_STAGED")
    if route_changed:
        _validate_staged_route_transition(
            repository, staged, head_state, staged_state, roadmap_relative
        )
    if head_state.get("active_role") == "DEEPSEEK" and staged.intersection(
        {"AGENT_STATE.md", "NEXT_TASK.md", "SUPERVISOR_LOG.md"}
    ):
        raise RouteGuardError("BUILDER_SUPERVISOR_FILE_STAGED")
    _require_staged_authority_matches_worktree(repository, staged)
    if head_state.get("task_id") == "ICRA-071":
        forbidden = sorted(
            path for path in staged
            if path not in ICRA071_ALLOWED_STAGED_FILES
            and not path.startswith("results/icra27/icra071/")
        )
        if forbidden:
            raise RouteGuardError(
                "ICRA071_PRODUCT_CHANGE_FORBIDDEN:" + ",".join(forbidden)
            )
    _verify_defaults(repository)


def run_pre_push_guard(repository: Path, ref_updates: str = "") -> None:
    check_hooks_path(repository)
    status = _git(repository, "status", "--porcelain", "--untracked-files=no")
    if status.returncode != 0 or status.stdout.strip():
        raise RouteGuardError("PUSH_TRACKED_WORKTREE_NOT_CLEAN")
    head = _git(repository, "rev-parse", "HEAD")
    if head.returncode != 0:
        raise RouteGuardError("PUSH_HEAD_QUERY_FAILED")
    head_commit = head.stdout.strip()
    for line in ref_updates.splitlines():
        fields = line.split()
        if len(fields) != 4:
            raise RouteGuardError("PUSH_REF_UPDATE_MALFORMED")
        _, local_sha, _, _ = fields
        if local_sha == "0" * 40:
            raise RouteGuardError("PUSH_DELETE_NOT_AUTHORIZED")
        if local_sha != head_commit:
            raise RouteGuardError("PUSHED_COMMIT_NOT_CHECKED_OUT_HEAD")
    _verify_defaults(repository)


def main(arguments: list[str] | None = None) -> int:
    repository_default = Path(__file__).resolve().parents[2]
    defaults = _default_paths(repository_default)
    parser = argparse.ArgumentParser()
    parser.add_argument("--repository", type=Path, default=repository_default)
    parser.add_argument("--route-lock", type=Path, default=defaults["route_lock"])
    parser.add_argument("--state", type=Path, default=defaults["state"])
    parser.add_argument("--task", type=Path, default=defaults["task"])
    parser.add_argument("--scope", type=Path, default=defaults["scope"])
    parser.add_argument("--plan", type=Path, default=defaults["plan"])
    parser.add_argument("--plan-review", type=Path, default=defaults["plan_review"])
    parser.add_argument("--commit-message", type=Path)
    parser.add_argument("--install-hooks", action="store_true")
    parser.add_argument("--check-hooks", action="store_true")
    parser.add_argument("--hook", choices=("pre-commit", "pre-push"))
    args = parser.parse_args(arguments)
    try:
        repository = args.repository.resolve()
        modes = sum(bool(value) for value in (
            args.commit_message, args.install_hooks, args.check_hooks, args.hook,
        ))
        if modes > 1:
            raise RouteGuardError("CLI_MODES_CONFLICT")
        if args.commit_message:
            validate_commit_message(args.commit_message)
            print("ICRA_ROUTE_GUARD_PASS:COMMIT_MESSAGE_REQUIREMENT_BOUND")
            return 0
        if args.install_hooks:
            install_hooks_path(repository)
            print("ICRA_ROUTE_GUARD_PASS:HOOKS_PATH_INSTALLED:.githooks")
            return 0
        if args.check_hooks:
            check_hooks_path(repository)
            print("ICRA_ROUTE_GUARD_PASS:HOOKS_PATH_VERIFIED:.githooks")
            return 0
        if args.hook == "pre-commit":
            run_pre_commit_guard(repository)
            print("ICRA_ROUTE_GUARD_PASS:PRE_COMMIT")
            return 0
        if args.hook == "pre-push":
            run_pre_push_guard(repository, sys.stdin.read())
            print("ICRA_ROUTE_GUARD_PASS:PRE_PUSH")
            return 0
        verify_repository_route(
            repository, args.route_lock, args.state, args.task, args.scope,
            args.plan, args.plan_review,
        )
    except (OSError, RouteGuardError) as exc:
        reason = exc.reason if isinstance(exc, RouteGuardError) else "INPUT_READ_FAILED"
        print(f"ICRA_ROUTE_GUARD_FAIL:{reason}", file=sys.stderr)
        return 2
    print("ICRA_ROUTE_GUARD_PASS:REPOSITORY_CONSISTENT_NOT_USER_AUTHENTICATION")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
