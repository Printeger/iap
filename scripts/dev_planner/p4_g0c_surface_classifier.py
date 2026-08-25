#!/usr/bin/env python3
"""Fail-closed static inventory for the P4-G0C production mutation surface."""

from __future__ import annotations

import ast
from pathlib import Path
from typing import Mapping


class SurfaceClassificationError(RuntimeError):
    """Raised when a production expression cannot be classified exactly."""


def _call_name(node: ast.AST) -> str:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        prefix = _call_name(node.value)
        return f"{prefix}.{node.attr}" if prefix else node.attr
    return ""


def _import_aliases(tree: ast.AST) -> dict[str, str]:
    aliases = {
        "os": "os", "shutil": "shutil", "subprocess": "subprocess",
        "pathlib": "pathlib", "Path": "Path",
    }
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for item in node.names:
                if item.name in {"os", "shutil", "subprocess", "pathlib"}:
                    aliases[item.asname or item.name] = item.name
        elif isinstance(node, ast.ImportFrom) and node.module in {
            "os", "shutil", "subprocess", "pathlib",
        }:
            for item in node.names:
                target = (
                    "Path" if node.module == "pathlib" and item.name == "Path"
                    else f"{node.module}.{item.name}"
                )
                aliases[item.asname or item.name] = target
    return aliases


def _normalized_call_name(node: ast.AST, aliases: Mapping[str, str]) -> str:
    name = _call_name(node)
    if not name:
        return ""
    first, separator, remainder = name.partition(".")
    normalized = aliases.get(first, first)
    return f"{normalized}.{remainder}" if separator else normalized


def _scope_nodes(scope: ast.AST):
    pending = list(getattr(scope, "body", ()))
    while pending:
        node = pending.pop()
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            continue
        yield node
        pending.extend(ast.iter_child_nodes(node))


def _scopes(tree: ast.Module):
    yield "<module>", tree
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            yield node.name, node


def _function_assignments(function: ast.AST) -> dict[str, ast.AST]:
    assignments: dict[str, ast.AST] = {}
    for node in _scope_nodes(function):
        if (
            isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
        ):
            assignments[node.targets[0].id] = node.value
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            assignments[node.target.id] = node.value
    return assignments


def _resolve_environment_value(
    node: ast.AST,
    assignments: Mapping[str, ast.AST],
    seen: frozenset[str] = frozenset(),
) -> tuple[str, str]:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return (
            "absolute_literal" if node.value.startswith("/") else "literal",
            node.value,
        )
    if isinstance(node, ast.Name):
        if node.id in seen or node.id not in assignments:
            raise SurfaceClassificationError(
                f"unresolved environment variable expression:{node.id}"
            )
        return _resolve_environment_value(
            assignments[node.id], assignments, seen | {node.id}
        )
    if isinstance(node, (ast.List, ast.Tuple)):
        parts = [
            _resolve_environment_value(item, assignments, seen)
            for item in node.elts
        ]
        return "substitution_list", repr(parts)
    if isinstance(node, ast.BinOp) and isinstance(node.op, (ast.Div, ast.Add)):
        left = _resolve_environment_value(node.left, assignments, seen)
        right = _resolve_environment_value(node.right, assignments, seen)
        kind = "trusted_join" if "trusted" in left[0] else "joined_path"
        return kind, f"{left!r}/{right!r}"
    if isinstance(node, ast.Call):
        name = _call_name(node.func)
        if name == "LaunchConfiguration":
            if not node.args or not isinstance(node.args[0], ast.Constant):
                raise SurfaceClassificationError(
                    "unresolved LaunchConfiguration environment value"
                )
            return "launch_configuration", str(node.args[0].value)
        if name == "get_package_share_directory":
            if not node.args or not isinstance(node.args[0], ast.Constant):
                raise SurfaceClassificationError("unresolved package share")
            return "trusted_package_share", str(node.args[0].value)
        if name in {"os.path.join", "Path"}:
            parts = [
                _resolve_environment_value(item, assignments, seen)
                for item in node.args
            ]
            if not parts:
                raise SurfaceClassificationError("empty path constructor")
            kind = (
                "trusted_join"
                if any("trusted" in part[0] for part in parts)
                else "joined_path"
            )
            return kind, repr(parts)
    raise SurfaceClassificationError(
        f"unresolved environment expression:{ast.dump(node, include_attributes=False)}"
    )


def _condition_operator(call: ast.Call) -> str:
    conditions = {
        keyword.arg: keyword.value
        for keyword in call.keywords
        if keyword.arg is not None
    }
    condition = conditions.get("condition")
    if not isinstance(condition, ast.Call) or _call_name(condition.func) != "IfCondition":
        return "unconditional"
    if (
        len(condition.args) != 1
        or condition.keywords
        or not isinstance(condition.args[0], ast.Call)
    ):
        raise SurfaceClassificationError("unresolved environment condition")
    comparison = condition.args[0]
    operator = _call_name(comparison.func)
    if operator not in {"EqualsSubstitution", "NotEqualsSubstitution"}:
        raise SurfaceClassificationError(
            f"unresolved environment condition operator:{operator}"
        )
    if len(comparison.args) != 2 or comparison.keywords:
        raise SurfaceClassificationError(
            "environment condition comparison shape mismatch"
        )
    experiment, expected = comparison.args
    if not (
        isinstance(experiment, ast.Call)
        and _call_name(experiment.func) == "LaunchConfiguration"
        and len(experiment.args) == 1
        and not experiment.keywords
        and isinstance(experiment.args[0], ast.Constant)
        and experiment.args[0].value == "experiment"
    ):
        raise SurfaceClassificationError(
            "environment condition experiment operand mismatch"
        )
    if not isinstance(expected, ast.Name) or expected.id not in {
        "P4_G0C_EXPERIMENT_V3", "P4_G0C_EXPERIMENT_V4"
    }:
        raise SurfaceClassificationError(
            "environment condition registered calibration constant mismatch"
        )
    return operator


def classify_environment_actions(source: str) -> list[dict[str, object]]:
    tree = ast.parse(source)
    records: list[dict[str, object]] = []
    for scope_name, scope in _scopes(tree):
        assignments = _function_assignments(scope)
        for node in _scope_nodes(scope):
            if not (
                isinstance(node, ast.Call)
                and _call_name(node.func) == "SetEnvironmentVariable"
                and len(node.args) >= 2
            ):
                continue
            if not isinstance(node.args[0], ast.Constant) or not isinstance(
                node.args[0].value, str
            ):
                raise SurfaceClassificationError("unresolved environment name")
            name = node.args[0].value
            value_kind, value = _resolve_environment_value(
                node.args[1], assignments
            )
            condition = _condition_operator(node)
            if name == "QT_X11_NO_MITSHM":
                if value_kind != "literal" or value != "1" or condition != "unconditional":
                    raise SurfaceClassificationError("invalid Qt scalar action")
                classification = "non_path_scalar"
                r3_reachable = True
            elif name == "FASTRTPS_DEFAULT_PROFILES_FILE":
                if value_kind != "trusted_join" or condition != "unconditional":
                    raise SurfaceClassificationError("untrusted FAST DDS profile")
                if "fastdds_udp_only.xml" not in value:
                    raise SurfaceClassificationError("wrong FAST DDS profile")
                classification = "immutable_read_only_path"
                r3_reachable = True
            elif name == "XDG_RUNTIME_DIR":
                classification = "registered_mutable_path"
                if (
                    value_kind == "launch_configuration"
                    and value == "p4.g0c.child_xdg_runtime_dir"
                    and condition == "EqualsSubstitution"
                ):
                    r3_reachable = True
                elif (
                    value_kind == "absolute_literal"
                    and value == "/tmp/runtime-root"
                    and condition == "NotEqualsSubstitution"
                ):
                    r3_reachable = False
                else:
                    raise SurfaceClassificationError(
                        "unregistered or conflicting XDG action"
                    )
            else:
                raise SurfaceClassificationError(
                    f"unclassified environment action:{name}"
                )
            records.append({
                "function": scope_name,
                "line": node.lineno,
                "name": name,
                "classification": classification,
                "r3_reachable": r3_reachable,
                "value_kind": value_kind,
                "value": value,
                "condition": condition,
            })
    counts = {name: 0 for name in {item["name"] for item in records}}
    for record in records:
        counts[str(record["name"])] += 1
    expected_counts = {
        "FASTRTPS_DEFAULT_PROFILES_FILE": 1,
        "QT_X11_NO_MITSHM": 1,
        "XDG_RUNTIME_DIR": 3,
    }
    if counts != expected_counts:
        raise SurfaceClassificationError(
            f"environment action multiset mismatch:{counts}"
        )
    return records


_PATH_READ_METHODS = {
    "absolute", "as_posix", "exists", "expanduser", "is_absolute",
    "is_dir", "is_file", "is_symlink", "iterdir", "lstat", "match",
    "read_bytes", "read_text", "relative_to", "resolve", "rglob",
    "samefile", "stat", "with_name", "with_suffix",
}
_PATH_MUTATION_METHODS = {
    "chmod": "Path.chmod",
    "hardlink_to": "Path.hardlink_to",
    "mkdir": "Path.mkdir",
    "open": "Path.open",
    "rename": "Path.rename",
    "replace": "Path.replace",
    "rmdir": "Path.rmdir",
    "symlink_to": "Path.symlink_to",
    "touch": "Path.touch",
    "unlink": "Path.unlink",
    "write_bytes": "Path.write_bytes",
    "write_text": "Path.write_text",
}
_PATH_CLASS_READ_CALLS = {"Path.cwd", "Path.home", "pathlib.Path.cwd", "pathlib.Path.home"}

_OS_READ_CALLS = {
    "os.access", "os.close", "os.environ.get", "os.fstat", "os.getegid",
    "os.geteuid", "os.getgid", "os.getpid", "os.getuid", "os.lstat",
    "os.path.abspath", "os.path.basename", "os.path.dirname",
    "os.path.exists", "os.path.isabs", "os.path.join", "os.path.realpath",
    "os.path.relpath", "os.path.samefile", "os.path.split", "os.readlink",
    "os.stat", "os.uname",
}
_OS_MUTATION_CALLS = {
    "os.chmod": (0, None), "os.chown": (0, None),
    "os.fchmod": (0, None), "os.fchown": (0, None),
    "os.ftruncate": (0, None), "os.lchown": (0, None),
    "os.link": (1, None), "os.makedirs": (0, None),
    "os.mkdir": (0, None), "os.remove": (0, None),
    "os.removedirs": (0, None), "os.rename": (0, 1),
    "os.renames": (0, 1), "os.replace": (0, 1),
    "os.rmdir": (0, None), "os.symlink": (1, None),
    "os.truncate": (0, None), "os.unlink": (0, None),
    "os.utime": (0, None),
}
_SHUTIL_READ_CALLS = {"shutil.disk_usage", "shutil.which"}
_SHUTIL_MUTATION_CALLS = {
    "shutil.chown": (0, None), "shutil.copy": (1, None),
    "shutil.copy2": (1, None), "shutil.copyfile": (1, None),
    "shutil.copytree": (1, None), "shutil.move": (1, 0),
    "shutil.rmtree": (0, None),
}
_SUBPROCESS_HELPERS = {
    "subprocess.Popen", "subprocess.call", "subprocess.check_call",
    "subprocess.check_output", "subprocess.run",
}
_SUBPROCESS_READ_ONLY_KEYWORDS = {
    "bufsize", "capture_output", "check", "close_fds", "creationflags",
    "cwd", "encoding", "env", "errors", "executable", "input",
    "pass_fds", "pipesize", "preexec_fn", "process_group",
    "restore_signals", "shell", "start_new_session", "startupinfo",
    "stdin", "text", "timeout", "universal_newlines",
}
_UNKNOWN_MODULE_MUTATION_METHODS = {
    "chmod", "chown", "copy", "copy2", "copyfile", "copytree", "link",
    "makedirs", "move", "remove", "removedirs", "renames", "rmdir",
    "rmtree", "symlink", "truncate", "utime",
}


def _semantic_root(classification: str) -> str:
    return classification.split(":", 1)[1]


def _resolve_mutation_target(
    node: ast.AST,
    function: str,
    assignments: Mapping[str, ast.AST],
    root_bindings: Mapping[tuple[str, str], str],
    seen: frozenset[str] = frozenset(),
) -> str | None:
    if isinstance(node, ast.Name):
        binding = root_bindings.get((function, node.id))
        if binding is not None:
            return binding
        if node.id in seen or node.id not in assignments:
            return None
        return _resolve_mutation_target(
            assignments[node.id], function, assignments, root_bindings,
            seen | {node.id},
        )
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Div):
        base = _resolve_mutation_target(
            node.left, function, assignments, root_bindings, seen
        )
        component = node.right
        if isinstance(component, ast.Name):
            if component.id in seen or component.id not in assignments:
                return None
            component = assignments[component.id]
        if not (
            base is not None
            and isinstance(component, ast.Constant)
            and isinstance(component.value, str)
            and component.value not in {"", ".", ".."}
            and not Path(component.value).is_absolute()
            and len(Path(component.value).parts) == 1
        ):
            return None
        return f"derived:{_semantic_root(base)}"
    if isinstance(node, ast.Call):
        name = _call_name(node.func)
        if name == "Path" and node.args:
            return _resolve_mutation_target(
                node.args[0], function, assignments, root_bindings, seen
            )
        if isinstance(node.func, ast.Attribute) and node.func.attr in {
            "expanduser", "resolve",
        }:
            base = _resolve_mutation_target(
                node.func.value, function, assignments, root_bindings, seen
            )
            if base is None:
                return None
            return base
    if isinstance(node, ast.Attribute) and node.attr == "name":
        return _resolve_mutation_target(
            node.value, function, assignments, root_bindings, seen
        )
    return None


def _write_mode(
    call: ast.Call, *, default: str, positional_index: int
) -> str:
    mode_node = (
        call.args[positional_index]
        if len(call.args) > positional_index else None
    )
    for keyword in call.keywords:
        if keyword.arg == "mode":
            mode_node = keyword.value
    if mode_node is None:
        return default
    if not isinstance(mode_node, ast.Constant) or not isinstance(
        mode_node.value, str
    ):
        raise SurfaceClassificationError("unresolved file-open mode")
    return mode_node.value


def _os_open_is_write(
    node: ast.AST, assignments: Mapping[str, ast.AST], seen: frozenset[str] = frozenset()
) -> bool:
    if isinstance(node, ast.Name):
        if node.id in seen or node.id not in assignments:
            raise SurfaceClassificationError(f"unresolved os.open flags:{node.id}")
        return _os_open_is_write(
            assignments[node.id], assignments, seen | {node.id}
        )
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.BitOr):
        left_is_write = _os_open_is_write(node.left, assignments, seen)
        right_is_write = _os_open_is_write(node.right, assignments, seen)
        return left_is_write or right_is_write
    if isinstance(node, ast.Attribute):
        flag = _call_name(node)
        if flag in {
            "os.O_WRONLY", "os.O_RDWR", "os.O_CREAT", "os.O_TRUNC",
            "os.O_APPEND",
        }:
            return True
        if flag in {
            "os.O_RDONLY", "os.O_DIRECTORY", "os.O_NOFOLLOW", "os.O_CLOEXEC",
        }:
            return False
    if isinstance(node, ast.Constant) and isinstance(node.value, int):
        if node.value == 0:
            return False
    raise SurfaceClassificationError(
        f"unresolved os.open flags:{ast.dump(node, include_attributes=False)}"
    )


def _record_mutation(
    records: list[dict[str, object]],
    *,
    api: str,
    target: ast.AST,
    node: ast.Call,
    function: str,
    assignments: Mapping[str, ast.AST],
    root_bindings: Mapping[tuple[str, str], str],
    target_policies: Mapping[tuple[str, str, str], str],
    source_name: str,
    secondary: ast.AST | None = None,
) -> None:
    target_text = ast.unparse(target)
    classification = target_policies.get((function, api, target_text))
    if classification is None:
        classification = _resolve_mutation_target(
            target, function, assignments, root_bindings
        )
    if classification is None:
        raise SurfaceClassificationError(
            f"{source_name}:unresolved mutation target:{function}:{api}:"
            f"{ast.unparse(target)}"
        )
    if secondary is not None:
        secondary_classification = _resolve_mutation_target(
            secondary, function, assignments, root_bindings
        )
        if secondary_classification is None:
            raise SurfaceClassificationError(
                f"unresolved secondary mutation target:{function}:{api}:"
                f"{ast.unparse(secondary)}"
            )
        if _semantic_root(secondary_classification) != _semantic_root(classification):
            raise SurfaceClassificationError(
                f"cross-root mutation:{function}:{api}"
            )
        if classification.startswith("registered:"):
            classification = f"derived:{_semantic_root(classification)}"
    records.append({
        "source": source_name,
        "function": function,
        "line": node.lineno,
        "api": api,
        "target": target_text,
        "classification": classification,
    })


def _dynamic_namespace_call(
    call: ast.Call, aliases: Mapping[str, str]
) -> str | None:
    if not isinstance(call.func, ast.Call):
        return None
    lookup = call.func
    if _normalized_call_name(lookup.func, aliases) != "getattr" or not lookup.args:
        return None
    namespace = _normalized_call_name(lookup.args[0], aliases)
    return namespace or ast.unparse(lookup.args[0])


def classify_mutations(
    source: str,
    *,
    source_name: str,
    root_bindings: Mapping[tuple[str, str], str],
    target_policies: Mapping[tuple[str, str, str], str] | None = None,
) -> list[dict[str, object]]:
    tree = ast.parse(source)
    aliases = _import_aliases(tree)
    records: list[dict[str, object]] = []
    target_policies = target_policies or {}
    for scope_name, scope in _scopes(tree):
        assignments = _function_assignments(scope)
        for node in _scope_nodes(scope):
            if not isinstance(node, ast.Call):
                continue
            dynamic_namespace = _dynamic_namespace_call(node, aliases)
            if dynamic_namespace is not None:
                raise SurfaceClassificationError(
                    f"{source_name}:dynamic namespace call:{dynamic_namespace}"
                )
            name = _normalized_call_name(node.func, aliases)
            if isinstance(node.func, ast.Name) and name == "open":
                mode = _write_mode(node, default="r", positional_index=1)
                if any(flag in mode for flag in "wax+"):
                    if not node.args:
                        raise SurfaceClassificationError(
                            f"{source_name}:builtin open lacks target"
                        )
                    _record_mutation(
                        records, api="builtin.open", target=node.args[0],
                        node=node, function=scope_name, assignments=assignments,
                        root_bindings=root_bindings,
                        target_policies=target_policies,
                        source_name=source_name,
                    )
                continue
            if name == "os.open":
                if len(node.args) < 2:
                    raise SurfaceClassificationError(
                        f"{source_name}:os.open lacks flags"
                    )
                if _os_open_is_write(node.args[1], assignments):
                    _record_mutation(
                        records, api="os.open", target=node.args[0], node=node,
                        function=scope_name, assignments=assignments,
                        root_bindings=root_bindings,
                        target_policies=target_policies,
                        source_name=source_name,
                    )
                continue
            if name in _OS_MUTATION_CALLS:
                target_index, secondary_index = _OS_MUTATION_CALLS[name]
                if len(node.args) <= target_index:
                    raise SurfaceClassificationError(
                        f"{source_name}:{name} lacks target"
                    )
                _record_mutation(
                    records, api=name, target=node.args[target_index],
                    secondary=(
                        node.args[secondary_index]
                        if secondary_index is not None
                        and len(node.args) > secondary_index else None
                    ),
                    node=node, function=scope_name, assignments=assignments,
                    root_bindings=root_bindings,
                    target_policies=target_policies, source_name=source_name,
                )
                continue
            if name.startswith("os."):
                if name in _OS_READ_CALLS:
                    continue
                raise SurfaceClassificationError(
                    f"{source_name}:unknown os member:{name}"
                )
            if name in _SHUTIL_MUTATION_CALLS:
                target_index, secondary_index = _SHUTIL_MUTATION_CALLS[name]
                if len(node.args) <= target_index:
                    raise SurfaceClassificationError(
                        f"{source_name}:{name} lacks target"
                    )
                _record_mutation(
                    records, api=name, target=node.args[target_index],
                    secondary=(
                        node.args[secondary_index]
                        if secondary_index is not None else None
                    ),
                    node=node, function=scope_name, assignments=assignments,
                    root_bindings=root_bindings,
                    target_policies=target_policies, source_name=source_name,
                )
                continue
            if name.startswith("shutil."):
                if name in _SHUTIL_READ_CALLS:
                    continue
                raise SurfaceClassificationError(
                    f"{source_name}:unknown shutil member:{name}"
                )
            if name in _SUBPROCESS_HELPERS:
                continue
            if name.startswith("subprocess."):
                raise SurfaceClassificationError(
                    f"{source_name}:unknown subprocess member:{name}"
                )
            if name in {"Path", "pathlib.Path"} or name in _PATH_CLASS_READ_CALLS:
                continue
            if name.startswith(("Path.", "pathlib.Path.")):
                raise SurfaceClassificationError(
                    f"{source_name}:unknown Path member:{name}"
                )
            if name.startswith("pathlib."):
                raise SurfaceClassificationError(
                    f"{source_name}:unknown pathlib member:{name}"
                )
            if (
                isinstance(node.func, ast.Attribute)
                and node.func.attr in _UNKNOWN_MODULE_MUTATION_METHODS
            ):
                raise SurfaceClassificationError(
                    f"{source_name}:unknown module-qualified mutation:{name}"
                )
            if isinstance(node.func, ast.Attribute):
                method = node.func.attr
                if method == "open":
                    mode = _write_mode(node, default="r", positional_index=0)
                    if any(flag in mode for flag in "wax+"):
                        _record_mutation(
                            records, api="Path.open", target=node.func.value,
                            node=node, function=scope_name,
                            assignments=assignments, root_bindings=root_bindings,
                            target_policies=target_policies,
                            source_name=source_name,
                        )
                    continue
                receiver = _resolve_mutation_target(
                    node.func.value, scope_name, assignments, root_bindings
                )
                if method in _PATH_MUTATION_METHODS:
                    if method == "replace" and len(node.args) >= 2:
                        continue
                    api = _PATH_MUTATION_METHODS[method]
                    policy_key = (
                        scope_name, api, ast.unparse(node.func.value)
                    )
                    if receiver is None and policy_key not in target_policies:
                        raise SurfaceClassificationError(
                            f"{source_name}:unresolved Path mutation receiver:"
                            f"{scope_name}:{method}"
                        )
                    secondary = (
                        node.args[0]
                        if method in {"rename", "replace"} and node.args
                        else None
                    )
                    _record_mutation(
                        records, api=api,
                        target=node.func.value, secondary=secondary, node=node,
                        function=scope_name, assignments=assignments,
                        root_bindings=root_bindings,
                        target_policies=target_policies,
                        source_name=source_name,
                    )
                    continue
                if receiver is not None and method not in _PATH_READ_METHODS:
                    raise SurfaceClassificationError(
                        f"{source_name}:unknown path mutation primitive:"
                        f"{scope_name}:{method}"
                    )
    return records


def classify_process_output_arguments(
    source: str,
    *,
    target_policies: Mapping[tuple[str, str, str], str],
    root_bindings: Mapping[tuple[str, str], str] | None = None,
) -> list[dict[str, object]]:
    tree = ast.parse(source)
    aliases = _import_aliases(tree)
    records: list[dict[str, object]] = []
    root_bindings = root_bindings or {}
    write_parameters = {
        "csv_path", "p1.debug_csv_path", "p2.debug_csv_path",
        "p3.debug_csv_path", "p4.debug_csv_path", "summary_path",
    }
    read_parameters = {
        "config_path", "gate0.candidate_events_path", "gate0.control_points_path",
        "gate0.evidence_manifest_path", "manifest_path", "p1.evidence_manifest_path",
    }

    def command_outputs(
        command: ast.AST | None,
        prefix: str,
        assignments: Mapping[str, ast.AST],
        *,
        reject_dynamic_flags: bool,
    ) -> list[tuple[str, ast.AST]]:
        if not isinstance(command, (ast.List, ast.Tuple)):
            return []
        found: list[tuple[str, ast.AST]] = []
        index = 0
        while index < len(command.elts):
            item = command.elts[index]
            resolved = item
            if isinstance(item, ast.Name) and item.id in assignments:
                resolved = assignments[item.id]
            value = (
                resolved.value
                if isinstance(resolved, ast.Constant)
                and isinstance(resolved.value, str)
                else None
            )
            if value == "--":
                break
            if value in {"--bag-output", "--manifest"}:
                if index + 1 >= len(command.elts):
                    raise SurfaceClassificationError(
                        f"recognized process path flag lacks target:{prefix}:{value}"
                    )
                found.append((f"{prefix}.{value}", command.elts[index + 1]))
                index += 2
                continue
            if (
                value is not None
                and value.startswith("-")
                and any(
                    token in value.lower()
                    for token in ("dir", "file", "manifest", "output", "path")
                )
            ):
                raise SurfaceClassificationError(
                    f"unknown process path flag:{prefix}:{value}"
                )
            if reject_dynamic_flags and isinstance(item, ast.Name):
                raise SurfaceClassificationError(
                    f"unresolved subprocess flag or argument:{prefix}:{item.id}"
                )
            index += 1
        return found

    def is_memory_stream(node: ast.AST) -> bool:
        return (
            isinstance(node, ast.Constant) and node.value is None
        ) or (
            isinstance(node, ast.Attribute)
            and _normalized_call_name(node, aliases) in {
                "subprocess.DEVNULL", "subprocess.PIPE", "subprocess.STDOUT",
            }
        )

    for scope_name, scope in _scopes(tree):
        assignments = _function_assignments(scope)
        for node in _scope_nodes(scope):
            if not isinstance(node, ast.Call):
                continue
            dynamic_namespace = _dynamic_namespace_call(node, aliases)
            if dynamic_namespace == "subprocess":
                raise SurfaceClassificationError(
                    "dynamic subprocess member is forbidden"
                )
            name = _normalized_call_name(node.func, aliases)
            candidates: list[tuple[str, ast.AST]] = []
            if name in _SUBPROCESS_HELPERS:
                positional_streams: dict[str, ast.AST] = {}
                if len(node.args) > 4:
                    positional_streams["stdout"] = node.args[4]
                if len(node.args) > 5:
                    positional_streams["stderr"] = node.args[5]
                for stream_name, stream in positional_streams.items():
                    if not is_memory_stream(stream):
                        candidates.append((f"{name}.{stream_name}", stream))
                for keyword in node.keywords:
                    if keyword.arg in {"stdout", "stderr"}:
                        if keyword.arg in positional_streams:
                            raise SurfaceClassificationError(
                                f"conflicting subprocess stream:{name}:"
                                f"{keyword.arg}"
                            )
                        if is_memory_stream(keyword.value):
                            continue
                        candidates.append((
                            f"{name}.{keyword.arg}", keyword.value,
                        ))
                    elif keyword.arg not in _SUBPROCESS_READ_ONLY_KEYWORDS:
                        raise SurfaceClassificationError(
                            f"unknown subprocess output keyword:{name}:"
                            f"{keyword.arg}"
                        )
                candidates.extend(command_outputs(
                    node.args[0] if node.args else None,
                    name,
                    assignments,
                    reject_dynamic_flags=True,
                ))
            elif name.startswith("subprocess."):
                raise SurfaceClassificationError(
                    f"unknown subprocess member:{name}"
                )
            elif name == "ExecuteProcess":
                command = next(
                    (
                        keyword.value for keyword in node.keywords
                        if keyword.arg == "cmd"
                    ),
                    None,
                )
                if not isinstance(command, (ast.List, ast.Tuple)):
                    raise SurfaceClassificationError(
                        "unresolved ExecuteProcess command"
                    )
                candidates.extend(command_outputs(
                    command,
                    "ExecuteProcess",
                    assignments,
                    reject_dynamic_flags=True,
                ))
            elif name == "Node":
                parameters = next(
                    (
                        keyword.value for keyword in node.keywords
                        if keyword.arg == "parameters"
                    ),
                    None,
                )
                if parameters is not None:
                    for item in ast.walk(parameters):
                        if not isinstance(item, ast.Dict):
                            continue
                        for key, value in zip(item.keys, item.values):
                            if (
                                isinstance(key, ast.Constant)
                                and key.value in write_parameters
                            ):
                                candidates.append((
                                    f"Node.parameter.{key.value}", value,
                                ))
                            elif (
                                isinstance(key, ast.Constant)
                                and key.value in read_parameters
                            ):
                                candidates.append((
                                    f"Node.input.{key.value}", value,
                                ))
                            elif (
                                isinstance(key, ast.Constant)
                                and isinstance(key.value, str)
                                and (
                                    key.value.lower().endswith(("_dir", "_path"))
                                    or any(
                                        token in key.value.lower()
                                        for token in (
                                            "log_dir", "output_dir",
                                            "summary_path", "timing_csv_path",
                                        )
                                    )
                                )
                            ):
                                raise SurfaceClassificationError(
                                    f"unknown Node output parameter:{key.value}"
                                )
            for api, target in candidates:
                target_text = ast.unparse(target)
                classification = target_policies.get(
                    (scope_name, api, target_text)
                )
                if classification is None:
                    classification = _resolve_mutation_target(
                        target, scope_name, assignments, root_bindings
                    )
                if classification is None:
                    raise SurfaceClassificationError(
                        f"unclassified subprocess output:{scope_name}:"
                        f"{api}:{target_text}"
                    )
                records.append({
                    "function": scope_name,
                    "line": node.lineno,
                    "api": api,
                    "target": target_text,
                    "classification": classification,
                })
    return records


def _classify_configuration_path_sinks(source: str) -> list[dict[str, object]]:
    tree = ast.parse(source)
    records: list[dict[str, object]] = []
    function = next(
        (
            node for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "_materialize_iap_logging_config"
        ),
        None,
    )
    if function is None:
        raise SurfaceClassificationError("logging materializer is missing")
    for node in ast.walk(function):
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if not (
            isinstance(target, ast.Subscript)
            and isinstance(target.slice, ast.Constant)
            and isinstance(target.slice.value, str)
        ):
            continue
        field = target.slice.value
        if not (field.endswith("_path") or field.endswith("_dir")):
            continue
        value_text = ast.unparse(node.value)
        if field == "log_dir" and value_text == "str(requested)":
            classification = "registered:iap_log_root"
        elif field == "timing_csv_path" and value_text == "str(timing_path)":
            classification = "derived:iap_log_root"
        else:
            raise SurfaceClassificationError(
                f"unclassified logging path sink:{field}:{value_text}"
            )
        records.append({
            "function": function.name,
            "line": node.lineno,
            "api": f"config.assignment.{field}",
            "target": value_text,
            "classification": classification,
        })
    if [item["api"] for item in records].count("config.assignment.log_dir") != 2:
        raise SurfaceClassificationError("logging directory sink count mismatch")
    if [item["api"] for item in records].count(
        "config.assignment.timing_csv_path"
    ) != 1:
        raise SurfaceClassificationError("timing path sink count mismatch")
    return records


def _classify_runner_launch_bindings(
    source: str,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    tree = ast.parse(source)
    function = next(
        (
            node for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "launch_command"
        ),
        None,
    )
    if function is None:
        raise SurfaceClassificationError("runner launch_command is missing")
    assignments = _function_assignments(function)
    dictionaries: list[tuple[int, ast.Dict]] = []
    for node in ast.walk(function):
        if (
            isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
            and node.targets[0].id == "values"
            and isinstance(node.value, ast.Dict)
        ):
            dictionaries.append((node.lineno, node.value))
        elif (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "values"
            and node.func.attr == "update"
            and len(node.args) == 1
            and isinstance(node.args[0], ast.Dict)
        ):
            dictionaries.append((node.lineno, node.args[0]))
    if len(dictionaries) != 2:
        raise SurfaceClassificationError("runner launch values surface mismatch")
    final_values: dict[str, ast.AST] = {}
    for _, dictionary in sorted(dictionaries):
        local_keys: set[str] = set()
        for key, value in zip(dictionary.keys, dictionary.values):
            if not isinstance(key, ast.Constant) or not isinstance(key.value, str):
                raise SurfaceClassificationError("unresolved runner launch key")
            if key.value in local_keys:
                raise SurfaceClassificationError(
                    f"duplicate runner launch key:{key.value}"
                )
            local_keys.add(key.value)
            final_values[key.value] = value

    child_environment: dict[str, str] = {}
    mutable_outputs: dict[str, str] = {}
    immutable_inputs: set[str] = set()
    mutation_records: list[dict[str, object]] = []
    local_output_names = {
        "manifest_path": ("p4_g0c_run_manifest.json", "run_manifest_path"),
        "csv_path": ("p4_decisions.csv", "decision_csv_path"),
    }
    for key, value in final_values.items():
        path_shaped = any(
            token in key.lower()
            for token in ("path", "dir", "home", "root", "tmp")
        )
        if not path_shaped:
            continue
        if (
            isinstance(value, ast.Subscript)
            and isinstance(value.value, ast.Name)
            and isinstance(value.slice, ast.Constant)
        ):
            semantic = str(value.slice.value)
            if value.value.id == "child":
                child_environment[key] = semantic
                continue
            if value.value.id == "outputs":
                mutable_outputs[key] = semantic
            else:
                raise SurfaceClassificationError(
                    f"unresolved runner launch subscript:{key}"
                )
        elif key in {
            "p4.g0c.fixture_path", "p4.g0c.protocol_path",
            "p4.g0c.registry_path",
        }:
            if not (
                isinstance(value, ast.Attribute)
                and isinstance(value.value, ast.Name)
                and value.value.id == "bundle"
            ):
                raise SurfaceClassificationError(
                    f"untrusted runner launch input:{key}"
                )
            immutable_inputs.add(key)
            continue
        elif key in {"p4.g0c.csv_path", "p4.g0c.run_manifest_path"}:
            names = {
                item.id for item in ast.walk(value) if isinstance(item, ast.Name)
            }
            matches = names & set(local_output_names)
            if len(matches) != 1:
                raise SurfaceClassificationError(
                    f"unresolved local runner output:{key}"
                )
            local_name = matches.pop()
            assignment = assignments.get(local_name)
            expected_file, semantic = local_output_names[local_name]
            if not (
                isinstance(assignment, ast.BinOp)
                and isinstance(assignment.op, ast.Div)
                and isinstance(assignment.left, ast.Name)
                and assignment.left.id == "run_dir"
                and isinstance(assignment.right, ast.Constant)
                and assignment.right.value == expected_file
            ):
                raise SurfaceClassificationError(
                    f"noncanonical local runner output:{key}"
                )
            mutable_outputs[key] = semantic
        else:
            raise SurfaceClassificationError(
                f"unclassified runner launch path argument:{key}:"
                f"{ast.unparse(value)}"
            )
        mutation_records.append({
            "function": function.name,
            "line": getattr(value, "lineno", function.lineno),
            "api": f"ros2.launch.argument.{key}",
            "target": ast.unparse(value),
            "classification": f"registered:{mutable_outputs[key]}",
        })
    return ({
        "child_environment": child_environment,
        "mutable_output_arguments": mutable_outputs,
        "immutable_input_arguments": immutable_inputs,
    }, mutation_records)


_RUNNER_ROOT_BINDINGS = {
    ("prepare_launch_environment", "environment_root"): "derived:runs_root",
    ("prepare_launch_environment", "path"): "derived:runs_root",
    ("_execute_launch", "run_dir"): "derived:runs_root",
    ("_validate_and_finalize_run", "run_dir"): "derived:runs_root",
    ("_validate_and_finalize_run", "manifest_path"): (
        "registered:run_manifest_path"
    ),
    ("_validate_and_finalize_run", "inventory_path"): "derived:runs_root",
    ("_persist_result", "runs_root"): "registered:runs_root",
    ("run", "run_dir"): "derived:runs_root",
    ("run", "runs_root"): "registered:runs_root",
}
_RUNNER_TARGET_POLICIES = {
    (
        "prepare_launch_environment", "os.fchmod", "directory_fd",
    ): "registered:XDG_RUNTIME_DIR",
    (
        "_execute_launch", "Path.open", "run_dir / 'stdout.log'",
    ): "registered:stdout_log_path",
    (
        "_validate_and_finalize_run", "Path.write_text", "manifest_path",
    ): "registered:run_manifest_path",
    (
        "run", "Path.write_text", "run_dir / 'launch_command.json'",
    ): "registered:launch_command_path",
    (
        "_execute_launch", "subprocess.Popen.stdout", "output",
    ): "registered:stdout_log_path",
}
_LAUNCH_ROOT_BINDINGS = {
    ("_materialize_gnss_scenario", "path"): "derived:export_root_dir",
    ("_override_odometry_initialization_mode", "path"): (
        "derived:runtime_root_dir"
    ),
    ("_materialize_iap_logging_config", "config_path"): (
        "derived:runtime_root_dir"
    ),
    ("_materialize_iap_logging_config", "referenced_path"): (
        "derived:runtime_root_dir"
    ),
    ("_runtime_config", "export_dir"): "registered:export_root_dir",
    ("_runtime_config", "runtime_root"): "registered:runtime_root_dir",
    ("_runtime_config", "runtime_config_dir"): "derived:runtime_root_dir",
    ("_runtime_config", "config_path"): "derived:runtime_root_dir",
    ("_runtime_config", "config_ros_path"): "derived:runtime_root_dir",
    ("_runtime_config", "config_gnss_path"): "derived:runtime_root_dir",
    ("_runtime_config", "config_odometry_path"): "derived:runtime_root_dir",
    ("_launch_setup", "bag_root_dir"): "registered:bag_output_dir",
    ("_launch_setup", "manifest_path"): "derived:export_root_dir",
    ("_launch_setup", "g0c_path"): "registered:run_manifest_path",
}
_LAUNCH_TARGET_POLICIES = {
    (
        "_launch_setup", "Path.mkdir", "g0c_path.parent",
    ): "derived:runs_root",
    (
        "_launch_setup", "Path.write_text", "g0c_path",
    ): "registered:run_manifest_path",
    (
        "_launch_setup", "ExecuteProcess.--bag-output", "bag_output_dir",
    ): "registered:bag_output_dir",
    (
        "_launch_setup", "ExecuteProcess.--manifest",
        "evidence['manifest_path']",
    ): "derived:export_root_dir",
    (
        "_ego_planner_node", "Node.parameter.p1.debug_csv_path",
        "p1_debug_path",
    ): "derived:export_root_dir",
    (
        "_ego_planner_node", "Node.parameter.p2.debug_csv_path",
        "p2_debug_path",
    ): "derived:export_root_dir",
    (
        "_ego_planner_node", "Node.parameter.p3.debug_csv_path",
        "p3_debug_path",
    ): "derived:export_root_dir",
    (
        "_ego_planner_node", "Node.parameter.p4.debug_csv_path",
        "p4_debug_path",
    ): "registered:decision_csv_path",
    (
        "_ego_planner_node", "Node.input.p1.evidence_manifest_path",
        "evidence['manifest_path']",
    ): "immutable_read_only_path",
    (
        "_ego_planner_node", "Node.input.gate0.evidence_manifest_path",
        "LaunchConfiguration('gate0.evidence_manifest_path').perform(context)",
    ): "immutable_read_only_path",
    (
        "_ego_planner_node", "Node.input.gate0.candidate_events_path",
        "LaunchConfiguration('gate0.candidate_events_path').perform(context)",
    ): "immutable_read_only_path",
    (
        "_ego_planner_node", "Node.input.gate0.control_points_path",
        "LaunchConfiguration('gate0.control_points_path').perform(context)",
    ): "immutable_read_only_path",
    (
        "_launch_setup", "Node.parameter.csv_path",
        "str(Path(export_dir) / 'test_planner_integrity_validation.csv')",
    ): "derived:export_root_dir",
    (
        "_launch_setup", "Node.parameter.summary_path",
        "str(Path(export_dir) / 'test_planner_validation_summary.json')",
    ): "derived:export_root_dir",
    (
        "_launch_setup", "Node.input.manifest_path",
        "evidence['manifest_path']",
    ): "immutable_read_only_path",
    (
        "_launch_setup", "Node.input.config_path", "runtime_config_path",
    ): "derived:runtime_root_dir",
}

_EXPECTED_CHILD_ENVIRONMENT = {
    "HOME", "ROS_HOME", "ROS_LOG_DIR", "TMPDIR", "XDG_RUNTIME_DIR",
}
_EXPECTED_OUTPUT_SEMANTICS = {
    "bag_output_dir", "decision_csv_path", "export_root_dir", "iap_log_root",
    "launch_command_path", "run_manifest_path", "runtime_root_dir",
    "stdout_log_path",
}


def _named_function(tree: ast.Module, name: str) -> ast.FunctionDef:
    matches = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    ]
    if len(matches) != 1:
        raise SurfaceClassificationError(f"container function mismatch:{name}")
    return matches[0]


def _simple_assignments(function: ast.FunctionDef) -> dict[str, str]:
    assignments: dict[str, str] = {}
    for node in ast.walk(function):
        if (
            isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
        ):
            assignments[node.targets[0].id] = ast.unparse(node.value)
    return assignments


def _returned_mapping(function: ast.FunctionDef) -> dict[str, ast.AST]:
    returns = [node for node in ast.walk(function) if isinstance(node, ast.Return)]
    if len(returns) != 1 or not isinstance(returns[0].value, ast.Dict):
        raise SurfaceClassificationError(
            f"container return mapping mismatch:{function.name}"
        )
    result: dict[str, ast.AST] = {}
    for key, value in zip(returns[0].value.keys, returns[0].value.values):
        if not isinstance(key, ast.Constant) or not isinstance(key.value, str):
            raise SurfaceClassificationError(
                f"container return key mismatch:{function.name}"
            )
        result[key.value] = value
    return result


def _nested_mapping(node: ast.AST, label: str) -> dict[str, ast.AST]:
    if not isinstance(node, ast.Dict):
        raise SurfaceClassificationError(f"container nested mapping mismatch:{label}")
    result: dict[str, ast.AST] = {}
    for key, value in zip(node.keys, node.values):
        if not isinstance(key, ast.Constant) or not isinstance(key.value, str):
            raise SurfaceClassificationError(f"container key mismatch:{label}")
        result[key.value] = value
    return result


def _is_canonical_literal_descendant(node: ast.AST, root: str) -> bool:
    if (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "str"
        and len(node.args) == 1
    ):
        node = node.args[0]
    components = 0
    while isinstance(node, ast.BinOp) and isinstance(node.op, ast.Div):
        if not (
            isinstance(node.right, ast.Constant)
            and isinstance(node.right.value, str)
            and node.right.value not in {"", ".", ".."}
            and not Path(node.right.value).is_absolute()
            and len(Path(node.right.value).parts) == 1
        ):
            return False
        components += 1
        node = node.left
    return components > 0 and isinstance(node, ast.Name) and node.id == root


def classify_runner_container_contract(
    runner_source: str, protocol_source: str
) -> list[dict[str, object]]:
    """Prove the sole fresh runs-root container from production ASTs."""
    runner_tree = ast.parse(runner_source)
    protocol_tree = ast.parse(protocol_source)
    run_function = _named_function(runner_tree, "run")
    run_assignments = _simple_assignments(run_function)
    required_assignments = {
        "requested_root": "Path(runs_root).expanduser()",
        "runs_root": "requested_root.resolve()",
        "plan": "expand_run_plan(bundle.protocol, runs_root)",
        "preflight_path": "runs_root / 'preflight'",
        "run_dir": "Path(record['run_dir'])",
    }
    if any(
        run_assignments.get(name) != expression
        for name, expression in required_assignments.items()
    ):
        raise SurfaceClassificationError("runs_root canonical ownership mismatch")
    run_calls = {ast.unparse(node) for node in ast.walk(run_function) if isinstance(node, ast.Call)}
    if not {
        "requested_root.is_symlink()", "runs_root.is_dir()",
        "runs_root.is_symlink()", "runs_root.iterdir()", "run_dir.mkdir(parents=True, exist_ok=False)",
    }.issubset(run_calls):
        raise SurfaceClassificationError("runs_root freshness guard mismatch")
    run_literals = {
        node.value for node in ast.walk(run_function)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }
    if "dirty runs root is forbidden: " not in run_literals:
        raise SurfaceClassificationError("runs_root dirty-reuse guard missing")

    persist_function = _named_function(runner_tree, "_persist_result")
    persist_calls = {
        ast.unparse(node.func) for node in ast.walk(persist_function)
        if isinstance(node, ast.Call)
    }
    if "(runs_root / 'p4_g0c_runner_state.json').write_text" not in persist_calls:
        raise SurfaceClassificationError("runner-state child mismatch")

    derive_function = _named_function(
        runner_tree, "derive_launch_environment_inventory"
    )
    if _simple_assignments(derive_function).get("root") != "Path(runs_root).resolve()":
        raise SurfaceClassificationError("launch-environment root mismatch")

    expand_function = _named_function(protocol_tree, "expand_run_plan")
    expand_mapping_nodes = [
        node for node in ast.walk(expand_function) if isinstance(node, ast.Dict)
        and any(
            isinstance(key, ast.Constant) and key.value == "run_dir"
            for key in node.keys
        )
    ]
    if len(expand_mapping_nodes) != 1:
        raise SurfaceClassificationError("run directory mapping mismatch")
    expand_mapping = _nested_mapping(expand_mapping_nodes[0], "run_dir")
    if ast.unparse(expand_mapping["run_dir"]) != "str(root / run_id)":
        raise SurfaceClassificationError("run directory is not a container child")

    binding_function = _named_function(
        protocol_tree, "expected_launch_environment_binding"
    )
    binding_assignments = _simple_assignments(binding_function)
    if binding_assignments.get("root") != "Path(runs_root).resolve()" or (
        binding_assignments.get("run") != "Path(run_dir).resolve()"
        or binding_assignments.get("environment_root")
        != "root / 'launch_environment'"
    ):
        raise SurfaceClassificationError("container binding roots mismatch")
    binding = _returned_mapping(binding_function)
    child_environment = _nested_mapping(
        binding.get("child_environment"), "child_environment"
    )
    mutable_outputs = _nested_mapping(
        binding.get("mutable_output_paths"), "mutable_output_paths"
    )
    if set(child_environment) != _EXPECTED_CHILD_ENVIRONMENT or not all(
        _is_canonical_literal_descendant(value, "environment_root")
        for value in child_environment.values()
    ):
        raise SurfaceClassificationError("child environment descendants mismatch")
    if set(mutable_outputs) != _EXPECTED_OUTPUT_SEMANTICS or not all(
        _is_canonical_literal_descendant(value, "run")
        for value in mutable_outputs.values()
    ):
        raise SurfaceClassificationError("mutable output descendants mismatch")
    return [{
        "canonical_descendants": {
            "launch_environment", "preflight", "run_dir",
        },
        "child_environment": set(child_environment),
        "output_semantics": set(mutable_outputs),
        "ownership_guards": {
            "canonicalize", "reject_dirty", "reject_symlink",
            "reject_wrong_type",
        },
        "runner_state_child": "p4_g0c_runner_state.json",
        "semantic_root": "runs_root",
    }]


def validate_production_contract(
    environment_actions: list[dict[str, object]],
    mutations: list[dict[str, object]],
    runner_launch_bindings: Mapping[str, object],
    containers: object,
) -> dict[str, object]:
    """Reject any semantic root outside the exact r3 environment/output seam."""
    child_environment = runner_launch_bindings.get("child_environment")
    if not isinstance(child_environment, dict):
        raise SurfaceClassificationError("missing runner child environment")
    actual_child_environment = set(child_environment.values())
    if actual_child_environment != _EXPECTED_CHILD_ENVIRONMENT:
        raise SurfaceClassificationError(
            f"child environment contract mismatch:{actual_child_environment}"
        )
    action_names = [str(item.get("name")) for item in environment_actions]
    if sorted(action_names) != sorted([
        "FASTRTPS_DEFAULT_PROFILES_FILE", "QT_X11_NO_MITSHM",
        "XDG_RUNTIME_DIR", "XDG_RUNTIME_DIR", "XDG_RUNTIME_DIR",
    ]):
        raise SurfaceClassificationError(
            f"environment action contract mismatch:{action_names}"
        )
    if not isinstance(containers, list) or len(containers) != 1:
        raise SurfaceClassificationError("runs_root container count mismatch")
    container = containers[0]
    expected_container = {
        "canonical_descendants": {
            "launch_environment", "preflight", "run_dir",
        },
        "child_environment": _EXPECTED_CHILD_ENVIRONMENT,
        "output_semantics": _EXPECTED_OUTPUT_SEMANTICS,
        "ownership_guards": {
            "canonicalize", "reject_dirty", "reject_symlink",
            "reject_wrong_type",
        },
        "runner_state_child": "p4_g0c_runner_state.json",
        "semantic_root": "runs_root",
    }
    if not isinstance(container, dict) or container != expected_container:
        raise SurfaceClassificationError("runs_root container contract mismatch")
    output_semantics: set[str] = set()
    environment_semantics: set[str] = set()
    container_semantics: set[str] = set()
    for record in mutations:
        classification = record.get("classification")
        if not isinstance(classification, str):
            raise SurfaceClassificationError("mutation classification is not text")
        if classification == "immutable_read_only_path":
            continue
        if ":" not in classification:
            raise SurfaceClassificationError(
                f"unresolved production classification:{classification}"
            )
        root = _semantic_root(classification)
        if root in _EXPECTED_OUTPUT_SEMANTICS:
            output_semantics.add(root)
        elif root in _EXPECTED_CHILD_ENVIRONMENT:
            environment_semantics.add(root)
        elif root == "runs_root":
            container_semantics.add(root)
        else:
            raise SurfaceClassificationError(
                f"unexpected production semantic root:{root}"
            )
    if output_semantics != _EXPECTED_OUTPUT_SEMANTICS:
        raise SurfaceClassificationError(
            f"output contract mismatch:{output_semantics}"
        )
    if container_semantics != {"runs_root"}:
        raise SurfaceClassificationError(
            f"container semantic mismatch:{container_semantics}"
        )
    return {
        "canonical_descendants": container["canonical_descendants"],
        "child_environment": actual_child_environment,
        "container_semantics": container_semantics,
        "environment_mutation_semantics": environment_semantics,
        "ownership_guards": container["ownership_guards"],
        "output_semantics": output_semantics,
        "runner_state_child": container["runner_state_child"],
    }


def production_surface_inventory(repository_root: Path) -> dict[str, object]:
    """Return classified records without weakening the exact contract check."""
    launch_path = repository_root / "launch" / "test_planner.launch.py"
    runner_path = (
        repository_root / "scripts" / "dev_planner"
        / "run_p4_g0c_calibration.py"
    )
    protocol_path = (
        repository_root / "scripts" / "dev_planner" / "p4_g0c_protocol.py"
    )
    environment_actions = classify_environment_actions(launch_path.read_text())
    runner_source = runner_path.read_text()
    launch_source = launch_path.read_text()
    protocol_source = protocol_path.read_text()
    containers = classify_runner_container_contract(
        runner_source, protocol_source
    )
    mutations = classify_mutations(
        runner_source,
        source_name="runner",
        root_bindings=_RUNNER_ROOT_BINDINGS,
        target_policies=_RUNNER_TARGET_POLICIES,
    )
    mutations.extend(classify_mutations(
        launch_source,
        source_name="launch",
        root_bindings=_LAUNCH_ROOT_BINDINGS,
        target_policies=_LAUNCH_TARGET_POLICIES,
    ))
    mutations.extend(classify_process_output_arguments(
        runner_source, target_policies=_RUNNER_TARGET_POLICIES,
        root_bindings=_RUNNER_ROOT_BINDINGS,
    ))
    mutations.extend(classify_process_output_arguments(
        launch_source, target_policies=_LAUNCH_TARGET_POLICIES,
        root_bindings=_LAUNCH_ROOT_BINDINGS,
    ))
    mutations.extend(_classify_configuration_path_sinks(launch_source))
    runner_launch_bindings, runner_launch_mutations = (
        _classify_runner_launch_bindings(runner_source)
    )
    mutations.extend(runner_launch_mutations)
    return {
        "containers": containers,
        "environment_actions": environment_actions,
        "mutations": mutations,
        "runner_launch_bindings": runner_launch_bindings,
    }


def production_surface(repository_root: Path) -> dict[str, object]:
    """Return the production surface only when its exact contract is closed."""
    surface = production_surface_inventory(repository_root)
    surface["contract"] = validate_production_contract(
        surface["environment_actions"],
        surface["mutations"],
        surface["runner_launch_bindings"],
        surface["containers"],
    )
    return surface
