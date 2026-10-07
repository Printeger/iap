#!/usr/bin/env python3
"""Read-only arrival evidence; lifecycle and safety qualification remain separate."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'launch/_includes'))
from run_directory import resolve_run_directory, finalize_run, write_subordinate_manifest


def arrival_transitions(text):
    # PRESET_TARGET resets its waypoint with planNextWaypoint before WAIT_TARGET.
    # That synchronous call changes EXEC_TRAJ to REPLAN_TRAJ. INIT is startup.
    return [dict(line_number=index, source_state=match.group(1), source_line=line)
            for index, line in enumerate(text.splitlines(), 1)
            if (match := re.search(r'\[FSM\]: from (EXEC_TRAJ|REPLAN_TRAJ) to WAIT_TARGET\s*$', line))]


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def audit(source):
    process_file = source / 'metadata/manifests/forest_process_result.json'
    process_bytes = process_file.read_bytes()
    process = json.loads(process_bytes)
    revision = process['revision']
    fsm_path = 'src/iap/planner/plan_manage/src/ego_replan_fsm.cpp'
    code = subprocess.run(['git', '-C', str(REPO), 'show', f'{revision}:{fsm_path}'],
                          check=True, capture_output=True, text=True).stdout
    # Historical evidence is interpreted against its own implementation.
    if (code.count('changeFSMExecState(WAIT_TARGET, "FSM")') != 2 or
            't_cur > info->duration_ - 1e-2' not in code or
            '(odom_pos_-end_pt_).norm()<tracking_error_limit_m_ && odom_vel_.norm()<.1' not in code):
        raise ValueError('UNSUPPORTED_ARRIVAL_SEAM: inspect this revision before interpreting logs')
    log = source / 'runtime/launch.log'
    log_bytes = log.read_bytes()
    log_hash = hashlib.sha256(log_bytes).hexdigest()
    if process.get('logs_sha256', {}).get('runtime/launch.log') != log_hash:
        raise ValueError('LOG_IDENTITY: original launch log is unbound or changed')
    text = log_bytes.decode()
    transitions = arrival_transitions(text)
    lines = text.splitlines()
    for transition in transitions:
        index = transition['line_number'] - 1
        transition['context'] = lines[max(0, index-3):index+4]
        transition['preset_reset_immediately_before'] = bool(index and
            '[TRIG]: from EXEC_TRAJ to REPLAN_TRAJ' in lines[index-1])
    return dict(identity='ORIGINAL_FSM_ARRIVAL_AUDIT', source_run=str(source),
                revision=revision, original_fsm_arrival_observed=bool(transitions),
                transitions=transitions, arrival_rule_changed=False,
                safety_qualified=False, formal_B_or_D_qualified=False,
                lifecycle_is_arrival=False,
                interpretation='Noninitial FSM WAIT_TARGET transition at original arrival seam; '
                               'PRESET_TARGET may reset to REPLAN_TRAJ before logging WAIT_TARGET.',
                input_sha256={str(log): log_hash,
                              str(process_file): hashlib.sha256(process_bytes).hexdigest()},
                original_fsm_source_sha256=hashlib.sha256(code.encode()).hexdigest())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source_runs', type=Path, nargs='+')
    args = parser.parse_args()
    run = resolve_run_directory(entrypoint='task_arrival_audit',
                                scenario='icra_dense_forest_four_fork_v2')
    status = 'failed'
    record = dict(command=[sys.executable, *sys.argv],
                  input_runs=[str(p) for p in args.source_runs],
                  script_sha256=sha(Path(__file__)), original_inputs_modified=False,
                  safety_qualified=False, formal_qualified=False)
    try:
        results = [audit(source.resolve(strict=True)) for source in args.source_runs]
        output = run / 'export/planner/task_arrival_audit.json'
        with output.open('x') as stream:
            json.dump(results, stream, indent=2)
            stream.write('\n')
        record['artifact_sha256'] = {str(output.relative_to(run)): sha(output)}
        status = 'completed'
        print(json.dumps(dict(run=str(run), arrivals=[dict(source=r['source_run'],
            observed=r['original_fsm_arrival_observed']) for r in results])))
    except Exception as error:
        record['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        record['lifecycle'] = status
        write_subordinate_manifest(run, 'task_arrival_audit', record)
        finalize_run(run, lifecycle=status)


if __name__ == '__main__':
    main()
