# ICRA-053 synthetic verification

IAP-RQ-423. ICRA-053 registers `XDG_RUNTIME_DIR` as the fifth exact r3 child
environment key at `launch_environment/xdg_runtime`, mode `0700`. It extends
the existing eight-output contract without changing calibration science,
lineage or v1/v2 behavior.

Every Python command in this task, including RED development commands, used:

```bash
TMPDIR=/home/dev/ws_iap/src/iap/results/icra27/icra053/tmp
```

The first attempted unittest command used package-style paths and exited 1
because `test/` is not a package. Correct single-file RED commands then exited
1 for the intended behavior: production AST exposed the unconditional
`/tmp/runtime-root`, runner lacked XDG evidence/command propagation, and the
analyzer rejected an incomplete five-key baseline. After implementation, the
same three seams passed: launch contract 11/11, runner environment 4/4 and the
focused analyzer XDG test 1/1.

Formal commands and results:

- `env TMPDIR=$PWD/results/icra27/icra053/tmp python3 -m unittest discover -s test -p 'test_p4_g0c_*.py' -v` — exit 0, 87 tests.
- `env TMPDIR=$PWD/results/icra27/icra053/tmp python3 test/test_test_planner_launch.py -v` — exit 0, 16 tests.
- `env TMPDIR=$PWD/results/icra27/icra053/tmp python3 -m unittest discover -s test -p 'test_*.py'` — exit 0, 442 tests.
- in-memory syntax compile over four production files and five focused test
  files — exit 0, 9 files.
- fatal-only flake8, four-file canonical JSON and `git diff --check` — exit 0.

The structural test parses production launch construction rather than importing
the shared registry: it enumerates all top-level `SetEnvironmentVariable`
actions, proves r3 selects the registered XDG launch argument, proves legacy
non-r3 alone retains `/tmp/runtime-root`, and independently checks the exact
five environment/eight output keys passed into the production r3 binding.

The initial Spec review correctly found that the output half still inspected
only that declared binding. Remediation now also parses the production runner
and launch ASTs independently: five child arguments, six launch output
arguments, the `launch_command.json`/`stdout.log` direct runner writes and the
runtime/export/log/bag/CSV/manifest sink chains must normalize to the same
eight output keys. Path-valued environment actions are filtered independently
so unrelated scalar environment variables are outside this contract. The
post-remediation focused and complete discoveries again pass 87/87 and
442/442.

Runner coverage includes absent and malicious caller XDG values plus missing,
extra, changed, wrong-type, outside, lexical-parent, alias, symlink, duplicate
and wrong-mode evidence. Every rejection records typed
`LAUNCH_ENVIRONMENT_NOT_READY` before GPU, launch or attempted-ID mutation.
The nominal boundary proves task-local propagation and actual `0700` mode.
Analyzer coverage applies remove/change/wrong-type to all 13 bindings
(13 x 3 = 39) after refreshing legitimate provenance; two additional mode
cases reject filesystem and runner-state drift. No threshold draft is emitted.

No external temp/evidence, build/install/log product, CTest, GPU, ROS, live
runner/analyzer CLI, main flow, smoke or qualification was created or run.
The task TMPDIR is empty after verification. Protected PDF, ICRA-051 state and
external ROS log hashes remain unchanged.
