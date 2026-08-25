# ICRA-056 executed commands and one-shot ledger

All commands ran from `/home/dev/ws_iap/src/iap`. Complete environments,
stdout/stderr, exits and retained products remain below
`results/icra27/icra056/`; this compact file records the one-shot boundaries.

## Fresh CUDA build — invoked once, exit 0

The exact 17-package command from `NEXT_TASK.md` ran in a clean
noninteractive Jazzy-only subshell with repository-local build, merged install,
log, HOME and TMPDIR roots. It used sequential execution, Release,
`BUILD_TESTING=OFF`, `BUILD_WITH_CUDA=ON`,
`/usr/local/cuda/bin/nvcc`, OpenCV OFF and viewer OFF. Summary: 17 packages
finished in 4m47s. It was not retried.

## Static CUDA closure — PASS

The fresh install contains exactly the 17 selected package indexes,
`BUILD_WITH_CUDA:BOOL=ON`, and all six declared ordinary non-symlink ELF
libraries. `ldd -r` for `libodometry_estimation_gpu.so` has no missing or
undefined entry and no historical/default workspace root. Frozen launch,
config, manifest, protocol, registry, lineage and fixture hashes match.

The first diagnostic used unavailable `/usr/bin/file` and a relative-path
`gnss_comm` aggregate, so it transparently retained false-negative lines. A
read-only resolution used `readelf` and the original absolute-path aggregate;
all six ELF checks and the original `de422a4b...16a` external identity pass.
No build, install or runner invocation was repeated.

## Standalone dependency preflight — invoked once, contract BLOCKED

The command used exact repository-local HOME, ROS_HOME, ROS_LOG_DIR, TMPDIR and
XDG_RUNTIME_DIR plus identical ordered prefixes:

```bash
AMENT_PREFIX_PATH="$PWD/results/icra27/icra056/install:/opt/ros/jazzy" \
P4_G0C_ALLOWED_PREFIXES="$PWD/results/icra27/icra056/install:/opt/ros/jazzy" \
python3 scripts/dev_planner/run_p4_g0c_calibration.py \
  --dependency-preflight-only \
  --runs-root "$PWD/results/icra27/icra056/dependency_preflight"
```

It reported the expected 18 packages, 13 executables, one component, 14
configs and six libraries, with zero GPU, launch or identity attempt. However,
the immutable state serialized `dependency_preflight.manifest_path` as
`results/icra27/icra056/install/lib/libsub_mapping.so`, not the bound v3
manifest. Root cause is production-local variable reuse in
`validate_runtime_dependencies()`. Per the output-binding fail-closed rule,
the typed result is `BLOCKED_DEPENDENCY_MANIFEST_PATH_BINDING`.

## Commands not invoked

- Full r3 runner: zero invocations; `runs/` does not exist.
- Built-in GPU preflight: zero invocations.
- Registered identities: zero attempted, zero complete, zero retries.
- Analyzer: zero invocations; analysis and threshold draft do not exist.
- No threshold mutation, G0C PASS, G0D, P5, smoke or qualification command.
