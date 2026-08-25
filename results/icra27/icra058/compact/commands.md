# ICRA-058 executed-boundary ledger

All commands ran from `/home/dev/ws_iap/src/iap`. Child processes were
started with `env -i`; only repository-local caller paths, the exact adopted
prefix pair, a fixed executable search path and bytecode suppression were
supplied before sourcing Jazzy and the adopted local setup. No inherited
environment was dumped or persisted.

## Adopted CUDA closure — PASS, no build

Read-only checks used only the adopted package index, exact IAP cache, six
manifest-declared libraries, fourteen manifest-declared configs and the
registered protocol/registry/lineage/fixture/launch files. Representative
executed commands were:

```bash
find results/icra27/icra056/install/share/ament_index/resource_index/packages \
  -mindepth 1 -maxdepth 1 -type f -printf '%f\n'
rg -n '^(CMAKE_BUILD_TYPE|BUILD_TESTING|BUILD_WITH_CUDA|CMAKE_CUDA_COMPILER)(:|=)' \
  results/icra27/icra056/build/iap/CMakeCache.txt
env -i PATH=/usr/local/cuda/bin:/usr/bin:/bin /usr/local/cuda/bin/nvcc --version
sha256sum config/icra27/p4_g0c_runtime_dependencies_v3.json \
  config/icra27/p4_g0c_protocol_v3.json \
  config/icra27/p4_threshold_registry_v3.json \
  config/icra27/p4_g0c_replacement_lineage_v3.json \
  config/icra27/p4_g0c_live_fixture_v1.json \
  launch/test_planner.launch.py \
  results/icra27/icra056/install/share/iap/launch/test_planner.launch.py
```

The six explicit runtime paths were each checked with `test -f`, `test ! -L`,
`readelf -h` and `sha256sum`. The GPU library additionally passed sanitized
`ldd -r` policy scanning and `ctypes.CDLL(..., RTLD_NOW)`. All seventeen
package names passed `ros2 pkg prefix` equality with the adopted install.

Pre-live diagnostics had several correctable mechanics errors: unavailable
JSON tooling, an over-narrow metadata depth, setup-script incompatibility with
nounset, an unavailable scanner inside the sanitized shell, one mistyped
config SHA and one wrong protocol-summary key. Each invalid diagnostic was
discarded before live and corrected read-only. One exact CMake metadata read
also emitted an embedded historical path environment; it contained no
credential-like value, was not persisted, and that file was excluded from all
later commands and evidence.

## Standalone dependency preflight — one invocation, exit 0

```bash
env -i \
  HOME="$PWD/results/icra27/icra058/home" \
  ROS_HOME="$PWD/results/icra27/icra058/ros_home" \
  ROS_LOG_DIR="$PWD/results/icra27/icra058/ros_logs" \
  TMPDIR="$PWD/results/icra27/icra058/tmp" \
  XDG_RUNTIME_DIR="$PWD/results/icra27/icra058/xdg_runtime" \
  PATH=/usr/local/cuda/bin:/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  PYTHONDONTWRITEBYTECODE=1 \
  /bin/bash --noprofile --norc -c '
    set -e -o pipefail
    source /opt/ros/jazzy/setup.bash
    source /home/dev/ws_iap/src/iap/results/icra27/icra056/install/local_setup.bash
    export AMENT_PREFIX_PATH=/home/dev/ws_iap/src/iap/results/icra27/icra056/install:/opt/ros/jazzy
    export P4_G0C_ALLOWED_PREFIXES=/home/dev/ws_iap/src/iap/results/icra27/icra056/install:/opt/ros/jazzy
    exec python3 /home/dev/ws_iap/src/iap/scripts/dev_planner/run_p4_g0c_calibration.py \
      --dependency-preflight-only \
      --runs-root /home/dev/ws_iap/src/iap/results/icra27/icra058/dependency_preflight
  '
```

## Full runner — one invocation, exit 2

The same exact sanitized command was invoked once without
`--dependency-preflight-only` and with:

```bash
python3 /home/dev/ws_iap/src/iap/scripts/dev_planner/run_p4_g0c_calibration.py \
  --runs-root /home/dev/ws_iap/src/iap/results/icra27/icra058/runs
```

Built-in GPU preflight passed. The first registered identity ran for its
90-second interval and both required processes survived until controlled
shutdown, but inventory parsing rejected the decision CSV as
`typed_identity`. Runner state is immutable `FAILED`, 1 attempted / 0 complete
/ 1 launch / 0 retry.

One final process audit briefly observed a matching process that self-exited
before its command line could be identified. No signal was sent. Repeated
audits are clean with zero matching runner/analyzer/required process.

## Commands not invoked

- `colcon` or any build/install command: zero.
- A second full runner, alternate root or identity retry: zero.
- Analyzer: zero; scientific/evidence failure makes it ineligible.
- Threshold mutation, G0C PASS, G0D, P5, alternate scenario or cleanup: zero.
