# ICRA regenerable build/install retirement inventory — 2026-08-26

Status: **COMPLETED / 61 REGENERABLE ROOTS RETIRED**

Authority: user-directed four-layer workflow; `IAP-RQ-424`

Archive anchor: `6a6bdd3e674dd58fafae4153e5a2b5cb5225d730`

## Boundary

The paths below are generated build/install products under this repository's `results/icra27` tree. They contain
no Git-tracked files. The inventory intentionally excludes every `log*` or `static_*` subtree, raw/compact/run/
manifest/analysis evidence, P4-v1 scientific evidence, the protected PDF, and the shared workspace
`/home/dev/ws_iap/{build,install,log}`.

Deletion is permanent; reconstruction requires the retained source commit and command records. Before deletion,
the executor must re-resolve every literal path, reject symlinks or anything outside `results/icra27`, prove no
tracked-file intersection and prove no colcon/CMake/ROS process is using a listed root. No wildcard or unresolved
environment variable may be used as a deletion target.

Pre-deletion filesystem state: `/dev/nvme0n1p4` size `694831472640`, used `632343797760`, available
`27116900352` bytes. Candidate count: `61`; expected reclaim: `122694791115` bytes.

| Path | Bytes |
|---|---:|
| `results/icra27/icra006/build` | 2059293154 |
| `results/icra27/icra007/build` | 2070132806 |
| `results/icra27/icra014/install` | 353954178 |
| `results/icra27/icra015/install` | 684848313 |
| `results/icra27/icra016/install` | 685325315 |
| `results/icra27/icra017/install` | 684822723 |
| `results/icra27/icra018/install` | 684822723 |
| `results/icra27/icra019/build_bspline_opt` | 204594251 |
| `results/icra27/icra019/build_ego` | 1115249614 |
| `results/icra27/icra019/build_iap` | 2449061196 |
| `results/icra27/icra019/build_path_searching` | 57430807 |
| `results/icra27/icra019/build_plan_env` | 222600857 |
| `results/icra27/icra019/install` | 684822890 |
| `results/icra27/icra019/install_bspline_opt` | 53954965 |
| `results/icra27/icra019/install_path_searching` | 5824009 |
| `results/icra27/icra019/install_plan_env` | 67876739 |
| `results/icra27/icra026/build_bspline_opt` | 203750994 |
| `results/icra27/icra026/build_ego` | 1177845697 |
| `results/icra27/icra026/build_iap` | 2435863838 |
| `results/icra27/icra026/build_path_searching` | 56700209 |
| `results/icra27/icra026/build_plan_env` | 226151327 |
| `results/icra27/icra026/install` | 684836848 |
| `results/icra27/icra026/install_bspline_opt` | 53955261 |
| `results/icra27/icra026/install_ego` | 164880939 |
| `results/icra27/icra026/install_path_searching` | 5824153 |
| `results/icra27/icra026/install_plan_env` | 67879332 |
| `results/icra27/icra027/build_iap` | 2460504279 |
| `results/icra27/icra027/install` | 692092607 |
| `results/icra27/icra028/build_iap` | 2461247356 |
| `results/icra27/icra028/install` | 692096381 |
| `results/icra27/icra031/build_iap` | 2461267714 |
| `results/icra27/icra031/install` | 692102521 |
| `results/icra27/icra032/build_ego` | 1180107083 |
| `results/icra27/icra032/build_iap` | 2461299095 |
| `results/icra27/icra032/install` | 692104186 |
| `results/icra27/icra032/install_ego` | 164883275 |
| `results/icra27/icra046/build_bspline` | 365991306 |
| `results/icra27/icra046/build_iap` | 2138110904 |
| `results/icra27/icra046/build_path_searching` | 56703832 |
| `results/icra27/icra046/build_plan_env` | 226151028 |
| `results/icra27/icra046/build_plan_manage` | 1188048396 |
| `results/icra27/icra046/build_quadrotor_msgs` | 7857089 |
| `results/icra27/icra046/install_bspline` | 62380680 |
| `results/icra27/icra046/install_iap` | 594769364 |
| `results/icra27/icra046/install_path_searching` | 5795334 |
| `results/icra27/icra046/install_plan_env` | 67879033 |
| `results/icra27/icra046/install_plan_manage` | 167361336 |
| `results/icra27/icra046/install_quadrotor_msgs` | 2781337 |
| `results/icra27/icra050/build` | 1215166234 |
| `results/icra27/icra050/install` | 475998486 |
| `results/icra27/icra051/build` | 1215987660 |
| `results/icra27/icra051/install` | 476034269 |
| `results/icra27/icra063/env/install_tdd_iap` | 0 |
| `results/icra27/icra068/build` | 1221909134 |
| `results/icra27/icra068/install` | 477687148 |
| `results/icra27/icra070/install` | 41093249 |
| `results/icra27/icra070/install_v2` | 476487546 |
| `results/icra27/icra072/build` | 62004545810 |
| `results/icra27/icra072/install` | 13539615608 |
| `results/icra27/icra072/tdd_final_closure/attempt_16_green/build` | 4516579604 |
| `results/icra27/icra072/tdd_final_closure/attempt_16_green/install` | 1029849093 |

## Post-deletion record

The inventory was committed and pushed at `3f06a455d3b4ca6539f44e674bb3907fea72a16d` before deletion. Immediately
before deletion, all 61 roots still matched the literal inventory, resolved below
`/home/dev/ws_iap/src/iap/results/icra27/`, were directories rather than symlinks, contained zero tracked files,
and totaled `122694791115` logical bytes. No colcon/CMake/ROS process was active. The protected PDF hash and all
three shared workspace roots passed their preconditions.

An initial `rm -rf` form was rejected by the command executor before process creation and changed no path. The
same checks were repeated, then each validated literal root was removed with non-following `find -depth -delete`.
Post-delete verification found `0/61` roots remaining.

Filesystem observations (`df -B1`):

| Point | Size | Used | Available |
|---|---:|---:|---:|
| Immediately before deletion | 694831472640 | 632348499968 | 27112198144 |
| Immediately after deletion and `sync` | 694831472640 | 509306290176 | 150154407936 |

Observed available-space increase and used-space decrease were both `123042209792` bytes. This filesystem block
accounting differs from the summed logical candidate size by `347418677` bytes; both values are recorded rather
than conflated.

Preservation checks after deletion:

- shared build `/home/dev/ws_iap/build`: `4026917714` bytes;
- shared install `/home/dev/ws_iap/install`: `361883196` bytes;
- shared log `/home/dev/ws_iap/log`: `174476233` bytes;
- protected PDF SHA-256:
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`;
- ICRA-068 live/compact, ICRA-070 compact, ICRA-072 compact, all three registered development smokes and ordinary
  logs remain present.

The deleted roots are permanently absent and can be recovered only by rebuilding from retained source and
commands. No Git-tracked file, raw/compact/live evidence, ordinary log, shared workspace directory or protected
PDF was deleted.
