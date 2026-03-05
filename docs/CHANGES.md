# Changes Log (IAP)

> 规则：任何代码改动必须在这里记录，并包含 IAP-RQ-XXX。

## Unreleased
- (none)

## 2026-03-05
- IAP-RQ-000: Fix git `core.hooksPath` → `.githooks`; pre-commit doc-guard verified. AGENTS.md + CHANGES/TRACEABILITY/REQS confirmed present.
- IAP-RQ-002: Add `apps/iap_status.cpp` (smoke-test demo), `launch/iap_demo.launch.py`, `.clangd` (CompilationDatabase path). CMakeLists.txt: `IAP_VERSION` define, install launch/. `colcon build` 产生 compile_commands.json; workspace root symlink.
  - `ros2 launch iap iap_demo.launch.py` 可用，输出 `iap_status: OK`
- IAP-RQ-001: Generate `docs/SPEC_VS_IMPL.md`：对照 spec/ 与代码现状，汇总已实现/待实现条目，建议新目录结构。
- IAP-RQ-001: Renamed ROS2 package from `glim` to `iap`.
  - `src/glim/` → `src/iap/` (C++ source directory)
  - `include/glim/` → `include/iap/` (public header directory)
  - `cmake/glim-config.cmake.in` → `cmake/iap-config.cmake.in`
  - CMakeLists.txt: `add_library(glim)` → `add_library(iap)`, all install targets/exports renamed.
  - All `glim_LIBRARIES` → `iap_LIBRARIES`; install paths `share/glim` → `share/iap`, `bin/glim` → `bin/iap`.
  - `colcon build --packages-select iap` passes; `ros2 pkg list | grep iap` shows `iap`.
- Init: add traceability & agent rules. (IAP-RQ-000)