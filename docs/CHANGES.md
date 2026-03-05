# Changes Log (IAP)

> 规则：任何代码改动必须在这里记录，并包含 IAP-RQ-XXX。

## Unreleased
- (none)

## 2026-03-05
- IAP-RQ-001: Renamed ROS2 package from `glim` to `iap`.
  - `src/glim/` → `src/iap/` (C++ source directory)
  - `include/glim/` → `include/iap/` (public header directory)
  - `cmake/glim-config.cmake.in` → `cmake/iap-config.cmake.in`
  - CMakeLists.txt: `add_library(glim)` → `add_library(iap)`, all install targets/exports renamed.
  - All `glim_LIBRARIES` → `iap_LIBRARIES`; install paths `share/glim` → `share/iap`, `bin/glim` → `bin/iap`.
  - `colcon build --packages-select iap` passes; `ros2 pkg list | grep iap` shows `iap`.
- Init: add traceability & agent rules. (IAP-RQ-000)