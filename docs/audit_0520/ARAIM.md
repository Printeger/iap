Let me do a deep dive into the ARAIM module's actual code structure before giving a critical assessment.

Deep dive ARAIM module structure

Searched for files matching `**/src/iap/src/iap/integrity/araim.cpp`, 1 match

Searched for files matching `**/src/iap/include/iap/integrity/araim.h*`, 1 match

Searched for files matching `**/src/iap/src/iap/gnss/visibility_predictor.cpp`, 1 match

Searched for files matching `**/src/iap/include/iap/gnss/visibility_predictor.h*`, 1 match

Searched for files matching `**/test*araim*.cpp`, 2 matches

Searched for files matching `**/src/iap/CMakeLists.txt`, 1 match

Read [](file:///home/dev/ws_iap/src/iap/src/iap/integrity/araim.cpp#1-1), lines 1 to 500

Read [](file:///home/dev/ws_iap/src/iap/include/iap/integrity/araim.hpp)

Read [](file:///home/dev/ws_iap/src/iap/include/iap/gnss/visibility_predictor.hpp)

Read [](file:///home/dev/ws_iap/src/iap/src/iap/gnss/visibility_predictor.cpp)

Read [](file:///home/dev/ws_iap/src/iap/src/iap/integrity/araim.cpp#300-300), lines 300 to 569

Read [](file:///home/dev/ws_iap/src/iap/src/iap/integrity/integrity_monitor.cpp#1-1), lines 1 to 300

Read [](file:///home/dev/ws_iap/src/iap/src/iap/integrity/integrity_extension.cpp#1-1), lines 1 to 300

Read [](file:///home/dev/ws_iap/src/iap/src/iap/planner/predicted_araim.cpp)

Searched for regex `#include.*araim`, 27 results

Read [](file:///home/dev/ws_iap/src/iap/test/test_predicted_araim.cpp)

Read [](file:///home/dev/ws_iap/src/iap/test/test_araim.cpp#1-1), lines 1 to 200

Searched for files matching `**/src/iap/include/iap/integrity/araim_types.hpp`, 1 match

Read [](file:///home/dev/ws_iap/src/iap/CMakeLists.txt#1-1), lines 1 to 500

Read [](file:///home/dev/ws_iap/src/iap/include/iap/integrity/araim_types.hpp)

Read memory

Created memory file [](file:///root/.vscode-server/data/User/workspaceStorage/fbb37a1416bd056fc35d64822ada45f0/GitHub.copilot-chat/memory-tool/memories/OWNjNThmZDgtZDAxYi00YjRiLTllOWQtODQzNmExOTM5NmIy/araim_deep_dive.md)

Let me read the key files directly.

Searched for files matching `**/araim*`, 14 matches

Read [](file:///home/dev/ws_iap/src/iap/include/iap/integrity/araim.hpp#1-1), lines 1 to 150

Read [](file:///home/dev/ws_iap/src/iap/src/iap/integrity/araim.cpp#1-1), lines 1 to 100

Read [](file:///home/dev/ws_iap/src/iap/src/iap/integrity/araim.cpp#100-100), lines 100 to 300

Read [](file:///home/dev/ws_iap/src/iap/include/iap/integrity/araim_types.hpp#1-1), lines 1 to 100

Read [](file:///home/dev/ws_iap/src/iap/include/iap/integrity/araim_types.hpp#100-100), lines 100 to 140

Searched for text `araim|Araim|araim_` (`**/integrity_monitor.cpp`), 20 results

Searched for text `araim|integrity|libiap` (`**/CMakeLists.txt`), 20 results

Read [](file:///home/dev/ws_iap/src/iap/CMakeLists.txt#90-90), lines 90 to 130

Read [](file:///home/dev/ws_iap/src/iap/CMakeLists.txt#70-70), lines 70 to 90

Read [](file:///home/dev/ws_iap/src/iap/CMakeLists.txt#35-35), lines 35 to 70

Read [](file:///home/dev/ws_iap/src/iap/test/test_araim.cpp#1-1), lines 1 to 80

Read [](file:///home/dev/ws_iap/src/iap/src/iap/planner/predicted_araim.cpp#1-1), lines 1 to 30

Now I have a thorough picture. Let me compile the critical assessment.

---

## ARAIM Module — Critical Assessment

### Current State Summary

| Aspect | Status |
|--------|--------|
| **Header** | `include/iap/integrity/araim.hpp` (147 lines) |
| **Types** | `include/iap/integrity/araim_types.hpp` (139 lines) |
| **Implementation** | `src/iap/integrity/araim.cpp` (~500+ lines) |
| **Compilation** | Part of `libiap.so` (monolithic shared library, ~50 source files bundled together) |
| **ROS Dependency** | None — pure C++ computation |
| **Downstream Users** | `IntegrityMonitor` (via `araim_.run()`), `PredictedAraimComputer` (via `araim_.predict_geometry()`) |
| **Test** | `test/test_araim.cpp` (12 unit tests for Q_inv, PL formula, state transitions) |
| **Parameters** | `Araim::Params` nested struct, populated from araim_params.yaml via `IntegrityMonitor` |

### ✅ What Meets Design Expectations

1. **Algorithm correctness.** The 3-term PL formula (§1.11), solution separation (§1.9), FDE (§1.10), and dynamic budget allocation (§1.8) are faithfully implemented and match the spec in `docs/dev_ARAIM/`.

2. **Clean API surface.** Two operating modes — `run()` for real epochs with residuals, `predict_geometry()` for planning with r=0 geometry proxy. Well-designed public/private split with static helper methods.

3. **No ROS dependency.** The class is pure computation, making it independently testable, reusable outside ROS, and easy to unit-test.

4. **Const-correctness.** All compute methods are `const`. No mutable state during computation.

5. **Comprehensive types.** `FaultHypothesis`, `SubsetSolution`, `AraimResult` are first-class types with per-axis fields, making debugging and logging straightforward.

### ❌ Critical Issues

#### 1. Monolithic `libiap.so` — ARAIM should be its own compilation unit

araim.cpp is compiled as one of ~50 source files squashed into a single `libiap.so`. This means:
- **Any change to ARAIM re-links the entire IAP library** (including odometry, mapping, viewer, etc.)
- **Test executables link against the entire `libiap.so`** instead of a focused ARAIM target
- **No independent versioning** — ARAIM can't evolve at its own pace
- **Circular dependency risk** — araim.hpp includes `gnss_types.hpp`, which is in the same library, creating a hidden coupling that could grow

**Recommendation:** Extract ARAIM into a `libiap_araim` OBJECT library or separate shared library:
```cmake
add_library(iap_araim OBJECT
  src/iap/integrity/araim.cpp
  src/iap/integrity/araim_types.hpp  # header-only, but listed for IDE visibility
)
target_link_libraries(iap PUBLIC iap_araim)
# Tests link against iap_araim directly, not the full libiap.so
```

#### 2. `Araim::Params` is nested inside the class — should be in araim_types.hpp

The params struct (60+ lines, 20+ fields) occupies half of araim.hpp. It should live in araim_types.hpp alongside `AraimResult` so that:
- Downstream code can forward-declare or include just the params
- Config loading code doesn't need the full `Araim` class
- The params can be reused by `PredictedAraimComputer` without pulling in the full engine

```cpp
// Current (problematic):
#include <iap/integrity/araim.hpp>  // pulls in Eigen, gnss_types, etc.
Araim::Params p;

// Better:
#include <iap/integrity/araim_types.hpp>  // lightweight, no Eigen/gnss deps
AraimParams p;  // standalone struct
```

#### 3. Tight coupling to `GnssEpoch` via header dependency

araim.hpp directly includes `gnss_types.hpp` and references `GnssEpoch` in public method signatures. This creates a compile-time dependency from the integrity module to the GNSS module. The `GnssEpoch` type is a heavy struct with satellite observations, timestamps, etc.

**Recommendation:** Either:
- **(Preferred)** Make `GnssEpoch` a forward-declared type and move `build_G()`, `build_W()`, `build_r()` to a separate translation unit that does the include, or
- Define an abstract interface for the epoch data that ARAIM consumes (N×4 design matrix + weight vector + residual vector), decoupling ARAIM from the specific GNSS data format

#### 4. araim.cpp is too large — `compute_core()` needs decomposition

The implementation file is 500+ lines, dominated by `compute_core()` which handles:
- Normal matrix assembly
- LDLT factorization
- Dynamic budget allocation (K factor computation)
- Fault-free PL computation
- Per-hypothesis subset solutions
- Worst-case PL accumulation
- FDE detection

This function is hard to review, test in isolation, and maintain. Each of these concerns should be a separate private method.

#### 5. Anonymous-namespace helper functions should be static members or in a detail header

Functions like `infer_constellation_id()`, `constellation_prior()`, `factorize_normal_matrix()`, and `covariance_from_factorization()` live in an anonymous namespace in araim.cpp. This is technically correct but:
- They can't be unit-tested directly
- If another `.cpp` in the same library has an identically-named anonymous-namespace function, silent ODR violations are possible
- The constellation ID mapping logic (GPS=1-32, GLO=33-56, GAL=57-88, BDS=89-152) should be in a shared GNSS utility, not buried in the ARAIM implementation

**Recommendation:** Move to either:
- `static` private methods of `Araim` (testable via friend or public test helper)
- A `detail/araim_math.hpp` header with inline functions

#### 6. File location: `src/iap/integrity/araim.cpp` is acceptable but suboptimal

The `integrity/` directory currently contains 5 files:
```
src/iap/integrity/
  araim.cpp                ← pure algorithm
  lidar_araim.cpp          ← pure algorithm
  fgo_information_manager.cpp ← iSAM2 interface
  integrity_monitor.cpp    ← orchestrator
  integrity_extension.cpp  ← compiled as SEPARATE .so (libintegrity_extension.so)
```

The directory mixes algorithm code (araim, lidar_araim) with system integration code (monitor, extension). A cleaner separation would be:
```
src/iap/integrity/
  araim/                   ← algorithm-only, no ROS/iSAM2 deps
    araim.cpp
    araim_types.hpp
  lidar_araim/
    lidar_araim.cpp
  integrity_monitor.cpp    ← orchestrator (depends on araim, lidar_araim, fgo_info)
  integrity_extension.cpp  ← ROS2 plugin (compiled separately)
  fgo_information_manager.cpp
```

But given the current project scale, this is a **nice-to-have**, not a blocker.

#### 7. `Q_inv()` is public static but shouldn't be in the `Araim` class

The inverse Q-function (Abramowitz & Stegun 26.2.23 approximation) is a general mathematical utility. It's currently a public `static` method of `Araim`, which means any code needing the normal quantile function must `#include <iap/integrity/araim.hpp>`. It should be in a shared math utility like `iap/util/math_utils.hpp`.

#### 8. Test coverage is unit-only, no integration or regression tests

test_araim.cpp has 12 tests covering Q_inv accuracy, PL formula, and basic state transitions. Missing:
- **Regression tests** with known GNSS epochs and golden-output PL values
- **Stress tests** with degenerate geometry (coplanar satellites, singular G^T W G)
- **Performance tests** — ARAIM runs per smoother update (10-30 Hz), so latency matters
- **FDE integration tests** — inject a fault, verify detection + exclusion
- **Constellation fault hypothesis tests** — GPS constellation fault vs. single-sat fault

#### 9. No explicit NaN/Inf guards in the compute path

As flagged in the audit (J.3), the ARAIM computation chain has no explicit NaN/Inf guards. While `factorize_normal_matrix()` checks `d.allFinite()`, the PL accumulation step (`worst_PL_E = max(worst_PL_E, ss.PL_E)`) would propagate NaN silently. Add `Eigen::isfinite()` checks in the PL accumulation loop.

#### 10. `PredictedAraimComputer` duplicates the `Araim` dependency pattern

`PredictedAraimComputer` (`src/iap/planner/predicted_araim.cpp`) also owns an `Araim` instance and calls `predict_geometry()`. This creates a planner → integrity dependency. While this is correct architecturally (the planner needs advisory PL), the fact that it's compiled into the same `libiap.so` obscures the module boundary.

### Summary Table

| # | Issue | Severity | Recommendation |
|---|-------|----------|----------------|
| 1 | Monolithic `libiap.so` | **HIGH** | Extract `libiap_araim` OBJECT library |
| 2 | `Params` nested in class | **MEDIUM** | Move to araim_types.hpp as standalone struct |
| 3 | Header dependency on `GnssEpoch` | **MEDIUM** | Forward-declare or abstract epoch interface |
| 4 | `compute_core()` too large | **MEDIUM** | Decompose into 5-6 private methods |
| 5 | Anonymous-namespace helpers | **LOW** | Move to static members or `detail/` header |
| 6 | File location in mixed `integrity/` dir | **LOW** | Consider `integrity/araim/` subdirectory |
| 7 | `Q_inv()` in wrong class | **LOW** | Move to `iap/util/math_utils.hpp` |
| 8 | Missing integration/regression tests | **HIGH** | Add golden-output and FDE integration tests |
| 9 | No NaN/Inf guards in PL loop | **HIGH** | Add `isfinite()` checks |
| 10 | `PredictedAraimComputer` coupling | **LOW** | Fine for now; document the dependency |

### Verdict

The ARAIM module is **algorithmically sound** and **well-documented**, but its **structural integration** (monolithic library, tight header coupling, oversized implementation) creates friction for independent development, testing, and maintenance. The extraction into a separate OBJECT library with a standalone params struct is the highest-impact, lowest-risk refactoring to perform first.