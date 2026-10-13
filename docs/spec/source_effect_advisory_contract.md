# G2 source effects and one fused Advisory — v1

Status: active research, unqualified. The user's 2026-10-13 source-effect Goal
supersedes the old A0/A1 execution budget and G2→G3→G4 schedule. This round
executes only G2-L/G/F; no new planner, G3 or G4 development is authorized.
The original local increment contract, numerical gates and historical failures
remain unchanged. Full joint innovation/PL coverage is not a proxy prerequisite.

## Objects and evidence

- L: original native GLIO 1.000 s relative increment, starting physical IMU XY,
  plus relative attitude, weak directions, real matches, effective factor support,
  faults, continuity and all missing requests. No re-registration or truth fit.
- G: native optimized `E(0) + R(0) * p_world_imu` in ECEF, compared to simulator
  `origin_ecef + R_ecef_enu * p_truth`. Producer origin LLA is
  `[31.2304,121.4737,25]`; one analytic WGS84/ENU relation covers every condition.
  Errors are projected into that fixed producer ENU basis. Estimator E/R changes
  are part of the error, not an evaluation alignment. Truth is offline only.
- Source effects require physical measurement changes, actual admission/injection,
  estimator response and independent labels. Satellite counts or nominal FIM
  alone cannot establish global anchoring. Kernels/loss sigma are not noise laws.

Global state samples use the original post-update owner frame/stamp and update
identity; no old E/R is promoted to fresh when keys are absent. Duplicate frame
updates are retained, with the greatest completed update sequence selected
before seeing error. Truth endpoints retain the original 0.050 s bracket gate.
Local endpoint selection/IMU gates remain exactly the existing v1.2 evaluator.

## Bounded physical pilot

One matched four-cell pilot, seed 9100, original four-fork scene/map seed 41021,
original start/goal, GLIO/FGO/FDE/monitor/configuration and budgets. Advisory and
posterior preference are OFF. Each run is 105 s; this is not task arrival
acceptance. Original EGO supplies motion, without planner changes. Motion and
self-built map differences are recorded as mediators/confounders.

| Cell | GNSS physical measurements | LiDAR physical returns |
|---|---|---|
| GG/LG | Original 1 m PR / .03 m/s Doppler, open sky, GPS+BDS NAV | 10 m range |
| GG/LB | Same GNSS | 3 m range |
| GB/LG | Same GNSS, all GPS/BDS observables dropped during [30,60] s | 10 m range |
| GB/LB | Same physical dropout | 3 m range |

Time is the original GNSS generator's elapsed time from its first epoch. NLOS,
map occlusion, skymask, multipath are disabled in both GNSS arms so the planned
difference is observable absence/recovery. All original estimator weights,
canopy/elevation sigma floors and integrity gates remain. Dropout is applied at
the measurement producer, not to the final FIM or monitor result. These are
provisional "good/bad" labels; actual contrast must be established from data.
The original producer skips noise draws for dropped observables. Common seeds
pair startup; recovery white-noise streams can diverge. Record this consequence
and use independent matched-run uncertainty rather than claiming identical
post-recovery physical draws. The initial noiseless fix/ideal IMU are retained;
they can make absolute drift too small to identify an anchoring benefit.

Pair-start requests have a fixed 2 s cadence, first delay 10 s, 46 one-second
pairs / 92 inputs. Keep the original .5 s pre / 2.3 s post report selection,
complete request ledger and unique frozen payloads; no new raw/full GPU dump,
observer or source-family profile. Existing factor/debug/source quality evidence
supplies support diagnosis. GNSS producer draws are recorded separately.

Before the pilot, test global-variable availability across absence/recovery
with the real native solver. A lifetime/identity bug may receive a minimal
software correction without adding a prior, changing measurement weights,
altering the original 5 s lag or changing motion authority. The original failed
case remains evidence; correction does not prove physical source effects.

## Formal freezing and effects

After a discriminating pilot, freeze one evaluator, protocol and, only if
supported, one new evidence-based fused candidate. Formal seed groups are
9101/9102/9103, each with all four cells (maximum twelve new runs). No pilot is
upgraded to independent validation; no failed candidate seed replacement.
Use run/group as the independent unit, not windows. Do not collect formal runs
when a failed physical contrast or decisive mechanism prevents interpretation.

Predeclared time phases: pre [15,28], absence [32,58], post [65,95] s. Report
native availability, admitted PR/Doppler counts and effective sigma, raw producer
noise/drop/NLOS, actual match support and motion/region coverage in each phase.
G main effects: absolute horizontal/3D error, robust phase-wise change and error
growth, and actual recovery correction (world-pose versus E/R contributions).
Use phase medians and last-minus-first time-quartile median growth; at least six
valid native global states per phase. A phase remains one run, never six
independent samples. Recovery is the first actually injected post-dropout
update: split the exact ECEF change as previous-R × delta-world-position and
delta-anchor + delta-R × current-position. This algebraic split is not a causal
factor attribution. Truth displacement and error change remain separate.
Recovery can worsen error or local continuity; both outcomes remain visible.
L main effect: original native XY error, both GNSS strata reported, with original
regions/missing; pose increment and weak-direction diagnostics are supplemental.

Pilot contrast floors for interpreting an effect are 0.001 m local and 0.02 m
global, as detection limits only, not PL or task tolerances. Formal conclusions
require the same effect direction across all three independent paired groups,
run-level contrasts and uncertainty reported without window-as-N inference.
Near-floor or confounded/unavailable differences are INCONCLUSIVE. Native local
ordering retains .7 concordance, .5 Spearman, >=6 windows, >=2 original regions
and all three runs passing. Better global effects never erase failed local gates.

## Fusion decision and closure

Only actual L/G evidence can support one minimal online-computable candidate.
No A2/A3 probe/trace/weight sweeps. There is one final `s(p)` and no single-source
planning cost or second map. Check meaningful output sensitivity, joint Pareto
consistency and fixed offline route/region exposure. Do not invent a total
ordering for local/global tradeoffs without a declared task/error-scale basis.
Detection floors above do not supply that utility basis.

QUALIFIED requires both independent effects and the frozen fused/local/global
gates. PARTIALLY QUALIFIED, NOT SUPPORTED UNDER CURRENT CONDITIONS or
INCONCLUSIVE are valid completed research deliveries when backed by bounded
experiments and a concrete six-question conclusion matrix. None grants real G3
inputs or overall IAP success. Keep all old G1/M2–M5 and V5/A0/A1 failures.
