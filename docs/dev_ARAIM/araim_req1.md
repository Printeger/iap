You are working inside the current repository. Your job is to inspect, plan, implement, validate, and report — not to speculate.

You must first read the real code and infer the actual architecture from the repository itself. Do not assume the architecture described below is fully correct. Use it only as intent/background, then verify everything in code before changing anything.

==================================================
MISSION
==================================================

Upgrade the current ARAIM / integrity module in three areas, in this exact priority order:

1) CPU parallelization of the hypothesis loop
2) remove or reduce the per-hypothesis “rebuild matrix + explicit inverse” path
3) strengthen the FGO snapshot exported to the integrity module

You must:
- inspect the repository first
- write a concrete repository-specific plan
- then implement the plan
- then validate correctness and performance
- then report results

Do not stop after the plan. Continue through implementation and validation unless blocked by a real technical issue.

==================================================
KNOWN INTENT / BACKGROUND (TO VERIFY IN CODE)
==================================================

From prior inspection, the current implementation likely behaves like this:

- ARAIM is currently an epoch-level WLS / solution-separation engine.
- It does NOT fully re-run the nonlinear smoother/FGO for each fault hypothesis.
- Subset hypotheses are evaluated by modifying current epoch geometry/weights and recomputing subset quantities.
- The smoother currently exports only a weak snapshot (for example sigma_p / lambda_p-like summaries), mainly for fallback PL proxy / reporting.
- Existing hypotheses likely include:
  - single satellite
  - constellation
  - single trunk
- Trunk hypothesis may currently be bookkeeping-only or only partially wired in the WLS path.
- Hypothesis evaluation may still be serial.
- The code may explicitly form and invert per-hypothesis matrices.

You must confirm or correct all of the above from the actual code.

==================================================
NON-NEGOTIABLE WORKFLOW
==================================================

PHASE 1 — REPOSITORY INSPECTION
You must identify:
- entry points
- relevant files
- classes/functions
- control/data flow
- current algorithmic behavior
- existing build/test infrastructure
- available concurrency infrastructure
- available linear algebra backends / decomposition utilities
- current snapshot extraction path from smoother/FGO into integrity code

Then write a repository-specific plan before editing code.

PHASE 2 — IMPLEMENTATION
After the plan, implement the changes directly in the repository.
Keep the project buildable after each step.

PHASE 3 — VALIDATION
Build the code.
Run tests.
Add focused tests if coverage is missing.
Benchmark before/after.
Check numerical equivalence to baseline.
Then report results.

Do not skip validation.

==================================================
OBJECTIVES IN DETAIL
==================================================

--------------------------------
OBJECTIVE A — PARALLELIZE HYPOTHESIS LOOP
--------------------------------

Requirements:
- Find the exact hypothesis loop.
- Parallelize it safely on CPU.
- Prefer the least invasive solution that matches the repository style and build system.
- Use existing concurrency infrastructure if available; otherwise choose a practical solution (OpenMP / TBB / thread pool / std::async) that fits the repo.
- Ensure thread safety:
  - no racy writes
  - deterministic aggregation if needed
  - no hidden mutation of shared state
- Keep the nominal/full-solution computation outside the parallel loop if possible.

Deliverable:
- repository-specific explanation of what was parallelized
- implementation details
- proof/reasoning of thread safety
- numerical equivalence vs serial baseline

--------------------------------
OBJECTIVE B — AVOID NAIVE PER-HYPOTHESIS FULL INVERSE
--------------------------------

Requirements:
- Inspect exactly how the current code computes the nominal solution and each subset solution.
- Replace explicit inverse usage where reasonable with decomposition-based solves.
- Reuse nominal/shared quantities across hypotheses.
- Prefer correctness-preserving improvements over aggressive but fragile hacks.

Acceptable optimization strategies include:
- LDLT / LLT / QR solve path instead of explicit inverse
- reuse of nominal factorization where possible
- low-rank update/downdate for single-satellite / constellation hypotheses if practical
- Woodbury / Sherman-Morrison style updates if numerically safe in this codebase
- caching and reusing matrix assembly components
- computing only the actually required covariance projections/diagonals instead of full inverse, where possible

Important:
- If a full low-rank downdate path is too invasive for this pass, implement the strongest safe intermediate optimization:
  - remove explicit inverse
  - reduce repeated assembly
  - reuse decompositions and shared intermediates
- Then document what remains for a future pass.

Deliverable:
- old computational path vs new computational path
- exact files/functions changed
- validation of numerical consistency
- timing improvement

--------------------------------
OBJECTIVE C — STRENGTHEN FGO SNAPSHOT
--------------------------------

Requirements:
- Inspect the current smoother/FGO callback and snapshot extraction path.
- Replace the current weak snapshot with a stronger explicit integrity snapshot.

At minimum, attempt to export:
- timestamp/frame/epoch id
- nominal current state reference
- marginal position covariance block
- richer covariance/info blocks if available
- active factor metadata relevant to integrity:
  - factor type/source
  - satellite id / constellation id
  - trunk / landmark id
  - grouping information
- anything else already cheaply available that can support future FGO-aware ARAIM

Constraints:
- do not create dangerous ownership/lifetime bugs
- do not explode memory use
- keep it practical and minimally invasive
- if some desired data are unavailable from the current APIs, say exactly what is missing and expose the cleanest future extension point

Deliverable:
- stronger snapshot data structure/interface
- extraction code
- transport/consumption path updates
- comments documenting current use vs future intended use

==================================================
ALGORITHMIC GUARDRAILS
==================================================

You must NOT silently change the current algorithm into a full nonlinear per-hypothesis FGO re-optimization system.

Preserve current semantics unless a change is necessary for:
- correctness
- thread safety
- performance infrastructure
- snapshot plumbing

In particular:
- if the current ARAIM is still WLS-centered, keep it WLS-centered for this task
- if trunk hypotheses are not yet mathematically wired into the WLS subset solve, do not invent unsupported math silently
- if a limitation remains, state it explicitly

==================================================
REQUIRED OUTPUT FORMAT
==================================================

As you work, produce the following sections in your final report:

1. Repository inspection summary
   - relevant files
   - classes/functions
   - confirmed current behavior
   - corrected assumptions

2. Implementation plan
   - ordered tasks
   - rationale
   - risks

3. Code changes
   - grouped by logical step
   - exact files modified
   - what each change does

4. Validation
   - build status
   - tests run / tests added
   - numerical equivalence results
   - benchmark results

5. Remaining limitations
   - what is still not solved
   - what should be the next pass

==================================================
SUCCESS CRITERIA
==================================================

This task is only complete if:
- the plan is based on actual code, not guesswork
- the hypothesis loop is safely parallelized
- the naive per-hypothesis full inverse path is improved or eliminated where practical
- the FGO snapshot is meaningfully strengthened
- the repository builds
- numerical equivalence is checked
- performance evidence is provided
- unresolved issues are stated clearly

Start now with repository inspection, then continue through implementation and validation.