# Safety Planner v1 Acceptance Contract

**Status:** Draft

**Owner:** IAP Safety Planner

**Version:** 1.0.0-draft

This document defines version-specific PASS/FAIL evidence for the Safety
Planner far-boundary contract. It owns scenario values and test thresholds; it
does not redefine planner authority or safety semantics.

Unfilled values are open acceptance decisions and create no PASS claim while
this document remains `Draft`.

## 1. Standard scenario

The standard integration scenario is:

```text
icra_dense_forest_four_fork_v2
```

An acceptance run must bind:

- scenario/catalog version or content hash;
- launch profile and resolved parameters;
- task mode;
- HAL/VAL policy and values;
- planner revision and dirty/clean state;
- simulator, map, predictor, and Oracle versions;
- random seed, run duration, and repetition index; and
- the run manifest and required evidence identities.

The current reference profile uses `MISSION_BEST_EFFORT` with configured HAL
and VAL. Exact values remain owned by the versioned scenario/profile and must
be copied into the retained run manifest rather than inferred from defaults.

## 2. Online and Oracle separation

The evidence flow is:

```text
online sensors and advisory evidence
        ↓
Safety Planner
        ↓
published trajectory / commands / odometry
        ↓
simulation Oracle/reference evaluator
```

The Oracle may use complete simulation geometry and information unavailable to
the online system. Online planner, predictor, assurance, publication, and
runtime code must never read Oracle data.

For the run's configured HAL/VAL, the evaluator may derive:

```text
R_oracle(p, t)
  = max(HPL_oracle(p, t) / HAL,
        VPL_oracle(p, t) / VAL)

IM_oracle(p, t)
  = min(HAL - HPL_oracle(p, t),
        VAL - VPL_oracle(p, t))
```

The Oracle is a simulation reference, not a claim of stronger physical ground
truth than its model supports.

## 3. Acceptance layers

### 3.1 Run validity

Before behavior can PASS, the run must establish:

- committed revision and clean IAP worktree;
- valid run manifest and artifact layout;
- required processes remained healthy for the evaluated interval;
- required topics and evidence identities are present and time-consistent;
- planner used online evidence only; and
- teardown did not truncate authoritative evidence.

A top-level launch exit code of zero is insufficient.

### 3.2 Mission and physical safety

The actual vehicle execution must:

- reach the scenario goal within the configured tolerance and duration;
- remain collision-free under the simulation reference; and
- avoid an unauthorized emergency termination or indefinite HOLD.

Goal tolerance, duration, and physical sampling rules are open values listed in
Section 6.

### 3.3 Navigation-integrity behavior

Evaluation follows the configured task mode.

For `STRICT_GLOBAL`, actual execution must satisfy the versioned Oracle
integrity boundary and unknown-evidence policy.

For `MISSION_BEST_EFFORT`, PASS requires the versioned mission policy rather
than pretending every `R_oracle >= 1` sample is a physical collision. The
policy must define permitted exposure, required route preference, recovery or
exit behavior, and disallowed exposure. Local motion safety remains mandatory.

The evaluator samples actual execution. Checking only the search guide or an
unpublished B-spline cannot establish PASS.

### 3.4 Execution correctness

Evidence must show that:

- every published normal candidate had a matching final assurance PASS;
- rejected, stale, late, cancelled, or identity-inconsistent work was not
  executed;
- a failed replacement did not overwrite an independently valid active
  trajectory; and
- runtime replan or emergency actions carry a typed policy reason.

## 4. Causality tests

Causality tests are separate cases in the acceptance suite. They are not
repeated implicitly for every baseline integration run.

### 4.1 Risk-causality A/B case

Hold physical geometry, start, goal, and controller configuration fixed while
changing only the versioned advisory-risk fixture. The executed route must
change in the direction predicted by the task policy when physical obstacles
alone do not force that change.

### 4.2 Mirror metamorphic case

Use a fixture whose physical geometry, start, goal, and risk field have a
defined mirror transform. After mirroring only the declared fixture inputs,
the executed route response must satisfy the versioned mirror tolerance.

This test applies only when the complete fixture and evaluation model meet the
declared symmetry assumptions.

## 5. Repetition and result aggregation

The suite must define before execution:

- repetition count;
- allowed seeds;
- per-run hard failures;
- aggregate success requirement;
- treatment of infrastructure failures; and
- rules for retrying an invalid run.

A failed behavioral run cannot be discarded as an outlier after inspecting its
result. Infrastructure-invalid runs remain distinct from planner FAIL results.

## 6. Open numerical decisions

The following values must be fixed before this contract becomes `Active`:

| Item | Required definition |
|---|---|
| Goal | Position/velocity tolerance and hold time |
| Duration | Maximum mission and HOLD duration |
| Collision | Vehicle envelope, temporal/spatial sampling, contact tolerance |
| Oracle integrity | Task-mode-specific risk/exposure/recovery rule |
| Unknown evidence | Permitted duration/extent and required response |
| Sampling | Command, odometry, B-spline, and Oracle interpolation rules |
| Causality | Minimum route-response effect size |
| Mirror test | Transform and symmetry tolerance |
| Repetitions | Count, seeds, and aggregate PASS rule |
| Performance | Planning deadline, missed-deadline allowance, resource limits |

These values belong here or in a referenced, versioned machine-readable
profile. They must not be hidden in analyzer defaults.

## 7. Required retained evidence

The run artifact must retain enough bounded evidence to reconstruct:

- resolved scenario and policy configuration;
- physical and advisory generation identities;
- planning attempts and typed outcomes;
- published trajectory identities and assurance bindings;
- actual command/odometry execution;
- collision and Oracle evaluation;
- runtime replan/emergency actions; and
- final per-run and aggregate verdicts.

RViz is diagnostic evidence and is never a PASS criterion.

## 8. Activation conditions

This acceptance contract may become `Active` only after:

1. every open numerical decision has a versioned value;
2. the analyzer and run manifest bind those values explicitly;
3. baseline, risk-causality, and applicable mirror cases have executable test
   procedures; and
4. a retained run on a committed, clean revision demonstrates the complete
   evidence chain.
