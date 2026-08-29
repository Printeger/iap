# Current technical task — ICRA-077A bounded formal-identity repair

> Active gate: `ICRA-077_HELD_OUT_CONFIRMATION`
> Milestone: `ICRA-077A_LAYER4_FORMAL_IDENTITY_REPAIR`
> Scope: repair the formal experiment configuration freeze and evidence identity model; do not access held-out outcomes or execute ICRA-077B

This file is a technical work brief, not an agent ownership, handoff, review, or approval state. Ordinary
development may iterate and commit normally under `AGENTS.md`. The restrictions below apply only when running
the named formal experiment protocol.

## Starting authority and retained evidence

Preserve replay-004/005, verification-003/004 and freeze-006/007 as immutable Review-rejected history. The latest
offline candidate still validates 1017 source / 958 install / 360 order records and no held-out/GPU/ROS/live work
occurred, but ICRA-077A remains BLOCKED/NOT PASS for exactly two Review findings:

1. current protected-route fields and the protocol fingerprint can be rewritten together because the validator
   does not derive the protected identity from the frozen roadmap Git blob;
2. current evidence ambiguously combines an `icra077a` schema with `task=ICRA-076`, generic PASS and
   `icra077_authorized=false` while retaining ICRA-076 BLOCKED/user-bypassed/NOT PASS.

ICRA-076's accepted wrong-domain and omitted-thirdparty-input debts remain unchanged and must not be repaired,
hidden or relabelled. Route `P0_P4_V2_P5`, P4-v1 `SCIENTIFIC_NO_GO`, P4-v2, ICRA-071, protected claims, formal
arms, scenes, thresholds, seeds/order and campaign activation remain unchanged.

## Repair 1 — immutable protected-route cross-binding

1. After validating the exact-three governance blob records, read the frozen roadmap content only through its
   verified Git blob OID at the frozen pushed commit. Parse that content with the canonical strict route parser
   and derive the canonical protected-field object/hash from the frozen blob itself.
2. Parse the current roadmap independently with the same formal-configuration parser. Require its exact frozen
   scientific fields to equal the frozen-blob fields. Ordinary explanatory prose may change without altering
   the experiment configuration.
3. Any configured fingerprint must equal the value derived from the verified frozen blob; it cannot silently
   authorize changed scientific inputs. A future formal-configuration change requires a separately selected new
   frozen commit/blob identity; this task does not implement such a change.
4. Add fail-closed adversaries for coordinated mutation of current scientific configuration + config SHA, each
   frozen field, frozen-blob/config mismatch, duplicate/unknown keys and legitimate later prose. The reproduced
   `P0_P5_CONTROL` joint mutation must fail.

## Repair 2 — unambiguous evidence identity

1. Replace ambiguous task/result/authorization fields across the new replay, verification and freeze schemas
   with exact, non-conflicting identities that distinguish at least:
   - active task ID `ICRA-077`;
   - active milestone `ICRA-077A_LAYER4_FORMAL_IDENTITY_REPAIR`;
   - artifact purpose `ICRA076_PREREGISTRATION_AUTHORITY_REPAIR`;
   - validation result, which is not a scientific Gate PASS;
   - ICRA-076 disposition `BLOCKED_USER_BYPASSED_NOT_PASS`;
   - downstream `icra077b_authorized=false` until formal protocol validation passes.
2. Do not retain a generic `task=ICRA-076/result=PASS` pair or an unsuffixed `icra077_authorized` field in the
   fresh schema. Validate exact keys/values and cross-bind the identities across replay, verification and freeze.
3. Add adversaries for stale task, milestone, artifact purpose, generic/scientific PASS ambiguity, ICRA-076
   relabelling and premature ICRA-077B authorization. Existing freeze-007 remains unchanged.

## Fresh pushed-source evidence and exit

1. Fetch-confirm divergence `0 0`; preserve the protected PDF and exact untracked state. Implement only the two
   repairs above with focused tests and minimal schema/config/validator changes.
2. Commit and push repair source before evidence. Fetch-confirm `0 0`, run the exact shared six-package build in
   `/home/dev/ws_iap/{build,install,log}`, then run all existing focused ICRA-073/074/075/076/077A tests, paired-
   tamper/identity adversaries and static validator.
3. Produce fresh non-overwriting identities no earlier than replay-006, verification-005 and freeze-008. Bind
   pushed repair source, both accepted ICRA-076 debts, exact-three blobs, frozen-derived/current-matched protected
   route, strict executable/install bytes and the unambiguous evidence identities.
4. After evidence creation, rerun the exact suite and validate freeze-008 at the final pushed evidence HEAD plus
   a controlled later-prose fixture. Preserve the formal evidence required by the protocol.
5. Return `ICRA077A_FORMAL_IDENTITY_REPAIR_VALIDATED` only after all scoped changes/evidence are pushed
   and fetch-confirmed `0 0`; otherwise retain the first typed blocker.

## Forbidden scope

- No held-out opening/generation/analysis, GPU, ROS, live, 360 rows, ICRA-077B, ICRA-078, threshold/SESOI/sample-
  size/seed/order change, retry/exclusion, qualification, campaign or effect/publication claim.
- Do not broaden the exact-three governance set or weaken product/config/launch/runner/analyzer/test/build-input/
  install current-byte validation, output containment or source admission.
- Preserve `/home/dev/ws_iap/{build,install,log}`, all raw/compact/live/scientific evidence, ordinary logs,
  hidden user artifacts and untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`.
- Advance to ICRA-077B or ICRA-078 only after the preceding formal protocol validation passes.
