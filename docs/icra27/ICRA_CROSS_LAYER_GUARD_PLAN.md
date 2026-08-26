# ICRA-071 user-owned research-route guard plan

Status: **Implemented / Review REQUEST_CHANGES / non-blocking backlog under user decision 002**
Requirements: `IAP-RQ-000`, `IAP-RQ-423`, `IAP-RQ-424`

## Purpose and authority boundary

ICRA-071 prevents an implementation agent, Supervisor verdict or runner outcome from silently changing the
user's research route. Its canonical input is the `ICRA_USER_ROUTE_LOCK_V1` JSON block in
`docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md`.

The frozen route is `P0_P4_V2_P5`. P0+P5 remains a matched control, not an alternate main route. ICRA-071 is
pure repository governance: it changes no P0/P4/P5 algorithm, starts no ROS process, runs no GPU preflight,
creates no product build/install and issues no campaign.

```text
USER route lock
  -> static route/state/plan verifier
    -> pre-commit + pre-push + commit-msg guards
      -> focused mutation tests
        -> Supervisor Review
          -> governance backlog repair

USER decision 002
  -> ICRA-072 development vertical slice may proceed independently of that backlog
```

## Canonical route-lock parser

Add one parser that reads exactly one JSON object between the begin/end sentinels. It must reject:

- absent, duplicated, nested or reversed sentinels;
- invalid JSON, duplicate keys, unknown/missing schema fields or noncanonical value types;
- `route_owner != USER`, an empty `user_decision_id`, a non-40-hex approval anchor, or an anchor not reachable
  from the current repository history;
- duplicate required modules/arms/scenes, an empty research question/claim or unsupported guard strength;
- a protected route-lock edit made by the active Builder task.

The parser exposes a typed immutable value. Other guards consume it; they may not maintain literal mirrors of
the route, modules, claim or arms.

## Route, state and task verifier

Provide one deterministic repository-local command equivalent to:

```bash
python3 scripts/dev_planner/verify_icra_research_route.py \
  --route-lock docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md \
  --state AGENT_STATE.md \
  --task NEXT_TASK.md
```

It requires no ROS, GPU, build/install, network or untracked input and checks:

1. `AGENT_STATE.md.conference_route` equals `active_route` and the required modules/claim/campaign barrier agree
   with the route lock and active scope/plan headers.
2. Only one task and role are active. `TASK_READY` points to the same task/gate/owner in `NEXT_TASK.md`.
3. While ICRA-071 itself is active it permits only governance files. Under decision 002, ICRA-072 independently
   permits only the bounded development vertical-slice product surface in its task; this does not mark the guard PASS.
4. P0+P5 is named only as `P0_P5_CONTROL` or immutable history, never as the active conference route.
5. P4-v1 remains `SCIENTIFIC_NO_GO`; P4-v2 remains unqualified. No document may relabel old evidence.
6. Campaign state remains blocked before ICRA-079 Review PASS and a distinct user campaign decision.
7. A state containing `SCIENTIFIC_NO_GO` cannot simultaneously activate a different route/task unless the
   canonical lock contains a new user decision bound to the exact prior pushed anchor.
8. Without such a decision, the only NO-GO transition is
   `BLOCKED_AWAITING_USER_RESEARCH_DECISION`, `active_role=SUPERVISOR`, `next_task=NONE`.

The verifier emits a stable typed reason and nonzero exit for each failure. Passing it proves repository
consistency only; it does not authenticate that a same-permission process is the human user.

## Staged-diff research authority guard

The pre-commit path compares the staged route-lock protected fields with `HEAD`. If any protected field changes,
it requires all of the following in the staged state:

- a distinct `user_decision_id` and exact old/new values;
- an approval anchor equal to the pushed pre-change HEAD;
- synchronized scope, implementation plan, plan review, state, task, Supervisor log, requirements,
  traceability and changes documentation;
- `active_role=SUPERVISOR` until the route changeset and post-push rotation audit complete;
- no product/config/experiment or evidence file in the same staged changeset.

The Builder task must always fail if it stages the route-lock sentinel block, `AGENT_STATE.md`, `NEXT_TASK.md`
or a user-decision record. The Supervisor may stage them only for a user-directed route decision.

## Repository hooks

ICRA-071 replaces the current ineffective hook setup:

- add a repository-local installation/check command for `git config --local core.hooksPath .githooks`;
- reject absent, absolute or stale `core.hooksPath`;
- repair pre-commit matching for root-relative `src/`, `include/`, `apps/`, `msg/`, `cmake/`, `launch/`,
  `config/`, `scripts/`, `test/`, `tests/`, `tools/`, `docker/`, `data/`, `thirdparty/`, `.githooks/`, root
  `CMakeLists.txt`, `package.xml` and toolchain/build/config files;
- add an extension/category fallback outside generated/evidence trees so a new source/interface/config root
  cannot bypass synchronization solely because its directory is absent from the enumerated set;
- code/interface/config changes require staged `DEV_LOG.md`, `docs/CHANGES.md` and
  `docs/TRACEABILITY.md`;
- route/scope/claim files invoke the route verifier and staged-diff authority guard;
- add pre-push to re-run the clean-tree route verifier against the commits being pushed;
- add commit-msg validation requiring at least one valid `IAP-RQ-NNN` for every commit;
- remove `IAP_SKIP_DOCS` and add no replacement bypass environment variable.

The hooks must preserve unrelated tracked/untracked work and never stage files themselves.

## Adversarial test matrix

Tests construct mutations from one valid temporary repository fixture and prove typed failure for:

- active route changed to P0+P5 with no new user decision;
- P4 removed from required modules or treatment arms;
- primary max-risk claim changed to mean/CVaR or weakened to pure numerical noise;
- contingency/campaign activated by Supervisor verdict, runner output or Builder handoff;
- scientific NO_GO followed directly by an alternate `TASK_READY`;
- forged stale approval anchor, reused decision ID or unsynchronized plan/state/task;
- Builder staging route lock, Supervisor state or task files;
- every enumerated code/interface/config root and a new-root extension fallback without all three synchronized
  development documents;
- commit message without a valid requirement ID;
- missing/absolute hooks path, direct stale hook matcher and second installation;
- documentation-only historical text that does not alter protected active fields.

Positive cases cover the current user decision, an unchanged route with a normal Builder handoff, the required
NO-GO blocked state, and a synthetic Supervisor route proposal that does not activate anything.

## Documentation and evidence

The Builder records exact commands/exits and focused/full hermetic results in `DEV_LOG.md`, `docs/CHANGES.md`,
`docs/TRACEABILITY.md` and compact ICRA-071 evidence. Evidence binds the route-lock hash, verifier/hook hashes,
test inventory, current commit and `core.hooksPath` result. It must not edit the route-lock sentinel, Supervisor
state/task/verdict or historical r6 artifacts.

## Acceptance

ICRA-071 passes Builder handoff only when:

1. the canonical route-lock parser and route/state/task verifier pass the current repository;
2. every registered route/claim/authority mutation fails for its exact typed reason;
3. pre-commit, pre-push and commit-msg use the same implementation and contain no bypass variable;
4. root-relative code matching and three-document synchronization are proven;
5. repository-local `core.hooksPath=.githooks` is installed and verified without global Git mutation;
6. focused tests and complete hermetic static discovery pass;
7. no P4 product, ROS, GPU, live, campaign, build/install or retained evidence mutation occurs.

User decision `USER-ICRA-ROUTE-20260826-002` no longer makes this Review PASS a prerequisite for ICRA-072.
The repair remains required governance debt, but ICRA-072 may build the development-only end-to-end vertical
slice. This does not authorize a scientific/effect claim, G0D qualification, P5 qualification or campaign.

## Local-enforcement limitation

The user selected repository-local protection. It prevents ordinary automation mistakes but is not a security
boundary: a process with the same filesystem/Git authority can edit hooks or use `--no-verify`. Documentation,
tests and verdicts must state this limitation. Non-bypassable enforcement would require a protected GitHub
branch and an independent user approval identity, which are outside ICRA-071.
