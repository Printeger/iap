# IAP Topology Graph v0.1 - Scope and Legend

This document set is generated from the template in [src/iap/docs/iap_topology_minimum_template.md](src/iap/docs/iap_topology_minimum_template.md).

## Scope

1. Covered modules: odometry, sub-mapping, global-mapping, gnss, viewer.
2. Covered lanes: T0_Main, T1_Odom, T2_Bridge, T3_Sub, T4_Global, T5_GNSS, T6_Viewer.
3. Expansion depth: each L0 function expanded to at most L3.
4. Variable granularity:
- S level: persistent state variables.
- C level: key containers and handoff queues.
- L level: local variables listed only.

## Legend

| Symbol | Meaning |
|---|---|
| L0 | Trunk function node |
| L1-L3 | Expanded callees |
| HANDOFF | Cross-thread object transfer |
| callback | Slot-based event transfer |
| R | Read member variable |
| W | Write member variable |

## Files in this set

1. Mainline graph: [src/iap/docs/topology/10_swimlane_mainline.md](src/iap/docs/topology/10_swimlane_mainline.md)
2. Function cards: [src/iap/docs/topology/20_function_cards.md](src/iap/docs/topology/20_function_cards.md)
3. Variable ledger: [src/iap/docs/topology/30_variable_ledger.md](src/iap/docs/topology/30_variable_ledger.md)
4. Handoff matrix: [src/iap/docs/topology/40_cross_thread_handoffs.md](src/iap/docs/topology/40_cross_thread_handoffs.md)
