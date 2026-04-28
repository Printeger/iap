# IAP Sim Build Source

`src/iap/sim/ego_planner_swarm_ws/src` is the intended source tree for the
migrated EGO simulator/planner packages used by IAP demos.

The top-level `src/ego-planner-swarm` copy is intentionally ignored by colcon
via `COLCON_IGNORE` to prevent package-name collisions.

Use:

```bash
src/iap/tools/build_iap_sim.sh
```

Verify package discovery:

```bash
source /opt/ros/jazzy/setup.bash
colcon list --base-paths src/iap src/gnss_comm src/iap/sim/ego_planner_swarm_ws/src
```
