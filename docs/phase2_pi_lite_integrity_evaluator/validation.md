# Phase 2 PI-lite Validation

Build:

```bash
cd /home/dev/ws_iap
bash src/iap/tools/build_phase1_ego_planner_closed_loop.sh
source install/setup.bash
```

Run demo10:

```bash
ros2 launch iap demo10_ego_planner_pi_lite_eval.launch.py \
  start_rviz:=false \
  run_duration_s:=60 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_gnss:=true \
  use_araim:=true \
  phase2_pl_model:=constant_current \
  phase2_al_model:=cloud_clearance
```

Run Phase 1 validation:

```bash
python3 src/iap/tools/phase1/validate_phase1_closed_loop.py \
  --run-dir /home/dev/ws_iap/src/iap/log/latest \
  --official
```

Run Phase 2 offline alignment and validation:

```bash
python3 src/iap/tools/phase2/analyze_phase2_integrity_eval.py \
  --run-dir /home/dev/ws_iap/src/iap/log/latest

python3 src/iap/tools/phase2/validate_phase2_integrity_eval.py \
  --run-dir /home/dev/ws_iap/src/iap/log/latest
```

The validator fails when required Phase 2 outputs are missing, online truth usage is reported, the odom source is not `/drone_0_visual_slam/odom`, required finite columns contain NaN/inf, `AL_pred` is entirely unavailable, ARAIM is present but no finite offline IM/PL can be aligned, or Phase 1 official validation fails.

Warnings are expected for conservative PL, mostly unsafe predicted margins, high tracking error, or missing actual AL/IM fields in incomplete runs.
