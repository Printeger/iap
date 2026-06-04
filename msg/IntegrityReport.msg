# IAP current certified monitor result (§1.11)
# Published by the integrity monitor. The PL fields are monitor-fused
# certified outputs: PL_mon_q = max(PL_G_q, PL_L_q).
#
# Step 1: im now = min(HAL-HPL, VAL-VPL), not AL-PL.
# Step 3: source breakdown, H/V margins, fusion source, numerical failure
# flags appended (no existing fields removed or reordered).

std_msgs/Header header

# Three-state: 0=SAFE, 1=SAFE_EXCLUDED, 2=UNSAFE
uint8  integrity_state

# Current certified monitor Protection Levels [m]
float64 hpl              # monitor_fused_hpl = max(pl_e, pl_n) [m]
float64 vpl              # monitor_fused_vpl = pl_u [m]
float64 pl_e             # monitor_fused_pl_e [m]
float64 pl_n             # monitor_fused_pl_n [m]
float64 pl_u             # monitor_fused_pl_u [m]

# Alert Limits [m]
float64 hal              # horizontal alert limit [m]
float64 val              # vertical alert limit [m]

# Current monitor Integrity Margin (Step 1: now min(im_h, im_v), not AL-PL)
float64 im               # monitor_integrity_margin = min(im_h, im_v) [m]

# H/V safety margins (Step 1)
float64 im_h             # horizontal margin = HAL - HPL [m]
float64 im_v             # vertical margin = VAL - VPL [m]
float64 im_min           # = min(im_h, im_v), same as `im` for backward compat

# Fault-free PL
float64 pl_ff            # fault-free horizontal PL [m]
float64 pl_ff_v          # fault-free vertical PL [m]

# K multipliers used
float64 k_ff_used
float64 k_fa_used

# GNSS quality
int32   n_sv_used            # number of satellites used
int32   n_constellations     # number of constellations
float64 pdop
float64 sigma_h              # fault-free horizontal sigma [m]

# ARAIM diagnostics
int32   n_hypotheses
int32   n_detected           # number of faults detected (FDE)
int32[] excluded_prns        # excluded satellite PRNs
int32[] excluded_trunk_ids   # excluded trunk IDs

# Trunk geometry
int32   n_trunks_observed
float64 tdop

# =========================================================================
# Step 3: Source breakdown, fusion diagnostics, numerical failure flags
# All fields appended after the original schema — no reordering.
# =========================================================================

# --- GNSS source breakdown ---
bool    gnss_valid
float64 gnss_hpl
float64 gnss_vpl
float64 gnss_pl_e
float64 gnss_pl_n
float64 gnss_pl_u
float64 gnss_pl_ff
float64 gnss_k_ff_used
float64 gnss_k_fa_used
int32   gnss_n_hyp
int32   gnss_n_det

# --- LiDAR source breakdown ---
bool    lidar_valid
float64 lidar_hpl
float64 lidar_vpl
float64 lidar_pl_e
float64 lidar_pl_n
float64 lidar_pl_u
int32   lidar_n_hyp
int32   lidar_n_det
string  lidar_worst_mode

# --- Fallback source breakdown ---
# TODO(Step 4): replace with explicit FallbackSourceResult struct
bool    fallback_valid       # true when fallback covariance was usable
float64 fallback_hpl         # fallback HPL before any GNSS/LiDAR override
float64 fallback_vpl         # fallback VPL before any GNSS/LiDAR override

# --- Final fusion diagnostics ---
string  fusion_mode          # current fusion policy (max_pl until Step 4)
string  final_hpl_source     # "GNSS", "LIDAR", "FALLBACK", or "UNKNOWN"
string  final_vpl_source
string  final_pl_source

# --- Numerical failure flags (Step 2) ---
bool    fallback_pl_invalid
bool    gnss_araim_invalid
bool    lidar_integrity_invalid
bool    hal_invalid
bool    val_invalid
bool    im_invalid
bool    any_nan_rejected
bool    any_inf_rejected
bool    negative_variance_rejected
bool    degenerate_geometry
string  failure_reason
