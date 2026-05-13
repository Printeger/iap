# IAP current certified monitor result (§1.11)
# Published by the integrity monitor. The PL fields are monitor-fused
# certified outputs: PL_mon_q = max(PL_G_q, PL_L_q).

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

# Current monitor Integrity Margin
float64 im               # monitor_integrity_margin = AL - PL [m]

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
