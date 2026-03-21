# IAP per-axis ARAIM result (§1.11)
# Published by integrity monitor at each GNSS epoch

std_msgs/Header header

# Three-state: 0=SAFE, 1=SAFE_EXCLUDED, 2=UNSAFE
uint8  integrity_state

# Protection Levels [m]
float64 hpl              # max(pl_e, pl_n) [m]
float64 vpl              # pl_u [m]
float64 pl_e             # East protection level [m]
float64 pl_n             # North protection level [m]
float64 pl_u             # Up protection level [m]

# Alert Limits [m]
float64 hal              # horizontal alert limit [m]
float64 val              # vertical alert limit [m]

# Integrity Margin
float64 im               # AL - PL [m] (positive = safe)

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
