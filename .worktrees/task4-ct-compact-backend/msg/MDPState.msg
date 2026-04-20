# IAP MDP state vector for RL interface (§9)
# This is the state representation exposed to an RL agent

std_msgs/Header header

# Integrity metrics
float64 hpl              # [m]
float64 vpl              # [m]
float64 hal              # [m]
float64 val              # [m]
float64 im               # AL - PL [m]
uint8   integrity_state  # 0=SAFE, 1=SAFE_EXCLUDED, 2=UNSAFE

# GNSS quality features
int32   n_sv_used
float64 pdop
float64 sigma_h          # [m]

# Trunk environment features
int32   n_trunks_visible
float64 nearest_trunk_dist   # [m]
float64 tdop

# Vehicle state
float64 altitude_agl     # [m]
float64 speed            # ‖v‖ [m/s]
float64 heading          # [rad]

# Distance to goal
float64 dist_to_goal     # [m]
