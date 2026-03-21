# IAP Trunk landmark state for visualization and debugging
# Published per-trunk after smoother update

std_msgs/Header header

int32   trunk_id             # landmark ID in the map
float64 center_x             # trunk center in world frame [m]
float64 center_y
float64 radius               # estimated trunk radius [m]
float64 confidence           # [0,1] association confidence
float64 sigma_x              # position uncertainty X [m]
float64 sigma_y              # position uncertainty Y [m]
int32   n_observations       # number of times this trunk has been observed
bool    active               # true if in the current smoother window
