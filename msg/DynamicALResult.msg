# IAP Dynamic Alert Limit result (§1.12)

std_msgs/Header header

float64 hal                  # horizontal alert limit from trunk geometry [m]
float64 val                  # vertical alert limit from altitude/canopy [m]
float64 al                   # min(hal, val) [m]

int32   nearest_trunk_id     # closest trunk landmark ID (-1 if none)
float64 nearest_trunk_dist   # distance to nearest trunk [m]

float64 current_altitude     # AGL altitude [m]
float64 canopy_height_est    # estimated canopy height [m]
bool    al_from_trunk        # true if AL = HAL, false if AL = VAL
