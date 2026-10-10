int32 drone_id

int32 order
int64 traj_id
builtin_interfaces/Time start_time
uint8 IMMEDIATE=0
uint8 AT_TIME=1
# Withdraw only the matching server queue; no curve or new authorization.
uint8 CANCEL_PENDING=2
uint8 REPLACE_PENDING=3
uint8 start_mode
# Expected server identities for a scheduled request; replacement is compare-and-swap.
int64 predecessor_id
int64 replace_pending_id

float64[] knots
geometry_msgs/Point[] pos_pts

float64[] yaw_pts
float64 yaw_dt
