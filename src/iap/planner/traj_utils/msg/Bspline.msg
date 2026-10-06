int32 drone_id

int32 order
int64 traj_id
builtin_interfaces/Time start_time
uint8 IMMEDIATE=0
uint8 AT_TIME=1
uint8 start_mode

float64[] knots
geometry_msgs/Point[] pos_pts

float64[] yaw_pts
float64 yaw_dt
