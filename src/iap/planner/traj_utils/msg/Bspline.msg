int32 drone_id

int32 order
int64 traj_id
uint64 execution_instance_id
string curve_hash
builtin_interfaces/Time start_time

uint64 parent_execution_instance_id
int64 parent_traj_id
builtin_interfaces/Time parent_start_time
string parent_curve_hash
float64 parent_switch_elapsed_s

float64[] knots
geometry_msgs/Point[] pos_pts

float64[] yaw_pts
float64 yaw_dt

