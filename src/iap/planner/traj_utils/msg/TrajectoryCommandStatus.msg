uint8 QUEUED=0
uint8 ACTIVATED=1
uint8 CANCELED=2
uint8 REJECTED=3

std_msgs/Header header
uint8 state

uint64 execution_instance_id
int64 trajectory_id
builtin_interfaces/Time start_time
string curve_hash

uint64 parent_execution_instance_id
int64 parent_trajectory_id
builtin_interfaces/Time parent_start_time
string parent_curve_hash

builtin_interfaces/Time planned_event_time
builtin_interfaces/Time actual_event_time
string rejection_reason
