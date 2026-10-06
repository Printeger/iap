std_msgs/Header header
int64 frame_id
float64 scan_end_stamp_s
uint64 sensor_receipt_steady_ns
geometry_msgs/Pose t_map_lidar
sensor_msgs/PointCloud2 deskewed_hits_lidar
string frame_contract_id

# Explicit beam outcomes associated with this registered scan. The arrays are
# empty and complete=false when no synchronized simulator evidence exists.
string sensor_model_id
uint32 horizontal_samples
uint32 vertical_samples
float64 horizontal_fov_rad
float64 vertical_min_rad
float64 vertical_max_rad
float64 min_range_m
float64 max_range_m
bool beam_evidence_complete
uint8[] beam_outcomes
float32[] beam_direction_x
float32[] beam_direction_y
float32[] beam_direction_z
float32[] beam_ranges_m
string beam_content_hash

# Diagnostic association at this scan's serialization; never observation
# authorization. Missing old producers leave reason empty.
string beam_binding_reason
uint64 beam_received_count
uint64 beam_invalid_count
uint64 beam_evicted_count
float64 beam_history_oldest_stamp_s
float64 beam_history_newest_stamp_s
float64 beam_same_start_end_stamp_s

# True only for the first estimator frame that defines the planner-map datum.
# Its pose is not the result of an ICP alignment against an older frame; scan
# error is still charged by LocalMotionAssurance.
bool source_is_map_reference

# Immutable SLAM registration health associated with this source scan. Active
# window consumers must not reconstruct it from shorter-lived downstream
# history because keyframes can remain active for much longer.
bool source_health_valid
float64 source_health_stamp_s
bool source_icp_degenerate
float64 source_icp_rmse
float64 source_icp_condition
float64 source_icp_gamma_lidar
# Compatibility diagnostics for absolute map/world localization only. These
# values are not a current-to-source relative error and must not be deducted
# from planner-local obstacle clearance.
float64 source_lidar_pl_e
float64 source_lidar_pl_n
float64 source_lidar_pl_u
