# Diagnostic capture from one actual smoother finish. See gnss/postopt_evidence.hpp.
# Row-major means and covariance; tangent order X,V,B,R,E,actual clocks.
# The covariance belongs to linearization_means, not optimized_means.
string model
uint64 update_sequence
int64 frame_id
float64 state_stamp
float64 gnss_stamp
uint64 epoch_source_identity
string used_constellations
bool optimized_valid
bool covariance_valid
string failure_reason
string propagation
uint64[<=9] keys
uint32[<=9] tangent_dimensions
uint32[<=9] mean_dimensions
float64[<=45] optimized_means
float64[<=45] linearization_means
float64[<=841] joint_covariance_row_major
float64[<=45] propagated_optimized_means
float64[<=45] propagated_linearization_means
float64[<=841] propagated_joint_covariance
float64[<=841] propagation_transition
float64[<=841] propagation_noise
float64[<=2048] imu_measurements
float64[<=3] imu_noise
float64[<=6] imu_bias_hat
