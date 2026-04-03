#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Compact handoff summary from the hybrid CT local frontend to the compact backend.

#include <iap/odometry/bspline_lidar_factor_result.hpp>
#include <iap/odometry/spline_state_layout.hpp>

#include <cstddef>
#include <string>
#include <vector>

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/types/point_cloud.hpp>

namespace iap {

struct CTBackendSummary {
  gtsam::KeyVector active_pose_keys;
  std::vector<int> active_control_indices;
  std::size_t pose_key_count{0};
  std::size_t lidar_factor_count{0};
  bool has_velocity_state{false};
  bool has_bias_state{false};
};

struct CTLocalFrontendDebugStats {
  std::size_t bucket_count{0};
  double local_solve_time_ms{0.0};
  std::size_t lidar_residual_count{0};
  std::size_t imu_residual_count{0};
  std::vector<int> active_local_controls;
};

struct FrontendLMIterationProfileRow {
  int iteration_index{0};
  double cost_before{0.0};
  double cost_after{0.0};
  bool accepted{false};
  double lambda_before{0.0};
  double lambda_after{0.0};
  double linear_solve_ms{0.0};
};

struct FrontendBucketProfileRow {
  std::size_t source_frame_index{0};
  std::size_t bucket_index{0};
  std::string bucket_mode{"TIME_EPS"};
  double representative_time{0.0};
  std::size_t points_in_bucket{0};
  std::size_t valid_correspondence_count{0};
  double match_ratio{0.0};
  double inlier_ratio{0.0};
  std::size_t target_point_count{0};
  std::size_t candidate_evaluation_count{0};
  double lookup_or_correspondence_ms{0.0};
  double accumulation_ms{0.0};
  double factor_total_ms{0.0};
  std::size_t time_bucket_count{0};
  double mean_time_bucket_population{0.0};
  std::size_t max_time_bucket_population{0};
};

struct FrontendFrameProfile {
  int frame_id{-1};
  double stamp{0.0};
  std::string frontend_mode{"unknown"};
  bool frontend_only_mode{false};
  std::string bucket_mode{"TIME_EPS"};
  std::size_t actual_bucket_count{0};
  std::size_t total_source_points{0};
  double preprocess_ms{0.0};
  double target_map_prep_ms{0.0};
  std::size_t warning_count_for_frame{0};
  double bucket_build_ms{0.0};
  double lidar_factor_build_ms{0.0};
  double imu_factor_build_ms{0.0};
  double lm_solve_ms{0.0};
  double marginalization_ms{0.0};
  double backend_update_ms{0.0};
  double backend_optimize_ms{0.0};
  double publish_ms{0.0};
  double local_mapping_update_ms{0.0};
  double global_mapping_update_ms{0.0};
  double submap_registration_ms{0.0};
  std::size_t active_control_point_count{0};
  std::size_t active_pose_key_count{0};
  std::size_t local_state_dimension{0};
  std::size_t imu_sample_count{0};
  std::size_t imu_factor_count{0};
  std::size_t imu_residual_count{0};
  std::size_t lidar_factor_count{0};
  std::size_t lidar_residual_count{0};
  std::size_t gnss_factor_count{0};
  std::size_t local_residual_count{0};
  std::size_t carried_prior_count{0};
  std::size_t backend_factor_count{0};
  std::size_t backend_state_count{0};
  int lm_iteration_count{0};
  bool lm_trace_expected{false};
  bool lm_trace_emitted{false};
  int lm_trace_row_count{0};
  double lm_initial_cost{0.0};
  double lm_final_cost{0.0};
  int lm_rejected_step_count{0};
  int lm_damping_change_count{0};
  double target_snapshot_clone_ms{0.0};
  double target_voxel_lookup_prep_ms{0.0};
  double target_covariance_prep_ms{0.0};
  double source_to_target_transform_ms{0.0};
};

struct CTLocalFrontendProcessedOutput {
  gtsam_points::PointCloud::ConstPtr deskewed_source_cloud;
  std::vector<BSplineLidarFactorResult> lidar_results;
  BSplineLidarWindowProfileSummary lidar_window_summary;
  double total_lidar_factor_error{0.0};
  FrontendFrameProfile frame_profile;
  std::vector<FrontendBucketProfileRow> bucket_profiles;
  std::vector<FrontendLMIterationProfileRow> lm_iterations;
};

struct CTLocalFrontendResult {
  SplineStateLayout layout;
  gtsam::Values local_values;
  CTBackendSummary backend_summary;
  CTLocalFrontendDebugStats debug_stats;
  CTLocalFrontendProcessedOutput processed;
};

}  // namespace iap
