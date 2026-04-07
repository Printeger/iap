#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Compact handoff summary from the hybrid CT local frontend to the compact backend.

#include <iap/odometry/bspline_lidar_factor_result.hpp>
#include <iap/odometry/integrated_bspline_gicp_factor.hpp>
#include <iap/odometry/spline_state_layout.hpp>

#include <cstddef>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/types/point_cloud.hpp>

namespace iap {

struct CTBackendSummary {
  gtsam::KeyVector active_pose_keys;
  std::vector<int> active_control_indices;
  std::size_t pose_key_count{0};
  std::size_t lidar_factor_count{0};
  bool has_velocity_state{false};
  // Bias states remain explicit even when gravity is supplied as an external reference.
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

struct SolverUpdateProfileRow {
  int frame_id{-1};
  double frame_stamp{0.0};
  std::string solver_mode{"BATCH_LM"};
  bool frontend_only_mode{false};
  bool local_layer_enabled{false};
  bool navigation_layer_enabled{false};
  bool used_incremental_solver{false};
  bool fallback_used{false};
  std::size_t new_factor_count{0};
  std::size_t new_value_count{0};
  std::size_t new_stamp_count{0};
  std::size_t query_key_count{0};
  std::size_t retired_key_count{0};
  std::size_t active_control_point_count{0};
  std::size_t active_pose_key_count{0};
  std::size_t active_aux_key_count{0};
  std::size_t persistent_key_count{0};
  std::size_t local_state_dimension{0};
  std::size_t local_residual_count{0};
  double solver_update_ms{0.0};
  double estimate_query_ms{0.0};
  double fallback_rebuild_ms{0.0};
  double relinearization_ms{0.0};
  double linearization_ms{0.0};
  double elimination_ms{0.0};
  double delta_solve_ms{0.0};
  std::size_t relinearized_variable_count{0};
  std::size_t reeliminated_variable_count{0};
  std::size_t relinearized_factor_count{0};
  std::size_t linearized_factor_count{0};
  std::size_t bayes_tree_clique_count{0};
  std::size_t affected_variable_count{0};
  std::size_t observed_key_count{0};
  std::size_t new_factor_index_count{0};
  std::size_t current_nonlinear_factor_count{0};
  std::size_t active_window_imu_factor_count{0};
  std::size_t active_window_velocity_factor_count{0};
  std::size_t active_window_lidar_factor_count{0};
  std::size_t active_window_lidar_current_segment_factor_count{0};
  std::size_t active_window_lidar_old_segment_factor_count{0};
  std::size_t active_window_prior_factor_count{0};
  std::size_t active_window_shared_jkg_touching_factor_count{0};
  std::size_t recalculated_imu_factor_count{0};
  std::size_t recalculated_velocity_factor_count{0};
  std::size_t recalculated_lidar_factor_count{0};
  std::size_t recalculated_lidar_current_segment_factor_count{0};
  std::size_t recalculated_lidar_old_segment_factor_count{0};
  std::size_t recalculated_lidar_same_support_factor_count{0};
  std::size_t recalculated_lidar_cross_support_factor_count{0};
  std::size_t recalculated_prior_factor_count{0};
  std::size_t recalculated_shared_jkg_touching_factor_count{0};
  std::size_t relinearized_pose_variable_count{0};
  std::size_t relinearized_aux_variable_count{0};
  std::size_t relinearized_shared_variable_count{0};
  std::size_t affected_pose_key_count{0};
  std::size_t affected_aux_key_count{0};
  std::size_t affected_shared_key_count{0};
  double isam_reported_update_ms{0.0};
  int optimize_count{0};
  double initial_error{0.0};
  double final_error{0.0};
  double error_drop_ratio{0.0};
  int iteration_count{0};
  std::string solver_status{"unavailable"};
};

struct LidarFactorInternalProfileRow {
  int frame_id{-1};
  double frame_stamp{0.0};
  std::string bucket_mode{"TIME_EPS"};
  std::size_t bucket_count{0};
  std::size_t factor_index{0};
  double representative_time{0.0};
  std::size_t points_in_bucket{0};
  std::size_t source_point_count{0};
  std::size_t target_candidate_count{0};
  std::size_t valid_correspondence_count{0};
  std::size_t effective_residual_count{0};
  double factor_total_ms{0.0};
  double correspondence_ms{0.0};
  double covariance_lookup_ms{0.0};
  double residual_eval_ms{0.0};
  double jacobian_eval_ms{0.0};
  double match_ratio{0.0};
  double inlier_ratio{0.0};
  double best_distance_mean{0.0};
  double best_second_gap_mean{0.0};
  std::size_t support_control_count{0};
  std::size_t support_pose_key_count{0};
  std::size_t active_control_point_count{0};
};

struct FrontendFrameProfile {
  int frame_id{-1};
  double stamp{0.0};
  std::string frontend_mode{"unknown"};
  bool frontend_only_mode{false};
  bool use_legacy_two_stage_path{false};
  std::string frontend_seed_mode{"last_pose_copy"};
  std::string frontend_seed_source{"last_pose_copy"};
  bool frontend_seed_fallback_used{false};
  std::size_t frontend_seed_imu_sample_count{0};
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
  int optimize_count{0};
  bool local_layer_enabled{false};
  bool navigation_layer_enabled{false};
  std::size_t local_layer_factor_count{0};
  std::size_t navigation_layer_factor_count{0};
  std::size_t local_layer_active_state_count{0};
  std::size_t navigation_layer_active_state_count{0};
  std::string solver_mode{"BATCH_LM"};
  std::size_t new_factor_count{0};
  std::size_t new_value_count{0};
  std::size_t retired_key_count{0};
  bool fallback_used{false};
  bool carried_prior_replay_success{false};
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

struct FrontendPoseDiagnostics {
  bool valid{false};
  double query_time{0.0};
  double frontend_target_time{0.0};
  double bucket_query_time{0.0};
  double seed_integration_end_time{0.0};
  bool frontend_target_time_consistent{false};
  double layout_domain_begin{0.0};
  double layout_domain_end{0.0};
  double representative_time{0.0};
  double bucket_representative_time{0.0};
  std::size_t representative_bucket_index{0};
  std::size_t points_in_bucket{0};
  double match_ratio{0.0};
  double inlier_ratio{0.0};
  double factor_total_ms{0.0};
  std::size_t candidate_correspondence_count{0};
  std::size_t accepted_correspondence_count{0};
  double accept_ratio{0.0};
  double registration_delta_translation_norm{0.0};
  double registration_delta_rotation_rad{0.0};
  bool uses_local_lidar_layout_override{false};
  std::string seed_mode{"last_pose_copy"};
  std::string seed_source{"last_pose_copy"};
  bool seed_fallback_used{false};
  std::size_t seed_imu_sample_count{0};
  gtsam::Pose3 seed_pose;
  gtsam::Pose3 optimized_pose;
  gtsam::KeyVector lidar_support_keys;
  gtsam::KeyVector query_support_keys;
  std::vector<std::size_t> lidar_support_control_indices;
  std::vector<std::size_t> query_support_control_indices;
};

struct BSplineUnifiedGraphContext {
  std::shared_ptr<const SplineStateLayout> layout;
  double min_active_stamp{0.0};
  bool frontend_only_mode{false};
  bool local_layer_enabled{true};
  bool navigation_layer_enabled{true};
  gtsam::KeyVector existing_keys;
};

struct BSplineLayerActivation {
  bool enabled{false};
  bool include_clock_states{false};
  bool retain_shared_gnss_anchor{false};
  std::vector<int> active_control_indices;
  std::vector<std::size_t> active_auxiliary_indices;
  gtsam::KeyVector retained_keys;

  std::size_t active_state_count() const {
    return active_control_indices.size() + active_auxiliary_indices.size() + retained_keys.size();
  }
};

struct BSplineLocalLayerContribution {
  struct LidarFactorHandle {
    std::size_t source_frame_index{0};
    std::size_t bucket_index{0};
    double representative_time{0.0};
    SplineBucketContext bucket_ctx;
    std::vector<std::size_t> support_control_indices;
    std::shared_ptr<IntegratedSplineGICPFactor> factor;
  };

  gtsam::NonlinearFactorGraph graph;
  BSplineLayerActivation activation;
  CTLocalFrontendDebugStats debug_stats;
  CTLocalFrontendProcessedOutput processed;
  std::vector<LidarFactorHandle> lidar_factor_handles;
  std::size_t lidar_factor_count{0};
  std::size_t imu_factor_count{0};
  std::size_t velocity_factor_count{0};
  bool uses_shared_imu_state{false};

  std::size_t factor_count() const { return graph.size(); }
};

struct BSplineNavigationLayerContribution {
  gtsam::NonlinearFactorGraph graph;
  BSplineLayerActivation activation;
  std::size_t gnss_pr_factor_count{0};
  std::size_t gnss_dop_factor_count{0};
  std::size_t clock_factor_count{0};

  std::size_t factor_count() const { return graph.size(); }
};

struct CTLocalFrontendResult {
  SplineStateLayout layout;
  gtsam::Values local_values;
  CTBackendSummary backend_summary;
  CTLocalFrontendDebugStats debug_stats;
  CTLocalFrontendProcessedOutput processed;
  FrontendPoseDiagnostics pose_diagnostics;
};

struct CTLocalFrontendShadowResult {
  bool valid{false};
  FrontendPoseDiagnostics pose_diagnostics;
  CTLocalFrontendDebugStats debug_stats;
  CTLocalFrontendProcessedOutput processed;
};

}  // namespace iap
