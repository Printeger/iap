#include <iap/odometry/odometry_estimation_bspline.hpp>
// Commit 0 migration boundary:
// This translation unit still executes the existing fixed 4-control-point local
// spline window path. Follow-up work will introduce an explicit knot vector and
// a unified spline evaluator shared by IMU/GNSS/LiDAR factor assembly. For this
// commit we only freeze the migration boundary and CPU-first frontend default;
// math behavior, residual models, public interfaces, plugin names, shared-state
// integration, ROS topics, and log keywords stay unchanged.

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_points/ann/kdtree2.hpp>
#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <spdlog/spdlog.h>

#include <iap/common/log_config.hpp>
#include <iap/common/log_paths.hpp>
#include <iap/common/cloud_covariance_estimation.hpp>
#include <iap/common/imu_integration.hpp>
#include <iap/odometry/initial_state_estimation.hpp>
#include <iap/util/config.hpp>
#include <iap/util/shared_state.hpp>
#include <iap/odometry/callbacks.hpp>

#ifdef GTSAM_POINTS_USE_CUDA
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>
#include <iap/odometry/integrated_bspline_gicp_factor_gpu.hpp>
#include <iap/odometry/integrated_bspline_gicp_factor_gpu_kernel.hpp>
#endif

namespace glim {

namespace {

using Callbacks = OdometryEstimationCallbacks;

template <typename T>
void hash_combine(std::size_t* seed, const T& value) {
  const std::size_t hashed = std::hash<T>{}(value);
  *seed ^= hashed + 0x9e3779b97f4a7c15ULL + (*seed << 6U) + (*seed >> 2U);
}

iap::SplineKnotMode parse_knot_mode(const std::string& mode) {
  if (mode == "non_uniform" || mode == "non-uniform" || mode == "NON_UNIFORM") {
    return iap::SplineKnotMode::NonUniform;
  }
  return iap::SplineKnotMode::Uniform;
}

BSplineLidarTargetMode parse_lidar_target_mode(const std::string& mode) {
  if (mode == "GLOBAL_IVOX_REFERENCE" || mode == "global_ivox_reference" || mode == "global_ivox") {
    return BSplineLidarTargetMode::GLOBAL_IVOX_REFERENCE;
  }
  return BSplineLidarTargetMode::ACTIVE_WINDOW_SNAPSHOT;
}

const char* to_string(BSplineLidarTargetMode mode) {
  switch (mode) {
    case BSplineLidarTargetMode::ACTIVE_WINDOW_SNAPSHOT:
      return "active_window_snapshot";
    case BSplineLidarTargetMode::GLOBAL_IVOX_REFERENCE:
      return "global_ivox_reference";
  }
  return "unknown";
}

BSplineGpuLidarBackend parse_gpu_lidar_backend(const std::string& mode) {
  if (mode == "KERNEL" || mode == "kernel" || mode == "ct_kernel") {
    return BSplineGpuLidarBackend::KERNEL;
  }
  return BSplineGpuLidarBackend::BUCKET;
}

const char* to_string(BSplineGpuLidarBackend backend) {
  switch (backend) {
    case BSplineGpuLidarBackend::BUCKET:
      return "bucket";
    case BSplineGpuLidarBackend::KERNEL:
      return "kernel";
  }
  return "unknown";
}

iap::CTLocalFrontend::LidarBucketMode parse_lidar_bucket_mode(const std::string& mode) {
  if (mode == "FIXED_COUNT" || mode == "fixed_count" || mode == "fixed") {
    return iap::CTLocalFrontend::LidarBucketMode::FIXED_COUNT;
  }
  if (mode == "SINGLE_BUCKET" || mode == "single_bucket" || mode == "single") {
    return iap::CTLocalFrontend::LidarBucketMode::SINGLE_BUCKET;
  }
  return iap::CTLocalFrontend::LidarBucketMode::TIME_EPS;
}

const char* to_string(iap::CTLocalFrontend::LidarBucketMode mode) {
  switch (mode) {
    case iap::CTLocalFrontend::LidarBucketMode::TIME_EPS:
      return "time_eps";
    case iap::CTLocalFrontend::LidarBucketMode::FIXED_COUNT:
      return "fixed_count";
    case iap::CTLocalFrontend::LidarBucketMode::SINGLE_BUCKET:
      return "single_bucket";
  }
  return "unknown";
}

iap::BSplineUnifiedSolverMode parse_bspline_unified_solver_mode(const std::string& mode) {
  if (mode == "INCREMENTAL_SMOOTHER" || mode == "incremental_smoother" || mode == "incremental") {
    return iap::BSplineUnifiedSolverMode::INCREMENTAL_SMOOTHER;
  }
  return iap::BSplineUnifiedSolverMode::BATCH_LM;
}

const char* frontend_frame_profile_csv_header() {
  return "frame_id,stamp,frontend_mode,frontend_only_mode,use_legacy_two_stage_path,bucket_mode,actual_bucket_count,total_source_points,"
         "preprocess_ms,target_map_prep_ms,warning_count_for_frame,bucket_build_ms,lidar_factor_build_ms,"
         "imu_factor_build_ms,lm_solve_ms,marginalization_ms,backend_update_ms,backend_optimize_ms,publish_ms,"
         "local_mapping_update_ms,global_mapping_update_ms,submap_registration_ms,active_control_point_count,"
         "active_pose_key_count,local_state_dimension,optimize_count,local_layer_enabled,navigation_layer_enabled,"
         "local_layer_factor_count,navigation_layer_factor_count,local_layer_active_state_count,navigation_layer_active_state_count,"
         "solver_mode,new_factor_count,new_value_count,retired_key_count,fallback_used,carried_prior_replay_success,imu_sample_count,imu_factor_count,imu_residual_count,"
         "lidar_factor_count,lidar_residual_count,gnss_factor_count,local_residual_count,carried_prior_count,"
         "backend_factor_count,backend_state_count,lm_iteration_count,lm_trace_expected,lm_trace_emitted,"
         "lm_trace_row_count,lm_initial_cost,lm_final_cost,lm_rejected_step_count,lm_damping_change_count,"
         "target_snapshot_clone_ms,target_voxel_lookup_prep_ms,target_covariance_prep_ms,"
         "source_to_target_transform_ms\n";
}

const char* frontend_lidar_factor_profile_csv_header() {
  return "frame_id,stamp,source_frame_index,bucket_index,bucket_mode,representative_time,points_in_bucket,"
         "valid_correspondence_count,match_ratio,inlier_ratio,target_point_count,candidate_evaluation_count,"
         "lookup_or_correspondence_ms,accumulation_ms,factor_total_ms,time_bucket_count,"
         "mean_time_bucket_population,max_time_bucket_population\n";
}

const char* solver_update_profile_csv_header() {
  return "frame_id,frame_stamp,solver_mode,frontend_only_mode,local_layer_enabled,navigation_layer_enabled,"
         "used_incremental_solver,fallback_used,new_factor_count,new_value_count,new_stamp_count,query_key_count,"
         "retired_key_count,active_control_point_count,active_pose_key_count,active_aux_key_count,persistent_key_count,"
         "local_state_dimension,local_residual_count,solver_update_ms,estimate_query_ms,fallback_rebuild_ms,"
         "relinearization_ms,linearization_ms,elimination_ms,delta_solve_ms,relinearized_variable_count,"
         "reeliminated_variable_count,relinearized_factor_count,linearized_factor_count,bayes_tree_clique_count,"
         "affected_variable_count,observed_key_count,new_factor_index_count,current_nonlinear_factor_count,"
         "isam_reported_update_ms,optimize_count,initial_error,final_error,error_drop_ratio,iteration_count,"
         "solver_status\n";
}

const char* lidar_factor_internal_profile_csv_header() {
  return "frame_id,frame_stamp,bucket_mode,bucket_count,factor_index,representative_time,points_in_bucket,"
         "source_point_count,target_candidate_count,valid_correspondence_count,effective_residual_count,"
         "factor_total_ms,correspondence_ms,covariance_lookup_ms,residual_eval_ms,jacobian_eval_ms,match_ratio,"
         "inlier_ratio,best_distance_mean,best_second_gap_mean,support_control_count,support_pose_key_count,"
         "active_control_point_count\n";
}

const char* frontend_lm_iteration_csv_header() {
  return "frame_id,stamp,iteration_index,cost_before,cost_after,accepted,lambda_before,lambda_after,linear_solve_ms\n";
}

const char* frame_warning_profile_csv_header() {
  return "frame_id,stamp,warning_count,warning_categories,top_warning_message\n";
}

std::string csv_escape(const std::string& value) {
  if (value.find_first_of(",\"\n\r") == std::string::npos) {
    return value;
  }

  std::string escaped;
  escaped.reserve(value.size() + 2);
  escaped.push_back('"');
  for (const char ch : value) {
    if (ch == '"') {
      escaped.push_back('"');
    }
    escaped.push_back(ch);
  }
  escaped.push_back('"');
  return escaped;
}

double error_drop_ratio(double initial_error, double final_error) {
  if (!(initial_error > 0.0) || !std::isfinite(initial_error) || !std::isfinite(final_error)) {
    return 0.0;
  }
  return (initial_error - final_error) / initial_error;
}

iap::LidarFactorInternalProfileRow make_lidar_factor_internal_profile_row(
  int frame_id,
  double frame_stamp,
  const std::string& bucket_mode,
  std::size_t bucket_count,
  std::size_t factor_index,
  const iap::BSplineLocalLayerContribution::LidarFactorHandle& handle,
  const iap::BSplineLidarFactorResult& factor_result,
  std::size_t active_control_point_count) {
  iap::LidarFactorInternalProfileRow row;
  row.frame_id = frame_id;
  row.frame_stamp = frame_stamp;
  row.bucket_mode = bucket_mode;
  row.bucket_count = bucket_count;
  row.factor_index = factor_index;
  row.representative_time = handle.representative_time;
  row.points_in_bucket = handle.bucket_ctx.point_indices.size();
  row.source_point_count = factor_result.profile.source_point_count;
  row.target_candidate_count = factor_result.profile.candidate_evaluation_count;
  row.valid_correspondence_count = factor_result.profile.matched_point_count;
  row.effective_residual_count = factor_result.profile.inlier_point_count;
  row.factor_total_ms = factor_result.profile.total_ms;
  row.correspondence_ms = factor_result.profile.correspondence_ms;
  row.covariance_lookup_ms = 0.0;
  row.residual_eval_ms = 0.0;
  row.jacobian_eval_ms = 0.0;
  row.match_ratio = factor_result.profile.match_ratio;
  row.inlier_ratio = factor_result.profile.inlier_ratio;
  row.best_distance_mean = factor_result.profile.mean_match_distance;
  row.best_second_gap_mean = factor_result.profile.mean_score_gap;
  row.support_control_count = handle.bucket_ctx.support.ctrl_indices.size();
  row.support_pose_key_count = handle.bucket_ctx.support.pose_keys.size();
  row.active_control_point_count = active_control_point_count;
  return row;
}

void write_frontend_frame_profile_row(std::FILE* file, const iap::FrontendFrameProfile& profile) {
  if (!file) {
    return;
  }

  std::ostringstream row;
  row << std::fixed << std::setprecision(9);
  bool first = true;
  const auto add = [&](const auto& value) {
    if (!first) {
      row << ',';
    }
    first = false;
    row << value;
  };

  add(profile.frame_id);
  add(profile.stamp);
  add(profile.frontend_mode);
  add(profile.frontend_only_mode ? 1 : 0);
  add(profile.use_legacy_two_stage_path ? 1 : 0);
  add(profile.bucket_mode);
  add(profile.actual_bucket_count);
  add(profile.total_source_points);
  add(profile.preprocess_ms);
  add(profile.target_map_prep_ms);
  add(profile.warning_count_for_frame);
  add(profile.bucket_build_ms);
  add(profile.lidar_factor_build_ms);
  add(profile.imu_factor_build_ms);
  add(profile.lm_solve_ms);
  add(profile.marginalization_ms);
  add(profile.backend_update_ms);
  add(profile.backend_optimize_ms);
  add(profile.publish_ms);
  add(profile.local_mapping_update_ms);
  add(profile.global_mapping_update_ms);
  add(profile.submap_registration_ms);
  add(profile.active_control_point_count);
  add(profile.active_pose_key_count);
  add(profile.local_state_dimension);
  add(profile.optimize_count);
  add(profile.local_layer_enabled ? 1 : 0);
  add(profile.navigation_layer_enabled ? 1 : 0);
  add(profile.local_layer_factor_count);
  add(profile.navigation_layer_factor_count);
  add(profile.local_layer_active_state_count);
  add(profile.navigation_layer_active_state_count);
  add(profile.solver_mode);
  add(profile.new_factor_count);
  add(profile.new_value_count);
  add(profile.retired_key_count);
  add(profile.fallback_used ? 1 : 0);
  add(profile.carried_prior_replay_success ? 1 : 0);
  add(profile.imu_sample_count);
  add(profile.imu_factor_count);
  add(profile.imu_residual_count);
  add(profile.lidar_factor_count);
  add(profile.lidar_residual_count);
  add(profile.gnss_factor_count);
  add(profile.local_residual_count);
  add(profile.carried_prior_count);
  add(profile.backend_factor_count);
  add(profile.backend_state_count);
  add(profile.lm_iteration_count);
  add(profile.lm_trace_expected ? 1 : 0);
  add(profile.lm_trace_emitted ? 1 : 0);
  add(profile.lm_trace_row_count);
  add(profile.lm_initial_cost);
  add(profile.lm_final_cost);
  add(profile.lm_rejected_step_count);
  add(profile.lm_damping_change_count);
  add(profile.target_snapshot_clone_ms);
  add(profile.target_voxel_lookup_prep_ms);
  add(profile.target_covariance_prep_ms);
  add(profile.source_to_target_transform_ms);
  row << '\n';
  std::fputs(row.str().c_str(), file);
}

void write_frontend_lidar_factor_profile_rows(
  std::FILE* file,
  const int frame_id,
  const double stamp,
  const std::vector<iap::FrontendBucketProfileRow>& profiles) {
  if (!file) {
    return;
  }

  for (const auto& profile : profiles) {
    std::ostringstream row;
    row << std::fixed << std::setprecision(9);
    bool first = true;
    const auto add = [&](const auto& value) {
      if (!first) {
        row << ',';
      }
      first = false;
      row << value;
    };

    add(frame_id);
    add(stamp);
    add(profile.source_frame_index);
    add(profile.bucket_index);
    add(profile.bucket_mode);
    add(profile.representative_time);
    add(profile.points_in_bucket);
    add(profile.valid_correspondence_count);
    add(profile.match_ratio);
    add(profile.inlier_ratio);
    add(profile.target_point_count);
    add(profile.candidate_evaluation_count);
    add(profile.lookup_or_correspondence_ms);
    add(profile.accumulation_ms);
    add(profile.factor_total_ms);
    add(profile.time_bucket_count);
    add(profile.mean_time_bucket_population);
    add(profile.max_time_bucket_population);
    row << '\n';
    std::fputs(row.str().c_str(), file);
  }
}

void write_solver_update_profile_row(std::FILE* file, const iap::SolverUpdateProfileRow& row_data) {
  if (!file) {
    return;
  }

  std::ostringstream row;
  row << std::fixed << std::setprecision(9);
  bool first = true;
  const auto add = [&](const auto& value) {
    if (!first) {
      row << ',';
    }
    first = false;
    row << value;
  };

  add(row_data.frame_id);
  add(row_data.frame_stamp);
  add(row_data.solver_mode);
  add(row_data.frontend_only_mode ? 1 : 0);
  add(row_data.local_layer_enabled ? 1 : 0);
  add(row_data.navigation_layer_enabled ? 1 : 0);
  add(row_data.used_incremental_solver ? 1 : 0);
  add(row_data.fallback_used ? 1 : 0);
  add(row_data.new_factor_count);
  add(row_data.new_value_count);
  add(row_data.new_stamp_count);
  add(row_data.query_key_count);
  add(row_data.retired_key_count);
  add(row_data.active_control_point_count);
  add(row_data.active_pose_key_count);
  add(row_data.active_aux_key_count);
  add(row_data.persistent_key_count);
  add(row_data.local_state_dimension);
  add(row_data.local_residual_count);
  add(row_data.solver_update_ms);
  add(row_data.estimate_query_ms);
  add(row_data.fallback_rebuild_ms);
  add(row_data.relinearization_ms);
  add(row_data.linearization_ms);
  add(row_data.elimination_ms);
  add(row_data.delta_solve_ms);
  add(row_data.relinearized_variable_count);
  add(row_data.reeliminated_variable_count);
  add(row_data.relinearized_factor_count);
  add(row_data.linearized_factor_count);
  add(row_data.bayes_tree_clique_count);
  add(row_data.affected_variable_count);
  add(row_data.observed_key_count);
  add(row_data.new_factor_index_count);
  add(row_data.current_nonlinear_factor_count);
  add(row_data.isam_reported_update_ms);
  add(row_data.optimize_count);
  add(row_data.initial_error);
  add(row_data.final_error);
  add(row_data.error_drop_ratio);
  add(row_data.iteration_count);
  add(row_data.solver_status);
  row << '\n';
  std::fputs(row.str().c_str(), file);
}

void write_lidar_factor_internal_profile_rows(
  std::FILE* file,
  const std::vector<iap::LidarFactorInternalProfileRow>& rows_data) {
  if (!file) {
    return;
  }

  for (const auto& row_data : rows_data) {
    std::ostringstream row;
    row << std::fixed << std::setprecision(9);
    bool first = true;
    const auto add = [&](const auto& value) {
      if (!first) {
        row << ',';
      }
      first = false;
      row << value;
    };

    add(row_data.frame_id);
    add(row_data.frame_stamp);
    add(row_data.bucket_mode);
    add(row_data.bucket_count);
    add(row_data.factor_index);
    add(row_data.representative_time);
    add(row_data.points_in_bucket);
    add(row_data.source_point_count);
    add(row_data.target_candidate_count);
    add(row_data.valid_correspondence_count);
    add(row_data.effective_residual_count);
    add(row_data.factor_total_ms);
    add(row_data.correspondence_ms);
    add(row_data.covariance_lookup_ms);
    add(row_data.residual_eval_ms);
    add(row_data.jacobian_eval_ms);
    add(row_data.match_ratio);
    add(row_data.inlier_ratio);
    add(row_data.best_distance_mean);
    add(row_data.best_second_gap_mean);
    add(row_data.support_control_count);
    add(row_data.support_pose_key_count);
    add(row_data.active_control_point_count);
    row << '\n';
    std::fputs(row.str().c_str(), file);
  }
}

void write_frontend_lm_iteration_rows(
  std::FILE* file,
  const int frame_id,
  const double stamp,
  const std::vector<iap::FrontendLMIterationProfileRow>& iterations) {
  if (!file) {
    return;
  }

  for (const auto& iteration : iterations) {
    std::ostringstream row;
    row << std::fixed << std::setprecision(9);
    bool first = true;
    const auto add = [&](const auto& value) {
      if (!first) {
        row << ',';
      }
      first = false;
      row << value;
    };

    add(frame_id);
    add(stamp);
    add(iteration.iteration_index);
    add(iteration.cost_before);
    add(iteration.cost_after);
    add(iteration.accepted ? 1 : 0);
    add(iteration.lambda_before);
    add(iteration.lambda_after);
    add(iteration.linear_solve_ms);
    row << '\n';
    std::fputs(row.str().c_str(), file);
  }
}

void write_frame_warning_profile_row(
  std::FILE* file,
  const int frame_id,
  const double stamp,
  const std::size_t warning_count,
  const std::string& warning_categories,
  const std::string& top_warning_message) {
  if (!file) {
    return;
  }

  std::ostringstream row;
  row << std::fixed << std::setprecision(9);
  row << frame_id << ','
      << stamp << ','
      << warning_count << ','
      << csv_escape(warning_categories) << ','
      << csv_escape(top_warning_message) << '\n';
  std::fputs(row.str().c_str(), file);
}

double frontend_frame_wall_ms(const iap::FrontendFrameProfile& profile) {
  return
    profile.preprocess_ms +
    profile.target_map_prep_ms +
    profile.bucket_build_ms +
    profile.lidar_factor_build_ms +
    profile.imu_factor_build_ms +
    profile.lm_solve_ms +
    profile.marginalization_ms +
    profile.backend_update_ms +
    profile.backend_optimize_ms +
    profile.publish_ms +
    profile.local_mapping_update_ms +
    profile.global_mapping_update_ms +
    profile.submap_registration_ms;
}

const char* frontend_frame_top_stage(const iap::FrontendFrameProfile& profile) {
  const std::array<std::pair<const char*, double>, 13> stages = {{
    {"preprocess", profile.preprocess_ms},
    {"target_map_prep", profile.target_map_prep_ms},
    {"bucket_build", profile.bucket_build_ms},
    {"lidar_factor_build", profile.lidar_factor_build_ms},
    {"imu_factor_build", profile.imu_factor_build_ms},
    {"lm_solve", profile.lm_solve_ms},
    {"marginalization", profile.marginalization_ms},
    {"backend_update", profile.backend_update_ms},
    {"backend_optimize", profile.backend_optimize_ms},
    {"publish", profile.publish_ms},
    {"local_mapping_update", profile.local_mapping_update_ms},
    {"global_mapping_update", profile.global_mapping_update_ms},
    {"submap_registration", profile.submap_registration_ms},
  }};

  return std::max_element(
           stages.begin(),
           stages.end(),
           [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; })
    ->first;
}

iap::IntegratedBSplineGICPFactor::JacobianMode parse_lidar_jacobian_mode(const std::string& mode) {
  if (mode == "NUMERIC_FULL" || mode == "numeric_full" || mode == "numeric") {
    return iap::IntegratedBSplineGICPFactor::JacobianMode::NUMERIC_FULL;
  }
  return iap::IntegratedBSplineGICPFactor::JacobianMode::SEMI_ANALYTIC;
}

const char* to_string(iap::IntegratedBSplineGICPFactor::JacobianMode mode) {
  switch (mode) {
    case iap::IntegratedBSplineGICPFactor::JacobianMode::NUMERIC_FULL:
      return "numeric_full";
    case iap::IntegratedBSplineGICPFactor::JacobianMode::SEMI_ANALYTIC:
      return "semi_analytic";
  }
  return "unknown";
}

#ifdef GTSAM_POINTS_USE_CUDA
iap::IntegratedBSplineGICPFactorGPU::JacobianMode to_gpu_lidar_jacobian_mode(
  iap::IntegratedBSplineGICPFactor::JacobianMode mode) {
  switch (mode) {
    case iap::IntegratedBSplineGICPFactor::JacobianMode::NUMERIC_FULL:
      return iap::IntegratedBSplineGICPFactorGPU::JacobianMode::NUMERIC_FULL;
    case iap::IntegratedBSplineGICPFactor::JacobianMode::SEMI_ANALYTIC:
      return iap::IntegratedBSplineGICPFactorGPU::JacobianMode::SEMI_ANALYTIC;
  }
  return iap::IntegratedBSplineGICPFactorGPU::JacobianMode::SEMI_ANALYTIC;
}
#endif

iap::IntegratedBSplineGICPFactor::RobustKernel parse_lidar_robust_kernel(const std::string& mode) {
  if (mode == "HUBER" || mode == "huber") {
    return iap::IntegratedBSplineGICPFactor::RobustKernel::HUBER;
  }
  if (mode == "CAUCHY" || mode == "cauchy") {
    return iap::IntegratedBSplineGICPFactor::RobustKernel::CAUCHY;
  }
  return iap::IntegratedBSplineGICPFactor::RobustKernel::NONE;
}

const char* to_string(iap::IntegratedBSplineGICPFactor::RobustKernel mode) {
  switch (mode) {
    case iap::IntegratedBSplineGICPFactor::RobustKernel::NONE:
      return "none";
    case iap::IntegratedBSplineGICPFactor::RobustKernel::HUBER:
      return "huber";
    case iap::IntegratedBSplineGICPFactor::RobustKernel::CAUCHY:
      return "cauchy";
  }
  return "unknown";
}

iap::GnssHandler::Params make_gnss_handler_params(
  double min_elevation,
  double pr_noise_base,
  double dop_noise_base,
  double elev_noise_exp,
  double time_tolerance,
  const Eigen::Vector3d& lever_arm,
  const iap::CanopyNoiseParams& canopy_params) {
  iap::GnssHandler::Params params;
  params.pr_noise_base = pr_noise_base;
  params.dop_noise_base = dop_noise_base;
  params.elev_noise_exp = elev_noise_exp;
  params.time_tolerance = time_tolerance;
  params.min_elevation = min_elevation;
  params.lever_arm = lever_arm;
  params.canopy = canopy_params;
  return params;
}

iap::GnssEpochBuilder::Params make_gnss_epoch_builder_params(
  double min_elevation,
  double pr_noise_base,
  double dop_noise_base) {
  iap::GnssEpochBuilder::Params params;
  params.min_elevation = min_elevation;
  params.default_pr_sigma = pr_noise_base;
  params.default_dop_sigma = dop_noise_base;
  return params;
}

double sigma_from_covariance(const Eigen::Matrix3d& sigma_p) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(sigma_p, Eigen::EigenvaluesOnly);
  if (eig.info() != Eigen::Success) {
    return 0.0;
  }
  return std::sqrt(std::max(0.0, eig.eigenvalues().maxCoeff()));
}

gtsam::Key bspline_gyro_bias_key() {
  return gtsam::symbol('j', 0);
}

gtsam::Key bspline_accel_bias_key() {
  return gtsam::symbol('k', 0);
}

gtsam::Key bspline_gravity_key() {
  return gtsam::symbol('g', 0);
}

gtsam::KeyVector make_key_vector(const std::array<gtsam::Key, iap::kBSplineControlPointCount>& keys) {
  return gtsam::KeyVector(keys.begin(), keys.end());
}

gtsam::KeyVector make_key_vector(const iap::SplineLocalSupport& support) {
  return gtsam::KeyVector(support.pose_keys.begin(), support.pose_keys.end());
}

gtsam::KeyVector make_key_vector(
  const std::array<gtsam::Key, iap::kBSplineControlPointCount>& keys,
  std::initializer_list<gtsam::Key> extra_keys) {
  gtsam::KeyVector result(keys.begin(), keys.end());
  result.insert(result.end(), extra_keys.begin(), extra_keys.end());
  return result;
}

gtsam::KeyVector make_key_vector(
  const iap::SplineLocalSupport& support,
  std::initializer_list<gtsam::Key> extra_keys) {
  gtsam::KeyVector result(support.pose_keys.begin(), support.pose_keys.end());
  result.insert(result.end(), extra_keys.begin(), extra_keys.end());
  return result;
}

std::vector<std::size_t> support_control_indices(
  const iap::SplineStateLayout& layout,
  const std::vector<iap::SplineLocalSupport>& supports) {
  std::vector<std::size_t> indices;
  indices.reserve(supports.size() * iap::kBSplineControlPointCount);

  for (const auto& support : supports) {
    for (const auto ctrl_idx : support.ctrl_indices) {
      if (ctrl_idx >= layout.controls().size()) {
        continue;
      }

      const auto control_index = layout.controls()[ctrl_idx].index;
      if (std::find(indices.begin(), indices.end(), control_index) == indices.end()) {
        indices.push_back(control_index);
      }
    }
  }

  return indices;
}

void append_unique_key(gtsam::KeyVector* keys, gtsam::Key key) {
  if (!keys) {
    return;
  }
  if (std::find(keys->begin(), keys->end(), key) == keys->end()) {
    keys->push_back(key);
  }
}

gtsam::KeyVector sort_unique_keys(gtsam::KeyVector keys) {
  std::sort(keys.begin(), keys.end());
  keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
  return keys;
}

std::vector<std::size_t> sort_unique_control_indices(std::vector<std::size_t> indices) {
  std::sort(indices.begin(), indices.end());
  indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
  return indices;
}

std::vector<std::size_t> control_indices_from_activation(const iap::BSplineLayerActivation& activation) {
  std::vector<std::size_t> indices;
  indices.reserve(activation.active_control_indices.size());
  for (const auto control_index : activation.active_control_indices) {
    if (control_index >= 0) {
      indices.push_back(static_cast<std::size_t>(control_index));
    }
  }
  return sort_unique_control_indices(std::move(indices));
}

gtsam::KeyVector pose_keys_from_control_indices(const std::vector<std::size_t>& control_indices) {
  gtsam::KeyVector keys;
  keys.reserve(control_indices.size());
  for (const auto control_index : control_indices) {
    keys.push_back(iap::bspline_control_point_key(control_index));
  }
  return sort_unique_keys(std::move(keys));
}

}  // namespace

OdometryEstimationBSplineParams::OdometryEstimationBSplineParams() : OdometryEstimationCPUParams() {
  Config config(GlobalConfig::get_config_path("config_odometry"));
  spline_knot_mode = config.param<std::string>("odometry_estimation", "spline_knot_mode", "uniform");
  spline_nominal_dt = config.param<double>("odometry_estimation", "spline_nominal_dt", 0.0);
  spline_finite_difference_dt = config.param<double>("odometry_estimation", "spline_finite_difference_dt", 0.01);
  compatibility_sample_dt = config.param<double>("odometry_estimation", "compatibility_sample_dt", 0.01);
  publish_shared_trajectory = iap::get_log_config().shared_output.publish_shared_trajectory;
  attach_trajectory_to_frames = iap::get_log_config().shared_output.attach_trajectory_to_frames;
}

OdometryEstimationBSplineParams::~OdometryEstimationBSplineParams() {}

OdometryEstimationBSpline::OdometryEstimationBSpline(const OdometryEstimationBSplineParams& params)
: OdometryEstimationCPU(params),
  compatibility_sample_dt_(params.compatibility_sample_dt),
  publish_shared_trajectory_(params.publish_shared_trajectory),
  attach_trajectory_to_frames_(params.attach_trajectory_to_frames) {
  Config config(GlobalConfig::get_config_path("config_odometry"));
  const auto& log_config = iap::get_log_config();
  trajectory_params_.knot_mode = parse_knot_mode(params.spline_knot_mode);
  trajectory_params_.nominal_dt = params.spline_nominal_dt;
  trajectory_params_.finite_difference_dt = params.spline_finite_difference_dt;
  trajectory_params_.order = 3;
  frontend_mode_ = config.param<std::string>("odometry_estimation", "frontend_mode", "CT_LIDAR_CPU");
  frontend_only_mode_ = config.param<bool>("odometry_estimation", "frontend_only_mode", false);
  use_legacy_bspline_two_stage_path_ =
    config.param<bool>("odometry_estimation", "use_legacy_bspline_two_stage_path", false);
  unified_solver_mode_ = parse_bspline_unified_solver_mode(
    config.param<std::string>("odometry_estimation", "bspline_unified_solver_mode", "INCREMENTAL_SMOOTHER"));
  max_correspondence_distance_ = config.param<double>("odometry_estimation", "max_correspondence_distance", 1.5);
  lidar_bucket_config_.mode = parse_lidar_bucket_mode(
    config.param<std::string>("odometry_estimation", "ct_lidar_bucket_mode", "TIME_EPS"));
  lidar_bucket_config_.time_eps = config.param<double>("odometry_estimation", "ct_lidar_bucket_time_eps", 1e-3);
  lidar_bucket_config_.max_buckets_per_scan =
    config.param<int>("odometry_estimation", "ct_lidar_max_buckets_per_scan", 0);
  lidar_bucket_config_.fixed_buckets_per_scan =
    config.param<int>("odometry_estimation", "ct_lidar_fixed_buckets_per_scan", 8);
  lidar_target_mode_ = parse_lidar_target_mode(
    config.param<std::string>("odometry_estimation", "ct_lidar_target_mode", "ACTIVE_WINDOW_SNAPSHOT"));
  lidar_gpu_backend_ = parse_gpu_lidar_backend(
    config.param<std::string>("odometry_estimation", "ct_lidar_gpu_backend", "BUCKET"));
  lidar_jacobian_mode_ = parse_lidar_jacobian_mode(
    config.param<std::string>("odometry_estimation", "ct_lidar_jacobian_mode", "SEMI_ANALYTIC"));
  lidar_snapshot_frame_window_ = config.param<int>("odometry_estimation", "ct_lidar_snapshot_frame_window", 0);
  lidar_snapshot_min_frames_ = config.param<int>("odometry_estimation", "ct_lidar_snapshot_min_frames", 2);
  lidar_snapshot_min_points_ = config.param<int>("odometry_estimation", "ct_lidar_snapshot_min_points", 64);
  lidar_snapshot_max_age_ = config.param<double>("odometry_estimation", "ct_lidar_snapshot_max_age", 0.0);
  lidar_correspondence_candidate_count_ =
    config.param<int>("odometry_estimation", "ct_lidar_correspondence_candidates", 3);
  lidar_correspondence_accept_ratio_ =
    config.param<double>("odometry_estimation", "ct_lidar_correspondence_accept_ratio", 0.0);
  lidar_correspondence_min_score_gap_ =
    config.param<double>("odometry_estimation", "ct_lidar_correspondence_min_score_gap", 0.0);
  lidar_jacobian_numeric_eps_ = config.param<double>("odometry_estimation", "ct_lidar_jacobian_numeric_eps", 1e-4);
  lidar_outlier_mahalanobis_thresh_ =
    config.param<double>("odometry_estimation", "ct_lidar_outlier_mahalanobis_thresh", 0.0);
  lidar_robust_kernel_ = parse_lidar_robust_kernel(
    config.param<std::string>("odometry_estimation", "ct_lidar_robust_kernel", "NONE"));
  lidar_robust_kernel_width_ = config.param<double>("odometry_estimation", "ct_lidar_robust_kernel_width", 1.0);
  lidar_robust_weight_floor_ = config.param<double>("odometry_estimation", "ct_lidar_robust_weight_floor", 0.0);
  frontend_frame_profile_enabled_ = log_config.profiling.frontend_frame;
  lidar_factor_profile_ = log_config.profiling.lidar_factor;
  solver_update_profile_enabled_ = log_config.profiling.solver_update_profile;
  lidar_factor_internal_profile_enabled_ = log_config.profiling.lidar_factor_internal_profile;
  frontend_lm_iteration_profile_enabled_ = log_config.profiling.frontend_lm_iteration;
  frame_warning_profile_enabled_ = log_config.profiling.frame_warning_profile;
  target_map_prep_breakdown_enabled_ = log_config.profiling.target_map_prep_breakdown;
  graph_problem_size_enabled_ = log_config.profiling.graph_problem_size;
  lidar_validate_linearization_ = log_config.profiling.linearization_check;
  lidar_profile_numeric_reference_ = log_config.profiling.numeric_reference;
  pipeline_profile_ = iap::resolve_log_value<bool>(
    iap::log_bool({"log", "profiling"}, "pipeline"),
    config.param<bool>("odometry_estimation", "ct_profile_pipeline"),
    "odometry_estimation.ct_profile_pipeline",
    "log.profiling.pipeline",
    false);
  lidar_warn_degeneracy_ = log_config.warnings.lidar_degeneracy.enable;
  lidar_export_baseline_csv_ = log_config.export_outputs.baseline_csv;
  lidar_linearization_check_scale_ =
    config.param<double>("odometry_estimation", "ct_lidar_linearization_check_scale", 1e-4);
  lidar_linearization_warn_ratio_ =
    config.param<double>("odometry_estimation", "ct_lidar_linearization_warn_ratio", 0.25);
  lidar_numeric_reference_scale_ =
    config.param<double>("odometry_estimation", "ct_lidar_numeric_reference_scale", 1e-5);
  lidar_baseline_csv_path_ =
    iap::LogPaths::instance().export_path(log_config.export_outputs.baseline_csv_file).string();
  frontend_frame_profile_csv_path_ =
    iap::LogPaths::instance().profiling_path(log_config.profiling.frontend_frame_file).string();
  frontend_lidar_factor_profile_csv_path_ =
    iap::LogPaths::instance().profiling_path(log_config.profiling.lidar_factor_file).string();
  solver_update_profile_csv_path_ =
    iap::LogPaths::instance().profiling_path(log_config.profiling.solver_update_profile_file).string();
  lidar_factor_internal_profile_csv_path_ =
    iap::LogPaths::instance().profiling_path(log_config.profiling.lidar_factor_internal_profile_file).string();
  frontend_lm_iteration_csv_path_ =
    iap::LogPaths::instance().profiling_path(log_config.profiling.frontend_lm_iteration_file).string();
  frame_warning_profile_csv_path_ =
    iap::LogPaths::instance().profiling_path(log_config.profiling.frame_warning_profile_file).string();
  lidar_degeneracy_thresholds_.min_match_ratio =
    log_config.warnings.lidar_degeneracy.min_match_ratio;
  lidar_degeneracy_thresholds_.min_inlier_ratio =
    log_config.warnings.lidar_degeneracy.min_inlier_ratio;
  lidar_degeneracy_thresholds_.min_unique_target_ratio =
    log_config.warnings.lidar_degeneracy.min_unique_target_ratio;
  lidar_degeneracy_thresholds_.max_target_reuse_ratio =
    log_config.warnings.lidar_degeneracy.max_target_reuse_ratio;
  lidar_degeneracy_thresholds_.max_ambiguity_rejection_ratio =
    log_config.warnings.lidar_degeneracy.max_ambiguity_rejection_ratio;
  lidar_degeneracy_thresholds_.min_mean_score_gap =
    log_config.warnings.lidar_degeneracy.min_mean_score_gap;
  ctrl_point_anchor_inf_scale_ = config.param<double>("odometry_estimation", "ctrl_point_anchor_inf_scale", 1e6);
  ctrl_point_prediction_inf_scale_ = config.param<double>("odometry_estimation", "ctrl_point_prediction_inf_scale", 1e3);
  ctrl_point_smoothness_inf_scale_ = config.param<double>("odometry_estimation", "ctrl_point_smoothness_inf_scale", 1e2);
  ctrl_point_marginal_inf_scale_ = config.param<double>("odometry_estimation", "ctrl_point_marginal_inf_scale", 1e4);
  imu_ct_trans_inf_scale_ = config.param<double>("odometry_estimation", "imu_ct_trans_inf_scale", 10.0);
  imu_ct_rot_inf_scale_ = config.param<double>("odometry_estimation", "imu_ct_rot_inf_scale", 100.0);
  imu_ct_bias_inf_scale_ = config.param<double>("odometry_estimation", "imu_ct_bias_inf_scale", 1e3);
  imu_ct_gravity_inf_scale_ = config.param<double>("odometry_estimation", "imu_ct_gravity_inf_scale", 1e3);
  velocity_ct_inf_scale_ = config.param<double>("odometry_estimation", "velocity_ct_inf_scale", 1e3);
  imu_ct_sample_stride_ = config.param<int>("odometry_estimation", "imu_ct_sample_stride", 4);
  lm_max_iterations_ = config.param<int>("odometry_estimation", "lm_max_iterations", 8);

  Config gnss_config(GlobalConfig::get_config_path("config_gnss"));
  gnss_time_tolerance_ = gnss_config.param<double>("gnss", "time_tolerance", 0.1);
  gnss_min_elevation_ = gnss_config.param<double>("gnss", "min_elevation_deg", 10.0) * M_PI / 180.0;
  gnss_pr_noise_base_ = gnss_config.param<double>("gnss", "pr_noise_base", 5.0);
  gnss_dop_noise_base_ = gnss_config.param<double>("gnss", "dop_noise_base", 0.5);
  gnss_elev_noise_exp_ = gnss_config.param<double>("gnss", "elev_noise_exp", 2.0);
  gnss_sigma_ecef_origin_ = gnss_config.param<double>("gnss", "sigma_ecef_origin", 5.0);
  gnss_sigma_ecef_rot_ = gnss_config.param<double>("gnss", "sigma_ecef_rot", 0.087);
  gnss_lever_arm_ = gnss_config.param<Eigen::Vector3d>("gnss", "lever_arm", Eigen::Vector3d::Zero());
  gnss_canopy_params_.sigma_0 = gnss_config.param<double>("gnss", "canopy_sigma_0", 1.0);
  gnss_canopy_params_.sigma_mp = gnss_config.param<double>("gnss", "canopy_sigma_mp", 0.5);
  gnss_canopy_params_.sigma_c = gnss_config.param<double>("gnss", "canopy_sigma_c", 5.0);
  gnss_canopy_params_.alpha = gnss_config.param<double>("gnss", "canopy_alpha", 2.0);
  gnss_clock_between_params_.q_bias = gnss_config.param<double>("gnss", "clock_q_bias", 1.0);
  gnss_clock_between_params_.q_drift = gnss_config.param<double>("gnss", "clock_q_drift", 0.1);

  T_lidar_imu = params.T_lidar_imu;
  T_imu_lidar = T_lidar_imu.inverse();
  control_window_ = std::make_unique<iap::BSplineControlWindow>();
  fixed_lag_registry_.set_shared_imu_state(
    params.imu_bias.tail<3>(),
    params.imu_bias.head<3>(),
    Eigen::Vector3d::UnitZ() * 9.80665);
  gnss_epoch_builder_ = std::make_unique<iap::GnssEpochBuilder>(make_gnss_epoch_builder_params(
    gnss_min_elevation_,
    gnss_pr_noise_base_,
    gnss_dop_noise_base_));
  gnss_handler_ = std::make_unique<iap::GnssHandler>(make_gnss_handler_params(
    gnss_min_elevation_,
    gnss_pr_noise_base_,
    gnss_dop_noise_base_,
    gnss_elev_noise_exp_,
    gnss_time_tolerance_,
    gnss_lever_arm_,
    gnss_canopy_params_));
  ct_target_ivox_ = std::make_shared<gtsam_points::iVox>(params.ivox_resolution);
  ct_target_ivox_->voxel_insertion_setting().set_min_dist_in_cell(params.ivox_min_dist);
  ct_target_ivox_->set_lru_horizon(params.lru_thresh);
  ct_target_ivox_->set_neighbor_voxel_mode(1);
#ifdef GTSAM_POINTS_USE_CUDA
  if (frontend_mode_ == "CT_LIDAR_GPU" && lidar_gpu_backend_ == BSplineGpuLidarBackend::BUCKET) {
    ct_lidar_gpu_stream_buffers_ = std::make_unique<gtsam_points::StreamTempBufferRoundRobin>();
  }
#endif
  reset_unified_graph_solver();
  if (frontend_lm_iteration_profile_enabled_ && !frontend_frame_profile_enabled_) {
    logger->warn(
      "frontend LM iteration profiling is enabled while frontend frame profiling is disabled; only '{}' will be emitted",
      frontend_lm_iteration_csv_path_);
  }
  if (target_map_prep_breakdown_enabled_ && !frontend_frame_profile_enabled_) {
    logger->warn(
      "target_map_prep_breakdown requires log.profiling.frontend_frame=true; substage telemetry will remain disabled");
  }
  if (graph_problem_size_enabled_ && !frontend_frame_profile_enabled_) {
    logger->warn(
      "graph_problem_size requires log.profiling.frontend_frame=true; graph telemetry columns will remain disabled");
  }
  logger->info("odometry_bspline initialized frontend_mode={} frontend_only_mode={} use_legacy_bspline_two_stage_path={} bspline_unified_solver_mode={} lidar_gpu_backend={} lidar_bucket_mode={} lidar_bucket_time_eps={:.6f} lidar_max_buckets={} lidar_fixed_buckets={} knot_mode={} nominal_dt={:.4f} compatibility_sample_dt={:.4f} lidar_target_mode={} lidar_jacobian_mode={} lidar_k_candidates={} lidar_accept_ratio={:.3f} lidar_score_gap={:.3f} lidar_snapshot_window={} lidar_snapshot_min_frames={} lidar_snapshot_min_points={} lidar_snapshot_max_age={:.3f} lidar_outlier_thresh={:.3f} lidar_robust_kernel={} lidar_robust_width={:.3f} lidar_robust_w_floor={:.3f} lidar_profile={} lidar_validate={} ct_pipeline_profile={} lidar_baseline_csv={} lidar_baseline_path={}",
    frontend_mode_,
    frontend_only_mode_,
    use_legacy_bspline_two_stage_path_,
    iap::to_string(unified_solver_mode_),
    ::glim::to_string(lidar_gpu_backend_),
    ::glim::to_string(lidar_bucket_config_.mode),
    lidar_bucket_config_.time_eps,
    lidar_bucket_config_.max_buckets_per_scan,
    lidar_bucket_config_.fixed_buckets_per_scan,
    iap::to_string(trajectory_params_.knot_mode),
    trajectory_params_.nominal_dt,
    compatibility_sample_dt_,
    ::glim::to_string(lidar_target_mode_),
    ::glim::to_string(lidar_jacobian_mode_),
    lidar_correspondence_candidate_count_,
    lidar_correspondence_accept_ratio_,
    lidar_correspondence_min_score_gap_,
    lidar_snapshot_frame_window_,
    lidar_snapshot_min_frames_,
    lidar_snapshot_min_points_,
    lidar_snapshot_max_age_,
    lidar_outlier_mahalanobis_thresh_,
    ::glim::to_string(lidar_robust_kernel_),
    lidar_robust_kernel_width_,
    lidar_robust_weight_floor_,
    lidar_factor_profile_,
    lidar_validate_linearization_,
    pipeline_profile_,
    lidar_export_baseline_csv_,
    lidar_baseline_csv_path_);
}

OdometryEstimationBSpline::~OdometryEstimationBSpline() {
  if (publish_shared_trajectory_) {
    iap::IapSharedState::instance().set_continuous_trajectory_view(nullptr);
    iap::IapSharedState::instance().set_spline_control_access(nullptr);
    iap::IapSharedState::instance().clear_bspline_fixed_lag_telemetry();
  }
}

void OdometryEstimationBSpline::update_frames(int current, const gtsam::NonlinearFactorGraph& new_factors) {
  OdometryEstimationIMU::update_frames(current, new_factors);
  publish_continuous_trajectory(current);
}

EstimationFrame::ConstPtr OdometryEstimationBSpline::insert_frame(
  const PreprocessedFrame::Ptr& frame,
  std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  if (frontend_mode_ == "CT_LIDAR_CPU" || frontend_mode_ == "CT_LIDAR_GPU") {
    return insert_frame_ct_lidar(frame, marginalized_frames);
  }
  return insert_frame_reconstruct(frame, marginalized_frames);
}

EstimationFrame::ConstPtr OdometryEstimationBSpline::insert_frame_reconstruct(
  const PreprocessedFrame::Ptr& frame,
  std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  return OdometryEstimationCPU::insert_frame(frame, marginalized_frames);
}

gtsam_points::PointCloud::ConstPtr OdometryEstimationBSpline::create_lidar_source_cloud(
  const PreprocessedFrame::Ptr& raw_frame) const {
  auto frame_cpu = std::make_shared<gtsam_points::PointCloudCPU>(raw_frame->points);
  frame_cpu->add_times(raw_frame->times);
  covariance_estimation->estimate(raw_frame->points, raw_frame->neighbors, frame_cpu->normals_storage, frame_cpu->covs_storage);
  frame_cpu->normals = frame_cpu->normals_storage.data();
  frame_cpu->covs = frame_cpu->covs_storage.data();
  return frame_cpu;
}

void OdometryEstimationBSpline::initialize_control_window(
  const PreprocessedFrame::Ptr& raw_frame,
  const gtsam::Pose3& initial_pose) {
  control_window_->initialize(raw_frame->stamp, raw_frame->scan_end_time, initial_pose);
  fixed_lag_registry_.reset_from_window(*control_window_);
  reset_unified_graph_solver();
  refresh_active_window_layout();
  marginal_prior_ = ActiveSplineMarginalPrior();
  fixed_lag_registry_.clear_auxiliary_values();
  ct_target_revision_ = 0;
}

gtsam::Pose3 OdometryEstimationBSpline::predict_scan_end_pose(double scan_duration) const {
  if (!control_window_ || !control_window_->initialized()) {
    return gtsam::Pose3();
  }

  const gtsam::Pose3 last_start = control_window_->evaluate(0.0);
  const gtsam::Pose3 last_end = control_window_->evaluate(1.0);
  const double last_duration = std::max(1e-3, control_window_->segment_duration());
  const double scale = scan_duration / last_duration;
  const gtsam::Vector6 delta = gtsam::Pose3::Logmap(last_start.between(last_end));
  return last_end.compose(gtsam::Pose3::Expmap(scale * delta));
}

OdometryEstimationBSpline::ActiveSplineTargetReference OdometryEstimationBSpline::create_active_target_reference() const {
  using Clock = std::chrono::steady_clock;
  const auto t_start = Clock::now();
  const auto* cpu_params = static_cast<const OdometryEstimationCPUParams*>(params.get());
  ActiveSplineTargetReference target_ref;

  if (lidar_target_mode_ == BSplineLidarTargetMode::GLOBAL_IVOX_REFERENCE && ct_target_ivox_ &&
      !ct_target_ivox_->voxel_points().empty()) {
    target_ref.target_snapshot = ct_target_ivox_;
    target_ref.target_tree = target_ref.target_snapshot;
    target_ref.mode = BSplineLidarTargetMode::GLOBAL_IVOX_REFERENCE;
    target_ref.point_count = target_ref.target_snapshot->voxel_points().size();
    target_ref.build_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_start).count();
    target_ref.target_voxel_lookup_prep_ms = target_ref.build_ms;
    return target_ref;
  }

  const auto t_snapshot_setup_start = Clock::now();
  auto snapshot = std::make_shared<gtsam_points::iVox>(cpu_params->ivox_resolution);
  snapshot->voxel_insertion_setting().set_min_dist_in_cell(cpu_params->ivox_min_dist);
  snapshot->set_lru_horizon(cpu_params->lru_thresh);
  snapshot->set_neighbor_voxel_mode(1);
  target_ref.target_voxel_lookup_prep_ms +=
    std::chrono::duration<double, std::milli>(Clock::now() - t_snapshot_setup_start).count();

  std::vector<EstimationFrame::ConstPtr> active_frames(frames.inner_begin(), frames.inner_end());
  std::size_t first_frame_index = 0;
  if (lidar_snapshot_frame_window_ > 0 && active_frames.size() > static_cast<std::size_t>(lidar_snapshot_frame_window_)) {
    first_frame_index = active_frames.size() - static_cast<std::size_t>(lidar_snapshot_frame_window_);
  }

  bool inserted = false;
  double snapshot_start_stamp = 0.0;
  double snapshot_end_stamp = 0.0;
  for (std::size_t i = first_frame_index; i < active_frames.size(); ++i) {
    if (!active_frames[i] || !active_frames[i]->frame) {
      continue;
    }
    if (lidar_snapshot_max_age_ > 0.0 && !active_frames.empty()) {
      const double latest_stamp = active_frames.back()->stamp;
      if (latest_stamp - active_frames[i]->stamp > lidar_snapshot_max_age_) {
        continue;
      }
    }

    const auto t_clone_start = Clock::now();
    auto transformed = gtsam_points::PointCloudCPU::clone(*active_frames[i]->frame);
    target_ref.target_snapshot_clone_ms +=
      std::chrono::duration<double, std::milli>(Clock::now() - t_clone_start).count();
    const auto t_transform_start = Clock::now();
    for (int j = 0; j < transformed->size(); ++j) {
      transformed->points[j] = active_frames[i]->T_world_lidar * active_frames[i]->frame->points[j];
      transformed->covs[j] =
        active_frames[i]->T_world_lidar.matrix() * active_frames[i]->frame->covs[j] * active_frames[i]->T_world_lidar.matrix().transpose();
    }
    target_ref.source_to_target_transform_ms +=
      std::chrono::duration<double, std::milli>(Clock::now() - t_transform_start).count();
    const auto t_insert_start = Clock::now();
    snapshot->insert(*transformed);
    target_ref.target_voxel_lookup_prep_ms +=
      std::chrono::duration<double, std::milli>(Clock::now() - t_insert_start).count();
    inserted = true;
    if (target_ref.snapshot_frame_count == 0) {
      snapshot_start_stamp = active_frames[i]->stamp;
    }
    snapshot_end_stamp = active_frames[i]->stamp;
    target_ref.snapshot_frame_count++;
  }

  target_ref.snapshot_point_count = inserted ? snapshot->voxel_points().size() : 0;
  target_ref.snapshot_span_sec =
    target_ref.snapshot_frame_count == 0 ? 0.0 : std::max(0.0, snapshot_end_stamp - snapshot_start_stamp);

  bool snapshot_policy_accepted = inserted;
  if (lidar_snapshot_min_frames_ > 0 &&
      target_ref.snapshot_frame_count < static_cast<std::size_t>(lidar_snapshot_min_frames_)) {
    snapshot_policy_accepted = false;
  }
  if (lidar_snapshot_min_points_ > 0 &&
      target_ref.snapshot_point_count < static_cast<std::size_t>(lidar_snapshot_min_points_)) {
    snapshot_policy_accepted = false;
  }
  if (lidar_snapshot_max_age_ > 0.0 && target_ref.snapshot_span_sec > lidar_snapshot_max_age_) {
    snapshot_policy_accepted = false;
  }
  target_ref.snapshot_policy_accepted = snapshot_policy_accepted;

  const bool global_reference_available = ct_target_ivox_ && !ct_target_ivox_->voxel_points().empty();
  if (snapshot_policy_accepted || (!global_reference_available && inserted)) {
    target_ref.target_snapshot = snapshot;
    target_ref.target_tree = target_ref.target_snapshot;
    target_ref.mode = BSplineLidarTargetMode::ACTIVE_WINDOW_SNAPSHOT;
    target_ref.contributing_frames = target_ref.snapshot_frame_count;
    target_ref.point_count = target_ref.snapshot_point_count;
    target_ref.build_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_start).count();
    return target_ref;
  }

  target_ref.target_snapshot = global_reference_available ? ct_target_ivox_ : snapshot;
  target_ref.contributing_frames = inserted ? target_ref.snapshot_frame_count : 0;
  target_ref.target_tree = target_ref.target_snapshot;
  target_ref.mode = BSplineLidarTargetMode::GLOBAL_IVOX_REFERENCE;
  target_ref.point_count = target_ref.target_snapshot ? target_ref.target_snapshot->voxel_points().size() : 0;
  target_ref.build_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_start).count();
  return target_ref;
}

std::vector<OdometryEstimationBSpline::ActiveSplineIMUSample> OdometryEstimationBSpline::create_segment_imu_samples(
  const PreprocessedFrame::Ptr& raw_frame) const {
  std::vector<ActiveSplineIMUSample> samples;
  if (!imu_integration) {
    return samples;
  }

  const double scan_duration = std::max(1e-3, raw_frame->scan_end_time - raw_frame->stamp);
  const double finite_difference_dt = std::min(trajectory_params_.finite_difference_dt, 0.25 * scan_duration);
  const double sample_start = raw_frame->stamp + finite_difference_dt;
  const double sample_end = raw_frame->scan_end_time - finite_difference_dt;
  if (sample_end <= sample_start) {
    return samples;
  }

  std::vector<double> delta_times;
  std::vector<Eigen::Matrix<double, 7, 1>> imu_data;
  imu_integration->find_imu_data(sample_start, sample_end, delta_times, imu_data);
  if (imu_data.empty()) {
    return samples;
  }

  const std::size_t stride = static_cast<std::size_t>(std::max(1, imu_ct_sample_stride_));
  auto append_sample = [&](std::size_t idx) {
    const auto& imu = imu_data[idx];
    ActiveSplineIMUSample sample;
    sample.stamp = imu[0];
    sample.u = std::clamp((sample.stamp - raw_frame->stamp) / scan_duration, 0.0, 1.0);
    sample.linear_acc = imu.block<3, 1>(1, 0);
    sample.angular_vel = imu.block<3, 1>(4, 0);
    samples.push_back(sample);
  };

  for (std::size_t i = 0; i < imu_data.size(); i += stride) {
    append_sample(i);
  }

  if ((imu_data.size() - 1) % stride != 0) {
    append_sample(imu_data.size() - 1);
  }

  return samples;
}

std::shared_ptr<const iap::SplineStateLayout> OdometryEstimationBSpline::build_active_window_layout() const {
  const auto& states = fixed_lag_registry_.control_buffer().states();
  if (states.size() < iap::kBSplineControlPointCount) {
    return nullptr;
  }

  std::vector<double> knots;
  const auto& segments = fixed_lag_registry_.segments();
  const std::size_t expected_knot_count = states.size() + iap::kBSplineControlPointCount;
  if (!segments.empty() && states.size() == segments.size() + iap::kBSplineControlPointCount - 1) {
    const double start_stamp = segments.front().stamp;
    double last_knot = start_stamp;
    knots.assign(iap::kBSplineControlPointCount, start_stamp);
    for (const auto& segment : segments) {
      last_knot = std::max(last_knot, segment.scan_end);
      knots.push_back(last_knot);
    }
    knots.resize(expected_knot_count, last_knot);
  } else if (control_window_ && control_window_->initialized() && control_window_->knots().size() == expected_knot_count) {
    knots = control_window_->knots();
  } else {
    const double start_stamp = states[1].stamp;
    double last_knot = std::max(start_stamp + 1e-3, states[states.size() - 2].stamp);
    knots.assign(iap::kBSplineControlPointCount, start_stamp);
    knots.push_back(last_knot);
    knots.resize(expected_knot_count, last_knot);
  }

  auto layout = std::make_shared<iap::SplineStateLayout>();
  layout->set_controls(states);
  layout->set_knots(std::move(knots));

  iap::SplineSensorModel imu_model;
  imu_model.id = iap::SplineSensorId::Imu;
  imu_model.T_sensor_imu = Eigen::Isometry3d(T_imu_lidar.matrix());
  layout->set_sensor_model(iap::SplineSensorId::Imu, imu_model);

  iap::SplineSensorModel lidar_model;
  lidar_model.id = iap::SplineSensorId::Lidar;
  layout->set_sensor_model(iap::SplineSensorId::Lidar, lidar_model);

  iap::SplineSensorModel gnss_model;
  gnss_model.id = iap::SplineSensorId::Gnss;
  gnss_model.T_sensor_imu = Eigen::Translation3d(gnss_lever_arm_) * T_imu_lidar;
  layout->set_sensor_model(iap::SplineSensorId::Gnss, gnss_model);

  return layout;
}

void OdometryEstimationBSpline::refresh_active_window_layout() {
  active_window_layout_ = build_active_window_layout();
  active_window_evaluator_ = active_window_layout_
    ? std::make_shared<iap::SplineEvaluator>(active_window_layout_)
    : nullptr;
}

std::shared_ptr<const iap::SplineStateLayout> OdometryEstimationBSpline::create_segment_imu_layout(
  const ActiveSplineSegmentConstraint& segment) const {
  (void)segment;
  // Commit 7 legacy note:
  // Segment-local layout builders remain as fallback surfaces, but the main
  // scheduler path now binds factors against the unified active-window layout.
  return active_window_layout_ ? active_window_layout_ : build_active_window_layout();
}

std::shared_ptr<const iap::SplineStateLayout> OdometryEstimationBSpline::create_segment_lidar_layout(
  const ActiveSplineSegmentConstraint& segment) const {
  (void)segment;
  return active_window_layout_ ? active_window_layout_ : build_active_window_layout();
}

std::vector<iap::SplineBucketContext> OdometryEstimationBSpline::create_segment_lidar_buckets(
  const ActiveSplineSegmentConstraint& segment) const {
  const auto layout = active_window_layout_ ? active_window_layout_ : create_segment_lidar_layout(segment);
  if (!layout) {
    return {};
  }

  iap::CTLocalFrontend::SourceFrameInput source_frame;
  source_frame.source_cloud = segment.source;
  source_frame.scan_start = segment.stamp;
  source_frame.scan_end = segment.scan_end;
  return iap::CTLocalFrontend::create_lidar_buckets(*layout, source_frame, lidar_bucket_config_);
}

std::vector<iap::GnssEpoch> OdometryEstimationBSpline::consume_segment_gnss_epochs(
  double segment_start,
  double segment_end) {
  sync_gnss_epochs_from_shared_state();
  if (!gnss_handler_) {
    return {};
  }
  return gnss_handler_->consume_epochs_in_range(segment_start, segment_end, gnss_time_tolerance_);
}

void OdometryEstimationBSpline::sync_gnss_epochs_from_shared_state() {
  if (!gnss_handler_ || !gnss_epoch_builder_) {
    return;
  }

  auto& shared = iap::IapSharedState::instance();
  if (const auto anchor = shared.get_gnss_anchor()) {
    gnss_epoch_builder_->set_anchor(*anchor);
    fixed_lag_registry_.set_shared_gnss_anchor(anchor->origin_ecef, gtsam::Rot3(anchor->R_ecef_world));
  }

  const auto iono_params = shared.get_gnss_iono_params();
  if (!iono_params.empty()) {
    gnss_epoch_builder_->set_iono_params(iono_params);
  }

  const auto ephemeris_updates = shared.consume_pending_gnss_ephemeris_updates();
  for (const auto& update : ephemeris_updates) {
    gnss_epoch_builder_->update_ephemeris(update);
  }

  const auto raw_batches = shared.consume_pending_gnss_raw_batches();
  for (auto& batch : raw_batches) {
    pending_raw_gnss_batches_.push_back(std::move(batch));
  }

  while (!pending_raw_gnss_batches_.empty()) {
    const auto build_result = gnss_epoch_builder_->build_epoch(pending_raw_gnss_batches_.front());

    if (build_result.status == iap::GnssEpochBuilder::BuildStatus::MissingAnchor ||
        build_result.status == iap::GnssEpochBuilder::BuildStatus::MissingEphemeris) {
      break;
    }

    if (build_result.status == iap::GnssEpochBuilder::BuildStatus::Success && build_result.epoch.has_value()) {
      gnss_handler_->insert_epoch(*build_result.epoch);
    }

    pending_raw_gnss_batches_.pop_front();
  }
}

void OdometryEstimationBSpline::prune_active_ct_state(
  double min_active_stamp,
  const iap::SplineActiveStateSet& active_state_set) {
  fixed_lag_registry_.prune_to_active_state_set(min_active_stamp, active_state_set, true);
}

void OdometryEstimationBSpline::update_marginal_prior_from_active_window() {
  marginal_prior_ = ActiveSplineMarginalPrior();

  const auto& control_buffer = fixed_lag_registry_.control_buffer();
  if (control_buffer.size() < 2) {
    return;
  }

  const auto& states = control_buffer.states();
  marginal_prior_.valid = true;
  marginal_prior_.control_indices = {states[0].index, states[1].index};
  marginal_prior_.first_pose = states[0].pose;
  marginal_prior_.relative_delta = states[0].pose.between(states[1].pose);
  marginal_prior_.auxiliary_index = states[1].index;

  const gtsam::Key velocity_key = iap::bspline_velocity_key(marginal_prior_.auxiliary_index);
  if (fixed_lag_registry_.auxiliary_values().exists(velocity_key)) {
    marginal_prior_.has_velocity = true;
    marginal_prior_.velocity = fixed_lag_registry_.auxiliary_values().at<gtsam::Vector3>(velocity_key);
  }

  const gtsam::Key clock_key = iap::bspline_clock_key(marginal_prior_.auxiliary_index);
  if (fixed_lag_registry_.auxiliary_values().exists(clock_key)) {
    marginal_prior_.has_clock = true;
    marginal_prior_.clock = fixed_lag_registry_.auxiliary_values().at<gtsam::Vector2>(clock_key);
  }
}

void OdometryEstimationBSpline::update_marginal_prior_information(
  const gtsam::NonlinearFactorGraph& graph,
  const gtsam::Values& values,
  const std::vector<gtsam::Key>& survivor_keys,
  const iap::BSplineCarriedPrior* previous_prior) {
  try {
    marginal_prior_.carried_prior =
      iap::build_bspline_carried_prior(graph, values, survivor_keys, previous_prior);
  } catch (const std::exception& e) {
    marginal_prior_.carried_prior = iap::BSplineCarriedPrior();
    logger->warn("failed to build bspline marginal survivor prior: {}", e.what());
  }
}

void OdometryEstimationBSpline::append_active_segment_constraint(
  const PreprocessedFrame::Ptr& raw_frame,
  const gtsam_points::PointCloud::ConstPtr& source) {
  ActiveSplineSegmentConstraint segment;
  segment.stamp = raw_frame->stamp;
  segment.scan_end = raw_frame->scan_end_time;
  segment.source = source;
  const auto target_ref = create_active_target_reference();
  segment.target_snapshot = target_ref.target_snapshot;
  segment.target_tree = target_ref.target_tree;
  segment.target_mode = target_ref.mode;
  segment.target_frame_count = target_ref.contributing_frames;
  segment.target_point_count = target_ref.point_count;
  segment.snapshot_frame_count = target_ref.snapshot_frame_count;
  segment.snapshot_point_count = target_ref.snapshot_point_count;
  segment.snapshot_span_sec = target_ref.snapshot_span_sec;
  segment.snapshot_policy_accepted = target_ref.snapshot_policy_accepted;
  segment.target_build_ms = target_ref.build_ms;
  segment.imu_samples = create_segment_imu_samples(raw_frame);

  const auto states = control_window_->states();
  for (std::size_t i = 0; i < iap::kBSplineControlPointCount; ++i) {
    segment.control_indices[i] = states[i].index;
  }
  segment.auxiliary_index = states[1].index;

  auto& appended = fixed_lag_registry_.append_segment(std::move(segment));
  appended.active_control_indices.assign(appended.control_indices.begin(), appended.control_indices.end());
  refresh_active_window_layout();
  if (active_window_layout_) {
    const auto supports = active_window_layout_->supports_in_range(
      raw_frame->stamp,
      std::max(raw_frame->scan_end_time, raw_frame->stamp + 1e-3),
      iap::SplineSensorId::Lidar);
    appended.active_control_indices = support_control_indices(*active_window_layout_, supports);
    if (!supports.empty()) {
      appended.span_begin_idx = supports.front().span_idx;
      appended.span_end_idx = supports.back().span_idx;
    }
  }
}

void OdometryEstimationBSpline::insert_target_cloud(const EstimationFrame::Ptr& frame) {
  auto transformed = gtsam_points::PointCloudCPU::clone(*frame->frame);
  for (int i = 0; i < transformed->size(); ++i) {
    transformed->points[i] = frame->T_world_lidar * frame->frame->points[i];
    transformed->covs[i] = frame->T_world_lidar.matrix() * frame->frame->covs[i] * frame->T_world_lidar.matrix().transpose();
  }
  ct_target_ivox_->insert(*transformed);
  ++ct_target_revision_;
}

void OdometryEstimationBSpline::update_frame_history(
  const EstimationFrame::Ptr& frame,
  std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  frames.push_back(frame);

  while (marginalized_cursor < frames.size() - 1) {
    const double span = frame->stamp - frames[marginalized_cursor]->stamp;
    if (span < params->smoother_lag - 0.1) {
      break;
    }

    marginalized_frames.push_back(frames[marginalized_cursor]);
    frames[marginalized_cursor].reset();
    marginalized_cursor++;
  }

  Callbacks::on_marginalized_frames(marginalized_frames);
}

bool OdometryEstimationBSpline::lidar_collect_window_results() const {
  return lidar_factor_profile_ || lidar_profile_numeric_reference_ || lidar_export_baseline_csv_ || lidar_warn_degeneracy_;
}

std::size_t OdometryEstimationBSpline::lidar_factor_config_signature(bool use_gpu_lidar) const {
  std::size_t seed = 0;
  hash_combine(&seed, use_gpu_lidar);
  hash_combine(&seed, static_cast<int>(lidar_gpu_backend_));
  hash_combine(&seed, static_cast<int>(lidar_target_mode_));
  hash_combine(&seed, static_cast<int>(lidar_jacobian_mode_));
  hash_combine(&seed, static_cast<int>(lidar_robust_kernel_));
  hash_combine(&seed, max_correspondence_distance_);
  hash_combine(&seed, lidar_jacobian_numeric_eps_);
  hash_combine(&seed, lidar_outlier_mahalanobis_thresh_);
  hash_combine(&seed, lidar_robust_kernel_width_);
  hash_combine(&seed, lidar_robust_weight_floor_);
  hash_combine(&seed, lidar_correspondence_candidate_count_);
  hash_combine(&seed, lidar_correspondence_accept_ratio_);
  hash_combine(&seed, lidar_correspondence_min_score_gap_);
  hash_combine(&seed, lidar_factor_profile_);
  hash_combine(&seed, lidar_warn_degeneracy_);
  hash_combine(&seed, lidar_export_baseline_csv_);
  return seed;
}

std::size_t OdometryEstimationBSpline::lidar_target_revision(const ActiveSplineSegmentConstraint& segment) const {
  return segment.target_mode == BSplineLidarTargetMode::GLOBAL_IVOX_REFERENCE ? ct_target_revision_ : 0U;
}

OdometryEstimationBSpline::ActiveSplineLidarFactorCacheKey
OdometryEstimationBSpline::make_lidar_factor_cache_key(
  const ActiveSplineSegmentConstraint& segment,
  bool use_gpu_lidar) const {
  ActiveSplineLidarFactorCacheKey key;
  key.valid = true;
  key.gpu = use_gpu_lidar;
  key.gpu_backend = lidar_gpu_backend_;
  key.target_mode = segment.target_mode;
  key.control_indices = segment.control_indices;
  key.source_identity = segment.source.get();
  key.target_identity = segment.target_snapshot.get();
  key.target_revision = lidar_target_revision(segment);
  key.config_signature = lidar_factor_config_signature(use_gpu_lidar);
  return key;
}

bool OdometryEstimationBSpline::same_lidar_factor_cache_base(
  const ActiveSplineLidarFactorCacheKey& lhs,
  const ActiveSplineLidarFactorCacheKey& rhs) const {
  return lhs.valid && rhs.valid &&
         lhs.gpu == rhs.gpu &&
         lhs.gpu_backend == rhs.gpu_backend &&
         lhs.target_mode == rhs.target_mode &&
         lhs.control_indices == rhs.control_indices &&
         lhs.bucket_u_signature == rhs.bucket_u_signature &&
         lhs.source_identity == rhs.source_identity &&
         lhs.config_signature == rhs.config_signature;
}

std::shared_ptr<iap::IntegratedBSplineGICPFactor> OdometryEstimationBSpline::get_or_create_cpu_lidar_factor(
  ActiveSplineSegmentConstraint& segment,
  bool* cache_hit) {
  const auto desired_key = make_lidar_factor_cache_key(segment, false);
  const bool key_match =
    segment.cached_cpu_factor &&
    same_lidar_factor_cache_base(segment.lidar_factor_cache, desired_key) &&
    segment.lidar_factor_cache.target_identity == desired_key.target_identity;

  if (!key_match) {
    auto factor = std::make_shared<iap::IntegratedBSplineGICPFactor>(
      std::array<gtsam::Key, iap::kBSplineControlPointCount>{
        iap::bspline_control_point_key(segment.control_indices[0]),
        iap::bspline_control_point_key(segment.control_indices[1]),
        iap::bspline_control_point_key(segment.control_indices[2]),
        iap::bspline_control_point_key(segment.control_indices[3])},
      segment.target_snapshot,
      segment.source,
      segment.target_tree);
    factor->set_num_threads(params->num_threads);
    factor->set_max_correspondence_distance(max_correspondence_distance_);
    factor->set_jacobian_mode(lidar_jacobian_mode_);
    factor->set_numeric_eps(lidar_jacobian_numeric_eps_);
    factor->set_correspondence_candidate_count(lidar_correspondence_candidate_count_);
    factor->set_correspondence_accept_ratio(lidar_correspondence_accept_ratio_);
    factor->set_correspondence_min_score_gap(lidar_correspondence_min_score_gap_);
    factor->set_outlier_mahalanobis_threshold(lidar_outlier_mahalanobis_thresh_);
    factor->set_robust_kernel(lidar_robust_kernel_, lidar_robust_kernel_width_);
    factor->set_robust_weight_floor(lidar_robust_weight_floor_);
    factor->set_enable_profiling(lidar_factor_profile_ || lidar_warn_degeneracy_ || lidar_export_baseline_csv_);
    segment.cached_cpu_factor = factor;
    segment.lidar_factor_cache = desired_key;
    if (cache_hit) {
      *cache_hit = false;
    }
  } else if (cache_hit) {
    *cache_hit = true;
  }

  return segment.cached_cpu_factor;
}

#ifdef GTSAM_POINTS_USE_CUDA
std::shared_ptr<iap::IntegratedBSplineGICPFactorGPU> OdometryEstimationBSpline::get_or_create_gpu_bucket_lidar_factor(
  const iap::SplineBucketContext& bucket_ctx,
  ActiveSplineSegmentConstraint& segment,
  CUstream_st* stream,
  std::shared_ptr<gtsam_points::TempBufferManager> temp_buffer,
  bool* cache_hit,
  bool* target_refreshed) {
  const bool enable_profile_surface =
    lidar_factor_profile_ || lidar_warn_degeneracy_ || lidar_export_baseline_csv_ || lidar_profile_numeric_reference_;
  auto factor = std::make_shared<iap::IntegratedBSplineGICPFactorGPU>(
    bucket_ctx,
    segment.target_snapshot,
    segment.source,
    stream,
    temp_buffer);
  factor->set_jacobian_mode(to_gpu_lidar_jacobian_mode(lidar_jacobian_mode_));
  factor->set_numeric_eps(lidar_jacobian_numeric_eps_);
  factor->set_max_correspondence_distance(max_correspondence_distance_);
  factor->set_correspondence_candidate_count(lidar_correspondence_candidate_count_);
  factor->set_correspondence_accept_ratio(lidar_correspondence_accept_ratio_);
  factor->set_correspondence_min_score_gap(lidar_correspondence_min_score_gap_);
  factor->set_outlier_mahalanobis_threshold(lidar_outlier_mahalanobis_thresh_);
  factor->set_robust_kernel(lidar_robust_kernel_, lidar_robust_kernel_width_);
  factor->set_robust_weight_floor(lidar_robust_weight_floor_);
  factor->set_enable_profiling(enable_profile_surface);
  if (cache_hit) {
    *cache_hit = false;
  }
  if (target_refreshed) {
    *target_refreshed = false;
  }
  return factor;
}

std::shared_ptr<iap::IntegratedBSplineGICPFactorGPUKernel> OdometryEstimationBSpline::get_or_create_gpu_kernel_lidar_factor(
  const iap::SplineBucketContext& bucket_ctx,
  ActiveSplineSegmentConstraint& segment,
  CUstream_st* stream,
  std::shared_ptr<gtsam_points::TempBufferManager> temp_buffer,
  bool* cache_hit,
  bool* target_refreshed) {
  auto desired_key = make_lidar_factor_cache_key(segment, true);
  desired_key.control_indices = bucket_ctx.support.ctrl_indices;
  desired_key.bucket_u_signature = std::hash<double>{}(bucket_ctx.support.u);
  desired_key.source_identity = bucket_ctx.point_indices.empty() ? nullptr : &bucket_ctx.point_indices;
  const bool enable_profile_surface =
    lidar_factor_profile_ || lidar_warn_degeneracy_ || lidar_export_baseline_csv_ || lidar_profile_numeric_reference_;
  const bool cache_base_match =
    segment.cached_gpu_kernel_factor && same_lidar_factor_cache_base(segment.lidar_factor_cache, desired_key);
  const bool can_refresh_target =
    cache_base_match &&
    segment.lidar_factor_cache.target_mode == desired_key.target_mode &&
    segment.lidar_factor_cache.source_identity == desired_key.source_identity;

  if (!cache_base_match) {
    auto factor = std::make_shared<iap::IntegratedBSplineGICPFactorGPUKernel>(
      bucket_ctx,
      segment.target_snapshot,
      segment.source,
      stream,
      temp_buffer);
    factor->set_jacobian_mode(lidar_jacobian_mode_);
    factor->set_numeric_eps(lidar_jacobian_numeric_eps_);
    factor->set_max_correspondence_distance(max_correspondence_distance_);
    factor->set_correspondence_candidate_count(lidar_correspondence_candidate_count_);
    factor->set_correspondence_accept_ratio(lidar_correspondence_accept_ratio_);
    factor->set_correspondence_min_score_gap(lidar_correspondence_min_score_gap_);
    factor->set_outlier_mahalanobis_threshold(lidar_outlier_mahalanobis_thresh_);
    factor->set_robust_kernel(lidar_robust_kernel_, lidar_robust_kernel_width_);
    factor->set_robust_weight_floor(lidar_robust_weight_floor_);
    factor->set_enable_profiling(enable_profile_surface);
    segment.cached_gpu_kernel_factor = factor;
    segment.lidar_factor_cache = desired_key;
    if (cache_hit) {
      *cache_hit = false;
    }
    if (target_refreshed) {
      *target_refreshed = false;
    }
  } else {
    segment.cached_gpu_kernel_factor->set_enable_profiling(enable_profile_surface);
    if (can_refresh_target &&
        (segment.lidar_factor_cache.target_identity != desired_key.target_identity ||
         segment.lidar_factor_cache.target_revision != desired_key.target_revision)) {
      segment.cached_gpu_kernel_factor->refresh_target(segment.target_snapshot);
      segment.lidar_factor_cache = desired_key;
      if (target_refreshed) {
        *target_refreshed = true;
      }
    } else if (target_refreshed) {
      *target_refreshed = false;
    }
    if (cache_hit) {
      *cache_hit = true;
    }
  }

  return segment.cached_gpu_kernel_factor;
}
#endif

EstimationFrame::ConstPtr OdometryEstimationBSpline::insert_frame_ct_lidar(
  const PreprocessedFrame::Ptr& raw_frame,
  std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  const bool use_gpu_lidar = frontend_mode_ == "CT_LIDAR_GPU";
  if (use_legacy_bspline_two_stage_path_ || use_gpu_lidar) {
    return insert_frame_ct_lidar_legacy_two_stage(raw_frame, marginalized_frames);
  }
  if (unified_solver_mode_ == iap::BSplineUnifiedSolverMode::INCREMENTAL_SMOOTHER) {
    return insert_frame_ct_lidar_incremental_graph(raw_frame, marginalized_frames);
  }
  return insert_frame_ct_lidar_unified_graph(raw_frame, marginalized_frames);
}

EstimationFrame::ConstPtr OdometryEstimationBSpline::insert_frame_ct_lidar_legacy_two_stage(
  const PreprocessedFrame::Ptr& raw_frame,
  std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  using Clock = std::chrono::steady_clock;
  const auto t_window_start = Clock::now();
  const auto elapsed_ms = [](const Clock::time_point& start, const Clock::time_point& end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
  };
  struct PipelineTiming {
    double gnss_mailbox_sync_ms = 0.0;
    double source_cloud_ms = 0.0;
    double segment_prepare_ms = 0.0;
    double target_build_ms = 0.0;
    double gnss_epoch_fetch_ms = 0.0;
    double marginalization_partition_ms = 0.0;
    double graph_build_ms = 0.0;
    double graph_lidar_factor_ms = 0.0;
    double graph_lidar_factor_new_build_ms = 0.0;
    double graph_lidar_factor_target_refresh_ms = 0.0;
    double graph_lidar_factor_reused_attach_ms = 0.0;
    double graph_velocity_factor_ms = 0.0;
    double imu_factor_assembly_ms = 0.0;
    double gnss_factor_assembly_ms = 0.0;
    double carried_prior_attach_ms = 0.0;
    double graph_prediction_prior_ms = 0.0;
    double graph_smoothness_ms = 0.0;
    double graph_shared_prior_ms = 0.0;
    double graph_clock_factor_ms = 0.0;
    double lm_optimize_ms = 0.0;
    double prune_active_ms = 0.0;
    double carried_prior_update_ms = 0.0;
    double marginalization_ms = 0.0;
    double postprocess_ms = 0.0;
    double post_lidar_result_ms = 0.0;
    double post_lidar_factor_error_ms = 0.0;
    double post_lidar_numeric_audit_ms = 0.0;
    double post_lidar_degeneracy_ms = 0.0;
    double post_lidar_result_pack_ms = 0.0;
    double post_lidar_window_aggregate_ms = 0.0;
    double post_lidar_csv_ms = 0.0;
    double post_lidar_log_ms = 0.0;
    double post_frame_state_ms = 0.0;
    double post_deskew_ms = 0.0;
    double post_covariance_ms = 0.0;
    double post_frame_store_ms = 0.0;
    double post_target_insert_ms = 0.0;
    double post_history_update_ms = 0.0;
    double post_publish_traj_ms = 0.0;
    double post_publish_telemetry_ms = 0.0;
    double post_callback_ms = 0.0;
    double window_wall_ms = 0.0;
    std::size_t graph_lidar_factor_cache_hit_count = 0;
    std::size_t graph_lidar_factor_cache_miss_count = 0;
    std::size_t graph_lidar_factor_refresh_count = 0;
  } pipeline_timing;

  Callbacks::on_insert_frame(raw_frame);
  {
    const auto t_sync_start = Clock::now();
    sync_gnss_epochs_from_shared_state();
    pipeline_timing.gnss_mailbox_sync_ms = elapsed_ms(t_sync_start, Clock::now());
  }

  const int current = frames.size();
  const double scan_duration = std::max(1e-3, raw_frame->scan_end_time - raw_frame->stamp);
  const bool use_gpu_lidar = frontend_mode_ == "CT_LIDAR_GPU";
  const bool collect_window_lidar_results = lidar_collect_window_results();

  EstimationFrame::Ptr new_frame(new EstimationFrame);
  new_frame->id = current;
  new_frame->stamp = raw_frame->stamp;
  new_frame->T_lidar_imu = T_lidar_imu;
  new_frame->raw_frame = raw_frame;
  new_frame->frame_id = FrameID::LIDAR;
  new_frame->v_world_imu.setZero();
  new_frame->imu_bias.head<3>() = fixed_lag_registry_.shared_state().accel_bias;
  new_frame->imu_bias.tail<3>() = fixed_lag_registry_.shared_state().gyro_bias;

  if (frames.empty()) {
    EstimationFrame::ConstPtr init_state;
    if (init_estimation) {
      init_estimation->insert_frame(raw_frame);
      init_state = init_estimation->initial_pose();
    }

    if (!init_state && init_estimation) {
      logger->debug("waiting for initial IMU state estimation to be finished (bspline ct frontend)");
      return nullptr;
    }

    const gtsam::Pose3 initial_pose = init_state
      ? gtsam::Pose3(init_state->T_world_lidar.matrix())
      : gtsam::Pose3();

    initialize_control_window(raw_frame, initial_pose);

    new_frame->T_world_lidar = Eigen::Isometry3d(initial_pose.matrix());
    new_frame->T_world_imu = new_frame->T_world_lidar * T_lidar_imu;
    new_frame->frame = create_lidar_source_cloud(raw_frame);
    new_frame->imu_bias = init_state ? init_state->imu_bias : params->imu_bias;
    new_frame->v_world_imu = init_state ? init_state->v_world_imu : Eigen::Vector3d::Zero();
    fixed_lag_registry_.set_shared_imu_state(
      new_frame->imu_bias.tail<3>(),
      new_frame->imu_bias.head<3>(),
      fixed_lag_registry_.shared_state().gravity);
    fixed_lag_registry_.clear_auxiliary_values();
    fixed_lag_registry_.auxiliary_values().insert(iap::bspline_velocity_key(control_window_->states()[1].index), new_frame->v_world_imu);

    Callbacks::on_new_frame(new_frame);
    insert_target_cloud(new_frame);
    update_frame_history(new_frame, marginalized_frames);
    update_marginal_prior_from_active_window();
    publish_continuous_trajectory(current);
    publish_fixed_lag_telemetry(current);

    std::vector<EstimationFrame::ConstPtr> active_frames(frames.inner_begin(), frames.inner_end());
    if (!active_frames.empty()) {
      Callbacks::on_update_new_frame(active_frames.back());
      Callbacks::on_update_frames(active_frames);
    }

    if (init_estimation) {
      init_estimation.reset();
    }
    return new_frame;
  }

  {
    const auto t_source_start = Clock::now();
    new_frame->frame = create_lidar_source_cloud(raw_frame);
    pipeline_timing.source_cloud_ms = elapsed_ms(t_source_start, Clock::now());
  }

  const auto ct_frontend_input = make_frontend_input(raw_frame, new_frame->frame);
  const auto ct_local_result = ct_local_frontend_.run(ct_frontend_input);
  if (frontend_only_mode_) {
    iap::FrontendFrameProfile frontend_frame_profile = ct_local_result.processed.frame_profile;
    frontend_frame_profile.frame_id = new_frame->id;
    frontend_frame_profile.stamp = raw_frame->stamp;
    frontend_frame_profile.frontend_mode = frontend_mode_;
    frontend_frame_profile.frontend_only_mode = true;
    frontend_frame_profile.use_legacy_two_stage_path = true;
    frontend_frame_profile.preprocess_ms = pipeline_timing.source_cloud_ms;
    frontend_frame_profile.target_map_prep_ms = ct_frontend_input.target_map_prep_ms;
    if (frontend_frame_profile_enabled_ && target_map_prep_breakdown_enabled_) {
      frontend_frame_profile.target_snapshot_clone_ms = ct_frontend_input.target_snapshot_clone_ms;
      frontend_frame_profile.target_voxel_lookup_prep_ms = ct_frontend_input.target_voxel_lookup_prep_ms;
      frontend_frame_profile.target_covariance_prep_ms = ct_frontend_input.target_covariance_prep_ms;
      frontend_frame_profile.source_to_target_transform_ms = ct_frontend_input.source_to_target_transform_ms;
    }
    frontend_frame_profile.marginalization_ms = 0.0;
    frontend_frame_profile.backend_update_ms = 0.0;
    frontend_frame_profile.backend_optimize_ms = 0.0;
    frontend_frame_profile.local_mapping_update_ms = 0.0;
    frontend_frame_profile.global_mapping_update_ms = 0.0;
    frontend_frame_profile.submap_registration_ms = 0.0;
    frontend_frame_profile.gnss_factor_count = 0;
    frontend_frame_profile.carried_prior_count = 0;
    frontend_frame_profile.backend_factor_count = 0;
    frontend_frame_profile.backend_state_count = 0;
    frontend_frame_profile.optimize_count = 1;
    frontend_frame_profile.local_layer_enabled = true;
    frontend_frame_profile.navigation_layer_enabled = false;
    frontend_frame_profile.local_layer_factor_count = ct_local_result.backend_summary.lidar_factor_count +
      ct_local_result.processed.frame_profile.imu_factor_count;
    frontend_frame_profile.navigation_layer_factor_count = 0;
    frontend_frame_profile.local_layer_active_state_count = ct_local_result.debug_stats.active_local_controls.size();
    frontend_frame_profile.navigation_layer_active_state_count = 0;
    frontend_frame_profile.carried_prior_replay_success = false;

    const auto frontend_layout = std::make_shared<const iap::SplineStateLayout>(ct_local_result.layout);
    auto frontend_evaluator = frontend_layout
      ? std::make_shared<iap::SplineEvaluator>(frontend_layout)
      : nullptr;
    auto evaluate_frontend_pose = [&](double stamp, const gtsam::Pose3& fallback) {
      if (!frontend_layout || !frontend_evaluator) {
        return fallback;
      }

      if (const auto support = frontend_layout->support_at(stamp, iap::SplineSensorId::Lidar)) {
        return frontend_evaluator->eval_pose(ct_local_result.local_values, *support, iap::SplineSensorId::Lidar);
      }
      return fallback;
    };

    const gtsam::Pose3 fallback_pose = frames.back()
      ? gtsam::Pose3(frames.back()->T_world_lidar.matrix())
      : gtsam::Pose3();
    const gtsam::Pose3 start_pose = evaluate_frontend_pose(raw_frame->stamp, fallback_pose);

    new_frame->T_world_lidar = Eigen::Isometry3d(start_pose.matrix());
    new_frame->T_world_imu = new_frame->T_world_lidar * T_lidar_imu;

    const auto& frontend_controls = ct_local_result.layout.controls();
    if (!frontend_controls.empty()) {
      const gtsam::Key velocity_key = iap::bspline_velocity_key(frontend_controls.back().index);
      if (ct_local_result.local_values.exists(velocity_key)) {
        new_frame->v_world_imu = ct_local_result.local_values.at<gtsam::Vector3>(velocity_key);
      }
    }
    if (ct_local_result.local_values.exists(gtsam::symbol('k', 0))) {
      new_frame->imu_bias.head<3>() = ct_local_result.local_values.at<gtsam::Vector3>(gtsam::symbol('k', 0));
    }
    if (ct_local_result.local_values.exists(gtsam::symbol('j', 0))) {
      new_frame->imu_bias.tail<3>() = ct_local_result.local_values.at<gtsam::Vector3>(gtsam::symbol('j', 0));
    }
    if (!frames.empty() && frames.back()) {
      new_frame->clk_bias = frames.back()->clk_bias;
      new_frame->clk_drift = frames.back()->clk_drift;
    }
    fixed_lag_registry_.set_shared_imu_state(
      new_frame->imu_bias.tail<3>(),
      new_frame->imu_bias.head<3>(),
      fixed_lag_registry_.shared_state().gravity);

    auto deskewed_frame = ct_local_result.processed.deskewed_source_cloud
      ? gtsam_points::PointCloudCPU::clone(*ct_local_result.processed.deskewed_source_cloud)
      : gtsam_points::PointCloudCPU::clone(*new_frame->frame);
    std::vector<Eigen::Vector4d> deskewed_points;
    deskewed_points.reserve(static_cast<std::size_t>(deskewed_frame->size()));
    for (int i = 0; i < deskewed_frame->size(); ++i) {
      deskewed_points.push_back(deskewed_frame->points[i]);
    }
    const auto deskewed_covs = covariance_estimation->estimate(deskewed_points, raw_frame->neighbors);
    for (int i = 0; i < deskewed_frame->size(); ++i) {
      deskewed_frame->points[i] = deskewed_points[static_cast<std::size_t>(i)];
      if (static_cast<std::size_t>(i) < deskewed_covs.size()) {
        deskewed_frame->covs[i] = deskewed_covs[static_cast<std::size_t>(i)];
      }
    }
    new_frame->frame = deskewed_frame;

    if (ct_local_result.processed.lidar_window_summary.valid) {
      new_frame->icp_quality.inlier_count =
        static_cast<int>(ct_local_result.processed.lidar_window_summary.total_inlier_point_count);
      new_frame->icp_quality.inlier_fraction =
        ct_local_result.processed.lidar_window_summary.weighted_inlier_ratio;
      new_frame->icp_quality.rmse = std::sqrt(
        ct_local_result.processed.total_lidar_factor_error /
        std::max(new_frame->icp_quality.inlier_count, 1));
    }

    Callbacks::on_new_frame(new_frame);
    insert_target_cloud(new_frame);
    update_frame_history(new_frame, marginalized_frames);
    {
      const auto t_publish_start = Clock::now();
      publish_continuous_trajectory_from_layout(frontend_layout, ct_local_result.local_values);
      if (publish_shared_trajectory_) {
        iap::IapSharedState::instance().clear_bspline_fixed_lag_telemetry();
      }
      frontend_frame_profile.publish_ms = elapsed_ms(t_publish_start, Clock::now());
    }

    std::vector<EstimationFrame::ConstPtr> active_frames(frames.inner_begin(), frames.inner_end());
    if (!active_frames.empty()) {
      Callbacks::on_update_new_frame(active_frames.back());
      Callbacks::on_update_frames(active_frames);
    }

    const auto frame_warning_profile = build_frame_warning_profile(
      frontend_frame_profile.frame_id,
      frontend_frame_profile.stamp,
      ct_frontend_input,
      ct_local_result,
      frontend_frame_profile);
    frontend_frame_profile.warning_count_for_frame = frame_warning_profile.warning_count;

    maybe_write_frontend_frame_profile(frontend_frame_profile);
    maybe_write_lidar_factor_profiles(
      frontend_frame_profile.frame_id,
      frontend_frame_profile.stamp,
      ct_local_result.processed.bucket_profiles);
    maybe_write_frontend_lm_iterations(
      frontend_frame_profile.frame_id,
      frontend_frame_profile.stamp,
      ct_local_result.processed.lm_iterations);
    maybe_write_frame_warning_profile(frame_warning_profile);
    log_frontend_only_stats(frontend_frame_profile);

    return new_frame;
  }

  const gtsam::Pose3 predicted_end_pose = predict_scan_end_pose(scan_duration);
  control_window_->advance(raw_frame->stamp, raw_frame->scan_end_time, predicted_end_pose);
  fixed_lag_registry_.append_window(*control_window_);
  refresh_active_window_layout();

  if (const auto anchor = iap::IapSharedState::instance().get_gnss_anchor()) {
    fixed_lag_registry_.set_shared_gnss_anchor(anchor->origin_ecef, gtsam::Rot3(anchor->R_ecef_world));
  }

  const double min_active_stamp = std::max(0.0, raw_frame->stamp - params->smoother_lag);
  const auto factor_source = gtsam_points::PointCloudCPU::clone(*new_frame->frame);
  {
    const auto t_segment_prepare_start = Clock::now();
    append_active_segment_constraint(raw_frame, factor_source);
    pipeline_timing.segment_prepare_ms = elapsed_ms(t_segment_prepare_start, Clock::now());
    if (!fixed_lag_registry_.segments().empty()) {
      pipeline_timing.target_build_ms = fixed_lag_registry_.segments().back().target_build_ms;
    }
  }
  if (!fixed_lag_registry_.segments().empty()) {
    const auto t_gnss_epoch_start = Clock::now();
    fixed_lag_registry_.segments().back().gnss_epochs = consume_segment_gnss_epochs(
      raw_frame->stamp,
      raw_frame->scan_end_time);
    pipeline_timing.gnss_epoch_fetch_ms = elapsed_ms(t_gnss_epoch_start, Clock::now());
  }

  gtsam::Values values = fixed_lag_registry_.control_buffer().values();
  const auto& active_states = fixed_lag_registry_.control_buffer().states();
  const iap::BSplineCarriedPrior previous_carried_prior = marginal_prior_.carried_prior;
  const gtsam::Key gyro_bias_key = bspline_gyro_bias_key();
  const gtsam::Key accel_bias_key = bspline_accel_bias_key();
  const gtsam::Key gravity_key = bspline_gravity_key();
  fixed_lag_registry_.seed_shared_values(values, fixed_lag_registry_.shared_state().gnss_anchor_initialized);

  gtsam::NonlinearFactorGraph graph;
  gtsam::NonlinearFactorGraph marginalization_graph;
  std::shared_ptr<iap::IntegratedBSplineGICPFactor> current_cpu_factor;
  std::vector<std::shared_ptr<iap::IntegratedSplineGICPFactor>> active_lidar_cpu_factors;
#ifdef GTSAM_POINTS_USE_CUDA
  std::shared_ptr<iap::IntegratedBSplineGICPFactorGPU> current_gpu_factor;
  std::vector<std::shared_ptr<iap::IntegratedBSplineGICPFactorGPU>> active_lidar_gpu_factors;
  std::shared_ptr<iap::IntegratedBSplineGICPFactorGPUKernel> current_gpu_kernel_factor;
  std::vector<std::shared_ptr<iap::IntegratedBSplineGICPFactorGPUKernel>> active_lidar_gpu_kernel_factors;
#endif
  std::size_t active_imu_factor_count = 0;
  std::size_t active_velocity_factor_count = 0;
  std::size_t active_gnss_pr_factor_count = 0;
  std::size_t active_gnss_dop_factor_count = 0;
  struct ActiveClockState {
    gtsam::Key key = 0;
    double stamp = 0.0;
    gtsam::Vector2 value = gtsam::Vector2::Zero();
  };
  std::vector<ActiveClockState> active_clock_states;
  const gtsam::Key ecef_origin_key = iap::bspline_ecef_origin_key();
  const gtsam::Key ecef_rot_key = iap::bspline_ecef_rot_key();
  const auto gnss_pr_sigma = [&](const iap::SatObs& sat) {
    const double canopy_sigma = iap::sigma_eff_canopy(gnss_canopy_params_, sat.kappa, sat.elevation);
    return std::max({1e-3, sat.pr_sigma, canopy_sigma, gnss_pr_noise_base_});
  };
  const auto gnss_dop_sigma = [&](const iap::SatObs& sat) {
    const double sin_el = std::sin(std::max(sat.elevation, gnss_min_elevation_));
    const double modeled = gnss_dop_noise_base_ / std::pow(std::max(0.052, sin_el), gnss_elev_noise_exp_);
    return std::max({1e-3, sat.dop_sigma, modeled});
  };
  auto segment_poses_from_values = [&](const ActiveSplineSegmentConstraint& segment) {
    std::array<gtsam::Pose3, iap::kBSplineControlPointCount> poses;
    for (std::size_t k = 0; k < iap::kBSplineControlPointCount; ++k) {
      poses[k] = values.at<gtsam::Pose3>(iap::bspline_control_point_key(segment.control_indices[k]));
    }
    return poses;
  };
  std::vector<ActiveClockState> seeded_clock_states;
  if (fixed_lag_registry_.shared_state().gnss_anchor_initialized) {
    for (const auto& segment : fixed_lag_registry_.segments()) {
      if (segment.gnss_epochs.empty()) {
        continue;
      }

      const gtsam::Key clock_key = iap::bspline_clock_key(segment.auxiliary_index);
      if (!values.exists(clock_key)) {
        gtsam::Vector2 init_clock = gtsam::Vector2::Zero();
        if (fixed_lag_registry_.auxiliary_values().exists(clock_key)) {
          init_clock = fixed_lag_registry_.auxiliary_values().at<gtsam::Vector2>(clock_key);
        } else if (!seeded_clock_states.empty()) {
          init_clock = seeded_clock_states.back().value;
          const double dt = std::max(0.0, segment.stamp - seeded_clock_states.back().stamp);
          init_clock(0) += init_clock(1) * dt;
        }
        values.insert(clock_key, init_clock);
      }

      seeded_clock_states.push_back(ActiveClockState{clock_key, segment.stamp, values.at<gtsam::Vector2>(clock_key)});
    }

  }
  for (const auto& segment : fixed_lag_registry_.segments()) {
    const gtsam::Key velocity_key = iap::bspline_velocity_key(segment.auxiliary_index);
    if (values.exists(velocity_key)) {
      continue;
    }

    if (fixed_lag_registry_.auxiliary_values().exists(velocity_key)) {
      values.insert(velocity_key, fixed_lag_registry_.auxiliary_values().at<gtsam::Vector3>(velocity_key));
      continue;
    }

    const auto segment_poses = segment_poses_from_values(segment);
    const gtsam::Vector3 velocity_guess =
      iap::IntegratedBSplineVelocityFactor::predict_velocity(
        segment_poses,
        0.0,
        std::max(1e-3, segment.scan_end - segment.stamp),
        trajectory_params_.finite_difference_dt);
    values.insert(velocity_key, velocity_guess);
  }

  const bool include_clock_states = !seeded_clock_states.empty();
  const iap::SplineActiveStateSet active_state_set = [&] {
    const auto t_partition_start = Clock::now();
    auto active_state_set = fixed_lag_registry_.active_state_set(values, min_active_stamp, include_clock_states);
    pipeline_timing.marginalization_partition_ms = elapsed_ms(t_partition_start, Clock::now());
    return active_state_set;
  }();
  const iap::BSplineMarginalizationPartition marginalization_partition =
    iap::build_bspline_marginalization_partition(active_state_set);
  const auto t_graph_build_start = Clock::now();
  const bool enable_lidar_profile_surface =
    lidar_factor_profile_ || lidar_warn_degeneracy_ || lidar_export_baseline_csv_;
  const auto factor_layout = active_window_layout_ ? active_window_layout_ : build_active_window_layout();
  for (std::size_t i = 0; i < fixed_lag_registry_.segments().size(); ++i) {
    auto& segment = fixed_lag_registry_.segments()[i];
    const gtsam::Key velocity_key = iap::bspline_velocity_key(segment.auxiliary_index);

    if (!values.exists(velocity_key)) {
      if (fixed_lag_registry_.auxiliary_values().exists(velocity_key)) {
        values.insert(velocity_key, fixed_lag_registry_.auxiliary_values().at<gtsam::Vector3>(velocity_key));
      } else {
        const auto segment_poses = segment_poses_from_values(segment);
        const gtsam::Vector3 velocity_guess =
          iap::IntegratedBSplineVelocityFactor::predict_velocity(
            segment_poses,
            0.0,
            std::max(1e-3, segment.scan_end - segment.stamp),
            trajectory_params_.finite_difference_dt);
        values.insert(velocity_key, velocity_guess);
      }
    }

    std::array<gtsam::Key, iap::kBSplineControlPointCount> segment_keys{};
    for (std::size_t k = 0; k < iap::kBSplineControlPointCount; ++k) {
      segment_keys[k] = iap::bspline_control_point_key(segment.control_indices[k]);
    }

    if (!use_gpu_lidar) {
      const auto t_graph_lidar_start = Clock::now();
      const auto t_lidar_prepare_start = Clock::now();
      const auto bucket_contexts = create_segment_lidar_buckets(segment);
      std::vector<std::shared_ptr<iap::IntegratedSplineGICPFactor>> segment_bucket_factors;
      segment_bucket_factors.reserve(bucket_contexts.size());
      for (const auto& bucket_ctx : bucket_contexts) {
        auto factor = std::make_shared<iap::IntegratedSplineGICPFactor>(
          bucket_ctx,
          segment.target_snapshot,
          segment.source,
          segment.target_tree);
        factor->set_num_threads(params->num_threads);
        factor->set_max_correspondence_distance(max_correspondence_distance_);
        factor->set_jacobian_mode(lidar_jacobian_mode_);
        factor->set_numeric_eps(lidar_jacobian_numeric_eps_);
        factor->set_correspondence_candidate_count(lidar_correspondence_candidate_count_);
        factor->set_correspondence_accept_ratio(lidar_correspondence_accept_ratio_);
        factor->set_correspondence_min_score_gap(lidar_correspondence_min_score_gap_);
        factor->set_outlier_mahalanobis_threshold(lidar_outlier_mahalanobis_thresh_);
        factor->set_robust_kernel(lidar_robust_kernel_, lidar_robust_kernel_width_);
        factor->set_robust_weight_floor(lidar_robust_weight_floor_);
        factor->set_enable_profiling(enable_lidar_profile_surface);
        segment_bucket_factors.push_back(factor);
      }
      const auto t_lidar_prepare_end = Clock::now();
      pipeline_timing.graph_lidar_factor_cache_miss_count++;
      pipeline_timing.graph_lidar_factor_new_build_ms += elapsed_ms(t_lidar_prepare_start, t_lidar_prepare_end);

      const auto t_lidar_attach_start = Clock::now();
      for (std::size_t bucket_idx = 0; bucket_idx < segment_bucket_factors.size(); ++bucket_idx) {
        const auto& factor = segment_bucket_factors[bucket_idx];
        const auto& bucket_ctx = bucket_contexts[bucket_idx];
        active_lidar_cpu_factors.push_back(factor);
        graph.add(factor);
        if (marginalization_partition.should_marginalize_factor(make_key_vector(bucket_ctx.support))) {
          marginalization_graph.add(factor);
        }
      }
      const auto t_lidar_attach_end = Clock::now();

      if (i + 1 == fixed_lag_registry_.segments().size()) {
        bool current_cache_hit = false;
        current_cpu_factor = get_or_create_cpu_lidar_factor(segment, &current_cache_hit);
        current_cpu_factor->set_enable_profiling(enable_lidar_profile_surface);
      }
      pipeline_timing.graph_lidar_factor_ms += elapsed_ms(t_graph_lidar_start, Clock::now());
    } else {
#ifdef GTSAM_POINTS_USE_CUDA
      switch (lidar_gpu_backend_) {
        case BSplineGpuLidarBackend::BUCKET: {
          const auto t_graph_lidar_start = Clock::now();
          const auto t_lidar_prepare_start = Clock::now();
          const auto bucket_contexts = create_segment_lidar_buckets(segment);
          std::vector<std::shared_ptr<iap::IntegratedBSplineGICPFactorGPU>> segment_bucket_factors;
          segment_bucket_factors.reserve(bucket_contexts.size());
          if (!ct_lidar_gpu_stream_buffers_) {
            ct_lidar_gpu_stream_buffers_ = std::make_unique<gtsam_points::StreamTempBufferRoundRobin>();
          }
          for (const auto& bucket_ctx : bucket_contexts) {
            auto stream_buffer = ct_lidar_gpu_stream_buffers_->get_stream_buffer();
            bool cache_hit = false;
            bool target_refreshed = false;
            auto factor = get_or_create_gpu_bucket_lidar_factor(
              bucket_ctx,
              segment,
              stream_buffer.first,
              stream_buffer.second,
              &cache_hit,
              &target_refreshed);
            factor->set_enable_profiling(enable_lidar_profile_surface);
            segment_bucket_factors.push_back(factor);
          }
          const auto t_lidar_prepare_end = Clock::now();
          pipeline_timing.graph_lidar_factor_cache_miss_count++;
          pipeline_timing.graph_lidar_factor_new_build_ms += elapsed_ms(t_lidar_prepare_start, t_lidar_prepare_end);

          const auto t_lidar_attach_start = Clock::now();
          for (std::size_t bucket_idx = 0; bucket_idx < segment_bucket_factors.size(); ++bucket_idx) {
            const auto& factor = segment_bucket_factors[bucket_idx];
            const auto& bucket_ctx = bucket_contexts[bucket_idx];
            active_lidar_gpu_factors.push_back(factor);
            graph.add(factor);
            if (marginalization_partition.should_marginalize_factor(make_key_vector(bucket_ctx.support))) {
              marginalization_graph.add(factor);
            }
          }
          const auto t_lidar_attach_end = Clock::now();
          pipeline_timing.graph_lidar_factor_reused_attach_ms += elapsed_ms(t_lidar_attach_start, t_lidar_attach_end);

          if (i + 1 == fixed_lag_registry_.segments().size() && !segment_bucket_factors.empty()) {
            current_gpu_factor = segment_bucket_factors.back();
          }
          pipeline_timing.graph_lidar_factor_ms += elapsed_ms(t_graph_lidar_start, Clock::now());
          break;
        }
        case BSplineGpuLidarBackend::KERNEL:
        {
          const auto t_graph_lidar_start = Clock::now();
          const auto bucket_contexts = create_segment_lidar_buckets(segment);
          if (!ct_lidar_gpu_stream_buffers_) {
            ct_lidar_gpu_stream_buffers_ = std::make_unique<gtsam_points::StreamTempBufferRoundRobin>();
          }
          std::vector<std::shared_ptr<iap::IntegratedBSplineGICPFactorGPUKernel>> segment_kernel_factors;
          segment_kernel_factors.reserve(bucket_contexts.size());
          const auto t_lidar_prepare_start = Clock::now();
          for (const auto& bucket_ctx : bucket_contexts) {
            auto stream_buffer = ct_lidar_gpu_stream_buffers_->get_stream_buffer();
            bool cache_hit = false;
            bool target_refreshed = false;
            auto factor = get_or_create_gpu_kernel_lidar_factor(
              bucket_ctx,
              segment,
              stream_buffer.first,
              stream_buffer.second,
              &cache_hit,
              &target_refreshed);
            factor->set_enable_profiling(enable_lidar_profile_surface);
            segment_kernel_factors.push_back(factor);
            if (cache_hit) {
              pipeline_timing.graph_lidar_factor_cache_hit_count++;
              if (target_refreshed) {
                pipeline_timing.graph_lidar_factor_refresh_count++;
              }
            } else {
              pipeline_timing.graph_lidar_factor_cache_miss_count++;
            }
          }
          const auto t_lidar_prepare_end = Clock::now();
          pipeline_timing.graph_lidar_factor_new_build_ms += elapsed_ms(t_lidar_prepare_start, t_lidar_prepare_end);

          const auto t_lidar_attach_start = Clock::now();
          for (std::size_t bucket_idx = 0; bucket_idx < segment_kernel_factors.size(); ++bucket_idx) {
            const auto& factor = segment_kernel_factors[bucket_idx];
            const auto& bucket_ctx = bucket_contexts[bucket_idx];
            active_lidar_gpu_kernel_factors.push_back(factor);
            graph.add(factor);
            if (marginalization_partition.should_marginalize_factor(make_key_vector(bucket_ctx.support))) {
              marginalization_graph.add(factor);
            }
          }
          const auto t_lidar_attach_end = Clock::now();
          pipeline_timing.graph_lidar_factor_reused_attach_ms += elapsed_ms(t_lidar_attach_start, t_lidar_attach_end);
          if (i + 1 == fixed_lag_registry_.segments().size() && !segment_kernel_factors.empty()) {
            current_gpu_kernel_factor = segment_kernel_factors.back();
          }
          pipeline_timing.graph_lidar_factor_ms += elapsed_ms(t_graph_lidar_start, Clock::now());
          break;
        }
      }
#else
      logger->error("CT_LIDAR_GPU requested but CUDA support is unavailable");
      return nullptr;
#endif
    }

    const auto t_graph_velocity_start = Clock::now();
    auto velocity_factor = std::make_shared<iap::IntegratedBSplineVelocityFactor>(
      segment_keys,
      velocity_key,
      0.0,
      std::max(1e-3, segment.scan_end - segment.stamp),
      velocity_ct_inf_scale_,
      trajectory_params_.finite_difference_dt);
    graph.add(velocity_factor);
    if (marginalization_partition.should_marginalize_factor(make_key_vector(segment_keys, {velocity_key}))) {
      marginalization_graph.add(velocity_factor);
    }
    active_velocity_factor_count++;
    pipeline_timing.graph_velocity_factor_ms += elapsed_ms(t_graph_velocity_start, Clock::now());

    const auto t_imu_factor_start = Clock::now();
    for (const auto& imu_sample : segment.imu_samples) {
      if (!factor_layout) {
        continue;
      }

      const auto support = factor_layout->support_at(imu_sample.stamp, iap::SplineSensorId::Imu);
      if (!support) {
        continue;
      }

      iap::SplineStampContext ctx;
      ctx.support = *support;
      ctx.sensor_id = iap::SplineSensorId::Imu;

      auto imu_factor = std::make_shared<iap::IntegratedSplineIMUFactor>(
        ctx,
        gyro_bias_key,
        accel_bias_key,
        gravity_key,
        imu_sample.angular_vel,
        imu_sample.linear_acc,
        imu_ct_trans_inf_scale_,
        imu_ct_rot_inf_scale_,
        factor_layout);
      graph.add(imu_factor);
      if (marginalization_partition.should_marginalize_factor(
            make_key_vector(ctx.support, {gyro_bias_key, accel_bias_key, gravity_key}))) {
        marginalization_graph.add(imu_factor);
      }
      active_imu_factor_count++;
    }
    pipeline_timing.imu_factor_assembly_ms += elapsed_ms(t_imu_factor_start, Clock::now());

    if (fixed_lag_registry_.shared_state().gnss_anchor_initialized && !segment.gnss_epochs.empty()) {
      const auto t_gnss_factor_start = Clock::now();
      const gtsam::Key clock_key = iap::bspline_clock_key(segment.auxiliary_index);
      active_clock_states.push_back(ActiveClockState{clock_key, segment.stamp, values.at<gtsam::Vector2>(clock_key)});
      if (!factor_layout) {
        pipeline_timing.gnss_factor_assembly_ms += elapsed_ms(t_gnss_factor_start, Clock::now());
        continue;
      }

      for (const auto& epoch : segment.gnss_epochs) {
        const auto support = factor_layout->support_at(epoch.stamp, iap::SplineSensorId::Gnss);
        if (!support) {
          continue;
        }

        iap::SplineStampContext ctx;
        ctx.support = *support;
        ctx.sensor_id = iap::SplineSensorId::Gnss;

        for (const auto& sat : epoch.sats) {
          if (sat.excluded || sat.elevation < gnss_min_elevation_) {
            continue;
          }

          iap::PseudorangeObservation pr_obs;
          pr_obs.pr_meas = sat.pr_meas;
          pr_obs.sat_pos = sat.sat_pos;
          pr_obs.tgd = sat.tgd;
          pr_obs.gps_sec = epoch.gps_sec;
          pr_obs.iono_params = epoch.iono_params;
          pr_obs.sigma = gnss_pr_sigma(sat);
          pr_obs.sat_id = sat.sat_id;
          pr_obs.constellation = sat.constellation;
          pr_obs.elevation = sat.elevation;

          auto pr_factor = std::make_shared<iap::IntegratedSplinePseudorangeFactor>(
            ctx,
            clock_key,
            ecef_origin_key,
            ecef_rot_key,
            pr_obs,
            factor_layout);
          graph.add(pr_factor);
          if (marginalization_partition.should_marginalize_factor(
                make_key_vector(ctx.support, {clock_key, ecef_origin_key, ecef_rot_key}))) {
            marginalization_graph.add(pr_factor);
          }
          active_gnss_pr_factor_count++;

          iap::DopplerObservation dop_obs;
          dop_obs.dop_meas = sat.dop_meas;
          dop_obs.sat_pos = sat.sat_pos;
          dop_obs.sat_vel = sat.sat_vel;
          dop_obs.anc_ecef_approx = fixed_lag_registry_.shared_state().ecef_origin;
          dop_obs.sigma = gnss_dop_sigma(sat);
          dop_obs.sat_id = sat.sat_id;
          dop_obs.constellation = sat.constellation;
          dop_obs.elevation = sat.elevation;

          auto dop_factor = std::make_shared<iap::IntegratedSplineDopplerFactor>(
            ctx,
            velocity_key,
            clock_key,
            ecef_rot_key,
            dop_obs,
            factor_layout);
          graph.add(dop_factor);
          if (marginalization_partition.should_marginalize_factor(
                make_key_vector(ctx.support, {velocity_key, clock_key, ecef_rot_key}))) {
            marginalization_graph.add(dop_factor);
          }
          active_gnss_dop_factor_count++;
        }
      }
      pipeline_timing.gnss_factor_assembly_ms += elapsed_ms(t_gnss_factor_start, Clock::now());
    }

  }

  if (!current_cpu_factor && 
#ifdef GTSAM_POINTS_USE_CUDA
      !current_gpu_factor &&
      !current_gpu_kernel_factor
#else
      true
#endif
  ) {
    logger->error("bspline ct frontend failed to create current segment factor");
    return nullptr;
  }

  const auto anchor_noise = gtsam::noiseModel::Isotropic::Precision(6, ctrl_point_anchor_inf_scale_);
  const auto pred_noise = gtsam::noiseModel::Isotropic::Precision(6, ctrl_point_prediction_inf_scale_);
  const auto smooth_noise = gtsam::noiseModel::Isotropic::Precision(6, ctrl_point_smoothness_inf_scale_);
  const auto marginal_noise = gtsam::noiseModel::Isotropic::Precision(6, ctrl_point_marginal_inf_scale_);
  const auto velocity_prior_noise = gtsam::noiseModel::Isotropic::Precision(3, velocity_ct_inf_scale_);
  const auto imu_bias_noise = gtsam::noiseModel::Isotropic::Precision(3, imu_ct_bias_inf_scale_);
  const auto gravity_noise = gtsam::noiseModel::Isotropic::Precision(3, imu_ct_gravity_inf_scale_);
  const auto gnss_ecef_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3::Constant(gnss_sigma_ecef_origin_));
  const auto gnss_ecef_rot_noise = gtsam::noiseModel::Isotropic::Sigma(3, gnss_sigma_ecef_rot_);
  const auto clock_prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector2() << params->clk_bias_noise, params->clk_drift_noise).finished());
  const auto& shared_state = fixed_lag_registry_.shared_state();

  const bool use_marginal_prior =
    marginal_prior_.valid &&
    active_states.size() >= 2 &&
    active_states[0].index == marginal_prior_.control_indices[0] &&
    active_states[1].index == marginal_prior_.control_indices[1];
  const bool use_information_marginal_prior =
    !marginal_prior_.carried_prior.empty() &&
    marginalization_partition.can_replay_keys(marginal_prior_.carried_prior.retained_keys, values);

  bool information_prior_attached = false;
  const auto t_carried_prior_attach_start = Clock::now();
  if (use_information_marginal_prior) {
    try {
      const auto replayed_prior = marginal_prior_.carried_prior.replay();
      for (const auto& factor : replayed_prior) {
        if (!factor) {
          continue;
        }
        graph.add(factor->clone());
      }
      information_prior_attached = true;
    } catch (const std::exception& e) {
      logger->warn("failed to attach bspline marginal information prior, fallback to handcrafted prior: {}", e.what());
      marginal_prior_.carried_prior = iap::BSplineCarriedPrior();
    }
  }

  if (!information_prior_attached && use_marginal_prior) {
    auto pose_prior = std::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      iap::bspline_control_point_key(active_states[0].index),
      marginal_prior_.first_pose,
      marginal_noise);
    auto delta_prior = std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      iap::bspline_control_point_key(active_states[0].index),
      iap::bspline_control_point_key(active_states[1].index),
      marginal_prior_.relative_delta,
      marginal_noise);
    graph.add(pose_prior);
    graph.add(delta_prior);
    marginalization_graph.add(pose_prior);
    marginalization_graph.add(delta_prior);
    if (marginal_prior_.has_velocity && values.exists(iap::bspline_velocity_key(marginal_prior_.auxiliary_index))) {
      auto velocity_prior = std::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        iap::bspline_velocity_key(marginal_prior_.auxiliary_index),
        marginal_prior_.velocity,
        velocity_prior_noise);
      graph.add(velocity_prior);
      marginalization_graph.add(velocity_prior);
    }
  } else if (!active_states.empty()) {
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      iap::bspline_control_point_key(active_states.front().index),
      active_states.front().pose,
      anchor_noise);
    if (active_states.size() >= 2) {
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        iap::bspline_control_point_key(active_states[1].index),
        active_states[1].pose,
      anchor_noise);
    }
  }
  pipeline_timing.carried_prior_attach_ms = elapsed_ms(t_carried_prior_attach_start, Clock::now());

  const auto t_graph_prediction_prior_start = Clock::now();
  if (active_states.size() >= 2) {
    const auto& pred_a = active_states[active_states.size() - 2];
    const auto& pred_b = active_states.back();
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      iap::bspline_control_point_key(pred_a.index),
      pred_a.pose,
      pred_noise);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      iap::bspline_control_point_key(pred_b.index),
      pred_b.pose,
      pred_noise);
  }
  pipeline_timing.graph_prediction_prior_ms = elapsed_ms(t_graph_prediction_prior_start, Clock::now());

  const auto t_graph_smoothness_start = Clock::now();
  for (std::size_t i = 0; i + 1 < active_states.size(); ++i) {
    const gtsam::Key key_i = iap::bspline_control_point_key(active_states[i].index);
    const gtsam::Key key_j = iap::bspline_control_point_key(active_states[i + 1].index);
    auto smooth_factor = std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      key_i,
      key_j,
      active_states[i].pose.between(active_states[i + 1].pose),
      smooth_noise);
    graph.add(smooth_factor);
    if (marginalization_partition.should_marginalize_factor(gtsam::KeyVector{key_i, key_j})) {
      marginalization_graph.add(smooth_factor);
    }
  }
  pipeline_timing.graph_smoothness_ms = elapsed_ms(t_graph_smoothness_start, Clock::now());

  const auto t_graph_shared_prior_start = Clock::now();
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gyro_bias_key, shared_state.gyro_bias, imu_bias_noise);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(accel_bias_key, shared_state.accel_bias, imu_bias_noise);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gravity_key, shared_state.gravity, gravity_noise);
  if (shared_state.gnss_anchor_initialized && values.exists(ecef_origin_key) && values.exists(ecef_rot_key)) {
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(ecef_origin_key, shared_state.ecef_origin, gnss_ecef_noise);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Rot3>>(ecef_rot_key, shared_state.ecef_rot, gnss_ecef_rot_noise);
  }
  pipeline_timing.graph_shared_prior_ms = elapsed_ms(t_graph_shared_prior_start, Clock::now());

  const auto t_graph_clock_factor_start = Clock::now();
  if (!active_clock_states.empty()) {
    const bool clock_constrained_by_information_prior =
      information_prior_attached &&
      std::find(
        marginal_prior_.carried_prior.retained_keys.begin(),
        marginal_prior_.carried_prior.retained_keys.end(),
        active_clock_states.front().key) != marginal_prior_.carried_prior.retained_keys.end();
    if (!clock_constrained_by_information_prior) {
      const bool use_clock_boundary_prior =
        use_marginal_prior &&
        marginal_prior_.has_clock &&
        active_clock_states.front().key == iap::bspline_clock_key(marginal_prior_.auxiliary_index);
      const gtsam::Vector2 boundary_clock =
        use_clock_boundary_prior ? marginal_prior_.clock : active_clock_states.front().value;
      auto clock_prior = std::make_shared<gtsam::PriorFactor<gtsam::Vector2>>(
        active_clock_states.front().key,
        boundary_clock,
        clock_prior_noise);
      graph.add(clock_prior);
      if (use_clock_boundary_prior) {
        marginalization_graph.add(clock_prior);
      }
    }
    for (std::size_t i = 1; i < active_clock_states.size(); ++i) {
      const double dt = std::max(1e-3, active_clock_states[i].stamp - active_clock_states[i - 1].stamp);
      auto clock_between = std::make_shared<iap::ClockBetweenFactor>(
        active_clock_states[i - 1].key,
        active_clock_states[i].key,
        dt,
        iap::ClockBetweenFactor::make_noise(dt, gnss_clock_between_params_));
      graph.add(clock_between);
      if (marginalization_partition.should_marginalize_factor(
            gtsam::KeyVector{active_clock_states[i - 1].key, active_clock_states[i].key})) {
        marginalization_graph.add(clock_between);
      }
    }
  }
  pipeline_timing.graph_clock_factor_ms = elapsed_ms(t_graph_clock_factor_start, Clock::now());
  pipeline_timing.graph_build_ms = elapsed_ms(t_graph_build_start, Clock::now());

  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setlambdaInitial(1e-4);
  lm_params.setAbsoluteErrorTol(1e-2);
  lm_params.setMaxIterations(lm_max_iterations_);

  const auto t_optimize_start = Clock::now();
  try {
    values = gtsam_points::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();
  } catch (const std::exception& e) {
    logger->error("bspline ct frontend optimization failed: {}", e.what());
  }
  pipeline_timing.lm_optimize_ms = elapsed_ms(t_optimize_start, Clock::now());

  // IAP-RQ-300 / IAP-RQ-410: Feed GNSS epochs into the compact backend graph
  // using the frontend layout. This runs after the monolithic LM solve so the
  // backend can use the same graph/values that the solver just updated.
  {
    const auto ct_backend_input = make_backend_input(ct_local_result);
    ct_compact_backend_.update(ct_local_result, ct_backend_input, &graph, &values);
  }

  const auto keys = control_window_->keys();
  logger->trace("bspline ct active_window={} active_segment_factors={} active_velocity_factors={} active_imu_factors={} active_gnss_pr_factors={} active_gnss_dop_factors={} current_segment_keys={} {} {} {}",
    active_states.size(),
    fixed_lag_registry_.segments().size(),
    active_velocity_factor_count,
    active_imu_factor_count,
    active_gnss_pr_factor_count,
    active_gnss_dop_factor_count,
    static_cast<std::uint64_t>(keys[0]),
    static_cast<std::uint64_t>(keys[1]),
    static_cast<std::uint64_t>(keys[2]),
    static_cast<std::uint64_t>(keys[3]));

  fixed_lag_registry_.control_buffer().update_from_values(values);
  control_window_->update_from_values(values);
  fixed_lag_registry_.update_shared_state_from_values(values);
  fixed_lag_registry_.clear_auxiliary_values();
  for (const auto& segment : fixed_lag_registry_.segments()) {
    const gtsam::Key velocity_key = iap::bspline_velocity_key(segment.auxiliary_index);
    if (values.exists(velocity_key) && !fixed_lag_registry_.auxiliary_values().exists(velocity_key)) {
      fixed_lag_registry_.auxiliary_values().insert(velocity_key, values.at<gtsam::Vector3>(velocity_key));
    }
    const gtsam::Key clock_key = iap::bspline_clock_key(segment.auxiliary_index);
    if (values.exists(clock_key) && !fixed_lag_registry_.auxiliary_values().exists(clock_key)) {
      fixed_lag_registry_.auxiliary_values().insert(clock_key, values.at<gtsam::Vector2>(clock_key));
    }
  }
  {
    const auto t_prune_start = Clock::now();
    prune_active_ct_state(min_active_stamp, active_state_set);
    refresh_active_window_layout();
    pipeline_timing.prune_active_ms = elapsed_ms(t_prune_start, Clock::now());
  }
  {
    const auto t_carried_prior_update_start = Clock::now();
    update_marginal_prior_from_active_window();
    update_marginal_prior_information(
      marginalization_graph,
      values,
      active_state_set.active_keys(),
      previous_carried_prior.empty() ? nullptr : &previous_carried_prior);
    pipeline_timing.carried_prior_update_ms = elapsed_ms(t_carried_prior_update_start, Clock::now());
  }
  pipeline_timing.marginalization_ms = pipeline_timing.prune_active_ms + pipeline_timing.carried_prior_update_ms;

  const auto t_postprocess_start = Clock::now();
  std::vector<iap::BSplineLidarFactorResult> lidar_results;
  lidar_results.reserve(std::max<std::size_t>(
    1,
    use_gpu_lidar
#ifdef GTSAM_POINTS_USE_CUDA
      ? (collect_window_lidar_results
          ? (lidar_gpu_backend_ == BSplineGpuLidarBackend::BUCKET
               ? active_lidar_gpu_factors.size()
               : active_lidar_gpu_kernel_factors.size())
          : 1U)
#else
      ? 0
#endif
      : (collect_window_lidar_results ? active_lidar_cpu_factors.size() : 1U)));
  iap::BSplineLidarFactorResult current_lidar_result;
  iap::IntegratedBSplineGICPFactor::NumericReferenceCheckResult current_numeric_check;
  iap::IntegratedBSplineGICPFactor::DegeneracyDiagnostics current_degeneracy;
  iap::BSplineLidarNumericAudit current_gpu_numeric_check;
  iap::BSplineLidarDegeneracyReport current_gpu_degeneracy;
  bool current_numeric_check_valid = false;
  bool current_degeneracy_valid = false;
  bool current_gpu_numeric_check_valid = false;
  bool current_gpu_degeneracy_valid = false;
  int current_lidar_result_index = -1;
  const auto t_post_lidar_result_start = Clock::now();

  auto process_cpu_factor = [&](const std::shared_ptr<iap::IntegratedSplineGICPFactor>& factor, bool current_factor) {
    const auto t_factor_error_start = Clock::now();
    const double factor_error = factor->error(values);
    pipeline_timing.post_lidar_factor_error_ms += elapsed_ms(t_factor_error_start, Clock::now());

    const iap::IntegratedSplineGICPFactor::DegeneracyDiagnostics* degeneracy_ptr = nullptr;
    if (lidar_warn_degeneracy_) {
      const auto t_degeneracy_start = Clock::now();
      current_degeneracy = factor->diagnose_degeneracy(lidar_degeneracy_thresholds_);
      pipeline_timing.post_lidar_degeneracy_ms += elapsed_ms(t_degeneracy_start, Clock::now());
      degeneracy_ptr = &current_degeneracy;
    }

    const iap::IntegratedSplineGICPFactor::NumericReferenceCheckResult* numeric_ptr = nullptr;
    if (lidar_profile_numeric_reference_ && current_factor) {
      const auto t_numeric_start = Clock::now();
      current_numeric_check = factor->check_against_numeric_full(values, lidar_numeric_reference_scale_);
      current_numeric_check_valid = current_numeric_check.valid;
      pipeline_timing.post_lidar_numeric_audit_ms += elapsed_ms(t_numeric_start, Clock::now());
      numeric_ptr = &current_numeric_check;
    }

    const auto t_result_pack_start = Clock::now();
    auto result = factor->make_result(factor_error, factor->num_inliers(), factor->inlier_fraction(), numeric_ptr, degeneracy_ptr);
    pipeline_timing.post_lidar_result_pack_ms += elapsed_ms(t_result_pack_start, Clock::now());
    if (current_factor) {
      current_lidar_result = result;
      current_degeneracy_valid = degeneracy_ptr && degeneracy_ptr->valid;
      current_lidar_result_index = static_cast<int>(lidar_results.size());
    }
    lidar_results.push_back(std::move(result));
  };

  auto update_current_cpu_reference = [&]() {
    if (!current_cpu_factor) {
      return;
    }

    const auto t_factor_error_start = Clock::now();
    const double factor_error = current_cpu_factor->error(values);
    pipeline_timing.post_lidar_factor_error_ms += elapsed_ms(t_factor_error_start, Clock::now());

    const iap::IntegratedBSplineGICPFactor::DegeneracyDiagnostics* degeneracy_ptr = nullptr;
    if (lidar_warn_degeneracy_) {
      const auto t_degeneracy_start = Clock::now();
      current_degeneracy = current_cpu_factor->diagnose_degeneracy(lidar_degeneracy_thresholds_);
      current_degeneracy_valid = current_degeneracy.valid;
      pipeline_timing.post_lidar_degeneracy_ms += elapsed_ms(t_degeneracy_start, Clock::now());
      degeneracy_ptr = &current_degeneracy;
    }

    const iap::IntegratedBSplineGICPFactor::NumericReferenceCheckResult* numeric_ptr = nullptr;
    if (lidar_profile_numeric_reference_) {
      const auto t_numeric_start = Clock::now();
      current_numeric_check = current_cpu_factor->check_against_numeric_full(values, lidar_numeric_reference_scale_);
      current_numeric_check_valid = current_numeric_check.valid;
      pipeline_timing.post_lidar_numeric_audit_ms += elapsed_ms(t_numeric_start, Clock::now());
      numeric_ptr = &current_numeric_check;
    }

    const auto t_result_pack_start = Clock::now();
    current_lidar_result = current_cpu_factor->make_result(
      factor_error,
      current_cpu_factor->num_inliers(),
      current_cpu_factor->inlier_fraction(),
      numeric_ptr,
      degeneracy_ptr);
    pipeline_timing.post_lidar_result_pack_ms += elapsed_ms(t_result_pack_start, Clock::now());
  };

#ifdef GTSAM_POINTS_USE_CUDA
  auto process_gpu_factor = [&](const std::shared_ptr<iap::IntegratedBSplineGICPFactorGPU>& factor, bool current_factor) {
    const auto t_factor_error_start = Clock::now();
    const double factor_error = factor->error(values);
    pipeline_timing.post_lidar_factor_error_ms += elapsed_ms(t_factor_error_start, Clock::now());

    const iap::BSplineLidarDegeneracyReport* degeneracy_ptr = nullptr;
    if (lidar_warn_degeneracy_) {
      const auto t_degeneracy_start = Clock::now();
      current_gpu_degeneracy = factor->diagnose_degeneracy(lidar_degeneracy_thresholds_);
      current_gpu_degeneracy_valid = current_gpu_degeneracy.valid;
      pipeline_timing.post_lidar_degeneracy_ms += elapsed_ms(t_degeneracy_start, Clock::now());
      degeneracy_ptr = &current_gpu_degeneracy;
    }

    const iap::BSplineLidarNumericAudit* numeric_ptr = nullptr;
    if (lidar_profile_numeric_reference_ && current_factor) {
      const auto t_numeric_start = Clock::now();
      current_gpu_numeric_check = factor->check_against_numeric_full(values, lidar_numeric_reference_scale_);
      current_gpu_numeric_check_valid = current_gpu_numeric_check.valid;
      pipeline_timing.post_lidar_numeric_audit_ms += elapsed_ms(t_numeric_start, Clock::now());
      numeric_ptr = &current_gpu_numeric_check;
    }

    const auto t_result_pack_start = Clock::now();
    auto result = factor->make_result(factor_error, factor->num_inliers(), factor->inlier_fraction(), numeric_ptr, degeneracy_ptr);
    pipeline_timing.post_lidar_result_pack_ms += elapsed_ms(t_result_pack_start, Clock::now());
    if (current_factor) {
      current_lidar_result = result;
      current_gpu_degeneracy_valid = degeneracy_ptr && degeneracy_ptr->valid;
      current_lidar_result_index = static_cast<int>(lidar_results.size());
    }
    lidar_results.push_back(std::move(result));
  };

  auto process_gpu_kernel_factor = [&](const std::shared_ptr<iap::IntegratedBSplineGICPFactorGPUKernel>& factor, bool current_factor) {
    const auto t_factor_error_start = Clock::now();
    const double factor_error = factor->error(values);
    pipeline_timing.post_lidar_factor_error_ms += elapsed_ms(t_factor_error_start, Clock::now());

    const iap::BSplineLidarDegeneracyReport* degeneracy_ptr = nullptr;
    if (lidar_warn_degeneracy_) {
      const auto t_degeneracy_start = Clock::now();
      current_gpu_degeneracy = factor->diagnose_degeneracy(lidar_degeneracy_thresholds_);
      current_gpu_degeneracy_valid = current_gpu_degeneracy.valid;
      pipeline_timing.post_lidar_degeneracy_ms += elapsed_ms(t_degeneracy_start, Clock::now());
      degeneracy_ptr = &current_gpu_degeneracy;
    }

    const iap::BSplineLidarNumericAudit* numeric_ptr = nullptr;
    if (lidar_profile_numeric_reference_ && current_factor) {
      const auto t_numeric_start = Clock::now();
      current_gpu_numeric_check = factor->check_against_numeric_full(values, lidar_numeric_reference_scale_);
      current_gpu_numeric_check_valid = current_gpu_numeric_check.valid;
      pipeline_timing.post_lidar_numeric_audit_ms += elapsed_ms(t_numeric_start, Clock::now());
      numeric_ptr = &current_gpu_numeric_check;
    }

    const auto t_result_pack_start = Clock::now();
    auto result = factor->make_result(factor_error, factor->num_inliers(), factor->inlier_fraction(), numeric_ptr, degeneracy_ptr);
    pipeline_timing.post_lidar_result_pack_ms += elapsed_ms(t_result_pack_start, Clock::now());
    if (current_factor) {
      current_lidar_result = result;
      current_gpu_degeneracy_valid = degeneracy_ptr && degeneracy_ptr->valid;
      current_lidar_result_index = static_cast<int>(lidar_results.size());
    }
    lidar_results.push_back(std::move(result));
  };
#endif

  if (!use_gpu_lidar) {
    if (collect_window_lidar_results) {
      for (const auto& factor : active_lidar_cpu_factors) {
        process_cpu_factor(factor, false);
      }
    }
    update_current_cpu_reference();
    if (!collect_window_lidar_results && current_cpu_factor) {
      lidar_results.push_back(current_lidar_result);
      current_lidar_result_index = 0;
    }
  } else {
#ifdef GTSAM_POINTS_USE_CUDA
    if (collect_window_lidar_results) {
      if (lidar_gpu_backend_ == BSplineGpuLidarBackend::BUCKET) {
        for (const auto& factor : active_lidar_gpu_factors) {
          process_gpu_factor(factor, factor == current_gpu_factor);
        }
      } else {
        for (const auto& factor : active_lidar_gpu_kernel_factors) {
          process_gpu_kernel_factor(factor, factor == current_gpu_kernel_factor);
        }
      }
    } else if (lidar_gpu_backend_ == BSplineGpuLidarBackend::BUCKET && current_gpu_factor) {
      process_gpu_factor(current_gpu_factor, true);
    } else if (lidar_gpu_backend_ == BSplineGpuLidarBackend::KERNEL && current_gpu_kernel_factor) {
      process_gpu_kernel_factor(current_gpu_kernel_factor, true);
    }
#endif
  }

  const auto t_post_lidar_aggregate_start = Clock::now();
  const auto lidar_window_summary = iap::aggregate_bspline_lidar_factor_results(lidar_results);
  pipeline_timing.post_lidar_window_aggregate_ms = elapsed_ms(t_post_lidar_aggregate_start, Clock::now());
  pipeline_timing.post_lidar_result_ms = elapsed_ms(t_post_lidar_result_start, Clock::now());
  {
    const auto t_post_lidar_csv_start = Clock::now();
    maybe_export_lidar_baseline_csv(raw_frame->stamp, lidar_results, current_lidar_result_index);
    pipeline_timing.post_lidar_csv_ms = elapsed_ms(t_post_lidar_csv_start, Clock::now());
  }

  const auto t_post_lidar_log_start = Clock::now();
  if (lidar_factor_profile_ && lidar_window_summary.valid) {
    if (!use_gpu_lidar) {
      logger->trace(
        "bspline ct lidar cpu-summary backend={} segments={} profiled={} warnings={} total_src={} total_tgt={} total_matched={} total_inliers={} weighted_match={:.3f} weighted_inlier={:.3f} mean_unique_ratio={:.3f} min_unique_ratio={:.3f} max_reuse_ratio={:.3f} max_ambiguity_rej={:.3f} max_numeric_rel={:.6f} max_axis_rel={:.6f} total_pose_ms={:.3f} total_corr_ms={:.3f} total_accum_ms={:.3f} total_factor_ms={:.3f} cand_eval={} mean_cand_per_src={:.2f} mean_bucket={:.2f} peak_bucket={}",
        iap::to_string(iap::BSplineLidarFactorBackend::CPU_GICP),
        lidar_window_summary.result_count,
        lidar_window_summary.valid_profile_count,
        lidar_window_summary.warning_result_count,
        lidar_window_summary.total_source_point_count,
        lidar_window_summary.total_target_point_count,
        lidar_window_summary.total_matched_point_count,
        lidar_window_summary.total_inlier_point_count,
        lidar_window_summary.weighted_match_ratio,
        lidar_window_summary.weighted_inlier_ratio,
        lidar_window_summary.mean_unique_target_ratio,
        lidar_window_summary.min_unique_target_ratio,
        lidar_window_summary.max_target_reuse_ratio,
        lidar_window_summary.max_ambiguity_rejection_ratio,
        lidar_window_summary.max_numeric_rel_error,
        lidar_window_summary.max_rotation_axis_rel_error,
        lidar_window_summary.total_pose_update_ms,
        lidar_window_summary.total_correspondence_ms,
        lidar_window_summary.total_accumulation_ms,
        lidar_window_summary.total_factor_ms,
        lidar_window_summary.total_candidate_evaluation_count,
        lidar_window_summary.mean_candidates_per_source,
        lidar_window_summary.mean_time_bucket_population,
        lidar_window_summary.max_time_bucket_population);
    } else {
      logger->trace(
        "bspline ct lidar gpu-summary backend={} segments={} profiled={} warnings={} total_src={} total_tgt={} total_matched={} total_inliers={} weighted_match={:.3f} weighted_inlier={:.3f} mean_unique_ratio={:.3f} min_unique_ratio={:.3f} max_reuse_ratio={:.3f} max_ambiguity_rej={:.3f} max_numeric_rel={:.6f} max_axis_rel={:.6f} total_pose_ms={:.3f} total_corr_ms={:.3f} total_accum_ms={:.3f} total_factor_ms={:.3f} cand_eval={} mean_cand_per_src={:.2f} mean_bucket={:.2f} peak_bucket={}",
        iap::to_string(iap::BSplineLidarFactorBackend::GPU_GICP),
        lidar_window_summary.result_count,
        lidar_window_summary.valid_profile_count,
        lidar_window_summary.warning_result_count,
        lidar_window_summary.total_source_point_count,
        lidar_window_summary.total_target_point_count,
        lidar_window_summary.total_matched_point_count,
        lidar_window_summary.total_inlier_point_count,
        lidar_window_summary.weighted_match_ratio,
        lidar_window_summary.weighted_inlier_ratio,
        lidar_window_summary.mean_unique_target_ratio,
        lidar_window_summary.min_unique_target_ratio,
        lidar_window_summary.max_target_reuse_ratio,
        lidar_window_summary.max_ambiguity_rejection_ratio,
        lidar_window_summary.max_numeric_rel_error,
        lidar_window_summary.max_rotation_axis_rel_error,
        lidar_window_summary.total_pose_update_ms,
        lidar_window_summary.total_correspondence_ms,
        lidar_window_summary.total_accumulation_ms,
        lidar_window_summary.total_factor_ms,
        lidar_window_summary.total_candidate_evaluation_count,
        lidar_window_summary.mean_candidates_per_source,
        lidar_window_summary.mean_time_bucket_population,
        lidar_window_summary.max_time_bucket_population);
      logger->trace("bspline ct lidar gpu-backend backend={}", ::glim::to_string(lidar_gpu_backend_));
    }
  }

  if (lidar_factor_profile_) {
    const auto& current_segment = fixed_lag_registry_.segments().back();
    const auto& profile = current_lidar_result.profile;
    if (!use_gpu_lidar) {
      logger->trace(
        "bspline ct lidar factor target_mode={} jacobian_mode={} k_candidates={} accept_ratio={:.3f} score_gap={:.3f} robust_kernel={} robust_width={:.3f} robust_w_floor={:.3f} outlier_thresh={:.3f} target_frames={} target_points={} snapshot_frames={} snapshot_points={} snapshot_span_s={:.3f} snapshot_policy={} target_build_ms={:.3f} stage={} time_buckets={} bucket_mean={:.2f} bucket_peak={} cand_eval={} cand_per_src={:.2f} matched={}/{} inliers={} rej_dist={} rej_ambiguity={} rej_outlier={} rej_robust={} match_ratio={:.3f} inlier_ratio={:.3f} mean_w={:.3f} uniq_targets={} uniq_ratio={:.3f} reuse_peak={} reuse_ratio={:.3f} mean_dist={:.4f} max_dist={:.4f} mean_score={:.4f} mean_gap={:.4f} mean_ratio={:.4f} pose_ms={:.3f} corr_ms={:.3f} accum_ms={:.3f} total_ms={:.3f} error={:.6f}",
        ::glim::to_string(current_segment.target_mode),
        ::glim::to_string(lidar_jacobian_mode_),
        lidar_correspondence_candidate_count_,
        lidar_correspondence_accept_ratio_,
        lidar_correspondence_min_score_gap_,
        ::glim::to_string(lidar_robust_kernel_),
        lidar_robust_kernel_width_,
        lidar_robust_weight_floor_,
        lidar_outlier_mahalanobis_thresh_,
        current_segment.target_frame_count,
        current_segment.target_point_count,
        current_segment.snapshot_frame_count,
        current_segment.snapshot_point_count,
        current_segment.snapshot_span_sec,
        current_segment.snapshot_policy_accepted,
        current_segment.target_build_ms,
        profile.stage,
        profile.time_bucket_count,
        profile.mean_time_bucket_population,
        profile.max_time_bucket_population,
        profile.candidate_evaluation_count,
        profile.mean_candidates_per_source,
        profile.matched_point_count,
        profile.source_point_count,
        profile.inlier_point_count,
        profile.rejected_distance_count,
        profile.rejected_ambiguity_count,
        profile.rejected_outlier_count,
        profile.rejected_robust_count,
        profile.match_ratio,
        profile.inlier_ratio,
        profile.mean_robust_weight,
        profile.unique_target_count,
        profile.unique_target_ratio,
        profile.max_target_reuse,
        profile.max_target_reuse_ratio,
        profile.mean_match_distance,
        profile.max_match_distance,
        profile.mean_match_score,
        profile.mean_score_gap,
        profile.mean_score_ratio,
        profile.pose_update_ms,
        profile.correspondence_ms,
        profile.accumulation_ms,
        profile.total_ms,
        profile.total_error);
    } else {
      logger->trace(
        "bspline ct lidar gpu-factor target_mode={} jacobian_mode={} k_candidates={} accept_ratio={:.3f} score_gap={:.3f} robust_kernel={} robust_width={:.3f} robust_w_floor={:.3f} outlier_thresh={:.3f} target_frames={} target_points={} snapshot_frames={} snapshot_points={} snapshot_span_s={:.3f} snapshot_policy={} target_build_ms={:.3f} stage={} time_buckets={} bucket_mean={:.2f} bucket_peak={} cand_eval={} cand_per_src={:.2f} matched={}/{} inliers={} rej_dist={} rej_ambiguity={} rej_outlier={} rej_robust={} match_ratio={:.3f} inlier_ratio={:.3f} mean_w={:.3f} uniq_targets={} uniq_ratio={:.3f} reuse_peak={} reuse_ratio={:.3f} mean_dist={:.4f} max_dist={:.4f} mean_score={:.4f} mean_gap={:.4f} mean_ratio={:.4f} pose_ms={:.3f} corr_ms={:.3f} accum_ms={:.3f} total_ms={:.3f} error={:.6f}",
        ::glim::to_string(current_segment.target_mode),
        ::glim::to_string(lidar_jacobian_mode_),
        lidar_correspondence_candidate_count_,
        lidar_correspondence_accept_ratio_,
        lidar_correspondence_min_score_gap_,
        ::glim::to_string(lidar_robust_kernel_),
        lidar_robust_kernel_width_,
        lidar_robust_weight_floor_,
        lidar_outlier_mahalanobis_thresh_,
        current_segment.target_frame_count,
        current_segment.target_point_count,
        current_segment.snapshot_frame_count,
        current_segment.snapshot_point_count,
        current_segment.snapshot_span_sec,
        current_segment.snapshot_policy_accepted,
        current_segment.target_build_ms,
        profile.stage,
        profile.time_bucket_count,
        profile.mean_time_bucket_population,
        profile.max_time_bucket_population,
        profile.candidate_evaluation_count,
        profile.mean_candidates_per_source,
        profile.matched_point_count,
        profile.source_point_count,
        profile.inlier_point_count,
        profile.rejected_distance_count,
        profile.rejected_ambiguity_count,
        profile.rejected_outlier_count,
        profile.rejected_robust_count,
        profile.match_ratio,
        profile.inlier_ratio,
        profile.mean_robust_weight,
        profile.unique_target_count,
        profile.unique_target_ratio,
        profile.max_target_reuse,
        profile.max_target_reuse_ratio,
        profile.mean_match_distance,
        profile.max_match_distance,
        profile.mean_match_score,
        profile.mean_score_gap,
        profile.mean_score_ratio,
        profile.pose_update_ms,
        profile.correspondence_ms,
        profile.accumulation_ms,
        profile.total_ms,
        current_lidar_result.factor_error);
    }
  }

  if (!use_gpu_lidar && lidar_validate_linearization_) {
    const auto check = current_cpu_factor->check_linearization(values, lidar_linearization_check_scale_);
    if (check.valid) {
      const auto level =
        check.rel_error > lidar_linearization_warn_ratio_ ? spdlog::level::warn : spdlog::level::trace;
      logger->log(
        level,
        "bspline ct lidar linearization target_mode={} perturb={:.2e} base_error={:.6f} predicted={:.6f} actual={:.6f} abs={:.6e} rel={:.6f}",
        ::glim::to_string(fixed_lag_registry_.segments().back().target_mode),
        check.perturbation_scale,
        check.base_error,
        check.predicted_error,
        check.actual_error,
        check.abs_error,
        check.rel_error);
    }
  }

  if (!use_gpu_lidar && lidar_profile_numeric_reference_) {
    if (current_numeric_check_valid) {
      const auto& check = current_numeric_check;
      const double max_rel_error = std::max(check.rotation_rel_error, check.translation_rel_error);
      const auto level =
        max_rel_error > lidar_linearization_warn_ratio_ ? spdlog::level::warn : spdlog::level::trace;
      logger->log(
        level,
        "bspline ct lidar numeric-reference target_mode={} perturb={:.2e} rot_pred_num={:.6f} rot_pred_semi={:.6f} rot_actual={:.6f} rot_abs={:.6e} rot_rel={:.6f} rot_axis_max_rel={:.6f} rot_axis_mean_rel={:.6f} rot_axis_worst={} trans_pred_num={:.6f} trans_pred_semi={:.6f} trans_actual={:.6f} trans_abs={:.6e} trans_rel={:.6f}",
        ::glim::to_string(fixed_lag_registry_.segments().back().target_mode),
        check.perturbation_scale,
        check.numeric_rotation_predicted_error,
        check.semi_rotation_predicted_error,
        check.rotation_actual_error,
        check.rotation_abs_error,
        check.rotation_rel_error,
        check.max_rotation_axis_rel_error,
        check.mean_rotation_axis_rel_error,
        check.worst_rotation_axis,
        check.numeric_translation_predicted_error,
        check.semi_translation_predicted_error,
        check.translation_actual_error,
        check.translation_abs_error,
        check.translation_rel_error);
    }
  }

  if (use_gpu_lidar && lidar_profile_numeric_reference_) {
#ifdef GTSAM_POINTS_USE_CUDA
    if (current_gpu_numeric_check_valid) {
      const auto& check = current_gpu_numeric_check;
      const double max_rel_error = std::max(check.rotation_rel_error, check.translation_rel_error);
      const auto level =
        max_rel_error > lidar_linearization_warn_ratio_ ? spdlog::level::warn : spdlog::level::trace;
      logger->log(
        level,
        "bspline ct lidar gpu numeric-reference target_mode={} perturb={:.2e} rot_pred_num={:.6f} rot_pred_semi={:.6f} rot_actual={:.6f} rot_abs={:.6e} rot_rel={:.6f} rot_axis_max_rel={:.6f} rot_axis_mean_rel={:.6f} rot_axis_worst={} trans_pred_num={:.6f} trans_pred_semi={:.6f} trans_actual={:.6f} trans_abs={:.6e} trans_rel={:.6f}",
        ::glim::to_string(fixed_lag_registry_.segments().back().target_mode),
        check.perturbation_scale,
        check.numeric_rotation_predicted_error,
        check.semi_rotation_predicted_error,
        check.rotation_actual_error,
        check.rotation_abs_error,
        check.rotation_rel_error,
        check.max_rotation_axis_rel_error,
        check.mean_rotation_axis_rel_error,
        check.worst_rotation_axis,
        check.numeric_translation_predicted_error,
        check.semi_translation_predicted_error,
        check.translation_actual_error,
        check.translation_abs_error,
        check.translation_rel_error);
    }
#endif
  }

  if (!use_gpu_lidar && lidar_warn_degeneracy_) {
    const auto& diagnostics = current_degeneracy;
    const bool snapshot_fallback =
      lidar_target_mode_ == BSplineLidarTargetMode::ACTIVE_WINDOW_SNAPSHOT && !fixed_lag_registry_.segments().back().snapshot_policy_accepted;
    if (snapshot_fallback || (current_degeneracy_valid && diagnostics.has_warning())) {
      std::string flags;
      auto append_flag = [&](const char* flag) {
        if (!flags.empty()) {
          flags += "|";
        }
        flags += flag;
      };

      if (snapshot_fallback) {
        append_flag("snapshot_fallback");
      }
      if (diagnostics.empty_target) {
        append_flag("empty_target");
      }
      if (diagnostics.low_match_ratio) {
        append_flag("low_match");
      }
      if (diagnostics.low_inlier_ratio) {
        append_flag("low_inlier");
      }
      if (diagnostics.low_target_diversity) {
        append_flag("low_target_diversity");
      }
      if (diagnostics.high_target_reuse) {
        append_flag("high_target_reuse");
      }
      if (diagnostics.high_ambiguity_rejection) {
        append_flag("high_ambiguity_rejection");
      }
      if (diagnostics.weak_score_separation) {
        append_flag("weak_score_separation");
      }
      if (flags.empty()) {
        flags = "none";
      }

      const auto& current_segment = fixed_lag_registry_.segments().back();
      const auto& profile = current_cpu_factor->last_profiling_stats();
      logger->warn(
        "bspline ct lidar degeneracy target_mode={} flags={} snapshot_policy={} target_frames={} target_points={} match_ratio={:.3f} inlier_ratio={:.3f} uniq_ratio={:.3f} reuse_ratio={:.3f} ambiguity_rej_ratio={:.3f} score_gap={:.4f} cand_eval={} bucket_peak={} bucket_mean={:.2f}",
        ::glim::to_string(current_segment.target_mode),
        flags,
        current_segment.snapshot_policy_accepted,
        current_segment.target_frame_count,
        current_segment.target_point_count,
        profile.match_ratio,
        profile.inlier_ratio,
        profile.unique_target_ratio,
        profile.max_target_reuse_ratio,
        diagnostics.ambiguity_rejection_ratio,
        profile.mean_score_gap,
        profile.candidate_evaluation_count,
        profile.max_time_bucket_population,
        profile.mean_time_bucket_population);
    }
  }

  if (use_gpu_lidar && lidar_warn_degeneracy_) {
#ifdef GTSAM_POINTS_USE_CUDA
    const auto& diagnostics = current_gpu_degeneracy;
    const bool snapshot_fallback =
      lidar_target_mode_ == BSplineLidarTargetMode::ACTIVE_WINDOW_SNAPSHOT && !fixed_lag_registry_.segments().back().snapshot_policy_accepted;
    if (snapshot_fallback || (current_gpu_degeneracy_valid && diagnostics.has_warning())) {
      std::string flags;
      auto append_flag = [&](const char* flag) {
        if (!flags.empty()) {
          flags += "|";
        }
        flags += flag;
      };

      if (snapshot_fallback) {
        append_flag("snapshot_fallback");
      }
      if (diagnostics.empty_target) {
        append_flag("empty_target");
      }
      if (diagnostics.low_match_ratio) {
        append_flag("low_match");
      }
      if (diagnostics.low_inlier_ratio) {
        append_flag("low_inlier");
      }
      if (diagnostics.low_target_diversity) {
        append_flag("low_target_diversity");
      }
      if (diagnostics.high_target_reuse) {
        append_flag("high_target_reuse");
      }
      if (diagnostics.high_ambiguity_rejection) {
        append_flag("high_ambiguity_rejection");
      }
      if (diagnostics.weak_score_separation) {
        append_flag("weak_score_separation");
      }
      if (flags.empty()) {
        flags = "none";
      }

      const auto& current_segment = fixed_lag_registry_.segments().back();
      const auto& profile = current_lidar_result.profile;
      logger->warn(
        "bspline ct lidar gpu degeneracy target_mode={} flags={} snapshot_policy={} target_frames={} target_points={} match_ratio={:.3f} inlier_ratio={:.3f} uniq_ratio={:.3f} reuse_ratio={:.3f} ambiguity_rej_ratio={:.3f} score_gap={:.4f} cand_eval={} bucket_peak={} bucket_mean={:.2f}",
        ::glim::to_string(current_segment.target_mode),
        flags,
        current_segment.snapshot_policy_accepted,
        current_segment.target_frame_count,
        current_segment.target_point_count,
        profile.match_ratio,
        profile.inlier_ratio,
        profile.unique_target_ratio,
        profile.max_target_reuse_ratio,
        diagnostics.ambiguity_rejection_ratio,
        profile.mean_score_gap,
        profile.candidate_evaluation_count,
        profile.max_time_bucket_population,
        profile.mean_time_bucket_population);
    }
#endif
  }

  pipeline_timing.post_lidar_log_ms = elapsed_ms(t_post_lidar_log_start, Clock::now());

  const auto t_post_frame_state_start = Clock::now();
  auto evaluate_lidar_pose = [&](double stamp, double legacy_u) {
    if (active_window_layout_ && active_window_evaluator_) {
      if (const auto support = active_window_layout_->support_at(stamp, iap::SplineSensorId::Lidar)) {
        return active_window_evaluator_->eval_pose(values, *support, iap::SplineSensorId::Lidar);
      }
    }
    return control_window_->evaluate(legacy_u);
  };
  const gtsam::Pose3 start_pose = evaluate_lidar_pose(raw_frame->stamp, 0.0);
  const gtsam::Pose3 end_pose = evaluate_lidar_pose(raw_frame->scan_end_time, 1.0);
  (void)end_pose;
  new_frame->T_world_lidar = Eigen::Isometry3d(start_pose.matrix());
  new_frame->T_world_imu = new_frame->T_world_lidar * T_lidar_imu;
  new_frame->v_world_imu = values.at<gtsam::Vector3>(iap::bspline_velocity_key(control_window_->states()[1].index));
  new_frame->imu_bias.head<3>() = fixed_lag_registry_.shared_state().accel_bias;
  new_frame->imu_bias.tail<3>() = fixed_lag_registry_.shared_state().gyro_bias;
  const gtsam::Key current_clock_key = iap::bspline_clock_key(control_window_->states()[1].index);
  if (values.exists(current_clock_key)) {
    const auto clock = values.at<gtsam::Vector2>(current_clock_key);
    new_frame->clk_bias = clock(0);
    new_frame->clk_drift = clock(1);
  } else if (!frames.empty() && frames.back()) {
    new_frame->clk_bias = frames.back()->clk_bias;
    new_frame->clk_drift = frames.back()->clk_drift;
  }
  if (current_lidar_result.valid) {
    const double factor_error = current_lidar_result.factor_error;
    new_frame->icp_quality.inlier_count = current_lidar_result.inlier_count;
    new_frame->icp_quality.inlier_fraction = current_lidar_result.inlier_fraction;
    new_frame->icp_quality.rmse =
      std::sqrt(factor_error / std::max(new_frame->icp_quality.inlier_count, 1));
  }
  pipeline_timing.post_frame_state_ms = elapsed_ms(t_post_frame_state_start, Clock::now());

  std::vector<Eigen::Vector4d> deskewed_points;
  const auto t_post_deskew_start = Clock::now();
  if (!use_gpu_lidar) {
    deskewed_points = current_cpu_factor->deskewed_source_points(values, true);
  } else {
#ifdef GTSAM_POINTS_USE_CUDA
    if (lidar_gpu_backend_ == BSplineGpuLidarBackend::BUCKET) {
      deskewed_points = current_gpu_factor->deskewed_source_points(values, true);
    } else if (current_gpu_kernel_factor) {
      deskewed_points = current_gpu_kernel_factor->deskewed_source_points(values, true);
    }
#endif
  }
  pipeline_timing.post_deskew_ms = elapsed_ms(t_post_deskew_start, Clock::now());
  const auto t_post_covariance_start = Clock::now();
  const auto deskewed_covs = covariance_estimation->estimate(deskewed_points, raw_frame->neighbors);
  pipeline_timing.post_covariance_ms = elapsed_ms(t_post_covariance_start, Clock::now());
  const auto t_post_frame_store_start = Clock::now();
  for (int i = 0; i < new_frame->frame->size(); ++i) {
    new_frame->frame->points[i] = deskewed_points[static_cast<std::size_t>(i)];
    new_frame->frame->covs[i] = deskewed_covs[static_cast<std::size_t>(i)];
  }
  pipeline_timing.post_frame_store_ms = elapsed_ms(t_post_frame_store_start, Clock::now());

  {
    const auto t_post_callback_start = Clock::now();
    Callbacks::on_new_frame(new_frame);
    pipeline_timing.post_callback_ms += elapsed_ms(t_post_callback_start, Clock::now());
  }
  {
    const auto t_post_target_insert_start = Clock::now();
    insert_target_cloud(new_frame);
    pipeline_timing.post_target_insert_ms = elapsed_ms(t_post_target_insert_start, Clock::now());
  }
  {
    const auto t_post_history_update_start = Clock::now();
    update_frame_history(new_frame, marginalized_frames);
    pipeline_timing.post_history_update_ms = elapsed_ms(t_post_history_update_start, Clock::now());
  }
  {
    const auto t_post_publish_traj_start = Clock::now();
    publish_continuous_trajectory(current);
    pipeline_timing.post_publish_traj_ms = elapsed_ms(t_post_publish_traj_start, Clock::now());
  }
  {
    const auto t_post_publish_telemetry_start = Clock::now();
    publish_fixed_lag_telemetry(current);
    pipeline_timing.post_publish_telemetry_ms = elapsed_ms(t_post_publish_telemetry_start, Clock::now());
  }

  std::vector<EstimationFrame::ConstPtr> active_frames(frames.inner_begin(), frames.inner_end());
  if (!active_frames.empty()) {
    const auto t_post_callback_start = Clock::now();
    Callbacks::on_update_new_frame(active_frames.back());
    Callbacks::on_update_frames(active_frames);
    pipeline_timing.post_callback_ms += elapsed_ms(t_post_callback_start, Clock::now());
  }

  pipeline_timing.postprocess_ms = elapsed_ms(t_postprocess_start, Clock::now());
  pipeline_timing.window_wall_ms = elapsed_ms(t_window_start, Clock::now());

  if (pipeline_profile_) {
    const double profiled_stage_sum =
      pipeline_timing.gnss_mailbox_sync_ms +
      pipeline_timing.source_cloud_ms +
      pipeline_timing.segment_prepare_ms +
      pipeline_timing.gnss_epoch_fetch_ms +
      pipeline_timing.marginalization_partition_ms +
      pipeline_timing.graph_build_ms +
      pipeline_timing.lm_optimize_ms +
      pipeline_timing.marginalization_ms +
      pipeline_timing.postprocess_ms;
    const double unprofiled_ms = std::max(0.0, pipeline_timing.window_wall_ms - profiled_stage_sum);
    logger->info(
      "bspline ct pipeline-summary frontend={} gpu_backend={} lidar_result_scope={} frame={} scan_dt={:.3f} smoother_lag={:.2f} active_states={} active_segments={} gnss_mailbox_sync_ms={:.3f} source_cloud_ms={:.3f} segment_prepare_ms={:.3f} target_build_ms={:.3f} gnss_epoch_fetch_ms={:.3f} marginalization_partition_ms={:.3f} graph_build_ms={:.3f} graph_lidar_factor_ms={:.3f} graph_lidar_factor_new_build_ms={:.3f} graph_lidar_factor_target_refresh_ms={:.3f} graph_lidar_factor_reused_attach_ms={:.3f} graph_lidar_factor_cache_hits={} graph_lidar_factor_cache_misses={} graph_lidar_factor_refreshes={} graph_velocity_factor_ms={:.3f} imu_factor_assembly_ms={:.3f} gnss_factor_assembly_ms={:.3f} carried_prior_attach_ms={:.3f} graph_prediction_prior_ms={:.3f} graph_smoothness_ms={:.3f} graph_shared_prior_ms={:.3f} graph_clock_factor_ms={:.3f} lm_optimize_ms={:.3f} marginalization_ms={:.3f} prune_active_ms={:.3f} carried_prior_update_ms={:.3f} postprocess_ms={:.3f} post_lidar_result_ms={:.3f} post_lidar_factor_error_ms={:.3f} post_lidar_numeric_audit_ms={:.3f} post_lidar_degeneracy_ms={:.3f} post_lidar_result_pack_ms={:.3f} post_lidar_window_aggregate_ms={:.3f} post_lidar_csv_ms={:.3f} post_lidar_log_ms={:.3f} post_frame_state_ms={:.3f} post_deskew_ms={:.3f} post_covariance_ms={:.3f} post_frame_store_ms={:.3f} post_target_insert_ms={:.3f} post_history_update_ms={:.3f} post_publish_traj_ms={:.3f} post_publish_telemetry_ms={:.3f} post_callback_ms={:.3f} wall_ms={:.3f} unprofiled_ms={:.3f} lidar_factor_ms={:.3f} lidar_pose_ms={:.3f} lidar_corr_ms={:.3f} factor_results={} imu_factors={} gnss_pr_factors={} gnss_dop_factors={} velocity_factors={}",
      frontend_mode_,
      ::glim::to_string(lidar_gpu_backend_),
      collect_window_lidar_results ? "window" : "current_only",
      new_frame->id,
      scan_duration,
      params->smoother_lag,
      fixed_lag_registry_.control_buffer().states().size(),
      fixed_lag_registry_.segments().size(),
      pipeline_timing.gnss_mailbox_sync_ms,
      pipeline_timing.source_cloud_ms,
      pipeline_timing.segment_prepare_ms,
      pipeline_timing.target_build_ms,
      pipeline_timing.gnss_epoch_fetch_ms,
      pipeline_timing.marginalization_partition_ms,
      pipeline_timing.graph_build_ms,
      pipeline_timing.graph_lidar_factor_ms,
      pipeline_timing.graph_lidar_factor_new_build_ms,
      pipeline_timing.graph_lidar_factor_target_refresh_ms,
      pipeline_timing.graph_lidar_factor_reused_attach_ms,
      pipeline_timing.graph_lidar_factor_cache_hit_count,
      pipeline_timing.graph_lidar_factor_cache_miss_count,
      pipeline_timing.graph_lidar_factor_refresh_count,
      pipeline_timing.graph_velocity_factor_ms,
      pipeline_timing.imu_factor_assembly_ms,
      pipeline_timing.gnss_factor_assembly_ms,
      pipeline_timing.carried_prior_attach_ms,
      pipeline_timing.graph_prediction_prior_ms,
      pipeline_timing.graph_smoothness_ms,
      pipeline_timing.graph_shared_prior_ms,
      pipeline_timing.graph_clock_factor_ms,
      pipeline_timing.lm_optimize_ms,
      pipeline_timing.marginalization_ms,
      pipeline_timing.prune_active_ms,
      pipeline_timing.carried_prior_update_ms,
      pipeline_timing.postprocess_ms,
      pipeline_timing.post_lidar_result_ms,
      pipeline_timing.post_lidar_factor_error_ms,
      pipeline_timing.post_lidar_numeric_audit_ms,
      pipeline_timing.post_lidar_degeneracy_ms,
      pipeline_timing.post_lidar_result_pack_ms,
      pipeline_timing.post_lidar_window_aggregate_ms,
      pipeline_timing.post_lidar_csv_ms,
      pipeline_timing.post_lidar_log_ms,
      pipeline_timing.post_frame_state_ms,
      pipeline_timing.post_deskew_ms,
      pipeline_timing.post_covariance_ms,
      pipeline_timing.post_frame_store_ms,
      pipeline_timing.post_target_insert_ms,
      pipeline_timing.post_history_update_ms,
      pipeline_timing.post_publish_traj_ms,
      pipeline_timing.post_publish_telemetry_ms,
      pipeline_timing.post_callback_ms,
      pipeline_timing.window_wall_ms,
      unprofiled_ms,
      lidar_window_summary.total_factor_ms,
      lidar_window_summary.total_pose_update_ms,
      lidar_window_summary.total_correspondence_ms,
      lidar_window_summary.result_count,
      active_imu_factor_count,
      active_gnss_pr_factor_count,
      active_gnss_dop_factor_count,
      active_velocity_factor_count);
  }

  return new_frame;
}

EstimationFrame::ConstPtr OdometryEstimationBSpline::insert_frame_ct_lidar_unified_graph(
  const PreprocessedFrame::Ptr& raw_frame,
  std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  using Clock = std::chrono::steady_clock;
  const auto t_window_start = Clock::now();
  const auto elapsed_ms = [](const Clock::time_point& start, const Clock::time_point& end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
  };

  struct UnifiedTiming {
    double gnss_mailbox_sync_ms = 0.0;
    double source_cloud_ms = 0.0;
    double segment_prepare_ms = 0.0;
    double target_build_ms = 0.0;
    double gnss_epoch_fetch_ms = 0.0;
    double graph_build_ms = 0.0;
    double carried_prior_attach_ms = 0.0;
    double graph_prediction_prior_ms = 0.0;
    double graph_smoothness_ms = 0.0;
    double graph_shared_prior_ms = 0.0;
    double graph_clock_factor_ms = 0.0;
    double lm_optimize_ms = 0.0;
    double prune_active_ms = 0.0;
    double carried_prior_update_ms = 0.0;
    double marginalization_ms = 0.0;
    double postprocess_ms = 0.0;
    double post_publish_traj_ms = 0.0;
    double post_publish_telemetry_ms = 0.0;
    double post_callback_ms = 0.0;
    double post_target_insert_ms = 0.0;
    double post_history_update_ms = 0.0;
    double window_wall_ms = 0.0;
  } timing;

  Callbacks::on_insert_frame(raw_frame);
  {
    const auto t_sync_start = Clock::now();
    sync_gnss_epochs_from_shared_state();
    timing.gnss_mailbox_sync_ms = elapsed_ms(t_sync_start, Clock::now());
  }

  const int current = frames.size();
  const double scan_duration = std::max(1e-3, raw_frame->scan_end_time - raw_frame->stamp);

  EstimationFrame::Ptr new_frame(new EstimationFrame);
  new_frame->id = current;
  new_frame->stamp = raw_frame->stamp;
  new_frame->T_lidar_imu = T_lidar_imu;
  new_frame->raw_frame = raw_frame;
  new_frame->frame_id = FrameID::LIDAR;
  new_frame->v_world_imu.setZero();
  new_frame->imu_bias.head<3>() = fixed_lag_registry_.shared_state().accel_bias;
  new_frame->imu_bias.tail<3>() = fixed_lag_registry_.shared_state().gyro_bias;

  if (frames.empty()) {
    EstimationFrame::ConstPtr init_state;
    if (init_estimation) {
      init_estimation->insert_frame(raw_frame);
      init_state = init_estimation->initial_pose();
    }

    if (!init_state && init_estimation) {
      logger->debug("waiting for initial IMU state estimation to be finished (bspline unified graph)");
      return nullptr;
    }

    const gtsam::Pose3 initial_pose = init_state
      ? gtsam::Pose3(init_state->T_world_lidar.matrix())
      : gtsam::Pose3();

    initialize_control_window(raw_frame, initial_pose);

    new_frame->T_world_lidar = Eigen::Isometry3d(initial_pose.matrix());
    new_frame->T_world_imu = new_frame->T_world_lidar * T_lidar_imu;
    new_frame->frame = create_lidar_source_cloud(raw_frame);
    new_frame->imu_bias = init_state ? init_state->imu_bias : params->imu_bias;
    new_frame->v_world_imu = init_state ? init_state->v_world_imu : Eigen::Vector3d::Zero();
    fixed_lag_registry_.set_shared_imu_state(
      new_frame->imu_bias.tail<3>(),
      new_frame->imu_bias.head<3>(),
      fixed_lag_registry_.shared_state().gravity);
    fixed_lag_registry_.clear_auxiliary_values();
    fixed_lag_registry_.auxiliary_values().insert(iap::bspline_velocity_key(control_window_->states()[1].index), new_frame->v_world_imu);

    Callbacks::on_new_frame(new_frame);
    insert_target_cloud(new_frame);
    update_frame_history(new_frame, marginalized_frames);
    update_marginal_prior_from_active_window();
    publish_continuous_trajectory(current);
    publish_fixed_lag_telemetry(current);

    std::vector<EstimationFrame::ConstPtr> active_frames(frames.inner_begin(), frames.inner_end());
    if (!active_frames.empty()) {
      Callbacks::on_update_new_frame(active_frames.back());
      Callbacks::on_update_frames(active_frames);
    }

    if (init_estimation) {
      init_estimation.reset();
    }
    return new_frame;
  }

  {
    const auto t_source_start = Clock::now();
    new_frame->frame = create_lidar_source_cloud(raw_frame);
    timing.source_cloud_ms = elapsed_ms(t_source_start, Clock::now());
  }

  const gtsam::Pose3 predicted_end_pose = predict_scan_end_pose(scan_duration);
  control_window_->advance(raw_frame->stamp, raw_frame->scan_end_time, predicted_end_pose);
  fixed_lag_registry_.append_window(*control_window_);
  refresh_active_window_layout();

  if (!frontend_only_mode_) {
    if (const auto anchor = iap::IapSharedState::instance().get_gnss_anchor()) {
      fixed_lag_registry_.set_shared_gnss_anchor(anchor->origin_ecef, gtsam::Rot3(anchor->R_ecef_world));
    }
  }

  const double min_active_stamp = std::max(0.0, raw_frame->stamp - params->smoother_lag);
  const auto factor_source = gtsam_points::PointCloudCPU::clone(*new_frame->frame);
  {
    const auto t_segment_prepare_start = Clock::now();
    append_active_segment_constraint(raw_frame, factor_source);
    timing.segment_prepare_ms = elapsed_ms(t_segment_prepare_start, Clock::now());
    if (!fixed_lag_registry_.segments().empty()) {
      timing.target_build_ms = fixed_lag_registry_.segments().back().target_build_ms;
    }
  }
  if (!fixed_lag_registry_.segments().empty() && !frontend_only_mode_) {
    const auto t_gnss_epoch_start = Clock::now();
    fixed_lag_registry_.segments().back().gnss_epochs = consume_segment_gnss_epochs(
      raw_frame->stamp,
      raw_frame->scan_end_time);
    timing.gnss_epoch_fetch_ms = elapsed_ms(t_gnss_epoch_start, Clock::now());
  }

  gtsam::Values values = fixed_lag_registry_.control_buffer().values();
  const iap::BSplineCarriedPrior previous_carried_prior = marginal_prior_.carried_prior;
  const bool navigation_layer_enabled = !frontend_only_mode_ && fixed_lag_registry_.shared_state().gnss_anchor_initialized;
  fixed_lag_registry_.seed_shared_values(values, navigation_layer_enabled);

  for (const auto& segment : fixed_lag_registry_.segments()) {
    const gtsam::Key velocity_key = iap::bspline_velocity_key(segment.auxiliary_index);
    if (values.exists(velocity_key)) {
      continue;
    }

    if (fixed_lag_registry_.auxiliary_values().exists(velocity_key)) {
      values.insert(velocity_key, fixed_lag_registry_.auxiliary_values().at<gtsam::Vector3>(velocity_key));
      continue;
    }

    std::array<gtsam::Pose3, iap::kBSplineControlPointCount> segment_poses{};
    for (std::size_t k = 0; k < iap::kBSplineControlPointCount; ++k) {
      segment_poses[k] = values.at<gtsam::Pose3>(iap::bspline_control_point_key(segment.control_indices[k]));
    }
    const gtsam::Vector3 velocity_guess =
      iap::IntegratedBSplineVelocityFactor::predict_velocity(
        segment_poses,
        0.0,
        std::max(1e-3, segment.scan_end - segment.stamp),
        trajectory_params_.finite_difference_dt);
    values.insert(velocity_key, velocity_guess);
  }

  auto factor_layout = active_window_layout_ ? active_window_layout_ : build_active_window_layout();
  if (!factor_layout) {
    logger->error("bspline unified graph failed to build active-window layout");
    return nullptr;
  }

  auto local_input = make_local_layer_input(values, factor_layout);
  local_input.graph_context.min_active_stamp = min_active_stamp;
  local_input.graph_context.frontend_only_mode = frontend_only_mode_;
  local_input.graph_context.local_layer_enabled = true;
  local_input.graph_context.navigation_layer_enabled = navigation_layer_enabled;

  auto navigation_input = make_navigation_layer_input(factor_layout, navigation_layer_enabled);
  navigation_input.graph_context.min_active_stamp = min_active_stamp;
  navigation_input.graph_context.frontend_only_mode = frontend_only_mode_;
  navigation_input.graph_context.local_layer_enabled = true;

  const auto t_graph_build_start = Clock::now();
  const auto local_contribution = ct_local_frontend_.assemble_local_layer(local_input);
  auto navigation_contribution = ct_compact_backend_.assemble_navigation_layer(navigation_input, &values);

  const bool include_clock_states = navigation_contribution.activation.include_clock_states;
  const iap::SplineActiveStateSet active_state_set =
    fixed_lag_registry_.active_state_set(values, min_active_stamp, include_clock_states);
  const iap::BSplineMarginalizationPartition marginalization_partition =
    iap::build_bspline_marginalization_partition(active_state_set);

  gtsam::NonlinearFactorGraph graph;
  gtsam::NonlinearFactorGraph marginalization_graph;
  auto append_factor_with_partition = [&](const gtsam::NonlinearFactor::shared_ptr& factor) {
    if (!factor) {
      return;
    }
    graph.push_back(factor);
    if (marginalization_partition.should_marginalize_factor(factor->keys())) {
      marginalization_graph.push_back(factor);
    }
  };
  for (const auto& factor : local_contribution.graph) {
    append_factor_with_partition(factor);
  }
  for (const auto& factor : navigation_contribution.graph) {
    append_factor_with_partition(factor);
  }
  timing.graph_build_ms += elapsed_ms(t_graph_build_start, Clock::now());

  const auto& active_states = fixed_lag_registry_.control_buffer().states();
  const gtsam::Key gyro_bias_key = bspline_gyro_bias_key();
  const gtsam::Key accel_bias_key = bspline_accel_bias_key();
  const gtsam::Key gravity_key = bspline_gravity_key();
  const gtsam::Key ecef_origin_key = iap::bspline_ecef_origin_key();
  const gtsam::Key ecef_rot_key = iap::bspline_ecef_rot_key();

  const auto anchor_noise = gtsam::noiseModel::Isotropic::Precision(6, ctrl_point_anchor_inf_scale_);
  const auto pred_noise = gtsam::noiseModel::Isotropic::Precision(6, ctrl_point_prediction_inf_scale_);
  const auto smooth_noise = gtsam::noiseModel::Isotropic::Precision(6, ctrl_point_smoothness_inf_scale_);
  const auto marginal_noise = gtsam::noiseModel::Isotropic::Precision(6, ctrl_point_marginal_inf_scale_);
  const auto velocity_prior_noise = gtsam::noiseModel::Isotropic::Precision(3, velocity_ct_inf_scale_);
  const auto imu_bias_noise = gtsam::noiseModel::Isotropic::Precision(3, imu_ct_bias_inf_scale_);
  const auto gravity_noise = gtsam::noiseModel::Isotropic::Precision(3, imu_ct_gravity_inf_scale_);
  const auto gnss_ecef_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3::Constant(gnss_sigma_ecef_origin_));
  const auto gnss_ecef_rot_noise = gtsam::noiseModel::Isotropic::Sigma(3, gnss_sigma_ecef_rot_);
  const auto clock_prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector2() << params->clk_bias_noise, params->clk_drift_noise).finished());
  const auto& shared_state = fixed_lag_registry_.shared_state();

  const bool use_marginal_prior =
    marginal_prior_.valid &&
    active_states.size() >= 2 &&
    active_states[0].index == marginal_prior_.control_indices[0] &&
    active_states[1].index == marginal_prior_.control_indices[1];
  const bool use_information_marginal_prior =
    !marginal_prior_.carried_prior.empty() &&
    marginalization_partition.can_replay_keys(marginal_prior_.carried_prior.retained_keys, values);

  bool information_prior_attached = false;
  std::size_t carried_prior_factor_count = 0;
  const auto t_carried_prior_attach_start = Clock::now();
  if (use_information_marginal_prior) {
    try {
      const auto replayed_prior = marginal_prior_.carried_prior.replay();
      carried_prior_factor_count = replayed_prior.size();
      for (const auto& factor : replayed_prior) {
        if (!factor) {
          continue;
        }
        graph.add(factor->clone());
      }
      information_prior_attached = true;
    } catch (const std::exception& e) {
      logger->warn("failed to attach unified bspline marginal information prior, fallback to handcrafted prior: {}", e.what());
      marginal_prior_.carried_prior = iap::BSplineCarriedPrior();
    }
  }

  if (!information_prior_attached && use_marginal_prior) {
    auto pose_prior = std::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      iap::bspline_control_point_key(active_states[0].index),
      marginal_prior_.first_pose,
      marginal_noise);
    auto delta_prior = std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      iap::bspline_control_point_key(active_states[0].index),
      iap::bspline_control_point_key(active_states[1].index),
      marginal_prior_.relative_delta,
      marginal_noise);
    graph.add(pose_prior);
    graph.add(delta_prior);
    marginalization_graph.add(pose_prior);
    marginalization_graph.add(delta_prior);
    carried_prior_factor_count += 2;
    if (marginal_prior_.has_velocity && values.exists(iap::bspline_velocity_key(marginal_prior_.auxiliary_index))) {
      auto velocity_prior = std::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        iap::bspline_velocity_key(marginal_prior_.auxiliary_index),
        marginal_prior_.velocity,
        velocity_prior_noise);
      graph.add(velocity_prior);
      marginalization_graph.add(velocity_prior);
      ++carried_prior_factor_count;
    }
  } else if (!active_states.empty()) {
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      iap::bspline_control_point_key(active_states.front().index),
      active_states.front().pose,
      anchor_noise);
    carried_prior_factor_count++;
    if (active_states.size() >= 2) {
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        iap::bspline_control_point_key(active_states[1].index),
        active_states[1].pose,
        anchor_noise);
      carried_prior_factor_count++;
    }
  }
  timing.carried_prior_attach_ms = elapsed_ms(t_carried_prior_attach_start, Clock::now());

  const auto t_graph_prediction_prior_start = Clock::now();
  if (active_states.size() >= 2) {
    const auto& pred_a = active_states[active_states.size() - 2];
    const auto& pred_b = active_states.back();
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      iap::bspline_control_point_key(pred_a.index),
      pred_a.pose,
      pred_noise);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      iap::bspline_control_point_key(pred_b.index),
      pred_b.pose,
      pred_noise);
  }
  timing.graph_prediction_prior_ms = elapsed_ms(t_graph_prediction_prior_start, Clock::now());

  const auto t_graph_smoothness_start = Clock::now();
  for (std::size_t i = 0; i + 1 < active_states.size(); ++i) {
    const gtsam::Key key_i = iap::bspline_control_point_key(active_states[i].index);
    const gtsam::Key key_j = iap::bspline_control_point_key(active_states[i + 1].index);
    auto smooth_factor = std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      key_i,
      key_j,
      active_states[i].pose.between(active_states[i + 1].pose),
      smooth_noise);
    graph.add(smooth_factor);
    if (marginalization_partition.should_marginalize_factor(gtsam::KeyVector{key_i, key_j})) {
      marginalization_graph.add(smooth_factor);
    }
  }
  timing.graph_smoothness_ms = elapsed_ms(t_graph_smoothness_start, Clock::now());

  const auto t_graph_shared_prior_start = Clock::now();
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gyro_bias_key, shared_state.gyro_bias, imu_bias_noise);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(accel_bias_key, shared_state.accel_bias, imu_bias_noise);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gravity_key, shared_state.gravity, gravity_noise);
  if (navigation_layer_enabled && shared_state.gnss_anchor_initialized && values.exists(ecef_origin_key) && values.exists(ecef_rot_key)) {
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(ecef_origin_key, shared_state.ecef_origin, gnss_ecef_noise);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Rot3>>(ecef_rot_key, shared_state.ecef_rot, gnss_ecef_rot_noise);
  }
  timing.graph_shared_prior_ms = elapsed_ms(t_graph_shared_prior_start, Clock::now());

  std::vector<std::pair<gtsam::Key, double>> active_clock_states;
  for (const auto& segment : fixed_lag_registry_.segments()) {
    if (!navigation_layer_enabled || segment.gnss_epochs.empty()) {
      continue;
    }
    const gtsam::Key clock_key = iap::bspline_clock_key(segment.auxiliary_index);
    if (values.exists(clock_key)) {
      active_clock_states.emplace_back(clock_key, segment.stamp);
    }
  }

  const auto t_graph_clock_factor_start = Clock::now();
  if (!active_clock_states.empty()) {
    const bool clock_constrained_by_information_prior =
      information_prior_attached &&
      std::find(
        marginal_prior_.carried_prior.retained_keys.begin(),
        marginal_prior_.carried_prior.retained_keys.end(),
        active_clock_states.front().first) != marginal_prior_.carried_prior.retained_keys.end();
    if (!clock_constrained_by_information_prior) {
      const bool use_clock_boundary_prior =
        use_marginal_prior &&
        marginal_prior_.has_clock &&
        active_clock_states.front().first == iap::bspline_clock_key(marginal_prior_.auxiliary_index);
      const gtsam::Vector2 boundary_clock =
        use_clock_boundary_prior ? marginal_prior_.clock : values.at<gtsam::Vector2>(active_clock_states.front().first);
      auto clock_prior = std::make_shared<gtsam::PriorFactor<gtsam::Vector2>>(
        active_clock_states.front().first,
        boundary_clock,
        clock_prior_noise);
      graph.add(clock_prior);
      if (use_clock_boundary_prior) {
        marginalization_graph.add(clock_prior);
      }
    }
    for (std::size_t i = 1; i < active_clock_states.size(); ++i) {
      const double dt = std::max(1e-3, active_clock_states[i].second - active_clock_states[i - 1].second);
      auto clock_between = std::make_shared<iap::ClockBetweenFactor>(
        active_clock_states[i - 1].first,
        active_clock_states[i].first,
        dt,
        iap::ClockBetweenFactor::make_noise(dt, gnss_clock_between_params_));
      graph.add(clock_between);
      if (marginalization_partition.should_marginalize_factor(
            gtsam::KeyVector{active_clock_states[i - 1].first, active_clock_states[i].first})) {
        marginalization_graph.add(clock_between);
      }
    }
  }
  timing.graph_clock_factor_ms = elapsed_ms(t_graph_clock_factor_start, Clock::now());

  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setlambdaInitial(1e-4);
  lm_params.setAbsoluteErrorTol(1e-2);
  lm_params.setMaxIterations(lm_max_iterations_);
  const double lm_initial_cost = graph.empty() ? 0.0 : graph.error(values);
  std::string batch_solver_status = graph.empty() ? "no_factors" : "ok";

  const auto t_optimize_start = Clock::now();
  try {
    values = gtsam_points::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();
  } catch (const std::exception& e) {
    logger->error("bspline unified graph optimization failed: {}", e.what());
    batch_solver_status = "exception";
  }
  timing.lm_optimize_ms = elapsed_ms(t_optimize_start, Clock::now());
  const double lm_final_cost = graph.empty() ? 0.0 : graph.error(values);

  fixed_lag_registry_.control_buffer().update_from_values(values);
  control_window_->update_from_values(values);
  fixed_lag_registry_.update_shared_state_from_values(values);
  fixed_lag_registry_.clear_auxiliary_values();
  for (const auto& segment : fixed_lag_registry_.segments()) {
    const gtsam::Key velocity_key = iap::bspline_velocity_key(segment.auxiliary_index);
    if (values.exists(velocity_key) && !fixed_lag_registry_.auxiliary_values().exists(velocity_key)) {
      fixed_lag_registry_.auxiliary_values().insert(velocity_key, values.at<gtsam::Vector3>(velocity_key));
    }
    const gtsam::Key clock_key = iap::bspline_clock_key(segment.auxiliary_index);
    if (values.exists(clock_key) && !fixed_lag_registry_.auxiliary_values().exists(clock_key)) {
      fixed_lag_registry_.auxiliary_values().insert(clock_key, values.at<gtsam::Vector2>(clock_key));
    }
  }

  {
    const auto t_prune_start = Clock::now();
    prune_active_ct_state(min_active_stamp, active_state_set);
    refresh_active_window_layout();
    timing.prune_active_ms = elapsed_ms(t_prune_start, Clock::now());
  }
  {
    const auto t_prior_update_start = Clock::now();
    update_marginal_prior_from_active_window();
    update_marginal_prior_information(
      marginalization_graph,
      values,
      active_state_set.active_keys(),
      previous_carried_prior.empty() ? nullptr : &previous_carried_prior);
    timing.carried_prior_update_ms = elapsed_ms(t_prior_update_start, Clock::now());
  }
  timing.marginalization_ms = timing.prune_active_ms + timing.carried_prior_update_ms;

  const auto t_postprocess_start = Clock::now();
  auto current_handle_it = local_contribution.lidar_factor_handles.empty()
    ? local_contribution.lidar_factor_handles.end()
    : std::prev(local_contribution.lidar_factor_handles.end());
  if (current_handle_it == local_contribution.lidar_factor_handles.end()) {
    logger->error("bspline unified graph failed to build current lidar factor");
    return nullptr;
  }

  std::vector<iap::BSplineLidarFactorResult> lidar_results;
  std::vector<iap::FrontendBucketProfileRow> bucket_profiles;
  std::vector<iap::LidarFactorInternalProfileRow> lidar_internal_rows;
  int current_lidar_result_index = -1;
  iap::BSplineLidarFactorResult current_lidar_result;
  const bool collect_window_lidar_results = lidar_collect_window_results();

  auto append_lidar_result = [&](const iap::BSplineLocalLayerContribution::LidarFactorHandle& handle, bool current_factor) {
    const double factor_error = handle.factor->error(values);
    auto factor_result = handle.factor->make_result(
      factor_error,
      handle.factor->num_inliers(),
      handle.factor->inlier_fraction());
    if (current_factor) {
      current_lidar_result = factor_result;
      current_lidar_result_index = static_cast<int>(lidar_results.size());
    }
    lidar_results.push_back(factor_result);

    iap::FrontendBucketProfileRow bucket_profile;
    bucket_profile.source_frame_index = handle.source_frame_index;
    bucket_profile.bucket_index = handle.bucket_index;
    bucket_profile.bucket_mode = iap::CTLocalFrontend::bucket_mode_name(lidar_bucket_config_.mode);
    bucket_profile.representative_time = handle.representative_time;
    bucket_profile.points_in_bucket = handle.bucket_ctx.point_indices.size();
    bucket_profile.valid_correspondence_count = factor_result.profile.matched_point_count;
    bucket_profile.match_ratio = factor_result.profile.match_ratio;
    bucket_profile.inlier_ratio = factor_result.profile.inlier_ratio;
    bucket_profile.target_point_count = factor_result.profile.target_point_count;
    bucket_profile.candidate_evaluation_count = factor_result.profile.candidate_evaluation_count;
    bucket_profile.lookup_or_correspondence_ms = factor_result.profile.correspondence_ms;
    bucket_profile.accumulation_ms = factor_result.profile.accumulation_ms;
    bucket_profile.factor_total_ms = factor_result.profile.total_ms;
    bucket_profile.time_bucket_count = factor_result.profile.time_bucket_count;
    bucket_profile.mean_time_bucket_population = factor_result.profile.mean_time_bucket_population;
    bucket_profile.max_time_bucket_population = factor_result.profile.max_time_bucket_population;
    bucket_profiles.push_back(std::move(bucket_profile));
    lidar_internal_rows.push_back(make_lidar_factor_internal_profile_row(
      new_frame->id,
      raw_frame->stamp,
      iap::CTLocalFrontend::bucket_mode_name(lidar_bucket_config_.mode),
      local_contribution.lidar_factor_handles.size(),
      lidar_internal_rows.size(),
      handle,
      factor_result,
      active_state_set.active_control_indices.size()));
  };

  if (collect_window_lidar_results) {
    for (const auto& handle : local_contribution.lidar_factor_handles) {
      append_lidar_result(handle, &handle == &(*current_handle_it));
    }
  } else {
    append_lidar_result(*current_handle_it, true);
  }

  const auto lidar_window_summary = iap::aggregate_bspline_lidar_factor_results(lidar_results);
  maybe_export_lidar_baseline_csv(raw_frame->stamp, lidar_results, current_lidar_result_index);

  auto evaluate_lidar_pose = [&](double stamp, double legacy_u) {
    if (active_window_layout_ && active_window_evaluator_) {
      if (const auto support = active_window_layout_->support_at(stamp, iap::SplineSensorId::Lidar)) {
        return active_window_evaluator_->eval_pose(values, *support, iap::SplineSensorId::Lidar);
      }
    }
    return control_window_->evaluate(legacy_u);
  };
  const gtsam::Pose3 start_pose = evaluate_lidar_pose(raw_frame->stamp, 0.0);
  new_frame->T_world_lidar = Eigen::Isometry3d(start_pose.matrix());
  new_frame->T_world_imu = new_frame->T_world_lidar * T_lidar_imu;
  new_frame->v_world_imu = values.at<gtsam::Vector3>(iap::bspline_velocity_key(control_window_->states()[1].index));
  new_frame->imu_bias.head<3>() = fixed_lag_registry_.shared_state().accel_bias;
  new_frame->imu_bias.tail<3>() = fixed_lag_registry_.shared_state().gyro_bias;
  const gtsam::Key current_clock_key = iap::bspline_clock_key(control_window_->states()[1].index);
  if (values.exists(current_clock_key)) {
    const auto clock = values.at<gtsam::Vector2>(current_clock_key);
    new_frame->clk_bias = clock(0);
    new_frame->clk_drift = clock(1);
  } else if (!frames.empty() && frames.back()) {
    new_frame->clk_bias = frames.back()->clk_bias;
    new_frame->clk_drift = frames.back()->clk_drift;
  }
  if (current_lidar_result.valid) {
    new_frame->icp_quality.inlier_count = current_lidar_result.inlier_count;
    new_frame->icp_quality.inlier_fraction = current_lidar_result.inlier_fraction;
    new_frame->icp_quality.rmse =
      std::sqrt(current_lidar_result.factor_error / std::max(new_frame->icp_quality.inlier_count, 1));
  }

  auto deskewed_points = current_handle_it->factor->deskewed_source_points(values, true);
  const auto deskewed_covs = covariance_estimation->estimate(deskewed_points, raw_frame->neighbors);
  for (int i = 0; i < new_frame->frame->size(); ++i) {
    new_frame->frame->points[i] = deskewed_points[static_cast<std::size_t>(i)];
    new_frame->frame->covs[i] = deskewed_covs[static_cast<std::size_t>(i)];
  }

  {
    const auto t_callback_start = Clock::now();
    Callbacks::on_new_frame(new_frame);
    timing.post_callback_ms += elapsed_ms(t_callback_start, Clock::now());
  }
  {
    const auto t_target_insert_start = Clock::now();
    insert_target_cloud(new_frame);
    timing.post_target_insert_ms = elapsed_ms(t_target_insert_start, Clock::now());
  }
  {
    const auto t_history_start = Clock::now();
    update_frame_history(new_frame, marginalized_frames);
    timing.post_history_update_ms = elapsed_ms(t_history_start, Clock::now());
  }
  {
    const auto t_publish_start = Clock::now();
    publish_continuous_trajectory(current);
    timing.post_publish_traj_ms = elapsed_ms(t_publish_start, Clock::now());
  }
  {
    const auto t_telemetry_start = Clock::now();
    publish_fixed_lag_telemetry(current);
    timing.post_publish_telemetry_ms = elapsed_ms(t_telemetry_start, Clock::now());
  }

  std::vector<EstimationFrame::ConstPtr> active_frames(frames.inner_begin(), frames.inner_end());
  if (!active_frames.empty()) {
    const auto t_callback_start = Clock::now();
    Callbacks::on_update_new_frame(active_frames.back());
    Callbacks::on_update_frames(active_frames);
    timing.post_callback_ms += elapsed_ms(t_callback_start, Clock::now());
  }

  timing.postprocess_ms = elapsed_ms(t_postprocess_start, Clock::now());
  timing.window_wall_ms = elapsed_ms(t_window_start, Clock::now());

  iap::FrontendFrameProfile frontend_frame_profile = local_contribution.processed.frame_profile;
  frontend_frame_profile.frame_id = new_frame->id;
  frontend_frame_profile.stamp = raw_frame->stamp;
  frontend_frame_profile.frontend_mode = frontend_mode_;
  frontend_frame_profile.frontend_only_mode = frontend_only_mode_;
  frontend_frame_profile.use_legacy_two_stage_path = false;
  frontend_frame_profile.preprocess_ms = timing.source_cloud_ms;
  frontend_frame_profile.target_map_prep_ms = timing.target_build_ms;
  frontend_frame_profile.lm_solve_ms = timing.lm_optimize_ms;
  frontend_frame_profile.marginalization_ms = timing.marginalization_ms;
  frontend_frame_profile.backend_update_ms = 0.0;
  frontend_frame_profile.backend_optimize_ms = 0.0;
  frontend_frame_profile.publish_ms = timing.post_publish_traj_ms + timing.post_publish_telemetry_ms + timing.post_callback_ms;
  frontend_frame_profile.local_mapping_update_ms = 0.0;
  frontend_frame_profile.global_mapping_update_ms = 0.0;
  frontend_frame_profile.submap_registration_ms = 0.0;
  frontend_frame_profile.active_control_point_count = active_state_set.active_control_indices.size();
  frontend_frame_profile.active_pose_key_count = active_state_set.active_pose_keys.size();
  frontend_frame_profile.optimize_count = 1;
  frontend_frame_profile.local_layer_enabled = local_contribution.activation.enabled;
  frontend_frame_profile.navigation_layer_enabled = navigation_contribution.activation.enabled;
  frontend_frame_profile.local_layer_factor_count = local_contribution.factor_count();
  frontend_frame_profile.navigation_layer_factor_count = navigation_contribution.factor_count();
  frontend_frame_profile.local_layer_active_state_count = local_contribution.activation.active_state_count();
  frontend_frame_profile.navigation_layer_active_state_count = navigation_contribution.activation.active_state_count();
  frontend_frame_profile.solver_mode = iap::to_string(iap::BSplineUnifiedSolverMode::BATCH_LM);
  frontend_frame_profile.new_factor_count = graph.size();
  frontend_frame_profile.new_value_count = values.size();
  frontend_frame_profile.retired_key_count = 0;
  frontend_frame_profile.fallback_used = (batch_solver_status != "ok" && batch_solver_status != "no_factors");
  frontend_frame_profile.carried_prior_replay_success = information_prior_attached;
  frontend_frame_profile.imu_factor_count = local_contribution.imu_factor_count;
  frontend_frame_profile.imu_residual_count = local_contribution.debug_stats.imu_residual_count;
  frontend_frame_profile.lidar_factor_count = local_contribution.lidar_factor_count;
  frontend_frame_profile.lidar_residual_count = local_contribution.debug_stats.lidar_residual_count;
  frontend_frame_profile.gnss_factor_count =
    navigation_contribution.gnss_pr_factor_count + navigation_contribution.gnss_dop_factor_count;
  frontend_frame_profile.carried_prior_count = carried_prior_factor_count;
  frontend_frame_profile.backend_factor_count = navigation_contribution.factor_count();
  frontend_frame_profile.backend_state_count = navigation_contribution.activation.active_state_count();
  frontend_frame_profile.local_residual_count =
    frontend_frame_profile.lidar_residual_count +
    frontend_frame_profile.imu_residual_count +
    frontend_frame_profile.gnss_factor_count +
    frontend_frame_profile.carried_prior_count;
  frontend_frame_profile.lm_initial_cost = lm_initial_cost;
  frontend_frame_profile.lm_final_cost = lm_final_cost;

  iap::SolverUpdateProfileRow solver_update_row;
  solver_update_row.frame_id = frontend_frame_profile.frame_id;
  solver_update_row.frame_stamp = frontend_frame_profile.stamp;
  solver_update_row.solver_mode = frontend_frame_profile.solver_mode;
  solver_update_row.frontend_only_mode = frontend_frame_profile.frontend_only_mode;
  solver_update_row.local_layer_enabled = frontend_frame_profile.local_layer_enabled;
  solver_update_row.navigation_layer_enabled = frontend_frame_profile.navigation_layer_enabled;
  solver_update_row.used_incremental_solver = false;
  solver_update_row.fallback_used = frontend_frame_profile.fallback_used;
  solver_update_row.new_factor_count = graph.size();
  solver_update_row.new_value_count = values.size();
  solver_update_row.new_stamp_count = 0;
  solver_update_row.query_key_count = 0;
  solver_update_row.retired_key_count = 0;
  solver_update_row.active_control_point_count = frontend_frame_profile.active_control_point_count;
  solver_update_row.active_pose_key_count = frontend_frame_profile.active_pose_key_count;
  solver_update_row.active_aux_key_count = active_state_set.active_aux_keys.size();
  solver_update_row.persistent_key_count = fixed_lag_registry_.active_shared_keys(navigation_layer_enabled).size();
  solver_update_row.local_state_dimension = frontend_frame_profile.local_state_dimension;
  solver_update_row.local_residual_count = frontend_frame_profile.local_residual_count;
  solver_update_row.solver_update_ms = timing.lm_optimize_ms;
  solver_update_row.estimate_query_ms = 0.0;
  solver_update_row.fallback_rebuild_ms = 0.0;
  solver_update_row.relinearization_ms = 0.0;
  solver_update_row.linearization_ms = 0.0;
  solver_update_row.elimination_ms = 0.0;
  solver_update_row.delta_solve_ms = timing.lm_optimize_ms;
  solver_update_row.relinearized_variable_count = active_state_set.active_keys().size();
  solver_update_row.reeliminated_variable_count = 0;
  solver_update_row.relinearized_factor_count = graph.size();
  solver_update_row.linearized_factor_count = graph.size();
  solver_update_row.bayes_tree_clique_count = 0;
  solver_update_row.affected_variable_count = active_state_set.active_keys().size();
  solver_update_row.observed_key_count = 0;
  solver_update_row.new_factor_index_count = 0;
  solver_update_row.current_nonlinear_factor_count = graph.size();
  solver_update_row.isam_reported_update_ms = 0.0;
  solver_update_row.optimize_count = 1;
  solver_update_row.initial_error = lm_initial_cost;
  solver_update_row.final_error = lm_final_cost;
  solver_update_row.error_drop_ratio = error_drop_ratio(lm_initial_cost, lm_final_cost);
  solver_update_row.iteration_count = frontend_frame_profile.lm_iteration_count;
  solver_update_row.solver_status = batch_solver_status;

  maybe_write_frontend_frame_profile(frontend_frame_profile);
  maybe_write_solver_update_profile(solver_update_row);
  maybe_write_lidar_factor_profiles(frontend_frame_profile.frame_id, frontend_frame_profile.stamp, bucket_profiles);
  maybe_write_lidar_factor_internal_profiles(lidar_internal_rows);
  if (frontend_only_mode_) {
    log_frontend_only_stats(frontend_frame_profile);
  }

  return new_frame;
}

EstimationFrame::ConstPtr OdometryEstimationBSpline::insert_frame_ct_lidar_incremental_graph(
  const PreprocessedFrame::Ptr& raw_frame,
  std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  using Clock = std::chrono::steady_clock;
  const auto t_window_start = Clock::now();
  const auto elapsed_ms = [](const Clock::time_point& start, const Clock::time_point& end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
  };

  struct IncrementalTiming {
    double gnss_mailbox_sync_ms{0.0};
    double source_cloud_ms{0.0};
    double segment_prepare_ms{0.0};
    double target_build_ms{0.0};
    double gnss_epoch_fetch_ms{0.0};
    double solver_update_ms{0.0};
    double prune_active_ms{0.0};
    double marginalization_ms{0.0};
    double postprocess_ms{0.0};
    double post_publish_traj_ms{0.0};
    double post_publish_telemetry_ms{0.0};
    double post_callback_ms{0.0};
    double post_target_insert_ms{0.0};
    double post_history_update_ms{0.0};
    double window_wall_ms{0.0};
  } timing;

  Callbacks::on_insert_frame(raw_frame);
  {
    const auto t_sync_start = Clock::now();
    sync_gnss_epochs_from_shared_state();
    timing.gnss_mailbox_sync_ms = elapsed_ms(t_sync_start, Clock::now());
  }

  const int current = frames.size();
  const double scan_duration = std::max(1e-3, raw_frame->scan_end_time - raw_frame->stamp);

  EstimationFrame::Ptr new_frame(new EstimationFrame);
  new_frame->id = current;
  new_frame->stamp = raw_frame->stamp;
  new_frame->T_lidar_imu = T_lidar_imu;
  new_frame->raw_frame = raw_frame;
  new_frame->frame_id = FrameID::LIDAR;
  new_frame->v_world_imu.setZero();
  new_frame->imu_bias.head<3>() = fixed_lag_registry_.shared_state().accel_bias;
  new_frame->imu_bias.tail<3>() = fixed_lag_registry_.shared_state().gyro_bias;

  if (frames.empty()) {
    EstimationFrame::ConstPtr init_state;
    if (init_estimation) {
      init_estimation->insert_frame(raw_frame);
      init_state = init_estimation->initial_pose();
    }

    if (!init_state && init_estimation) {
      logger->debug("waiting for initial IMU state estimation to be finished (bspline incremental graph)");
      return nullptr;
    }

    const gtsam::Pose3 initial_pose = init_state
      ? gtsam::Pose3(init_state->T_world_lidar.matrix())
      : gtsam::Pose3();

    initialize_control_window(raw_frame, initial_pose);

    new_frame->T_world_lidar = Eigen::Isometry3d(initial_pose.matrix());
    new_frame->T_world_imu = new_frame->T_world_lidar * T_lidar_imu;
    new_frame->frame = create_lidar_source_cloud(raw_frame);
    new_frame->imu_bias = init_state ? init_state->imu_bias : params->imu_bias;
    new_frame->v_world_imu = init_state ? init_state->v_world_imu : Eigen::Vector3d::Zero();
    fixed_lag_registry_.set_shared_imu_state(
      new_frame->imu_bias.tail<3>(),
      new_frame->imu_bias.head<3>(),
      fixed_lag_registry_.shared_state().gravity);
    fixed_lag_registry_.clear_auxiliary_values();
    fixed_lag_registry_.auxiliary_values().insert(
      iap::bspline_velocity_key(control_window_->states()[1].index),
      new_frame->v_world_imu);

    Callbacks::on_new_frame(new_frame);
    insert_target_cloud(new_frame);
    update_frame_history(new_frame, marginalized_frames);
    publish_continuous_trajectory(current);
    publish_fixed_lag_telemetry(current);

    std::vector<EstimationFrame::ConstPtr> active_frames(frames.inner_begin(), frames.inner_end());
    if (!active_frames.empty()) {
      Callbacks::on_update_new_frame(active_frames.back());
      Callbacks::on_update_frames(active_frames);
    }

    if (init_estimation) {
      init_estimation.reset();
    }
    return new_frame;
  }

  {
    const auto t_source_start = Clock::now();
    new_frame->frame = create_lidar_source_cloud(raw_frame);
    timing.source_cloud_ms = elapsed_ms(t_source_start, Clock::now());
  }

  const gtsam::Pose3 predicted_end_pose = predict_scan_end_pose(scan_duration);
  control_window_->advance(raw_frame->stamp, raw_frame->scan_end_time, predicted_end_pose);
  fixed_lag_registry_.append_window(*control_window_);
  refresh_active_window_layout();

  if (!frontend_only_mode_) {
    if (const auto anchor = iap::IapSharedState::instance().get_gnss_anchor()) {
      fixed_lag_registry_.set_shared_gnss_anchor(anchor->origin_ecef, gtsam::Rot3(anchor->R_ecef_world));
    }
  }

  const double min_active_stamp = std::max(0.0, raw_frame->stamp - params->smoother_lag);
  const auto factor_source = gtsam_points::PointCloudCPU::clone(*new_frame->frame);
  {
    const auto t_segment_prepare_start = Clock::now();
    append_active_segment_constraint(raw_frame, factor_source);
    timing.segment_prepare_ms = elapsed_ms(t_segment_prepare_start, Clock::now());
    if (!fixed_lag_registry_.segments().empty()) {
      timing.target_build_ms = fixed_lag_registry_.segments().back().target_build_ms;
    }
  }
  if (!fixed_lag_registry_.segments().empty() && !frontend_only_mode_) {
    const auto t_gnss_epoch_start = Clock::now();
    fixed_lag_registry_.segments().back().gnss_epochs = consume_segment_gnss_epochs(
      raw_frame->stamp,
      raw_frame->scan_end_time);
    timing.gnss_epoch_fetch_ms = elapsed_ms(t_gnss_epoch_start, Clock::now());
  }

  if (!unified_graph_solver_) {
    reset_unified_graph_solver();
  }

  auto factor_layout = active_window_layout_ ? active_window_layout_ : build_active_window_layout();
  if (!factor_layout) {
    logger->error("bspline incremental graph failed to build active-window layout");
    return nullptr;
  }

  auto& current_segment = fixed_lag_registry_.segments().back();
  const ActiveSplineSegmentConstraint* previous_segment =
    fixed_lag_registry_.segments().size() >= 2 ? &fixed_lag_registry_.segments()[fixed_lag_registry_.segments().size() - 2] : nullptr;
  const bool navigation_layer_enabled = !frontend_only_mode_;

  gtsam::Values mirror_values = fixed_lag_registry_.control_buffer().values();
  fixed_lag_registry_.seed_shared_values(
    mirror_values,
    navigation_layer_enabled && fixed_lag_registry_.shared_state().gnss_anchor_initialized);
  for (const auto key : fixed_lag_registry_.auxiliary_values().keys()) {
    if (!mirror_values.exists(key)) {
      mirror_values.insert(key, fixed_lag_registry_.auxiliary_values().at(key));
    }
  }

  const gtsam::Key current_velocity_key = iap::bspline_velocity_key(current_segment.auxiliary_index);
  if (!mirror_values.exists(current_velocity_key)) {
    if (fixed_lag_registry_.auxiliary_values().exists(current_velocity_key)) {
      mirror_values.insert(current_velocity_key, fixed_lag_registry_.auxiliary_values().at<gtsam::Vector3>(current_velocity_key));
    } else {
      std::array<gtsam::Pose3, iap::kBSplineControlPointCount> segment_poses{};
      for (std::size_t k = 0; k < iap::kBSplineControlPointCount; ++k) {
        segment_poses[k] = mirror_values.at<gtsam::Pose3>(iap::bspline_control_point_key(current_segment.control_indices[k]));
      }
      const gtsam::Vector3 velocity_guess =
        iap::IntegratedBSplineVelocityFactor::predict_velocity(
          segment_poses,
          0.0,
          std::max(1e-3, current_segment.scan_end - current_segment.stamp),
          trajectory_params_.finite_difference_dt);
      mirror_values.insert(current_velocity_key, velocity_guess);
    }
  }

  auto local_input = make_local_layer_delta_input(current_segment, factor_layout);
  local_input.graph_context.min_active_stamp = min_active_stamp;
  local_input.graph_context.existing_keys = unified_graph_solver_->current_active_keys();
  const auto local_contribution = ct_local_frontend_.assemble_local_layer(local_input);

  auto segment_support_control_indices = control_indices_from_activation(local_contribution.activation);
  if (segment_support_control_indices.empty()) {
    segment_support_control_indices = sort_unique_control_indices(std::vector<std::size_t>(
      current_segment.active_control_indices.begin(),
      current_segment.active_control_indices.end()));
  }
  if (segment_support_control_indices.empty()) {
    segment_support_control_indices.assign(current_segment.control_indices.begin(), current_segment.control_indices.end());
    segment_support_control_indices = sort_unique_control_indices(std::move(segment_support_control_indices));
  }
  current_segment.active_control_indices = segment_support_control_indices;

  const auto active_state_set =
    fixed_lag_registry_.active_state_set(mirror_values, min_active_stamp, navigation_layer_enabled);

  iap::BSplineGraphDelta delta;
  delta.layout = factor_layout;
  delta.current_frame_stamp = raw_frame->stamp;
  delta.min_active_stamp = min_active_stamp;
  delta.current_auxiliary_index = current_segment.auxiliary_index;
  delta.local_layer_enabled = local_contribution.activation.enabled;
  delta.navigation_layer_enabled = navigation_layer_enabled;
  delta.local_layer_factor_count = local_contribution.factor_count();
  delta.active_pose_keys = active_state_set.active_pose_keys;
  delta.active_aux_keys = active_state_set.active_aux_keys;
  delta.persistent_keys = sort_unique_keys(fixed_lag_registry_.active_shared_keys(
    navigation_layer_enabled && fixed_lag_registry_.shared_state().gnss_anchor_initialized));

  gtsam::KeyVector newly_announced_keys;
  const auto& active_states = fixed_lag_registry_.control_buffer().states();
  const auto& shared_state = fixed_lag_registry_.shared_state();
  const auto anchor_noise = gtsam::noiseModel::Isotropic::Precision(6, ctrl_point_anchor_inf_scale_);
  const auto pred_noise = gtsam::noiseModel::Isotropic::Precision(6, ctrl_point_prediction_inf_scale_);
  const auto smooth_noise = gtsam::noiseModel::Isotropic::Precision(6, ctrl_point_smoothness_inf_scale_);
  const auto velocity_prior_noise = gtsam::noiseModel::Isotropic::Precision(3, velocity_ct_inf_scale_);
  const auto imu_bias_noise = gtsam::noiseModel::Isotropic::Precision(3, imu_ct_bias_inf_scale_);
  const auto gravity_noise = gtsam::noiseModel::Isotropic::Precision(3, imu_ct_gravity_inf_scale_);
  const auto gnss_ecef_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3::Constant(gnss_sigma_ecef_origin_));
  const auto gnss_ecef_rot_noise = gtsam::noiseModel::Isotropic::Sigma(3, gnss_sigma_ecef_rot_);
  const auto clock_prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector2() << params->clk_bias_noise, params->clk_drift_noise).finished());

  const auto current_active_keys = unified_graph_solver_->current_active_keys();
  const auto key_known = [&](gtsam::Key key) {
    return std::find(current_active_keys.begin(), current_active_keys.end(), key) != current_active_keys.end() ||
           delta.new_values.exists(key);
  };

  std::vector<std::size_t> new_control_indices;
  new_control_indices.reserve(segment_support_control_indices.size());
  for (const auto control_index : segment_support_control_indices) {
    if (fixed_lag_registry_.control_index_announced(control_index)) {
      const auto pose_key = iap::bspline_control_point_key(control_index);
      delta.new_stamps[pose_key] = current_segment.scan_end;
      continue;
    }
    const gtsam::Key pose_key = iap::bspline_control_point_key(control_index);
    if (key_known(pose_key)) {
      delta.new_stamps[pose_key] = current_segment.scan_end;
      continue;
    }
    const auto state_it = std::find_if(active_states.begin(), active_states.end(), [&](const auto& state) {
      return state.index == control_index;
    });
    if (state_it == active_states.end()) {
      continue;
    }
    delta.new_values.insert(pose_key, state_it->pose);
    delta.new_stamps[pose_key] = current_segment.scan_end;
    append_unique_key(&newly_announced_keys, pose_key);
    new_control_indices.push_back(control_index);
  }

  if (!fixed_lag_registry_.auxiliary_index_announced(current_segment.auxiliary_index) && !key_known(current_velocity_key)) {
    delta.new_values.insert(current_velocity_key, mirror_values.at<gtsam::Vector3>(current_velocity_key));
    delta.new_stamps[current_velocity_key] = current_segment.scan_end;
    append_unique_key(&newly_announced_keys, current_velocity_key);
  }

  const gtsam::Key gyro_bias_key = bspline_gyro_bias_key();
  const gtsam::Key accel_bias_key = bspline_accel_bias_key();
  const gtsam::Key gravity_key = bspline_gravity_key();
  if (!fixed_lag_registry_.persistent_key_announced(gyro_bias_key) && !key_known(gyro_bias_key)) {
    delta.new_values.insert(gyro_bias_key, shared_state.gyro_bias);
    append_unique_key(&newly_announced_keys, gyro_bias_key);
  }
  if (!fixed_lag_registry_.persistent_key_announced(accel_bias_key) && !key_known(accel_bias_key)) {
    delta.new_values.insert(accel_bias_key, shared_state.accel_bias);
    append_unique_key(&newly_announced_keys, accel_bias_key);
  }
  if (!fixed_lag_registry_.persistent_key_announced(gravity_key) && !key_known(gravity_key)) {
    delta.new_values.insert(gravity_key, shared_state.gravity);
    append_unique_key(&newly_announced_keys, gravity_key);
  }

  if (delta.new_values.exists(gyro_bias_key)) {
    delta.new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gyro_bias_key, shared_state.gyro_bias, imu_bias_noise);
  }
  if (delta.new_values.exists(accel_bias_key)) {
    delta.new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(accel_bias_key, shared_state.accel_bias, imu_bias_noise);
  }
  if (delta.new_values.exists(gravity_key)) {
    delta.new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gravity_key, shared_state.gravity, gravity_noise);
  }

  const bool anchor_enabled = navigation_layer_enabled && shared_state.gnss_anchor_initialized;
  const gtsam::Key ecef_origin_key = iap::bspline_ecef_origin_key();
  const gtsam::Key ecef_rot_key = iap::bspline_ecef_rot_key();
  if (anchor_enabled && !fixed_lag_registry_.persistent_key_announced(ecef_origin_key) && !key_known(ecef_origin_key)) {
    delta.new_values.insert(ecef_origin_key, shared_state.ecef_origin);
    append_unique_key(&newly_announced_keys, ecef_origin_key);
    delta.new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(ecef_origin_key, shared_state.ecef_origin, gnss_ecef_noise);
  }
  if (anchor_enabled && !fixed_lag_registry_.persistent_key_announced(ecef_rot_key) && !key_known(ecef_rot_key)) {
    delta.new_values.insert(ecef_rot_key, shared_state.ecef_rot);
    append_unique_key(&newly_announced_keys, ecef_rot_key);
    delta.new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Rot3>>(ecef_rot_key, shared_state.ecef_rot, gnss_ecef_rot_noise);
  }

  // The incremental fixed-lag smoother may pull shared singleton states into
  // additional re-elimination cliques during timestamp-based retirement. Keep
  // their timestamps refreshed so retirement bookkeeping can reason about the
  // full connected component without trying to evict these persistent keys.
  delta.new_stamps[gyro_bias_key] = raw_frame->stamp;
  delta.new_stamps[accel_bias_key] = raw_frame->stamp;
  delta.new_stamps[gravity_key] = raw_frame->stamp;
  if (anchor_enabled) {
    delta.new_stamps[ecef_origin_key] = raw_frame->stamp;
    delta.new_stamps[ecef_rot_key] = raw_frame->stamp;
  }

  const bool first_incremental_segment = current_active_keys.empty();
  if (first_incremental_segment && segment_support_control_indices.size() >= 2) {
    const auto pose_key_0 = iap::bspline_control_point_key(segment_support_control_indices[0]);
    const auto pose_key_1 = iap::bspline_control_point_key(segment_support_control_indices[1]);
    const auto pose_0 = mirror_values.at<gtsam::Pose3>(pose_key_0);
    const auto pose_1 = mirror_values.at<gtsam::Pose3>(pose_key_1);
    delta.new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(pose_key_0, pose_0, anchor_noise);
    delta.new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(pose_key_1, pose_1, anchor_noise);
  }

  for (const auto control_index : segment_support_control_indices) {
    const auto pose_key = iap::bspline_control_point_key(control_index);
    if (!delta.new_values.exists(pose_key)) {
      continue;
    }
    delta.new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      pose_key,
      mirror_values.at<gtsam::Pose3>(pose_key),
      pred_noise);
  }
  for (const auto control_index : new_control_indices) {
    const auto curr_it = std::find_if(active_states.begin(), active_states.end(), [&](const auto& state) {
      return state.index == control_index;
    });
    if (curr_it == active_states.end() || curr_it == active_states.begin()) {
      continue;
    }
    const auto prev_it = std::prev(curr_it);
    const gtsam::Key prev_key = iap::bspline_control_point_key(prev_it->index);
    const gtsam::Key curr_key = iap::bspline_control_point_key(curr_it->index);
    delta.new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      prev_key,
      curr_key,
      prev_it->pose.between(curr_it->pose),
      smooth_noise);
  }
  if (delta.new_values.exists(current_velocity_key)) {
    delta.new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
      current_velocity_key,
      mirror_values.at<gtsam::Vector3>(current_velocity_key),
      velocity_prior_noise);
  }

  auto navigation_input = make_navigation_layer_delta_input(
    current_segment,
    previous_segment,
    factor_layout,
    current_active_keys,
    navigation_layer_enabled);
  navigation_input.graph_context.min_active_stamp = min_active_stamp;
  auto navigation_contribution = ct_compact_backend_.assemble_navigation_layer(navigation_input, &delta.new_values);
  delta.navigation_layer_factor_count = navigation_contribution.factor_count();

  const gtsam::Key current_clock_key = iap::bspline_clock_key(current_segment.auxiliary_index);
  if (delta.new_values.exists(current_clock_key)) {
    delta.new_stamps[current_clock_key] = current_segment.scan_end;
    append_unique_key(&newly_announced_keys, current_clock_key);
    if (!previous_segment) {
      delta.new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector2>>(
        current_clock_key,
        delta.new_values.at<gtsam::Vector2>(current_clock_key),
        clock_prior_noise);
    }
  }

  for (const auto& factor : local_contribution.graph) {
    if (factor) {
      delta.new_factors.push_back(factor);
    }
  }
  for (const auto& factor : navigation_contribution.graph) {
    if (factor) {
      delta.new_factors.push_back(factor);
    }
  }

  delta.query_keys = pose_keys_from_control_indices(segment_support_control_indices);
  delta.mirror_sync_keys = active_state_set.active_pose_keys;
  for (const auto key : current_segment.control_indices) {
    append_unique_key(&delta.query_keys, iap::bspline_control_point_key(key));
    append_unique_key(&delta.mirror_sync_keys, iap::bspline_control_point_key(key));
  }
  append_unique_key(&delta.query_keys, current_velocity_key);
  append_unique_key(&delta.mirror_sync_keys, current_velocity_key);
  if (navigation_contribution.activation.enabled) {
    append_unique_key(&delta.query_keys, current_clock_key);
    append_unique_key(&delta.mirror_sync_keys, current_clock_key);
  }
  for (const auto key : delta.persistent_keys) {
    append_unique_key(&delta.query_keys, key);
    append_unique_key(&delta.mirror_sync_keys, key);
  }
  delta.query_keys = sort_unique_keys(std::move(delta.query_keys));
  delta.mirror_sync_keys = sort_unique_keys(std::move(delta.mirror_sync_keys));

  const auto t_solver_update_start = Clock::now();
  const auto solver_result = unified_graph_solver_->apply_delta(delta);
  timing.solver_update_ms = elapsed_ms(t_solver_update_start, Clock::now());

  for (const auto control_index : segment_support_control_indices) {
    if (delta.new_values.exists(iap::bspline_control_point_key(control_index))) {
      fixed_lag_registry_.mark_control_index_announced(control_index);
    }
  }
  if (delta.new_values.exists(current_velocity_key)) {
    fixed_lag_registry_.mark_auxiliary_index_announced(current_segment.auxiliary_index);
  }
  if (delta.new_values.exists(current_clock_key)) {
    fixed_lag_registry_.mark_auxiliary_index_announced(current_segment.auxiliary_index);
  }
  for (const auto key : delta.persistent_keys) {
    if (delta.new_values.exists(key)) {
      fixed_lag_registry_.mark_persistent_key_announced(key);
    }
  }
  fixed_lag_registry_.retire_announced_keys(solver_result.retired_keys);

  fixed_lag_registry_.control_buffer().update_from_values(solver_result.estimate_subset);
  control_window_->update_from_values(solver_result.estimate_subset);
  fixed_lag_registry_.update_shared_state_from_values(solver_result.estimate_subset);
  fixed_lag_registry_.clear_auxiliary_values();
  for (const auto key : solver_result.active_aux_keys) {
    if (!solver_result.estimate_subset.exists(key)) {
      continue;
    }
    const char chr = gtsam::Symbol(key).chr();
    if (chr == 'u') {
      fixed_lag_registry_.auxiliary_values().insert(key, solver_result.estimate_subset.at<gtsam::Vector3>(key));
    } else if (chr == 'c') {
      fixed_lag_registry_.auxiliary_values().insert(key, solver_result.estimate_subset.at<gtsam::Vector2>(key));
    }
  }

  gtsam::Values evaluation_values = fixed_lag_registry_.control_buffer().values();
  for (const auto key : fixed_lag_registry_.auxiliary_values().keys()) {
    if (!evaluation_values.exists(key)) {
      evaluation_values.insert(key, fixed_lag_registry_.auxiliary_values().at(key));
    }
  }
  for (const auto key : solver_result.estimate_subset.keys()) {
    if (evaluation_values.exists(key)) {
      evaluation_values.update(key, solver_result.estimate_subset.at(key));
    } else {
      evaluation_values.insert(key, solver_result.estimate_subset.at(key));
    }
  }
  fixed_lag_registry_.seed_shared_values(
    evaluation_values,
    navigation_layer_enabled && fixed_lag_registry_.shared_state().gnss_anchor_initialized);

  iap::SplineActiveStateSet post_active_state_set =
    fixed_lag_registry_.active_state_set(evaluation_values, min_active_stamp, navigation_layer_enabled);
  {
    const auto t_prune_start = Clock::now();
    prune_active_ct_state(min_active_stamp, post_active_state_set);
    refresh_active_window_layout();
    timing.prune_active_ms = elapsed_ms(t_prune_start, Clock::now());
  }
  marginal_prior_ = ActiveSplineMarginalPrior();
  update_marginal_prior_from_active_window();
  timing.marginalization_ms = timing.prune_active_ms;

  const auto t_postprocess_start = Clock::now();
  auto current_handle_it = local_contribution.lidar_factor_handles.empty()
    ? local_contribution.lidar_factor_handles.end()
    : std::prev(local_contribution.lidar_factor_handles.end());
  if (current_handle_it == local_contribution.lidar_factor_handles.end()) {
    logger->error("bspline incremental graph failed to build current lidar factor");
    return nullptr;
  }

  std::vector<iap::BSplineLidarFactorResult> lidar_results;
  std::vector<iap::FrontendBucketProfileRow> bucket_profiles;
  std::vector<iap::LidarFactorInternalProfileRow> lidar_internal_rows;
  int current_lidar_result_index = -1;
  iap::BSplineLidarFactorResult current_lidar_result;
  const bool collect_window_lidar_results = lidar_collect_window_results();

  auto append_lidar_result = [&](const iap::BSplineLocalLayerContribution::LidarFactorHandle& handle, bool current_factor) {
    const double factor_error = handle.factor->error(evaluation_values);
    auto factor_result = handle.factor->make_result(
      factor_error,
      handle.factor->num_inliers(),
      handle.factor->inlier_fraction());
    if (current_factor) {
      current_lidar_result = factor_result;
      current_lidar_result_index = static_cast<int>(lidar_results.size());
    }
    lidar_results.push_back(factor_result);

    iap::FrontendBucketProfileRow bucket_profile;
    bucket_profile.source_frame_index = handle.source_frame_index;
    bucket_profile.bucket_index = handle.bucket_index;
    bucket_profile.bucket_mode = iap::CTLocalFrontend::bucket_mode_name(lidar_bucket_config_.mode);
    bucket_profile.representative_time = handle.representative_time;
    bucket_profile.points_in_bucket = handle.bucket_ctx.point_indices.size();
    bucket_profile.valid_correspondence_count = factor_result.profile.matched_point_count;
    bucket_profile.match_ratio = factor_result.profile.match_ratio;
    bucket_profile.inlier_ratio = factor_result.profile.inlier_ratio;
    bucket_profile.target_point_count = factor_result.profile.target_point_count;
    bucket_profile.candidate_evaluation_count = factor_result.profile.candidate_evaluation_count;
    bucket_profile.lookup_or_correspondence_ms = factor_result.profile.correspondence_ms;
    bucket_profile.accumulation_ms = factor_result.profile.accumulation_ms;
    bucket_profile.factor_total_ms = factor_result.profile.total_ms;
    bucket_profile.time_bucket_count = factor_result.profile.time_bucket_count;
    bucket_profile.mean_time_bucket_population = factor_result.profile.mean_time_bucket_population;
    bucket_profile.max_time_bucket_population = factor_result.profile.max_time_bucket_population;
    bucket_profiles.push_back(std::move(bucket_profile));
    lidar_internal_rows.push_back(make_lidar_factor_internal_profile_row(
      new_frame->id,
      raw_frame->stamp,
      iap::CTLocalFrontend::bucket_mode_name(lidar_bucket_config_.mode),
      local_contribution.lidar_factor_handles.size(),
      lidar_internal_rows.size(),
      handle,
      factor_result,
      post_active_state_set.active_control_indices.size()));
  };

  if (collect_window_lidar_results) {
    for (const auto& handle : local_contribution.lidar_factor_handles) {
      append_lidar_result(handle, &handle == &(*current_handle_it));
    }
  } else {
    append_lidar_result(*current_handle_it, true);
  }

  maybe_export_lidar_baseline_csv(raw_frame->stamp, lidar_results, current_lidar_result_index);

  auto evaluate_lidar_pose = [&](double stamp, double legacy_u) {
    if (active_window_layout_ && active_window_evaluator_) {
      if (const auto support = active_window_layout_->support_at(stamp, iap::SplineSensorId::Lidar)) {
        return active_window_evaluator_->eval_pose(evaluation_values, *support, iap::SplineSensorId::Lidar);
      }
    }
    return control_window_->evaluate(legacy_u);
  };
  const gtsam::Pose3 start_pose = evaluate_lidar_pose(raw_frame->stamp, 0.0);
  new_frame->T_world_lidar = Eigen::Isometry3d(start_pose.matrix());
  new_frame->T_world_imu = new_frame->T_world_lidar * T_lidar_imu;
  new_frame->v_world_imu = evaluation_values.exists(current_velocity_key)
    ? evaluation_values.at<gtsam::Vector3>(current_velocity_key)
    : Eigen::Vector3d::Zero();
  new_frame->imu_bias.head<3>() = fixed_lag_registry_.shared_state().accel_bias;
  new_frame->imu_bias.tail<3>() = fixed_lag_registry_.shared_state().gyro_bias;
  if (evaluation_values.exists(current_clock_key)) {
    const auto clock = evaluation_values.at<gtsam::Vector2>(current_clock_key);
    new_frame->clk_bias = clock(0);
    new_frame->clk_drift = clock(1);
  } else if (!frames.empty() && frames.back()) {
    new_frame->clk_bias = frames.back()->clk_bias;
    new_frame->clk_drift = frames.back()->clk_drift;
  }
  if (current_lidar_result.valid) {
    new_frame->icp_quality.inlier_count = current_lidar_result.inlier_count;
    new_frame->icp_quality.inlier_fraction = current_lidar_result.inlier_fraction;
    new_frame->icp_quality.rmse =
      std::sqrt(current_lidar_result.factor_error / std::max(new_frame->icp_quality.inlier_count, 1));
  }

  auto deskewed_points = current_handle_it->factor->deskewed_source_points(evaluation_values, true);
  const auto deskewed_covs = covariance_estimation->estimate(deskewed_points, raw_frame->neighbors);
  for (int i = 0; i < new_frame->frame->size(); ++i) {
    new_frame->frame->points[i] = deskewed_points[static_cast<std::size_t>(i)];
    new_frame->frame->covs[i] = deskewed_covs[static_cast<std::size_t>(i)];
  }

  {
    const auto t_callback_start = Clock::now();
    Callbacks::on_new_frame(new_frame);
    timing.post_callback_ms += elapsed_ms(t_callback_start, Clock::now());
  }
  {
    const auto t_target_insert_start = Clock::now();
    insert_target_cloud(new_frame);
    timing.post_target_insert_ms = elapsed_ms(t_target_insert_start, Clock::now());
  }
  {
    const auto t_history_start = Clock::now();
    update_frame_history(new_frame, marginalized_frames);
    timing.post_history_update_ms = elapsed_ms(t_history_start, Clock::now());
  }
  {
    const auto t_publish_start = Clock::now();
    publish_continuous_trajectory(current);
    timing.post_publish_traj_ms = elapsed_ms(t_publish_start, Clock::now());
  }
  {
    const auto t_telemetry_start = Clock::now();
    publish_fixed_lag_telemetry(current);
    timing.post_publish_telemetry_ms = elapsed_ms(t_telemetry_start, Clock::now());
  }

  std::vector<EstimationFrame::ConstPtr> active_frames(frames.inner_begin(), frames.inner_end());
  if (!active_frames.empty()) {
    const auto t_callback_start = Clock::now();
    Callbacks::on_update_new_frame(active_frames.back());
    Callbacks::on_update_frames(active_frames);
    timing.post_callback_ms += elapsed_ms(t_callback_start, Clock::now());
  }

  timing.postprocess_ms = elapsed_ms(t_postprocess_start, Clock::now());
  timing.window_wall_ms = elapsed_ms(t_window_start, Clock::now());

  iap::FrontendFrameProfile frontend_frame_profile = local_contribution.processed.frame_profile;
  frontend_frame_profile.frame_id = new_frame->id;
  frontend_frame_profile.stamp = raw_frame->stamp;
  frontend_frame_profile.frontend_mode = frontend_mode_;
  frontend_frame_profile.frontend_only_mode = frontend_only_mode_;
  frontend_frame_profile.use_legacy_two_stage_path = false;
  frontend_frame_profile.preprocess_ms = timing.source_cloud_ms;
  frontend_frame_profile.target_map_prep_ms = timing.target_build_ms;
  frontend_frame_profile.lm_solve_ms = timing.solver_update_ms;
  frontend_frame_profile.marginalization_ms = timing.marginalization_ms;
  frontend_frame_profile.backend_update_ms = 0.0;
  frontend_frame_profile.backend_optimize_ms = 0.0;
  frontend_frame_profile.publish_ms = timing.post_publish_traj_ms + timing.post_publish_telemetry_ms + timing.post_callback_ms;
  frontend_frame_profile.local_mapping_update_ms = 0.0;
  frontend_frame_profile.global_mapping_update_ms = 0.0;
  frontend_frame_profile.submap_registration_ms = 0.0;
  frontend_frame_profile.active_control_point_count = post_active_state_set.active_control_indices.size();
  frontend_frame_profile.active_pose_key_count = post_active_state_set.active_pose_keys.size();
  frontend_frame_profile.optimize_count = solver_result.optimize_count;
  frontend_frame_profile.local_layer_enabled = local_contribution.activation.enabled;
  frontend_frame_profile.navigation_layer_enabled = navigation_contribution.activation.enabled;
  frontend_frame_profile.local_layer_factor_count = local_contribution.factor_count();
  frontend_frame_profile.navigation_layer_factor_count = navigation_contribution.factor_count();
  frontend_frame_profile.local_layer_active_state_count = local_contribution.activation.active_state_count();
  frontend_frame_profile.navigation_layer_active_state_count = navigation_contribution.activation.active_state_count();
  frontend_frame_profile.solver_mode = iap::to_string(unified_solver_mode_);
  frontend_frame_profile.new_factor_count = delta.new_factors.size();
  frontend_frame_profile.new_value_count = delta.new_values.size();
  frontend_frame_profile.retired_key_count = solver_result.retired_keys.size();
  frontend_frame_profile.fallback_used = solver_result.fallback_used;
  frontend_frame_profile.carried_prior_replay_success = false;
  frontend_frame_profile.imu_factor_count = local_contribution.imu_factor_count;
  frontend_frame_profile.imu_residual_count = local_contribution.debug_stats.imu_residual_count;
  frontend_frame_profile.lidar_factor_count = local_contribution.lidar_factor_count;
  frontend_frame_profile.lidar_residual_count = local_contribution.debug_stats.lidar_residual_count;
  frontend_frame_profile.gnss_factor_count =
    navigation_contribution.gnss_pr_factor_count + navigation_contribution.gnss_dop_factor_count;
  frontend_frame_profile.carried_prior_count = 0;
  frontend_frame_profile.backend_factor_count = navigation_contribution.factor_count();
  frontend_frame_profile.backend_state_count = navigation_contribution.activation.active_state_count();
  frontend_frame_profile.local_residual_count =
    frontend_frame_profile.lidar_residual_count +
    frontend_frame_profile.imu_residual_count +
    frontend_frame_profile.gnss_factor_count;
  frontend_frame_profile.lm_initial_cost = solver_result.initial_cost;
  frontend_frame_profile.lm_final_cost = solver_result.final_cost;

  iap::SolverUpdateProfileRow solver_update_row;
  solver_update_row.frame_id = frontend_frame_profile.frame_id;
  solver_update_row.frame_stamp = frontend_frame_profile.stamp;
  solver_update_row.solver_mode = frontend_frame_profile.solver_mode;
  solver_update_row.frontend_only_mode = frontend_frame_profile.frontend_only_mode;
  solver_update_row.local_layer_enabled = frontend_frame_profile.local_layer_enabled;
  solver_update_row.navigation_layer_enabled = frontend_frame_profile.navigation_layer_enabled;
  solver_update_row.used_incremental_solver = solver_result.used_incremental_solver;
  solver_update_row.fallback_used = solver_result.fallback_used;
  solver_update_row.new_factor_count = delta.new_factors.size();
  solver_update_row.new_value_count = delta.new_values.size();
  solver_update_row.new_stamp_count = delta.new_stamps.size();
  solver_update_row.query_key_count = delta.query_keys.size();
  solver_update_row.retired_key_count = solver_result.retired_keys.size();
  solver_update_row.active_control_point_count = frontend_frame_profile.active_control_point_count;
  solver_update_row.active_pose_key_count = frontend_frame_profile.active_pose_key_count;
  solver_update_row.active_aux_key_count = solver_result.active_aux_keys.size();
  solver_update_row.persistent_key_count = delta.persistent_keys.size();
  solver_update_row.local_state_dimension = frontend_frame_profile.local_state_dimension;
  solver_update_row.local_residual_count = frontend_frame_profile.local_residual_count;
  solver_update_row.solver_update_ms = timing.solver_update_ms;
  solver_update_row.estimate_query_ms = solver_result.estimate_query_ms;
  solver_update_row.fallback_rebuild_ms = solver_result.fallback_rebuild_ms;
  solver_update_row.relinearization_ms = solver_result.relinearization_ms;
  solver_update_row.linearization_ms = solver_result.linearization_ms;
  solver_update_row.elimination_ms = solver_result.elimination_ms;
  solver_update_row.delta_solve_ms = solver_result.delta_solve_ms;
  solver_update_row.relinearized_variable_count = solver_result.relinearized_variable_count;
  solver_update_row.reeliminated_variable_count = solver_result.reeliminated_variable_count;
  solver_update_row.relinearized_factor_count = solver_result.relinearized_factor_count;
  solver_update_row.linearized_factor_count = solver_result.linearized_factor_count;
  solver_update_row.bayes_tree_clique_count = solver_result.bayes_tree_clique_count;
  solver_update_row.affected_variable_count = solver_result.affected_variable_count;
  solver_update_row.observed_key_count = solver_result.observed_key_count;
  solver_update_row.new_factor_index_count = solver_result.new_factor_index_count;
  solver_update_row.current_nonlinear_factor_count = solver_result.current_nonlinear_factor_count;
  solver_update_row.isam_reported_update_ms = solver_result.isam_reported_update_ms;
  solver_update_row.optimize_count = solver_result.optimize_count;
  solver_update_row.initial_error = solver_result.initial_cost;
  solver_update_row.final_error = solver_result.final_cost;
  solver_update_row.error_drop_ratio = error_drop_ratio(solver_result.initial_cost, solver_result.final_cost);
  solver_update_row.iteration_count = solver_result.iteration_count;
  solver_update_row.solver_status = solver_result.solver_status;

  maybe_write_frontend_frame_profile(frontend_frame_profile);
  maybe_write_solver_update_profile(solver_update_row);
  maybe_write_lidar_factor_profiles(frontend_frame_profile.frame_id, frontend_frame_profile.stamp, bucket_profiles);
  maybe_write_lidar_factor_internal_profiles(lidar_internal_rows);
  if (frontend_only_mode_) {
    log_frontend_only_stats(frontend_frame_profile);
  }

  return new_frame;
}

void OdometryEstimationBSpline::publish_continuous_trajectory(int current) {
  (void)current;

  if ((frontend_mode_ == "CT_LIDAR_CPU" || frontend_mode_ == "CT_LIDAR_GPU") && active_window_layout_) {
    gtsam::Values trajectory_values = fixed_lag_registry_.control_buffer().values();
    publish_continuous_trajectory_from_layout(active_window_layout_, trajectory_values);
    return;
  }

  auto trajectory = std::make_shared<iap::BSplineTrajectory>(trajectory_params_);
  std::vector<iap::SplineControlPoint> control_points;
  control_points.reserve(frames.inner_size());

  for (auto it = frames.inner_begin(); it != frames.inner_end(); ++it) {
    if (!(*it)) {
      continue;
    }

    iap::SplineControlPoint cp;
    cp.stamp = (*it)->stamp;
    cp.pose = (*it)->T_world_imu;
    cp.vel = (*it)->v_world_imu;
    cp.sigma = sigma_from_covariance((*it)->sigma_p);
    control_points.push_back(cp);
  }

  if (control_points.empty()) {
    return;
  }

  trajectory->set_control_points(control_points);

  if (trajectory->empty()) {
    return;
  }

  latest_trajectory_ = trajectory;

  update_frame_attachment(trajectory);
  update_compatibility_trajectory(trajectory);

  if (publish_shared_trajectory_) {
    iap::IapSharedState::instance().set_continuous_trajectory_view(trajectory);
    iap::IapSharedState::instance().set_spline_control_access(trajectory);
  }
}

void OdometryEstimationBSpline::publish_continuous_trajectory_from_layout(
  std::shared_ptr<const iap::SplineStateLayout> layout,
  const gtsam::Values& values) {
  auto trajectory = std::make_shared<iap::BSplineTrajectory>(trajectory_params_);
  trajectory->set_layout(std::move(layout), &values);

  if (trajectory->empty()) {
    return;
  }

  latest_trajectory_ = trajectory;

  update_frame_attachment(trajectory);
  update_compatibility_trajectory(trajectory);

  if (publish_shared_trajectory_) {
    iap::IapSharedState::instance().set_continuous_trajectory_view(trajectory);
    iap::IapSharedState::instance().set_spline_control_access(trajectory);
  }
}

void OdometryEstimationBSpline::publish_fixed_lag_telemetry(int current) const {
  (void)current;

  auto telemetry = fixed_lag_registry_.telemetry();

  if (publish_shared_trajectory_) {
    iap::IapSharedState::instance().set_bspline_fixed_lag_telemetry(telemetry);
  }

  logger->trace(
    "bspline fixed-lag telemetry state={} cps={} segs={} aux={} aux_values={} shared={} lag=[{:.3f}, {:.3f}] latest_segment=[{:.3f}, {:.3f}] anchor={}",
    iap::to_string(telemetry.lifecycle_state),
    telemetry.control_point_count,
    telemetry.segment_count,
    telemetry.active_auxiliary_count,
    telemetry.auxiliary_value_count,
    telemetry.active_shared_state_count,
    telemetry.lag_start_stamp,
    telemetry.lag_end_stamp,
    telemetry.latest_segment_stamp,
    telemetry.latest_segment_end,
    telemetry.gnss_anchor_initialized);
}

void OdometryEstimationBSpline::update_frame_attachment(const std::shared_ptr<iap::BSplineTrajectory>& trajectory) const {
  if (!attach_trajectory_to_frames_) {
    return;
  }

  const auto meta = trajectory->meta();
  for (auto it = frames.inner_begin(); it != frames.inner_end(); ++it) {
    if (!(*it)) {
      continue;
    }

    auto attachment = std::make_shared<iap::ContinuousTrajectoryAttachment>();
    attachment->trajectory_view = trajectory;
    attachment->control_access = trajectory;
    attachment->meta = meta;
    attachment->producer = "OdometryEstimationBSpline";
    (*it)->custom_data[iap::kContinuousTrajectoryAttachmentKey] = attachment;
  }
}

void OdometryEstimationBSpline::update_compatibility_trajectory(const std::shared_ptr<iap::BSplineTrajectory>& trajectory) const {
  if (compatibility_sample_dt_ <= 0.0) {
    return;
  }

  for (auto it = frames.inner_begin(); it != frames.inner_end(); ++it) {
    if (!(*it) || !(*it)->raw_frame) {
      continue;
    }

    const double scan_start = (*it)->stamp;
    const double scan_end = std::max(scan_start, (*it)->raw_frame->scan_end_time);
    if (scan_end < trajectory->start_time() || scan_start > trajectory->end_time()) {
      continue;
    }
    const auto samples = trajectory->sample_range(scan_start, scan_end, compatibility_sample_dt_);
    if (samples.empty()) {
      continue;
    }

    (*it)->imu_rate_trajectory.resize(8, samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i) {
      const auto& sample = samples[i];
      const Eigen::Quaterniond q(sample.pose.linear());
      (*it)->imu_rate_trajectory.col(static_cast<Eigen::Index>(i))
        << sample.stamp,
           sample.pose.translation(),
           q.x(), q.y(), q.z(), q.w();
    }
  }
}

void OdometryEstimationBSpline::maybe_export_lidar_baseline_csv(
  double stamp,
  const std::vector<iap::BSplineLidarFactorResult>& results,
  int current_factor_index) {
  if (!lidar_export_baseline_csv_) {
    return;
  }

  const auto export_data =
    iap::make_bspline_lidar_baseline_export(stamp, frontend_mode_.c_str(), results, current_factor_index);
  if (!export_data.valid) {
    return;
  }

  const std::filesystem::path csv_path(lidar_baseline_csv_path_);
  if (csv_path.has_parent_path()) {
    std::error_code ec;
    std::filesystem::create_directories(csv_path.parent_path(), ec);
    if (ec) {
      logger->warn(
        "failed to create CT LiDAR baseline CSV parent directory '{}': {}",
        csv_path.parent_path().string(),
        ec.message());
      return;
    }
  }

  std::FILE* file = std::fopen(lidar_baseline_csv_path_.c_str(), lidar_baseline_csv_header_written_ ? "a" : "w");
  if (!file) {
    logger->warn("failed to open CT LiDAR baseline CSV '{}': {}", lidar_baseline_csv_path_, std::strerror(errno));
    return;
  }

  if (!lidar_baseline_csv_header_written_) {
    iap::write_bspline_lidar_baseline_csv_header(file);
    lidar_baseline_csv_header_written_ = true;
  }

  iap::write_bspline_lidar_baseline_csv(file, export_data);
  std::fclose(file);

  if (!lidar_baseline_csv_first_row_logged_) {
    logger->info("bspline ct lidar baseline-csv first_rows_written path={}", lidar_baseline_csv_path_);
    lidar_baseline_csv_first_row_logged_ = true;
  }

  logger->trace(
    "bspline ct lidar baseline-csv path={} backend={} rows={} current_factor={} warnings={} total_ms={:.3f}",
    lidar_baseline_csv_path_,
    iap::to_string(export_data.backend),
    export_data.factor_results.size() + 1,
    export_data.current_factor_index,
    export_data.summary.warning_result_count,
    export_data.summary.total_factor_ms);
}

void OdometryEstimationBSpline::maybe_write_frontend_frame_profile(const iap::FrontendFrameProfile& profile) {
  if (!frontend_frame_profile_enabled_) {
    return;
  }

  std::FILE* file = std::fopen(frontend_frame_profile_csv_path_.c_str(), frontend_frame_profile_header_written_ ? "a" : "w");
  if (!file) {
    logger->warn(
      "failed to open frontend frame profile csv path={} errno={} ({})",
      frontend_frame_profile_csv_path_,
      errno,
      std::strerror(errno));
    return;
  }

  if (!frontend_frame_profile_header_written_) {
    std::fputs(frontend_frame_profile_csv_header(), file);
    frontend_frame_profile_header_written_ = true;
  }

  write_frontend_frame_profile_row(file, profile);
  std::fclose(file);
}

void OdometryEstimationBSpline::maybe_write_solver_update_profile(const iap::SolverUpdateProfileRow& row) {
  if (!solver_update_profile_enabled_) {
    return;
  }

  std::FILE* file = std::fopen(
    solver_update_profile_csv_path_.c_str(),
    solver_update_profile_header_written_ ? "a" : "w");
  if (!file) {
    logger->warn(
      "failed to open solver update profile csv path={} errno={} ({})",
      solver_update_profile_csv_path_,
      errno,
      std::strerror(errno));
    return;
  }

  if (!solver_update_profile_header_written_) {
    std::fputs(solver_update_profile_csv_header(), file);
    solver_update_profile_header_written_ = true;
  }

  write_solver_update_profile_row(file, row);
  std::fclose(file);
}

void OdometryEstimationBSpline::maybe_write_lidar_factor_profiles(
  const int frame_id,
  const double stamp,
  const std::vector<iap::FrontendBucketProfileRow>& profiles) {
  if (!lidar_factor_profile_ || profiles.empty()) {
    return;
  }

  std::FILE* file = std::fopen(
    frontend_lidar_factor_profile_csv_path_.c_str(),
    frontend_lidar_factor_profile_header_written_ ? "a" : "w");
  if (!file) {
    logger->warn(
      "failed to open frontend lidar factor profile csv path={} errno={} ({})",
      frontend_lidar_factor_profile_csv_path_,
      errno,
      std::strerror(errno));
    return;
  }

  if (!frontend_lidar_factor_profile_header_written_) {
    std::fputs(frontend_lidar_factor_profile_csv_header(), file);
    frontend_lidar_factor_profile_header_written_ = true;
  }

  write_frontend_lidar_factor_profile_rows(file, frame_id, stamp, profiles);
  std::fclose(file);
}

void OdometryEstimationBSpline::maybe_write_lidar_factor_internal_profiles(
  const std::vector<iap::LidarFactorInternalProfileRow>& rows) {
  if (!lidar_factor_internal_profile_enabled_ || rows.empty()) {
    return;
  }

  std::FILE* file = std::fopen(
    lidar_factor_internal_profile_csv_path_.c_str(),
    lidar_factor_internal_profile_header_written_ ? "a" : "w");
  if (!file) {
    logger->warn(
      "failed to open lidar factor internal profile csv path={} errno={} ({})",
      lidar_factor_internal_profile_csv_path_,
      errno,
      std::strerror(errno));
    return;
  }

  if (!lidar_factor_internal_profile_header_written_) {
    std::fputs(lidar_factor_internal_profile_csv_header(), file);
    lidar_factor_internal_profile_header_written_ = true;
  }

  write_lidar_factor_internal_profile_rows(file, rows);
  std::fclose(file);
}

void OdometryEstimationBSpline::maybe_write_frontend_lm_iterations(
  const int frame_id,
  const double stamp,
  const std::vector<iap::FrontendLMIterationProfileRow>& iterations) {
  if (!frontend_lm_iteration_profile_enabled_ || iterations.empty()) {
    return;
  }

  std::FILE* file = std::fopen(
    frontend_lm_iteration_csv_path_.c_str(),
    frontend_lm_iteration_header_written_ ? "a" : "w");
  if (!file) {
    logger->warn(
      "failed to open frontend lm iteration csv path={} errno={} ({})",
      frontend_lm_iteration_csv_path_,
      errno,
      std::strerror(errno));
    return;
  }

  if (!frontend_lm_iteration_header_written_) {
    std::fputs(frontend_lm_iteration_csv_header(), file);
    frontend_lm_iteration_header_written_ = true;
  }

  write_frontend_lm_iteration_rows(file, frame_id, stamp, iterations);
  std::fclose(file);
}

void OdometryEstimationBSpline::maybe_write_frame_warning_profile(const FrameWarningProfileRow& row) {
  if (!frame_warning_profile_enabled_) {
    return;
  }

  std::FILE* file = std::fopen(
    frame_warning_profile_csv_path_.c_str(),
    frame_warning_profile_header_written_ ? "a" : "w");
  if (!file) {
    logger->warn(
      "failed to open frame warning profile csv path={} errno={} ({})",
      frame_warning_profile_csv_path_,
      errno,
      std::strerror(errno));
    return;
  }

  if (!frame_warning_profile_header_written_) {
    std::fputs(frame_warning_profile_csv_header(), file);
    frame_warning_profile_header_written_ = true;
  }

  write_frame_warning_profile_row(
    file,
    row.frame_id,
    row.stamp,
    row.warning_count,
    row.warning_categories,
    row.top_warning_message);
  std::fclose(file);
}

OdometryEstimationBSpline::FrameWarningProfileRow OdometryEstimationBSpline::build_frame_warning_profile(
  const int frame_id,
  const double stamp,
  const iap::CTLocalFrontend::Input& input,
  const iap::CTLocalFrontendResult& local_result,
  const iap::FrontendFrameProfile& profile) const {
  FrameWarningProfileRow row;
  row.frame_id = frame_id;
  row.stamp = stamp;

  std::vector<std::string> categories;
  std::vector<std::string> messages;
  const auto add_warning = [&](const std::string& category, const std::string& message) {
    if (std::find(categories.begin(), categories.end(), category) == categories.end()) {
      categories.push_back(category);
    }
    messages.push_back(message);
  };

  if (!input.target_ivox || input.target_ivox->voxel_points().empty()) {
    add_warning("target_map", "target map unavailable for local CT frontend solve");
  }
  if (profile.total_source_points == 0) {
    add_warning("data_gap", "source cloud was empty for local CT frontend solve");
  }
  if (profile.imu_sample_count > 0 && profile.imu_residual_count == 0) {
    add_warning("data_gap", "imu samples were present but no IMU residuals were attached");
  }
  if (lidar_warn_degeneracy_ && local_result.processed.lidar_window_summary.valid) {
    const auto& summary = local_result.processed.lidar_window_summary;
    if (summary.weighted_match_ratio < lidar_degeneracy_thresholds_.min_match_ratio ||
        summary.weighted_inlier_ratio < lidar_degeneracy_thresholds_.min_inlier_ratio) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(3)
          << "low LiDAR support match_ratio=" << summary.weighted_match_ratio
          << " inlier_ratio=" << summary.weighted_inlier_ratio;
      add_warning("degeneracy", oss.str());
    }
  }
  if (profile.lm_trace_expected && !profile.lm_trace_emitted) {
    add_warning("optimizer", "lm iteration trace was expected but no rows were emitted");
  } else if (input.lm_max_iterations > 0 && profile.lm_iteration_count >= input.lm_max_iterations) {
    add_warning("optimizer", "lm solve reached the configured max iteration count");
  }
  if (messages.empty() && profile.total_source_points > 0 && profile.actual_bucket_count == 0) {
    add_warning("other", "no lidar buckets were emitted for a non-empty source cloud");
  }

  row.warning_count = messages.size();
  for (std::size_t i = 0; i < categories.size(); ++i) {
    if (i > 0) {
      row.warning_categories += ';';
    }
    row.warning_categories += categories[i];
  }
  row.top_warning_message = messages.empty() ? std::string{} : messages.front();
  return row;
}

void OdometryEstimationBSpline::log_frontend_only_stats(const iap::FrontendFrameProfile& profile) const {
  constexpr double kSlowFrontendFrameThresholdMs = 50.0;
  const double wall_ms = frontend_frame_wall_ms(profile);
  if (wall_ms < kSlowFrontendFrameThresholdMs) {
    return;
  }

  logger->info(
    "frame={} wall_ms={:.3f} top_stage={} buckets={} lidar_factors={} imu_residuals={} lm_iters={}",
    profile.frame_id,
    wall_ms,
    frontend_frame_top_stage(profile),
    profile.actual_bucket_count,
    profile.lidar_factor_count,
    profile.imu_residual_count,
    profile.lm_iteration_count);
}

// IAP-RQ-300 / IAP-RQ-410: Build CTLocalFrontend::Input from the current raw frame and frame history.
// This helper wires target selection, bucket config, prepared source cloud, and
// IMU samples into the local frontend contract.
iap::CTLocalFrontend::Input OdometryEstimationBSpline::make_frontend_input(
  const PreprocessedFrame::Ptr& raw_frame,
  const gtsam_points::PointCloud::ConstPtr& prepared_source_cloud) const {
  iap::CTLocalFrontend::Input input;

  if (!frames.empty() && frames.back()) {
    input.target_frame = frames.back();
  }

  auto source = std::make_shared<glim::RawPoints>();
  source->stamp = raw_frame->stamp;
  source->points = raw_frame->points;
  source->times = raw_frame->times;
  input.source_frames.push_back(iap::CTLocalFrontend::SourceFrameInput{
    source,
    prepared_source_cloud,
    raw_frame->stamp,
    raw_frame->scan_end_time,
  });

  const auto target_ref = create_active_target_reference();
  input.target_ivox =
    target_ref.target_snapshot && !target_ref.target_snapshot->voxel_points().empty()
      ? target_ref.target_snapshot
      : nullptr;
  input.target_map_prep_ms = target_ref.build_ms;
  if (frontend_frame_profile_enabled_ && target_map_prep_breakdown_enabled_) {
    input.target_snapshot_clone_ms = target_ref.target_snapshot_clone_ms;
    input.target_voxel_lookup_prep_ms = target_ref.target_voxel_lookup_prep_ms;
    input.target_covariance_prep_ms = target_ref.target_covariance_prep_ms;
    input.source_to_target_transform_ms = target_ref.source_to_target_transform_ms;
  }
  input.bucket_config = lidar_bucket_config_;
  input.lm_max_iterations = lm_max_iterations_;
  input.accelerometer_precision = imu_ct_trans_inf_scale_;
  input.gyroscope_precision = imu_ct_rot_inf_scale_;
  input.max_correspondence_distance = max_correspondence_distance_;
  input.enable_lm_iteration_trace = frontend_lm_iteration_profile_enabled_;
  input.enable_graph_problem_size = frontend_frame_profile_enabled_ && graph_problem_size_enabled_;

  const auto imu_samples = create_segment_imu_samples(raw_frame);
  input.imu_samples.reserve(imu_samples.size());
  for (const auto& sample : imu_samples) {
    input.imu_samples.push_back(iap::CTLocalFrontend::IMUSample{
      sample.stamp,
      sample.angular_vel,
      sample.linear_acc,
    });
  }

  return input;
}

iap::CTLocalFrontend::LayerInput OdometryEstimationBSpline::make_local_layer_input(
  const gtsam::Values& values,
  std::shared_ptr<const iap::SplineStateLayout> factor_layout) const {
  iap::CTLocalFrontend::LayerInput input;
  input.graph_context.layout = std::move(factor_layout);
  input.graph_context.frontend_only_mode = frontend_only_mode_;
  input.graph_context.local_layer_enabled = true;
  input.graph_context.navigation_layer_enabled = !frontend_only_mode_;
  input.bucket_config = lidar_bucket_config_;
  input.lm_max_iterations = lm_max_iterations_;
  input.accelerometer_precision = imu_ct_trans_inf_scale_;
  input.gyroscope_precision = imu_ct_rot_inf_scale_;
  input.velocity_precision = velocity_ct_inf_scale_;
  input.finite_difference_dt = trajectory_params_.finite_difference_dt;
  input.max_correspondence_distance = max_correspondence_distance_;
  input.enable_lidar_factor_profiling =
    lidar_factor_profile_ || lidar_warn_degeneracy_ || lidar_export_baseline_csv_ || lidar_factor_internal_profile_enabled_;
  input.enable_graph_problem_size = frontend_frame_profile_enabled_ && graph_problem_size_enabled_;
  input.jacobian_mode = lidar_jacobian_mode_;
  input.numeric_eps = lidar_jacobian_numeric_eps_;
  input.correspondence_candidate_count = lidar_correspondence_candidate_count_;
  input.correspondence_accept_ratio = lidar_correspondence_accept_ratio_;
  input.correspondence_min_score_gap = lidar_correspondence_min_score_gap_;
  input.outlier_mahalanobis_threshold = lidar_outlier_mahalanobis_thresh_;
  input.robust_kernel = lidar_robust_kernel_;
  input.robust_kernel_width = lidar_robust_kernel_width_;
  input.robust_weight_floor = lidar_robust_weight_floor_;

  for (std::size_t i = 0; i < fixed_lag_registry_.segments().size(); ++i) {
    const auto& segment = fixed_lag_registry_.segments()[i];
    iap::CTLocalFrontend::LayerSegmentInput layer_segment;
    layer_segment.source_frame_index = i;
    layer_segment.source_frame.source_cloud = segment.source;
    layer_segment.source_frame.scan_start = segment.stamp;
    layer_segment.source_frame.scan_end = segment.scan_end;
    layer_segment.target_ivox = segment.target_snapshot;
    layer_segment.target_tree = segment.target_tree;
    layer_segment.control_indices = segment.control_indices;
    layer_segment.auxiliary_index = segment.auxiliary_index;
    layer_segment.imu_samples.reserve(segment.imu_samples.size());
    for (const auto& imu_sample : segment.imu_samples) {
      layer_segment.imu_samples.push_back(iap::CTLocalFrontend::IMUSample{
        imu_sample.stamp,
        imu_sample.angular_vel,
        imu_sample.linear_acc,
      });
    }
    input.segments.push_back(std::move(layer_segment));
  }

  (void)values;
  return input;
}

iap::CTLocalFrontend::LayerInput OdometryEstimationBSpline::make_local_layer_delta_input(
  const ActiveSplineSegmentConstraint& segment,
  std::shared_ptr<const iap::SplineStateLayout> factor_layout) const {
  iap::CTLocalFrontend::LayerInput input;
  input.graph_context.layout = std::move(factor_layout);
  input.graph_context.min_active_stamp = std::max(0.0, segment.stamp - params->smoother_lag);
  input.graph_context.frontend_only_mode = frontend_only_mode_;
  input.graph_context.local_layer_enabled = true;
  input.graph_context.navigation_layer_enabled = !frontend_only_mode_;
  input.bucket_config = lidar_bucket_config_;
  input.lm_max_iterations = lm_max_iterations_;
  input.accelerometer_precision = imu_ct_trans_inf_scale_;
  input.gyroscope_precision = imu_ct_rot_inf_scale_;
  input.velocity_precision = velocity_ct_inf_scale_;
  input.finite_difference_dt = trajectory_params_.finite_difference_dt;
  input.max_correspondence_distance = max_correspondence_distance_;
  input.enable_lidar_factor_profiling =
    lidar_factor_profile_ || lidar_warn_degeneracy_ || lidar_export_baseline_csv_ || lidar_factor_internal_profile_enabled_;
  input.enable_graph_problem_size = frontend_frame_profile_enabled_ && graph_problem_size_enabled_;
  input.jacobian_mode = lidar_jacobian_mode_;
  input.numeric_eps = lidar_jacobian_numeric_eps_;
  input.correspondence_candidate_count = lidar_correspondence_candidate_count_;
  input.correspondence_accept_ratio = lidar_correspondence_accept_ratio_;
  input.correspondence_min_score_gap = lidar_correspondence_min_score_gap_;
  input.outlier_mahalanobis_threshold = lidar_outlier_mahalanobis_thresh_;
  input.robust_kernel = lidar_robust_kernel_;
  input.robust_kernel_width = lidar_robust_kernel_width_;
  input.robust_weight_floor = lidar_robust_weight_floor_;

  iap::CTLocalFrontend::LayerSegmentInput layer_segment;
  layer_segment.source_frame_index = fixed_lag_registry_.segments().empty() ? 0U : fixed_lag_registry_.segments().size() - 1U;
  layer_segment.source_frame.source_cloud = segment.source;
  layer_segment.source_frame.scan_start = segment.stamp;
  layer_segment.source_frame.scan_end = segment.scan_end;
  layer_segment.target_ivox = segment.target_snapshot;
  layer_segment.target_tree = segment.target_tree;
  layer_segment.control_indices = segment.control_indices;
  layer_segment.auxiliary_index = segment.auxiliary_index;
  layer_segment.imu_samples.reserve(segment.imu_samples.size());
  for (const auto& imu_sample : segment.imu_samples) {
    layer_segment.imu_samples.push_back(iap::CTLocalFrontend::IMUSample{
      imu_sample.stamp,
      imu_sample.angular_vel,
      imu_sample.linear_acc,
    });
  }
  input.segments.push_back(std::move(layer_segment));
  return input;
}

// IAP-RQ-300 / IAP-RQ-410: Build CTCompactBackend::Input from shared GNSS state.
// The local_result parameter is reserved for future layout-driven GNSS key selection (Task 6+).
iap::CTCompactBackend::Input OdometryEstimationBSpline::make_backend_input(
  const iap::CTLocalFrontendResult& local_result) const {
  // local_result.layout is available for Task 6+ key derivation; not used here yet.
  iap::CTCompactBackend::Input input;

  if (!fixed_lag_registry_.segments().empty()) {
    input.gnss_epochs = fixed_lag_registry_.segments().back().gnss_epochs;
  }

  const auto& shared = fixed_lag_registry_.shared_state();
  input.gnss_anchor_initialized = shared.gnss_anchor_initialized;
  input.ecef_origin = shared.ecef_origin;
  input.ecef_rot = shared.ecef_rot;
  input.gnss_pr_noise_base = gnss_pr_noise_base_;
  input.gnss_dop_noise_base = gnss_dop_noise_base_;
  input.gnss_min_elevation = gnss_min_elevation_;
  input.gnss_elev_noise_exp = gnss_elev_noise_exp_;
  // TODO(Task 6): forward gnss_canopy_params_ and gnss_clock_between_params_ to backend Input.
  // Note: epoch gating uses layout domain bounds; gnss_time_tolerance_ is applied
  // upstream in consume_segment_gnss_epochs before epochs reach the backend.
  input.gnss_lever_arm = gnss_lever_arm_;

  return input;
}

iap::CTCompactBackend::LayerInput OdometryEstimationBSpline::make_navigation_layer_input(
  std::shared_ptr<const iap::SplineStateLayout> factor_layout,
  bool navigation_layer_enabled) const {
  iap::CTCompactBackend::LayerInput input;
  input.graph_context.layout = std::move(factor_layout);
  input.graph_context.frontend_only_mode = frontend_only_mode_;
  input.graph_context.local_layer_enabled = true;
  input.graph_context.navigation_layer_enabled = navigation_layer_enabled;

  const auto& shared = fixed_lag_registry_.shared_state();
  input.gnss_anchor_initialized = shared.gnss_anchor_initialized;
  input.ecef_origin = shared.ecef_origin;
  input.ecef_rot = shared.ecef_rot;
  input.gnss_pr_noise_base = gnss_pr_noise_base_;
  input.gnss_dop_noise_base = gnss_dop_noise_base_;
  input.gnss_min_elevation = gnss_min_elevation_;
  input.gnss_elev_noise_exp = gnss_elev_noise_exp_;
  input.gnss_lever_arm = gnss_lever_arm_;

  for (const auto& segment : fixed_lag_registry_.segments()) {
    iap::CTCompactBackend::LayerSegmentInput layer_segment;
    layer_segment.stamp = segment.stamp;
    layer_segment.auxiliary_index = segment.auxiliary_index;
    layer_segment.gnss_epochs = segment.gnss_epochs;
    input.segments.push_back(std::move(layer_segment));
  }

  return input;
}

iap::CTCompactBackend::LayerInput OdometryEstimationBSpline::make_navigation_layer_delta_input(
  const ActiveSplineSegmentConstraint& current_segment,
  const ActiveSplineSegmentConstraint* previous_segment,
  std::shared_ptr<const iap::SplineStateLayout> factor_layout,
  const gtsam::KeyVector& existing_keys,
  bool navigation_layer_enabled) const {
  iap::CTCompactBackend::LayerInput input;
  input.graph_context.layout = std::move(factor_layout);
  input.graph_context.min_active_stamp = std::max(0.0, current_segment.stamp - params->smoother_lag);
  input.graph_context.frontend_only_mode = frontend_only_mode_;
  input.graph_context.local_layer_enabled = true;
  input.graph_context.navigation_layer_enabled = navigation_layer_enabled;
  input.graph_context.existing_keys = existing_keys;

  const auto& shared = fixed_lag_registry_.shared_state();
  input.gnss_anchor_initialized = shared.gnss_anchor_initialized;
  input.ecef_origin = shared.ecef_origin;
  input.ecef_rot = shared.ecef_rot;
  input.gnss_pr_noise_base = gnss_pr_noise_base_;
  input.gnss_dop_noise_base = gnss_dop_noise_base_;
  input.gnss_min_elevation = gnss_min_elevation_;
  input.gnss_elev_noise_exp = gnss_elev_noise_exp_;
  input.gnss_lever_arm = gnss_lever_arm_;

  iap::CTCompactBackend::LayerSegmentInput layer_segment;
  layer_segment.stamp = current_segment.stamp;
  layer_segment.auxiliary_index = current_segment.auxiliary_index;
  layer_segment.gnss_epochs = current_segment.gnss_epochs;
  if (previous_segment) {
    layer_segment.has_previous_auxiliary = true;
    layer_segment.previous_auxiliary_index = previous_segment->auxiliary_index;
    layer_segment.previous_stamp = previous_segment->stamp;
  }
  input.segments.push_back(std::move(layer_segment));
  return input;
}

void OdometryEstimationBSpline::reset_unified_graph_solver() {
  if (unified_solver_mode_ == iap::BSplineUnifiedSolverMode::INCREMENTAL_SMOOTHER) {
    gtsam::ISAM2Params incremental_params;
    if (params->use_isam2_dogleg) {
      incremental_params.setOptimizationParams(gtsam::ISAM2DoglegParams());
    }
    incremental_params.findUnusedFactorSlots = true;
    incremental_params.relinearizeSkip = params->isam2_relinearize_skip;
    incremental_params.setRelinearizeThreshold(params->isam2_relinearize_thresh);
    unified_graph_solver_ = std::make_unique<iap::IncrementalSmootherSolver>(params->smoother_lag, incremental_params);
  } else {
    unified_graph_solver_ = std::make_unique<iap::BatchLMSolver>(lm_max_iterations_);
  }
}

}  // namespace glim
