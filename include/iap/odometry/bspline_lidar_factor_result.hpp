#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Unified CT LiDAR factor profile/result types shared by CPU now and GPU later.

#include <array>
#include <cstddef>
#include <vector>

namespace iap {

enum class BSplineLidarFactorBackend {
  CPU_GICP,
  GPU_GICP,
};

inline const char* to_string(BSplineLidarFactorBackend backend) {
  switch (backend) {
    case BSplineLidarFactorBackend::CPU_GICP:
      return "CPU_GICP";
    case BSplineLidarFactorBackend::GPU_GICP:
      return "GPU_GICP";
  }
  return "UNKNOWN";
}

struct BSplineLidarFactorProfile {
  bool valid = false;
  BSplineLidarFactorBackend backend = BSplineLidarFactorBackend::CPU_GICP;
  std::size_t source_point_count = 0;
  std::size_t target_point_count = 0;
  std::size_t time_bucket_count = 0;
  std::size_t max_time_bucket_population = 0;
  std::size_t candidate_evaluation_count = 0;
  std::size_t matched_point_count = 0;
  std::size_t inlier_point_count = 0;
  std::size_t unique_target_count = 0;
  std::size_t max_target_reuse = 0;
  std::size_t comparative_score_count = 0;
  std::size_t rejected_distance_count = 0;
  std::size_t rejected_ambiguity_count = 0;
  std::size_t rejected_outlier_count = 0;
  std::size_t rejected_robust_count = 0;
  double match_ratio = 0.0;
  double inlier_ratio = 0.0;
  double mean_time_bucket_population = 0.0;
  double mean_candidates_per_source = 0.0;
  double unique_target_ratio = 0.0;
  double max_target_reuse_ratio = 0.0;
  double mean_match_distance = 0.0;
  double max_match_distance = 0.0;
  double mean_match_score = 0.0;
  double mean_score_gap = 0.0;
  double mean_score_ratio = 0.0;
  double mean_robust_weight = 1.0;
  double pose_update_ms = 0.0;
  double correspondence_ms = 0.0;
  double accumulation_ms = 0.0;
  double total_ms = 0.0;
  double total_error = 0.0;
  const char* stage = "none";
};

struct BSplineLidarNumericAudit {
  bool valid = false;
  double perturbation_scale = 0.0;
  double numeric_rotation_predicted_error = 0.0;
  double semi_rotation_predicted_error = 0.0;
  double rotation_actual_error = 0.0;
  double rotation_abs_error = 0.0;
  double rotation_rel_error = 0.0;
  double numeric_translation_predicted_error = 0.0;
  double semi_translation_predicted_error = 0.0;
  double translation_actual_error = 0.0;
  double translation_abs_error = 0.0;
  double translation_rel_error = 0.0;
  std::array<double, 3> axis_rotation_rel_error = {0.0, 0.0, 0.0};
  std::size_t worst_rotation_axis = 0;
  double max_rotation_axis_rel_error = 0.0;
  double mean_rotation_axis_rel_error = 0.0;
};

struct BSplineLidarDegeneracyReport {
  bool valid = false;
  bool empty_target = false;
  bool low_match_ratio = false;
  bool low_inlier_ratio = false;
  bool low_target_diversity = false;
  bool high_target_reuse = false;
  bool high_ambiguity_rejection = false;
  bool weak_score_separation = false;
  std::size_t warning_count = 0;
  double ambiguity_rejection_ratio = 0.0;
  double distance_rejection_ratio = 0.0;
  double outlier_rejection_ratio = 0.0;
  double robust_rejection_ratio = 0.0;

  bool has_warning() const { return warning_count > 0; }
};

struct BSplineLidarFactorResult {
  bool valid = false;
  BSplineLidarFactorBackend backend = BSplineLidarFactorBackend::CPU_GICP;
  double factor_error = 0.0;
  double rmse = 0.0;
  int inlier_count = 0;
  double inlier_fraction = 0.0;
  BSplineLidarFactorProfile profile;
  BSplineLidarNumericAudit numeric_audit;
  BSplineLidarDegeneracyReport degeneracy;
};

struct BSplineLidarWindowProfileSummary {
  bool valid = false;
  std::size_t result_count = 0;
  std::size_t valid_profile_count = 0;
  std::size_t warning_result_count = 0;
  std::size_t total_source_point_count = 0;
  std::size_t total_target_point_count = 0;
  std::size_t total_matched_point_count = 0;
  std::size_t total_inlier_point_count = 0;
  std::size_t total_candidate_evaluation_count = 0;
  double weighted_match_ratio = 0.0;
  double weighted_inlier_ratio = 0.0;
  double mean_unique_target_ratio = 0.0;
  double min_unique_target_ratio = 0.0;
  double max_target_reuse_ratio = 0.0;
  double max_ambiguity_rejection_ratio = 0.0;
  double max_numeric_rel_error = 0.0;
  double max_rotation_axis_rel_error = 0.0;
  double total_pose_update_ms = 0.0;
  double total_correspondence_ms = 0.0;
  double total_accumulation_ms = 0.0;
  double total_factor_ms = 0.0;
  double mean_candidates_per_source = 0.0;
  double mean_time_bucket_population = 0.0;
  std::size_t max_time_bucket_population = 0;
};

inline BSplineLidarWindowProfileSummary aggregate_bspline_lidar_factor_results(
  const std::vector<BSplineLidarFactorResult>& results) {
  BSplineLidarWindowProfileSummary summary;
  summary.result_count = results.size();
  if (results.empty()) {
    return summary;
  }

  double unique_target_ratio_sum = 0.0;
  double mean_candidates_per_source_sum = 0.0;
  double mean_time_bucket_population_sum = 0.0;
  bool min_unique_target_ratio_initialized = false;

  for (const auto& result : results) {
    if (!result.valid || !result.profile.valid) {
      continue;
    }

    summary.valid = true;
    summary.valid_profile_count++;
    summary.warning_result_count += result.degeneracy.has_warning() ? 1U : 0U;
    summary.total_source_point_count += result.profile.source_point_count;
    summary.total_target_point_count += result.profile.target_point_count;
    summary.total_matched_point_count += result.profile.matched_point_count;
    summary.total_inlier_point_count += result.profile.inlier_point_count;
    summary.total_candidate_evaluation_count += result.profile.candidate_evaluation_count;
    summary.weighted_match_ratio += static_cast<double>(result.profile.matched_point_count);
    summary.weighted_inlier_ratio += static_cast<double>(result.profile.inlier_point_count);
    unique_target_ratio_sum += result.profile.unique_target_ratio;
    mean_candidates_per_source_sum += result.profile.mean_candidates_per_source;
    mean_time_bucket_population_sum += result.profile.mean_time_bucket_population;
    summary.max_target_reuse_ratio = std::max(summary.max_target_reuse_ratio, result.profile.max_target_reuse_ratio);
    summary.max_ambiguity_rejection_ratio =
      std::max(summary.max_ambiguity_rejection_ratio, result.degeneracy.ambiguity_rejection_ratio);
    summary.max_numeric_rel_error = std::max(
      summary.max_numeric_rel_error,
      std::max(result.numeric_audit.rotation_rel_error, result.numeric_audit.translation_rel_error));
    summary.max_rotation_axis_rel_error =
      std::max(summary.max_rotation_axis_rel_error, result.numeric_audit.max_rotation_axis_rel_error);
    summary.total_pose_update_ms += result.profile.pose_update_ms;
    summary.total_correspondence_ms += result.profile.correspondence_ms;
    summary.total_accumulation_ms += result.profile.accumulation_ms;
    summary.total_factor_ms += result.profile.total_ms;
    summary.max_time_bucket_population =
      std::max(summary.max_time_bucket_population, result.profile.max_time_bucket_population);

    if (!min_unique_target_ratio_initialized || result.profile.unique_target_ratio < summary.min_unique_target_ratio) {
      summary.min_unique_target_ratio = result.profile.unique_target_ratio;
      min_unique_target_ratio_initialized = true;
    }
  }

  if (!summary.valid || summary.valid_profile_count == 0) {
    return summary;
  }

  summary.weighted_match_ratio =
    summary.total_source_point_count == 0 ? 0.0 : summary.weighted_match_ratio / summary.total_source_point_count;
  summary.weighted_inlier_ratio =
    summary.total_source_point_count == 0 ? 0.0 : summary.weighted_inlier_ratio / summary.total_source_point_count;
  summary.mean_unique_target_ratio = unique_target_ratio_sum / static_cast<double>(summary.valid_profile_count);
  summary.mean_candidates_per_source = mean_candidates_per_source_sum / static_cast<double>(summary.valid_profile_count);
  summary.mean_time_bucket_population = mean_time_bucket_population_sum / static_cast<double>(summary.valid_profile_count);
  return summary;
}

}  // namespace iap
