#include <iap/odometry/bspline_graph_solver.hpp>
#include <iap/odometry/bspline_factor_family.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

#include <algorithm>
#include <chrono>
#include <sstream>
#include <unordered_set>
#include <vector>

namespace iap {

namespace {

gtsam::KeyVector sort_unique_keys(gtsam::KeyVector keys) {
  std::sort(keys.begin(), keys.end());
  keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
  return keys;
}

gtsam::KeyVector merge_keys(
  const gtsam::KeyVector& a,
  const gtsam::KeyVector& b,
  const gtsam::KeyVector& c,
  const gtsam::KeyVector& d = {}) {
  gtsam::KeyVector merged;
  merged.reserve(a.size() + b.size() + c.size() + d.size());
  merged.insert(merged.end(), a.begin(), a.end());
  merged.insert(merged.end(), b.begin(), b.end());
  merged.insert(merged.end(), c.begin(), c.end());
  merged.insert(merged.end(), d.begin(), d.end());
  return sort_unique_keys(std::move(merged));
}

std::unordered_set<gtsam::Key> to_key_set(const gtsam::KeyVector& keys) {
  return std::unordered_set<gtsam::Key>(keys.begin(), keys.end());
}

gtsam::KeyVector difference(const gtsam::KeyVector& lhs, const gtsam::KeyVector& rhs) {
  const auto rhs_set = to_key_set(rhs);
  gtsam::KeyVector diff;
  diff.reserve(lhs.size());
  for (const auto key : lhs) {
    if (!rhs_set.count(key)) {
      diff.push_back(key);
    }
  }
  return sort_unique_keys(std::move(diff));
}

gtsam::Values filter_values(const gtsam::Values& values, const std::unordered_set<gtsam::Key>& keep) {
  gtsam::Values filtered;
  for (const auto key : values.keys()) {
    if (!keep.count(key)) {
      continue;
    }
    filtered.insert(key, values.at(key));
  }
  return filtered;
}

gtsam::Values extract_subset(const gtsam::Values& values, const gtsam::KeyVector& keys) {
  gtsam::Values subset;
  for (const auto key : keys) {
    if (!values.exists(key)) {
      continue;
    }
    subset.insert(key, values.at(key));
  }
  return subset;
}

gtsam::Values calculate_incremental_estimate_with_retry(
  const gtsam_points::IncrementalFixedLagSmootherExt* smoother,
  std::string* solver_status = nullptr) {
  if (!smoother) {
    return {};
  }

  try {
    return smoother->calculateEstimate();
  } catch (const std::out_of_range&) {
    if (solver_status) {
      *solver_status = "ok_estimate_linearization_point_fallback";
    }
    return smoother->getLinearizationPoint();
  }
}

bool factor_keys_active(
  const gtsam::NonlinearFactor::shared_ptr& factor,
  const std::unordered_set<gtsam::Key>& keep) {
  if (!factor) {
    return false;
  }
  for (const auto key : factor->keys()) {
    if (!keep.count(key)) {
      return false;
    }
  }
  return true;
}

double elapsed_ms(const std::chrono::steady_clock::time_point& start,
                  const std::chrono::steady_clock::time_point& end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

std::string format_keys(const gtsam::KeyVector& keys) {
  std::ostringstream oss;
  bool first = true;
  for (const auto key : keys) {
    if (!first) {
      oss << ",";
    }
    first = false;
    oss << gtsam::DefaultKeyFormatter(key);
  }
  return oss.str();
}

gtsam::KeyVector find_missing_factor_keys(
  const gtsam::NonlinearFactorGraph& graph,
  const std::unordered_set<gtsam::Key>& known_keys) {
  gtsam::KeyVector missing;
  for (const auto& factor : graph) {
    if (!factor) {
      continue;
    }
    for (const auto key : factor->keys()) {
      if (!known_keys.count(key)) {
        missing.push_back(key);
      }
    }
  }
  return sort_unique_keys(std::move(missing));
}

gtsam::NonlinearFactorGraph filter_graph(
  const gtsam::NonlinearFactorGraph& graph,
  const std::unordered_set<gtsam::Key>& keep) {
  gtsam::NonlinearFactorGraph filtered;
  for (const auto& factor : graph) {
    if (factor_keys_active(factor, keep)) {
      filtered.push_back(factor);
    }
  }
  return filtered;
}

gtsam::KeyVector partition_keys_by_symbol(const gtsam::KeyVector& keys, char symbol_chr) {
  gtsam::KeyVector filtered;
  for (const auto key : keys) {
    if (gtsam::Symbol(key).chr() == symbol_chr) {
      filtered.push_back(key);
    }
  }
  return sort_unique_keys(std::move(filtered));
}

gtsam::KeyVector active_aux_keys_from(const gtsam::KeyVector& keys) {
  gtsam::KeyVector filtered;
  for (const auto key : keys) {
    const char chr = gtsam::Symbol(key).chr();
    if (chr == 'u' || chr == 'c') {
      filtered.push_back(key);
    }
  }
  return sort_unique_keys(std::move(filtered));
}

BSplineFactorFamilyCounts count_active_window_factors(
  const gtsam::NonlinearFactorGraph& graph,
  const gtsam::KeyVector& active_keys) {
  const auto active_key_set = to_key_set(active_keys);
  BSplineFactorFamilyCounts counts;
  for (const auto& factor : graph) {
    if (!factor_keys_active(factor, active_key_set)) {
      continue;
    }
    accumulate_bspline_factor_family_counts(counts, factor);
  }
  return counts;
}

BSplineFactorFamilyCounts count_factor_indices(
  const gtsam::NonlinearFactorGraph& graph,
  gtsam::FactorIndices factor_indices) {
  std::sort(factor_indices.begin(), factor_indices.end());
  factor_indices.erase(std::unique(factor_indices.begin(), factor_indices.end()), factor_indices.end());

  BSplineFactorFamilyCounts counts;
  for (const auto idx : factor_indices) {
    if (idx >= graph.size()) {
      continue;
    }
    accumulate_bspline_factor_family_counts(counts, graph[idx]);
  }
  return counts;
}

BSplineKeyFamilyCounts count_key_families(const gtsam::KeyVector& keys) {
  BSplineKeyFamilyCounts counts;
  for (const auto key : keys) {
    accumulate_bspline_key_family_counts(counts, key);
  }
  return counts;
}

BSplineKeyFamilyCounts count_relinearized_variable_families(const gtsam::ISAM2Result::DetailedResults::StatusMap& statuses) {
  BSplineKeyFamilyCounts counts;
  for (const auto& [key, status] : statuses) {
    if (!status.isRelinearized) {
      continue;
    }
    accumulate_bspline_key_family_counts(counts, key);
  }
  return counts;
}

}  // namespace

const char* to_string(BSplineUnifiedSolverMode mode) {
  switch (mode) {
    case BSplineUnifiedSolverMode::BATCH_LM:
      return "BATCH_LM";
    case BSplineUnifiedSolverMode::INCREMENTAL_SMOOTHER:
      return "INCREMENTAL_SMOOTHER";
  }
  return "UNKNOWN";
}

BatchLMSolver::BatchLMSolver(int max_iterations) : max_iterations_(max_iterations) {}

void BatchLMSolver::reset() {
  values_.clear();
  factors_ = gtsam::NonlinearFactorGraph();
  current_active_keys_.clear();
}

BSplineSolverResult BatchLMSolver::apply_delta(const BSplineGraphDelta& delta) {
  const gtsam::KeyVector next_active_keys =
    merge_keys(delta.active_pose_keys, delta.active_aux_keys, delta.persistent_keys);
  const auto keep = to_key_set(next_active_keys);
  const gtsam::KeyVector previous_active_keys = current_active_keys_;

  factors_ = filter_graph(factors_, keep);
  values_ = filter_values(values_, keep);

  for (const auto key : delta.new_values.keys()) {
    if (values_.exists(key)) {
      values_.update(key, delta.new_values.at(key));
    } else {
      values_.insert(key, delta.new_values.at(key));
    }
  }
  for (const auto& factor : delta.new_factors) {
    if (factor) {
      factors_.push_back(factor);
    }
  }

  BSplineSolverResult result;
  result.optimize_count = 1;
  result.used_incremental_solver = false;
  result.relinearized_variable_count = next_active_keys.size();
  result.reeliminated_variable_count = 0;
  result.retired_keys = difference(previous_active_keys, next_active_keys);
  result.active_pose_keys = partition_keys_by_symbol(next_active_keys, 's');
  result.active_aux_keys = active_aux_keys_from(next_active_keys);
  result.relinearized_factor_count = 0;
  result.linearized_factor_count = 0;
  result.bayes_tree_clique_count = 0;
  result.affected_variable_count = next_active_keys.size();
  result.observed_key_count = 0;
  result.new_factor_index_count = 0;
  result.current_nonlinear_factor_count = factors_.size();
  result.active_window_imu_factor_count = 0;
  result.active_window_velocity_factor_count = 0;
  result.active_window_lidar_factor_count = 0;
  result.active_window_lidar_current_segment_factor_count = 0;
  result.active_window_lidar_old_segment_factor_count = 0;
  result.active_window_prior_factor_count = 0;
  result.active_window_shared_jkg_touching_factor_count = 0;
  result.recalculated_imu_factor_count = 0;
  result.recalculated_velocity_factor_count = 0;
  result.recalculated_lidar_factor_count = 0;
  result.recalculated_lidar_current_segment_factor_count = 0;
  result.recalculated_lidar_old_segment_factor_count = 0;
  result.recalculated_lidar_same_support_factor_count = 0;
  result.recalculated_lidar_cross_support_factor_count = 0;
  result.recalculated_prior_factor_count = 0;
  result.recalculated_shared_jkg_touching_factor_count = 0;
  result.relinearized_pose_variable_count = 0;
  result.relinearized_aux_variable_count = 0;
  result.relinearized_shared_variable_count = 0;
  result.affected_pose_key_count = 0;
  result.affected_aux_key_count = 0;
  result.affected_shared_key_count = 0;
  result.isam_reported_update_ms = 0.0;
  result.iteration_count = 0;
  result.solver_status = "no_factors";

  if (!factors_.empty()) {
    gtsam_points::LevenbergMarquardtExtParams lm_params;
    lm_params.setlambdaInitial(1e-4);
    lm_params.setAbsoluteErrorTol(1e-2);
    lm_params.setMaxIterations(max_iterations_);
    const auto t_opt_start = std::chrono::steady_clock::now();
    result.initial_cost = factors_.error(values_);
    try {
      values_ = gtsam_points::LevenbergMarquardtOptimizerExt(factors_, values_, lm_params).optimize();
      result.solver_status = "ok";
    } catch (const std::exception&) {
      result.fallback_used = true;
      result.solver_status = "exception";
    }
    result.delta_solve_ms = elapsed_ms(t_opt_start, std::chrono::steady_clock::now());
    result.final_cost = factors_.empty() ? 0.0 : factors_.error(values_);
    result.relinearized_factor_count = factors_.size();
    result.linearized_factor_count = factors_.size();
    result.current_nonlinear_factor_count = factors_.size();
  }

  current_active_keys_ = next_active_keys;
  const auto active_window_counts = count_active_window_factors(factors_, current_active_keys_);
  result.active_window_imu_factor_count = active_window_counts.imu_factor_count;
  result.active_window_velocity_factor_count = active_window_counts.velocity_factor_count;
  result.active_window_lidar_factor_count = active_window_counts.lidar_factor_count;
  result.active_window_lidar_current_segment_factor_count = 0;
  result.active_window_lidar_old_segment_factor_count = 0;
  result.active_window_prior_factor_count = active_window_counts.prior_factor_count;
  result.active_window_shared_jkg_touching_factor_count = active_window_counts.shared_jkg_touching_factor_count;
  const auto t_estimate_start = std::chrono::steady_clock::now();
  result.estimate_subset = extract_subset(values_, merge_keys(
    delta.query_keys,
    delta.mirror_sync_keys,
    delta.persistent_keys));
  result.estimate_query_ms = elapsed_ms(t_estimate_start, std::chrono::steady_clock::now());
  return result;
}

gtsam::Values BatchLMSolver::estimate_subset(const gtsam::KeyVector& keys) const {
  return extract_subset(values_, sort_unique_keys(keys));
}

bool BatchLMSolver::has_key(gtsam::Key key) const {
  return values_.exists(key);
}

gtsam::KeyVector BatchLMSolver::current_active_keys() const {
  return current_active_keys_;
}

IncrementalSmootherSolver::IncrementalSmootherSolver(double smoother_lag, const gtsam::ISAM2Params& isam2_params)
: smoother_lag_(smoother_lag), isam2_params_(isam2_params) {
  reset();
}

void IncrementalSmootherSolver::reset() {
  smoother_ = std::make_unique<gtsam_points::IncrementalFixedLagSmootherExt>(smoother_lag_, isam2_params_);
  current_active_keys_.clear();
  lidar_factor_metadata_by_index_.clear();
}

BSplineSolverResult IncrementalSmootherSolver::apply_delta(const BSplineGraphDelta& delta) {
  BSplineSolverResult result;
  result.optimize_count = 1;
  result.used_incremental_solver = true;
  const gtsam::KeyVector previous_active_keys = current_active_keys_;
  const gtsam::KeyVector active_window_keys =
    merge_keys(delta.active_pose_keys, delta.active_aux_keys, delta.persistent_keys);

  auto known_keys = to_key_set(smoother_->getLinearizationPoint().keys());
  for (const auto key : delta.new_values.keys()) {
    known_keys.insert(key);
  }
  const auto missing_factor_keys = find_missing_factor_keys(delta.new_factors, known_keys);
  if (!missing_factor_keys.empty()) {
    throw std::runtime_error(
      "Incremental delta references keys missing from solver/new_values: " + format_keys(missing_factor_keys));
  }

  const auto t_update_start = std::chrono::steady_clock::now();
  smoother_->update(delta.new_factors, delta.new_values, delta.new_stamps);
  result.delta_solve_ms = elapsed_ms(t_update_start, std::chrono::steady_clock::now());
  result.fallback_used = false;
  result.solver_status = "ok";
  const auto& isam_result = smoother_->getISAM2Result();
  const auto& isam_result_ext = smoother_->getISAM2ResultExt();
  result.initial_cost = isam_result.errorBefore.value_or(0.0);
  result.final_cost = isam_result.errorAfter.value_or(0.0);
  result.relinearized_variable_count = isam_result.variablesRelinearized;
  result.reeliminated_variable_count = isam_result.variablesReeliminated;
  result.relinearized_factor_count = isam_result.factorsRecalculated;
  result.linearized_factor_count = smoother_->getLinearFactors().size();
  result.bayes_tree_clique_count = isam_result.cliques;
  result.affected_variable_count = isam_result.markedKeys.size();
  result.observed_key_count = isam_result.observedKeys.size();
  result.new_factor_index_count = isam_result.newFactorsIndices.size();
  result.current_nonlinear_factor_count = smoother_->getFactors().size();
  result.active_window_imu_factor_count = 0;
  result.active_window_velocity_factor_count = 0;
  result.active_window_lidar_factor_count = 0;
  result.active_window_lidar_current_segment_factor_count = 0;
  result.active_window_lidar_old_segment_factor_count = 0;
  result.active_window_prior_factor_count = 0;
  result.active_window_shared_jkg_touching_factor_count = 0;
  result.recalculated_imu_factor_count = 0;
  result.recalculated_velocity_factor_count = 0;
  result.recalculated_lidar_factor_count = 0;
  result.recalculated_lidar_current_segment_factor_count = 0;
  result.recalculated_lidar_old_segment_factor_count = 0;
  result.recalculated_lidar_same_support_factor_count = 0;
  result.recalculated_lidar_cross_support_factor_count = 0;
  result.recalculated_prior_factor_count = 0;
  result.recalculated_shared_jkg_touching_factor_count = 0;
  result.relinearized_pose_variable_count = 0;
  result.relinearized_aux_variable_count = 0;
  result.relinearized_shared_variable_count = 0;
  result.affected_pose_key_count = 0;
  result.affected_aux_key_count = 0;
  result.affected_shared_key_count = 0;
  result.isam_reported_update_ms = isam_result_ext.elapsed_time * 1000.0;
  result.iteration_count = 0;

  const auto t_estimate_start = std::chrono::steady_clock::now();
  const gtsam::Values all_values = calculate_incremental_estimate_with_retry(smoother_.get(), &result.solver_status);
  result.estimate_query_ms = elapsed_ms(t_estimate_start, std::chrono::steady_clock::now());
  current_active_keys_ = sort_unique_keys(smoother_->getLinearizationPoint().keys());
  register_lidar_factor_metadata(
    lidar_factor_metadata_by_index_,
    isam_result.newFactorsIndices,
    delta.new_factors,
    delta.lidar_factor_metadata);
  const auto active_window_counts = count_active_window_factors(smoother_->getFactors(), active_window_keys);
  result.active_window_imu_factor_count = active_window_counts.imu_factor_count;
  result.active_window_velocity_factor_count = active_window_counts.velocity_factor_count;
  result.active_window_lidar_factor_count = active_window_counts.lidar_factor_count;
  const auto active_window_lidar_counts = count_active_window_lidar_factors(
    smoother_->getFactors(),
    active_window_keys,
    lidar_factor_metadata_by_index_,
    delta.current_lidar_source_frame_index,
    delta.current_lidar_support_control_indices);
  result.active_window_lidar_current_segment_factor_count =
    active_window_lidar_counts.current_segment_factor_count;
  result.active_window_lidar_old_segment_factor_count =
    active_window_lidar_counts.old_segment_factor_count;
  result.active_window_prior_factor_count = active_window_counts.prior_factor_count;
  result.active_window_shared_jkg_touching_factor_count = active_window_counts.shared_jkg_touching_factor_count;
  const auto recalculated_counts = count_factor_indices(smoother_->getFactors(), isam_result_ext.recalculatedFactorIndices);
  result.recalculated_imu_factor_count = recalculated_counts.imu_factor_count;
  result.recalculated_velocity_factor_count = recalculated_counts.velocity_factor_count;
  result.recalculated_lidar_factor_count = recalculated_counts.lidar_factor_count;
  const auto recalculated_lidar_counts = count_lidar_factor_indices(
    smoother_->getFactors(),
    isam_result_ext.recalculatedFactorIndices,
    lidar_factor_metadata_by_index_,
    delta.current_lidar_source_frame_index,
    delta.current_lidar_support_control_indices);
  result.recalculated_lidar_current_segment_factor_count =
    recalculated_lidar_counts.current_segment_factor_count;
  result.recalculated_lidar_old_segment_factor_count =
    recalculated_lidar_counts.old_segment_factor_count;
  result.recalculated_lidar_same_support_factor_count =
    recalculated_lidar_counts.same_support_factor_count;
  result.recalculated_lidar_cross_support_factor_count =
    recalculated_lidar_counts.cross_support_factor_count;
  result.recalculated_prior_factor_count = recalculated_counts.prior_factor_count;
  result.recalculated_shared_jkg_touching_factor_count = recalculated_counts.shared_jkg_touching_factor_count;
  const auto affected_counts = count_key_families(gtsam::KeyVector(isam_result.markedKeys.begin(), isam_result.markedKeys.end()));
  result.affected_pose_key_count = affected_counts.pose_key_count;
  result.affected_aux_key_count = affected_counts.aux_key_count;
  result.affected_shared_key_count = affected_counts.shared_key_count;
  if (isam_result.detail) {
    const auto relinearized_counts = count_relinearized_variable_families(isam_result.detail->variableStatus);
    result.relinearized_pose_variable_count = relinearized_counts.pose_key_count;
    result.relinearized_aux_variable_count = relinearized_counts.aux_key_count;
    result.relinearized_shared_variable_count = relinearized_counts.shared_key_count;
  }
  result.retired_keys = difference(previous_active_keys, current_active_keys_);
  result.active_pose_keys = partition_keys_by_symbol(current_active_keys_, 's');
  result.active_aux_keys = active_aux_keys_from(current_active_keys_);
  result.estimate_subset = extract_subset(all_values, merge_keys(
    delta.query_keys,
    delta.mirror_sync_keys,
    delta.persistent_keys));
  return result;
}

gtsam::Values IncrementalSmootherSolver::estimate_subset(const gtsam::KeyVector& keys) const {
  if (!smoother_) {
    return {};
  }
  return extract_subset(calculate_incremental_estimate_with_retry(smoother_.get()), sort_unique_keys(keys));
}

bool IncrementalSmootherSolver::has_key(gtsam::Key key) const {
  if (!smoother_) {
    return false;
  }
  return smoother_->getLinearizationPoint().exists(key);
}

gtsam::KeyVector IncrementalSmootherSolver::current_active_keys() const {
  return current_active_keys_;
}

}  // namespace iap
