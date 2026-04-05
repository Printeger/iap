#include <iap/odometry/bspline_graph_solver.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

#include <algorithm>
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
  result.retired_keys = difference(previous_active_keys, next_active_keys);
  result.active_pose_keys = partition_keys_by_symbol(next_active_keys, 's');
  result.active_aux_keys = active_aux_keys_from(next_active_keys);

  if (!factors_.empty()) {
    gtsam_points::LevenbergMarquardtExtParams lm_params;
    lm_params.setlambdaInitial(1e-4);
    lm_params.setAbsoluteErrorTol(1e-2);
    lm_params.setMaxIterations(max_iterations_);
    result.initial_cost = factors_.error(values_);
    try {
      values_ = gtsam_points::LevenbergMarquardtOptimizerExt(factors_, values_, lm_params).optimize();
    } catch (const std::exception&) {
      result.fallback_used = true;
    }
    result.final_cost = factors_.empty() ? 0.0 : factors_.error(values_);
  }

  current_active_keys_ = next_active_keys;
  result.estimate_subset = extract_subset(values_, merge_keys(
    delta.query_keys,
    result.active_pose_keys,
    result.active_aux_keys,
    delta.persistent_keys));
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
}

BSplineSolverResult IncrementalSmootherSolver::apply_delta(const BSplineGraphDelta& delta) {
  BSplineSolverResult result;
  result.optimize_count = 1;
  result.used_incremental_solver = true;
  const gtsam::KeyVector previous_active_keys = current_active_keys_;

  smoother_->update(delta.new_factors, delta.new_values, delta.new_stamps);
  result.fallback_used = false;

  const gtsam::Values all_values = smoother_->calculateEstimate();
  current_active_keys_ = sort_unique_keys(smoother_->getLinearizationPoint().keys());
  result.retired_keys = difference(previous_active_keys, current_active_keys_);
  result.active_pose_keys = partition_keys_by_symbol(current_active_keys_, 's');
  result.active_aux_keys = active_aux_keys_from(current_active_keys_);
  result.estimate_subset = extract_subset(all_values, merge_keys(
    delta.query_keys,
    result.active_pose_keys,
    result.active_aux_keys,
    delta.persistent_keys));
  return result;
}

gtsam::Values IncrementalSmootherSolver::estimate_subset(const gtsam::KeyVector& keys) const {
  if (!smoother_) {
    return {};
  }
  return extract_subset(smoother_->calculateEstimate(), sort_unique_keys(keys));
}

bool IncrementalSmootherSolver::has_key(gtsam::Key key) const {
  if (!smoother_) {
    return false;
  }

  try {
    (void)smoother_->calculateEstimate(key);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

gtsam::KeyVector IncrementalSmootherSolver::current_active_keys() const {
  return current_active_keys_;
}

}  // namespace iap
