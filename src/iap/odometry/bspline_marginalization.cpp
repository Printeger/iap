#include <iap/odometry/bspline_marginalization.hpp>

#include <algorithm>
#include <stdexcept>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

namespace iap {

namespace {

void append_unique_key(std::vector<gtsam::Key>& keys, gtsam::Key key) {
  if (std::find(keys.begin(), keys.end(), key) == keys.end()) {
    keys.push_back(key);
  }
}

std::vector<BSplineControlPointState> prune_buffer_states_copy(
  const std::vector<BSplineControlPointState>& buffer_states,
  double min_stamp) {
  auto states = buffer_states;
  if (states.empty()) {
    return states;
  }

  const auto first_active = std::find_if(states.begin(), states.end(), [&](const auto& state) {
    return state.stamp >= min_stamp;
  });

  if (first_active == states.end()) {
    if (states.size() > kBSplineControlPointCount) {
      states.erase(states.begin(), states.end() - static_cast<std::ptrdiff_t>(kBSplineControlPointCount));
    }
    return states;
  }

  const auto active_offset = std::distance(states.begin(), first_active);
  const auto support = std::min<std::ptrdiff_t>(
    active_offset,
    static_cast<std::ptrdiff_t>(kBSplineControlPointCount - 1));
  const auto keep_begin = first_active - support;

  if (keep_begin > states.begin()) {
    states.erase(states.begin(), keep_begin);
  }

  return states;
}

bool has_index(const std::vector<std::size_t>& indices, std::size_t index) {
  return std::find(indices.begin(), indices.end(), index) != indices.end();
}

void insert_bspline_value(gtsam::Values& dst, const gtsam::Values& src, gtsam::Key key) {
  const char c = gtsam::Symbol(key).chr();
  switch (c) {
    case 's':
      dst.insert(key, src.at<gtsam::Pose3>(key));
      return;
    case 'u':
    case 'g':
    case 'j':
    case 'k':
    case 'e':
      dst.insert(key, src.at<gtsam::Vector3>(key));
      return;
    case 'c':
      dst.insert(key, src.at<gtsam::Vector2>(key));
      return;
    case 'r':
      dst.insert(key, src.at<gtsam::Rot3>(key));
      return;
    default:
      throw std::runtime_error(std::string("unsupported bspline marginal key '") + c + "'");
  }
}

void append_cloned_gaussian_factors(
  gtsam::GaussianFactorGraph& dst,
  const gtsam::GaussianFactorGraph& src) {
  for (const auto& factor : src) {
    if (!factor) {
      continue;
    }
    dst.add(factor->clone());
  }
}

gtsam::GaussianFactorGraph relinearize_carried_prior(
  const BSplineCarriedPrior& prior,
  const gtsam::Values& values) {
  gtsam::GaussianFactorGraph linearized;
  if (prior.empty()) {
    return linearized;
  }

  const auto nonlinear_prior = prior.replay();
  const auto relinearized = nonlinear_prior.linearize(values);
  append_cloned_gaussian_factors(linearized, *relinearized);
  return linearized;
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

}  // namespace

bool BSplineMarginalizationPartition::FactorOwnershipInfo::is_survivor_only() const {
  return ownership == FactorOwnership::SurvivorOnly;
}

bool BSplineMarginalizationPartition::FactorOwnershipInfo::should_marginalize() const {
  return ownership == FactorOwnership::Removable;
}

bool BSplineMarginalizationPartition::contains_survivor(gtsam::Key key) const {
  return std::find(survivor_keys.begin(), survivor_keys.end(), key) != survivor_keys.end();
}

bool BSplineMarginalizationPartition::contains_removed(gtsam::Key key) const {
  return std::find(removable_keys.begin(), removable_keys.end(), key) != removable_keys.end();
}

BSplineMarginalizationPartition::FactorOwnershipInfo
BSplineMarginalizationPartition::classify_factor(const gtsam::KeyVector& factor_keys) const {
  FactorOwnershipInfo info;

  for (const auto key : factor_keys) {
    if (contains_removed(key)) {
      append_unique_key(info.removable_factor_keys, key);
      continue;
    }
    if (contains_survivor(key)) {
      append_unique_key(info.survivor_factor_keys, key);
      continue;
    }
    append_unique_key(info.foreign_keys, key);
  }

  if (!info.foreign_keys.empty()) {
    info.ownership = FactorOwnership::Foreign;
  } else if (!info.removable_factor_keys.empty()) {
    info.ownership = FactorOwnership::Removable;
  } else {
    info.ownership = FactorOwnership::SurvivorOnly;
  }

  return info;
}

bool BSplineMarginalizationPartition::should_marginalize_factor(const gtsam::KeyVector& factor_keys) const {
  return classify_factor(factor_keys).should_marginalize();
}

bool BSplineMarginalizationPartition::can_replay_keys(
  const std::vector<gtsam::Key>& keys,
  const gtsam::Values& values) const {
  if (keys.empty()) {
    return false;
  }

  const auto ownership = classify_factor(gtsam::KeyVector(keys.begin(), keys.end()));
  if (!ownership.is_survivor_only()) {
    return false;
  }

  return std::all_of(keys.begin(), keys.end(), [&](gtsam::Key key) { return values.exists(key); });
}

bool BSplineCarriedPrior::empty() const {
  return linear_graph.size() == 0 || retained_keys.empty();
}

gtsam::NonlinearFactorGraph BSplineCarriedPrior::replay() const {
  if (empty()) {
    return gtsam::NonlinearFactorGraph();
  }
  return gtsam::LinearContainerFactor::ConvertLinearGraph(linear_graph, linearization_point);
}

BSplineMarginalizationPartition build_bspline_marginalization_partition(
  const std::vector<BSplineControlPointState>& buffer_states,
  const std::vector<BSplineMarginalizationSegmentState>& segment_states,
  const gtsam::Values& values,
  double min_active_stamp,
  bool include_clock) {
  BSplineMarginalizationPartition partition;
  partition.min_active_stamp = min_active_stamp;

  const auto kept_states = prune_buffer_states_copy(buffer_states, min_active_stamp);
  partition.survivor_control_indices.reserve(kept_states.size());
  for (const auto& state : kept_states) {
    partition.survivor_control_indices.push_back(state.index);
    if (values.exists(bspline_control_point_key(state.index))) {
      append_unique_key(partition.survivor_keys, bspline_control_point_key(state.index));
    }
  }

  for (const auto& segment : segment_states) {
    if (segment.scan_end < min_active_stamp) {
      continue;
    }

    if (!has_index(partition.survivor_auxiliary_indices, segment.auxiliary_index)) {
      partition.survivor_auxiliary_indices.push_back(segment.auxiliary_index);
    }

    const gtsam::Key velocity_key = bspline_velocity_key(segment.auxiliary_index);
    if (values.exists(velocity_key)) {
      append_unique_key(partition.survivor_keys, velocity_key);
    }

    const gtsam::Key clock_key = bspline_clock_key(segment.auxiliary_index);
    if (include_clock && values.exists(clock_key)) {
      append_unique_key(partition.survivor_keys, clock_key);
    }
  }

  const std::array<gtsam::Key, 5> persistent_keys = {
    bspline_gyro_bias_key(),
    bspline_accel_bias_key(),
    bspline_gravity_key(),
    bspline_ecef_origin_key(),
    bspline_ecef_rot_key(),
  };
  for (const auto key : persistent_keys) {
    if (values.exists(key)) {
      append_unique_key(partition.survivor_keys, key);
    }
  }

  for (const auto& state : buffer_states) {
    if (has_index(partition.survivor_control_indices, state.index)) {
      continue;
    }
    if (!has_index(partition.removable_control_indices, state.index)) {
      partition.removable_control_indices.push_back(state.index);
    }
    if (values.exists(bspline_control_point_key(state.index))) {
      append_unique_key(partition.removable_keys, bspline_control_point_key(state.index));
    }
  }

  for (const auto& segment : segment_states) {
    if (segment.scan_end >= min_active_stamp || has_index(partition.survivor_auxiliary_indices, segment.auxiliary_index)) {
      continue;
    }

    if (!has_index(partition.removable_auxiliary_indices, segment.auxiliary_index)) {
      partition.removable_auxiliary_indices.push_back(segment.auxiliary_index);
    }

    const gtsam::Key velocity_key = bspline_velocity_key(segment.auxiliary_index);
    if (values.exists(velocity_key)) {
      append_unique_key(partition.removable_keys, velocity_key);
    }

    const gtsam::Key clock_key = bspline_clock_key(segment.auxiliary_index);
    if (include_clock && values.exists(clock_key)) {
      append_unique_key(partition.removable_keys, clock_key);
    }
  }

  return partition;
}

BSplineCarriedPrior build_bspline_carried_prior(
  const gtsam::NonlinearFactorGraph& removable_graph,
  const gtsam::Values& values,
  const std::vector<gtsam::Key>& survivor_keys,
  const BSplineCarriedPrior* previous_prior) {
  BSplineCarriedPrior carried_prior;
  if (survivor_keys.empty()) {
    return carried_prior;
  }

  gtsam::GaussianFactorGraph combined_linear_graph;
  if (previous_prior && !previous_prior->empty()) {
    append_cloned_gaussian_factors(combined_linear_graph, relinearize_carried_prior(*previous_prior, values));
  }
  if (removable_graph.size() != 0) {
    const auto linear_graph = removable_graph.linearize(values);
    append_cloned_gaussian_factors(combined_linear_graph, *linear_graph);
  }
  if (combined_linear_graph.size() == 0) {
    return carried_prior;
  }

  const auto linear_keys = combined_linear_graph.keys();

  std::vector<gtsam::Key> retained;
  retained.reserve(survivor_keys.size());
  for (const auto key : survivor_keys) {
    if (linear_keys.exists(key)) {
      retained.push_back(key);
    }
  }

  if (retained.empty()) {
    return carried_prior;
  }

  gtsam::Values linearization_point;
  for (const auto key : retained) {
    insert_bspline_value(linearization_point, values, key);
  }

  const auto marginal_graph = combined_linear_graph.marginal(retained);
  carried_prior.retained_keys = retained;
  carried_prior.linearization_point = linearization_point;
  carried_prior.linear_graph = *marginal_graph;
  return carried_prior;
}

}  // namespace iap
