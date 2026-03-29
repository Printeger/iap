#pragma once
// IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410:
// Utilities for partitioning fixed-lag B-spline states into survivor/removable
// sets and converting removable-factor subgraphs into replayable carried priors.

#include <iap/odometry/bspline_control_window.hpp>

#include <array>
#include <cstddef>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <vector>

namespace iap {

struct BSplineMarginalizationSegmentState {
  double scan_end = 0.0;
  std::array<std::size_t, kBSplineControlPointCount> control_indices{};
  std::size_t auxiliary_index = 0;
};

struct BSplineMarginalizationPartition {
  double min_active_stamp = 0.0;
  std::vector<std::size_t> survivor_control_indices;
  std::vector<std::size_t> removable_control_indices;
  std::vector<std::size_t> survivor_auxiliary_indices;
  std::vector<std::size_t> removable_auxiliary_indices;
  std::vector<gtsam::Key> survivor_keys;
  std::vector<gtsam::Key> removable_keys;

  bool contains_survivor(gtsam::Key key) const;
  bool contains_removed(gtsam::Key key) const;
  bool should_marginalize_factor(const gtsam::KeyVector& factor_keys) const;
};

BSplineMarginalizationPartition build_bspline_marginalization_partition(
  const std::vector<BSplineControlPointState>& buffer_states,
  const std::vector<BSplineMarginalizationSegmentState>& segment_states,
  const gtsam::Values& values,
  double min_active_stamp,
  bool include_clock);

gtsam::NonlinearFactorGraph build_bspline_carried_prior(
  const gtsam::NonlinearFactorGraph& removable_graph,
  const gtsam::Values& values,
  const std::vector<gtsam::Key>& survivor_keys,
  std::vector<gtsam::Key>* retained_keys = nullptr);

}  // namespace iap
