#pragma once
// IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410:
// Utilities for partitioning fixed-lag B-spline states into survivor/removable
// sets and converting removable-factor subgraphs into replayable carried priors.

#include <iap/odometry/bspline_control_window.hpp>

#include <array>
#include <cstddef>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <vector>

namespace iap {

struct BSplineMarginalizationSegmentState {
  double scan_end = 0.0;
  int span_begin_idx = -1;
  int span_end_idx = -1;
  std::vector<std::size_t> active_control_indices;
  std::array<std::size_t, kBSplineControlPointCount> control_indices{};
  std::size_t auxiliary_index = 0;
};

struct SplineActiveStateSet {
  double min_active_stamp = 0.0;
  std::vector<int> active_span_indices;
  std::vector<std::size_t> active_control_indices;
  std::vector<std::size_t> removable_control_indices;
  std::vector<std::size_t> active_auxiliary_indices;
  std::vector<std::size_t> removable_auxiliary_indices;
  std::vector<gtsam::Key> active_pose_keys;
  std::vector<gtsam::Key> active_aux_keys;
  std::vector<gtsam::Key> removable_keys;

  std::vector<gtsam::Key> active_keys() const;
  bool contains_active_key(gtsam::Key key) const;
  bool contains_removed_key(gtsam::Key key) const;
};

struct BSplineMarginalizationPartition {
  double min_active_stamp = 0.0;
  SplineActiveStateSet active_state_set;
  std::vector<std::size_t> survivor_control_indices;
  std::vector<std::size_t> removable_control_indices;
  std::vector<std::size_t> survivor_auxiliary_indices;
  std::vector<std::size_t> removable_auxiliary_indices;
  std::vector<gtsam::Key> survivor_keys;
  std::vector<gtsam::Key> removable_keys;

  enum class FactorOwnership {
    SurvivorOnly,
    Removable,
    Foreign,
  };

  struct FactorOwnershipInfo {
    FactorOwnership ownership = FactorOwnership::Foreign;
    std::vector<gtsam::Key> survivor_factor_keys;
    std::vector<gtsam::Key> removable_factor_keys;
    std::vector<gtsam::Key> foreign_keys;

    bool is_survivor_only() const;
    bool should_marginalize() const;
  };

  bool contains_survivor(gtsam::Key key) const;
  bool contains_removed(gtsam::Key key) const;
  FactorOwnershipInfo classify_factor(const gtsam::KeyVector& factor_keys) const;
  bool should_marginalize_factor(const gtsam::KeyVector& factor_keys) const;
  bool can_replay_keys(const std::vector<gtsam::Key>& keys, const gtsam::Values& values) const;
};

struct BSplineCarriedPrior {
  std::vector<gtsam::Key> retained_keys;
  gtsam::Values linearization_point;
  gtsam::GaussianFactorGraph linear_graph;

  bool empty() const;
  gtsam::NonlinearFactorGraph replay() const;
};

SplineActiveStateSet build_spline_active_state_set(
  const std::vector<BSplineControlPointState>& buffer_states,
  const std::vector<BSplineMarginalizationSegmentState>& segment_states,
  const gtsam::Values& values,
  double min_active_stamp,
  bool include_clock);

BSplineMarginalizationPartition build_bspline_marginalization_partition(
  const SplineActiveStateSet& active_state_set);

BSplineMarginalizationPartition build_bspline_marginalization_partition(
  const std::vector<BSplineControlPointState>& buffer_states,
  const std::vector<BSplineMarginalizationSegmentState>& segment_states,
  const gtsam::Values& values,
  double min_active_stamp,
  bool include_clock);

BSplineCarriedPrior build_bspline_carried_prior(
  const gtsam::NonlinearFactorGraph& removable_graph,
  const gtsam::Values& values,
  const std::vector<gtsam::Key>& survivor_keys,
  const BSplineCarriedPrior* previous_prior = nullptr);

}  // namespace iap
