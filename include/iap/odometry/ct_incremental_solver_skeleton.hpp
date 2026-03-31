#pragma once
// IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410:
// Explicit local-domain lifecycle helper for the continuous-time incremental
// solver refactor. It tracks the persistent key/segment add-remove surface used
// by the authoritative KERNEL fixed-lag solver owner.

#include <iap/odometry/bspline_fixed_lag_registry.hpp>
#include <iap/odometry/ct_solve_domain.hpp>

#include <gtsam/nonlinear/FixedLagSmoother.h>
#include <gtsam/nonlinear/Values.h>

#include <cstddef>
#include <vector>

namespace iap {

struct CTSolverLifecycleDelta {
  std::vector<std::size_t> active_segment_ordinals;
  std::vector<std::size_t> newly_active_segment_ordinals;
  std::vector<std::size_t> retired_segment_ordinals;
  std::vector<std::size_t> active_segment_ids;
  std::vector<std::size_t> newly_active_segment_ids;
  std::vector<std::size_t> retired_segment_ids;
  std::vector<gtsam::Key> active_control_keys;
  std::vector<gtsam::Key> active_auxiliary_keys;
  std::vector<gtsam::Key> active_shared_keys;
  std::vector<gtsam::Key> active_keys;
  std::vector<gtsam::Key> new_keys;
  std::vector<gtsam::Key> retired_keys;
  gtsam::Values new_values;
  gtsam::FixedLagSmootherKeyTimestampMap new_stamps;

  bool empty() const { return active_segment_ordinals.empty() && active_keys.empty(); }
};

class BSplineIncrementalSolverSkeleton {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void reset();

  CTSolverLifecycleDelta prepare_update(
    const BSplineSolveDomain& domain,
    const std::vector<BSplineControlPointState>& control_states,
    const BSplineFixedLagSharedState& shared_state,
    const gtsam::Values& authoritative_values,
    double current_stamp);

  const CTSolverLifecycleDelta& last_delta() const { return last_delta_; }

 private:
  std::vector<std::size_t> last_active_segment_ordinals_;
  std::vector<std::size_t> last_active_segment_ids_;
  std::vector<gtsam::Key> known_keys_;
  CTSolverLifecycleDelta last_delta_;
};

}  // namespace iap
