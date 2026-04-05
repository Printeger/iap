#pragma once

#include <iap/odometry/spline_state_layout.hpp>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_ext.hpp>

#include <cstddef>
#include <memory>
#include <string>

namespace iap {

enum class BSplineUnifiedSolverMode {
  BATCH_LM,
  INCREMENTAL_SMOOTHER,
};

const char* to_string(BSplineUnifiedSolverMode mode);

struct BSplineGraphDelta {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  gtsam::FixedLagSmootherKeyTimestampMap new_stamps;
  gtsam::KeyVector query_keys;
  gtsam::KeyVector mirror_sync_keys;
  gtsam::KeyVector active_pose_keys;
  gtsam::KeyVector active_aux_keys;
  gtsam::KeyVector persistent_keys;
  std::shared_ptr<const SplineStateLayout> layout;
  double current_frame_stamp{0.0};
  double min_active_stamp{0.0};
  std::size_t current_auxiliary_index{0};
  bool local_layer_enabled{false};
  bool navigation_layer_enabled{false};
  std::size_t local_layer_factor_count{0};
  std::size_t navigation_layer_factor_count{0};
};

struct BSplineSolverResult {
  gtsam::Values estimate_subset;
  gtsam::KeyVector retired_keys;
  gtsam::KeyVector active_pose_keys;
  gtsam::KeyVector active_aux_keys;
  double initial_cost{0.0};
  double final_cost{0.0};
  int optimize_count{0};
  bool fallback_used{false};
  bool used_incremental_solver{false};
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
  std::size_t active_window_prior_factor_count{0};
  std::size_t active_window_shared_jkg_touching_factor_count{0};
  double isam_reported_update_ms{0.0};
  int iteration_count{0};
  std::string solver_status{"unavailable"};
};

class IBSplineGraphSolver {
 public:
  virtual ~IBSplineGraphSolver() = default;

  virtual void reset() = 0;
  virtual BSplineSolverResult apply_delta(const BSplineGraphDelta& delta) = 0;
  virtual gtsam::Values estimate_subset(const gtsam::KeyVector& keys) const = 0;
  virtual bool has_key(gtsam::Key key) const = 0;
  virtual gtsam::KeyVector current_active_keys() const = 0;
  virtual bool using_incremental() const = 0;
};

class BatchLMSolver : public IBSplineGraphSolver {
 public:
  explicit BatchLMSolver(int max_iterations);

  void reset() override;
  BSplineSolverResult apply_delta(const BSplineGraphDelta& delta) override;
  gtsam::Values estimate_subset(const gtsam::KeyVector& keys) const override;
  bool has_key(gtsam::Key key) const override;
  gtsam::KeyVector current_active_keys() const override;
  bool using_incremental() const override { return false; }

 private:
  int max_iterations_{8};
  gtsam::Values values_;
  gtsam::NonlinearFactorGraph factors_;
  gtsam::KeyVector current_active_keys_;
};

class IncrementalSmootherSolver : public IBSplineGraphSolver {
 public:
  IncrementalSmootherSolver(double smoother_lag, const gtsam::ISAM2Params& isam2_params);

  void reset() override;
  BSplineSolverResult apply_delta(const BSplineGraphDelta& delta) override;
  gtsam::Values estimate_subset(const gtsam::KeyVector& keys) const override;
  bool has_key(gtsam::Key key) const override;
  gtsam::KeyVector current_active_keys() const override;
  bool using_incremental() const override { return true; }

private:
  double smoother_lag_{0.0};
  gtsam::ISAM2Params isam2_params_;
  std::unique_ptr<gtsam_points::IncrementalFixedLagSmootherExt> smoother_;
  gtsam::KeyVector current_active_keys_;
};

}  // namespace iap
