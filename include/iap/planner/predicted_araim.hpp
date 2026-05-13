#pragma once
// IAP-RQ-331: GNSS advisory PL proxy along candidate trajectory.

#include <iap/integrity/araim.hpp>
#include <iap/gnss/gnss_types.hpp>
#include <iap/gnss/visibility_predictor.hpp>
#include <iap/map/local_occupancy.hpp>
#include <Eigen/Core>
#include <string>

namespace iap {

// Geometry-only GNSS advisory proxy result. Despite the legacy type name, this
// is not a certified/current monitor ARAIM output.
struct PredictedAraimResult {
  bool valid = false;
  bool fallback = true;
  std::string fallback_reason = "not_evaluated";

  double hpl = 1e9;
  double vpl = 1e9;
  double pl_scalar = 1e9;
  double pl_e = 1e9;
  double pl_n = 1e9;
  double pl_u = 1e9;
  double pl_ff_h = 1e9;
  double pl_ff_v = 1e9;
  double sigma_h = 1e9;
  double sigma_v = 1e9;
  double pdop = 1e9;
  int n_vis = 0;
  int n_hypotheses = 0;
};

/**
 * @brief Computes a GNSS advisory PL proxy (GII) for planning.
 *
 * Workflow per waypoint (IAP-RQ-331):
 * 1. Call VisibilityPredictor::predict(pos, *epoch) → VisibilityResult
 * 2. Extract visible satellites with their σ_eff values
 * 3. Build SatGeometry list → call Araim::predict_geometry()
 * 4. Return non-certified advisory HPL/VPL proxy values.
 *
 * When no epoch is available, returns a conservative fallback value
 * (params.fallback_pl). When no occupancy grid is set, prediction assumes
 * open sky and still evaluates GNSS geometry. These outputs are advisory and
 * must not be used as certified monitor PL.
 */
class PredictedAraimComputer {
 public:
  struct Params {
    Araim::Params             araim_params;   ///< K_fa, K_md, K_ff, etc.
    VisibilityPredictor::Params vis_params;   ///< elevation mask, occ range, canopy
    double fallback_pl = 5.0;  ///< advisory proxy when GNSS epoch unavailable [m]
  };

  PredictedAraimComputer();
  explicit PredictedAraimComputer(const Params& p);

  /// Set the local occupancy grid for ray-cast visibility.  Pass nullptr to disable.
  void set_occupancy(const LocalOccupancyGrid* grid);

  /// Set the current GNSS epoch (satellite geometry source).  Pass nullptr to disable.
  void set_epoch(const GnssEpoch* epoch);

  /**
   * @brief Predict GNSS advisory PL proxy at a single world-frame position.
   *
   * @return advisory HPL proxy [m]; returns fallback_pl when unavailable.
   */
  double predict_araim_pl(const Eigen::Vector3d& pos_world) const;

  /**
   * @brief Predict GNSS advisory HPL/VPL proxy and debug fields.
   */
  PredictedAraimResult predict_araim_result(
      const Eigen::Vector3d& pos_world) const;

  const Params& params() const { return params_; }

 private:
  Params              params_;
  Araim               araim_;
  VisibilityPredictor vis_;
  const LocalOccupancyGrid* grid_  = nullptr;
  const GnssEpoch*          epoch_ = nullptr;
};

}  // namespace iap
