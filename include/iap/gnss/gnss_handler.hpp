#pragma once
// IAP-RQ-020: GNSS handler — bridges raw GNSS epochs to factor graph

#include <iap/gnss/gnss_types.hpp>
#include <iap/gnss/pseudorange_factor.hpp>
#include <iap/gnss/doppler_factor.hpp>
#include <iap/gnss/canopy_noise_model.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <Eigen/Core>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>

namespace iap {

/**
 * @brief Thread-safe buffer and factory for GNSS factors.
 *
 * Typical usage in the odometry estimator (IAP-RQ-020):
 * @code
 *   // On ROS callback thread:
 *   gnss_handler->insert_epoch(epoch);
 *
 *   // In on_smoother_update_(), after IMU pre-integration:
 *   auto gnss_factors = gnss_handler->get_factors(current_idx, frame_stamp,
 *                                                  origin_ecef);
 *   new_factors.add(gnss_factors);
 * @endcode
 *
 * Each call to get_factors() drains all epochs that fall within
 * `time_tolerance` seconds of `frame_stamp` and produces one
 * PseudorangeFactor + one DopplerFactor per non-excluded satellite.
 *
 * Satellite state (sat_pos, sat_vel) is stored in ECEF.  The receiver ECEF
 * position is reconstructed inside each factor as:
 *   P_ecef = R(0) * pose.translation() + E(0)
 * where E(0) and R(0) are shared factor-graph variables.
 */
class GnssHandler {
 public:
  struct Params {
    double pr_noise_base   = 5.0;   ///< base pseudorange noise [m]
    double dop_noise_base  = 0.5;   ///< base Doppler noise [m/s]
    double elev_noise_exp  = 2.0;   ///< exponent for elevation-dependent weighting
    double time_tolerance  = 0.1;   ///< epoch window around frame stamp [s]
    double min_elevation   = 0.1745; ///< min elevation mask [rad] (~10 deg)
    int    max_epoch_queue = 100;   ///< maximum buffered epochs (back-pressure guard)

    /// Canopy noise model parameters for σ²_eff (IAP-RQ-314).
    /// Used when SatObs::kappa > 0 to incorporate canopy-density weighting.
    CanopyNoiseParams canopy;

    /// GNSS antenna lever arm in body frame [m] (l^b_GNSS).
    /// Set from physical measurement (antenna phase center → IMU origin).
    Eigen::Vector3d lever_arm = Eigen::Vector3d::Zero();
  };

  GnssHandler();  ///< Default constructor (uses Params{} defaults)
  explicit GnssHandler(const Params& params);

  /// Insert a GNSS epoch (thread-safe; called on ROS subscriber callback).
  void insert_epoch(const GnssEpoch& epoch);

  /**
   * @brief Drain buffered epochs whose timestamps fall in
   *        [start - tolerance, end + tolerance].
   *
   * When @p tolerance is not provided, Params::time_tolerance is used.
   * Epochs older than the lower bound are discarded; future epochs stay queued.
   */
  std::vector<GnssEpoch> consume_epochs_in_range(
    double start,
    double end,
    std::optional<double> tolerance = std::nullopt);

  /**
   * @brief Collect all buffered epochs near @p frame_stamp and build factors.
   *
   * @param frame_idx    Smoother index i (used to form keys X(i), V(i), C(i))
   * @param frame_stamp  Timestamp of the LiDAR/IMU frame [s]
   * @param anc_ecef     ECEF coordinates of world-frame origin (passed to DopplerFactor
   *                     for Sagnac correction; also used for fallback geometry).
   * @param[out] out_epochs  (optional) filled with consumed epochs for logging
   * @return Factor graph containing PseudorangeFactor + DopplerFactor per sat.
   */
  gtsam::NonlinearFactorGraph get_factors(int                     frame_idx,
                                          double                  frame_stamp,
                                          const Eigen::Vector3d&  anc_ecef,
                                          std::vector<GnssEpoch>* out_epochs = nullptr);

  /// Number of epochs waiting in the queue.
  std::size_t queue_size() const;

 private:
  /// Canopy-aware pseudorange noise: uses full σ²_eff model when κ available,
  /// falls back to elevation-only scaling when κ ≈ 0.
  double pr_sigma(double elevation, double kappa) const;
  double dop_sigma(double elevation) const;

  Params params_;
  mutable std::mutex mutex_;
  std::deque<GnssEpoch> epoch_queue_;
};

}  // namespace iap
