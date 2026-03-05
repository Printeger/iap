#pragma once
// IAP-RQ-020: GNSS handler — bridges raw GNSS epochs to factor graph

#include <iap/gnss/gnss_types.hpp>
#include <iap/gnss/pseudorange_factor.hpp>
#include <iap/gnss/doppler_factor.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <memory>
#include <deque>
#include <mutex>

namespace iap {

/**
 * @brief Thread-safe buffer and factory for GNSS factors.
 *
 * Typical usage in the odometry estimator (IAP-RQ-020):
 * @code
 *   // On ROS callback thread:
 *   gnss_handler->insert_epoch(epoch);
 *
 *   // In insert_frame(), after IMU pre-integration:
 *   auto gnss_factors = gnss_handler->get_factors(current_idx, frame_stamp,
 *                                                  X_key, V_key, C_key);
 *   new_factors.add(gnss_factors);
 * @endcode
 *
 * Each call to get_factors() drains all epochs that fall within
 * `time_tolerance` seconds of `frame_stamp` and produces one
 * PseudorangeFactor + one DopplerFactor per non-excluded satellite.
 *
 * Ephemeris (sat_pos, sat_vel) is expected to be pre-resolved before
 * inserting the epoch — see the "可先接一个库" note in IAP-RQ-020.
 */
class GnssHandler {
 public:
  struct Params {
    double pr_noise_base   = 5.0;   ///< base pseudorange noise [m]
    double dop_noise_base  = 0.5;   ///< base Doppler noise [m/s]
    double elev_noise_exp  = 2.0;   ///< exponent for elevation-dependent weighting
    double time_tolerance  = 0.1;   ///< epoch window around frame stamp [s]
    double min_elevation   = 0.087; ///< min elevation mask [rad] (~5 deg)
    int    max_epoch_queue = 100;   ///< maximum buffered epochs (back-pressure guard)
  };

  GnssHandler();  ///< Default constructor (uses Params{} defaults)
  explicit GnssHandler(const Params& params);

  /// Insert a GNSS epoch (thread-safe; called on ROS subscriber callback).
  void insert_epoch(const GnssEpoch& epoch);

  /**
   * @brief Collect all buffered epochs near @p frame_stamp and build factors.
   *
   * @param frame_idx  Smoother index i (used to form keys X(i), V(i), C(i))
   * @param frame_stamp  Timestamp of the LiDAR/IMU frame [s]
   * @param[out] out_epochs  (optional) filled with consumed epochs for logging
   * @return Factor graph containing PseudorangeFactor + DopplerFactor per sat.
   */
  gtsam::NonlinearFactorGraph get_factors(int    frame_idx,
                                          double frame_stamp,
                                          std::vector<GnssEpoch>* out_epochs = nullptr);

  /// Number of epochs waiting in the queue.
  std::size_t queue_size() const;

 private:
  /// Elevation-dependent noise scaling: sigma = base / sin^exp(el).
  double pr_sigma(double elevation) const;
  double dop_sigma(double elevation) const;

  Params params_;
  mutable std::mutex mutex_;
  std::deque<GnssEpoch> epoch_queue_;
};

}  // namespace iap
