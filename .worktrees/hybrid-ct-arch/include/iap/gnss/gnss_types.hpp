#pragma once
// IAP-RQ-020: GNSS types — per-satellite observation data

#include <Eigen/Core>
#include <string>
#include <vector>

namespace iap {

/// @brief Per-satellite observation data for one epoch.
///
/// Each SatObs represents a single satellite observable channel;
/// pseudorange and Doppler are carried as separate fields so that
/// per-channel integrity gating (RQ-220) can admissibility-check them
/// independently.
struct SatObs {
  int  sat_id        = 0;     ///< satellite PRN / composite ID
  char constellation = 'G';  ///< 'G'=GPS, 'R'=GLONASS, 'E'=Galileo, 'C'=BeiDou

  // ---- Measurements -------------------------------------------------------
  double pr_meas  = 0.0;  ///< pseudorange measurement [m]
  double dop_meas = 0.0;  ///< Doppler measurement [m/s]  (positive = approach)
  double pr_sigma  = 5.0; ///< pseudorange 1-sigma noise [m]
  double dop_sigma = 0.5; ///< Doppler    1-sigma noise [m/s]

  // ---- Ephemeris-derived quantities (IAP-RQ-020: satellite state) ----------
  Eigen::Vector3d sat_pos = Eigen::Vector3d::Zero();  ///< satellite ECEF position [m]
  Eigen::Vector3d sat_vel = Eigen::Vector3d::Zero();  ///< satellite ECEF velocity [m/s]
  double tgd    = 0.0;   ///< group delay [s] (GPS/GAL/BDS: Ephem::tgd[0]; GLONASS: 0)
  double svddt  = 0.0;   ///< satellite clock drift rate [s/s] (from eph2vel/geph2vel)

  // ---- Geometry -----------------------------------------------------------
  double elevation = 0.0;  ///< elevation angle [rad] — used for noise weighting
  double azimuth   = 0.0;  ///< azimuth angle   [rad]

  // ---- Canopy density (populated by VisibilityPredictor, IAP-RQ-313) -----
  double kappa = 0.0;  ///< occupancy ratio along LOS ∈ [0,1]; 0 = clear sky

  // ---- Pseudorange residual (IAP-RQ-242: ARAIM solution separation) ------
  double pr_residual = 0.0;  ///< r = pr_meas − pr_pred [m];  filled by GnssHandler

  // ---- NIS gating (populated by RQ-220) -----------------------------------
  double nis_pr  = 0.0;  ///< normalised innovation squared — pseudorange
  double nis_dop = 0.0;  ///< normalised innovation squared — Doppler
  bool   excluded = false; ///< set true when FDE rejects the satellite
};

/// @brief All per-satellite observations at one GNSS epoch.
struct GnssEpoch {
  double stamp    = 0.0;           ///< UTC ROS timestamp [s]
  double gps_sec  = 0.0;           ///< GPS time [s since GPS epoch] — for iono/trop models
  std::vector<SatObs>    sats;     ///< per-satellite channels
  std::vector<double>    iono_params;  ///< Klobuchar params {α0..α3, β0..β3}; empty → skip iono
};

/// @brief Shared ECEF anchor state published by gnss_extension.
struct GnssAnchorState {
  double stamp = 0.0;
  Eigen::Vector3d origin_ecef = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_ecef_world = Eigen::Matrix3d::Identity();
};

}  // namespace iap
