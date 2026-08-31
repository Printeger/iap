#pragma once
// IAP-RQ-020: GNSS types — per-satellite observation data

#include <Eigen/Core>
#include <algorithm>
#include <cstdint>
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
  std::uint64_t source_identity = 0;  ///< frozen before consumer-derived mutation
};

/// Hash the immutable ROS measurement payload before either consumer consults
/// its local ephemeris/origin caches. This is a template so the GNSS core types
/// remain independent of ROS message headers.
template <typename GnssMeasMsgT>
inline std::uint64_t gnss_measurement_source_identity(
    const GnssMeasMsgT& message) {
  std::uint64_t hash = 1469598103934665603ull;
  const auto append = [&hash](const void* data, const std::size_t size) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t index = 0; index < size; ++index) {
      hash ^= static_cast<std::uint64_t>(bytes[index]);
      hash *= 1099511628211ull;
    }
  };
  const auto append_vector = [&append](const auto& values) {
    const std::uint64_t count = values.size();
    append(&count, sizeof(count));
    for (const auto& value : values) {
      append(&value, sizeof(value));
    }
  };

  const std::uint64_t measurement_count = message.meas.size();
  append(&measurement_count, sizeof(measurement_count));
  for (const auto& observation : message.meas) {
    append(&observation.time.week, sizeof(observation.time.week));
    append(&observation.time.tow, sizeof(observation.time.tow));
    append(&observation.sat, sizeof(observation.sat));
    append_vector(observation.freqs);
    append_vector(observation.cn0);
    append_vector(observation.lli);
    append_vector(observation.code);
    append_vector(observation.psr);
    append_vector(observation.psr_std);
    append_vector(observation.cp);
    append_vector(observation.cp_std);
    append_vector(observation.dopp);
    append_vector(observation.dopp_std);
    append_vector(observation.status);
  }
  return hash == 0 ? 1 : hash;
}

/// Stable identity for the common measurement epoch consumed by both
/// Integrity and the planning Predictor. Consumer-derived fields (residuals,
/// NIS, canopy kappa, and FDE exclusions) are deliberately excluded here:
/// Integrity receives those after smoother optimization while P0 reconstructs
/// the same source epoch directly from the range message. Certification-time
/// exclusions are bound by the overload below.
inline std::uint64_t gnss_epoch_identity(const GnssEpoch& epoch) {
  if (epoch.source_identity != 0) {
    return epoch.source_identity;
  }
  std::uint64_t hash = 1469598103934665603ull;
  const auto append = [&hash](const void* data, const std::size_t size) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t index = 0; index < size; ++index) {
      hash ^= static_cast<std::uint64_t>(bytes[index]);
      hash *= 1099511628211ull;
    }
  };
  append(&epoch.stamp, sizeof(epoch.stamp));
  append(&epoch.gps_sec, sizeof(epoch.gps_sec));
  const std::uint64_t count = epoch.sats.size();
  append(&count, sizeof(count));
  for (const auto& sat : epoch.sats) {
    append(&sat.sat_id, sizeof(sat.sat_id));
    append(&sat.constellation, sizeof(sat.constellation));
    append(&sat.pr_meas, sizeof(sat.pr_meas));
    append(&sat.dop_meas, sizeof(sat.dop_meas));
    append(&sat.pr_sigma, sizeof(sat.pr_sigma));
    append(&sat.dop_sigma, sizeof(sat.dop_sigma));
    for (Eigen::Index axis = 0; axis < sat.sat_pos.size(); ++axis) {
      append(&sat.sat_pos[axis], sizeof(sat.sat_pos[axis]));
      append(&sat.sat_vel[axis], sizeof(sat.sat_vel[axis]));
    }
    append(&sat.tgd, sizeof(sat.tgd));
    append(&sat.svddt, sizeof(sat.svddt));
    append(&sat.elevation, sizeof(sat.elevation));
    append(&sat.azimuth, sizeof(sat.azimuth));
  }
  const std::uint64_t iono_count = epoch.iono_params.size();
  append(&iono_count, sizeof(iono_count));
  for (const double parameter : epoch.iono_params) {
    append(&parameter, sizeof(parameter));
  }
  return hash;
}

/// Identity of the exact certified epoch, including the FDE/ARAIM satellite
/// exclusion set associated with the published protection levels.
inline std::uint64_t gnss_epoch_identity(
    const GnssEpoch& epoch, std::vector<int> excluded_sat_ids) {
  std::sort(excluded_sat_ids.begin(), excluded_sat_ids.end());
  excluded_sat_ids.erase(
      std::unique(excluded_sat_ids.begin(), excluded_sat_ids.end()),
      excluded_sat_ids.end());
  std::uint64_t hash = gnss_epoch_identity(epoch);
  if (excluded_sat_ids.empty()) {
    return hash;
  }
  const auto append = [&hash](const void* data, const std::size_t size) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t index = 0; index < size; ++index) {
      hash ^= static_cast<std::uint64_t>(bytes[index]);
      hash *= 1099511628211ull;
    }
  };
  const std::uint64_t count = excluded_sat_ids.size();
  append(&count, sizeof(count));
  for (const int sat_id : excluded_sat_ids) {
    append(&sat_id, sizeof(sat_id));
  }
  return hash;
}

}  // namespace iap
