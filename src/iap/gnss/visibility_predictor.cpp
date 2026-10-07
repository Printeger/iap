#include <stdexcept>
// IAP-RQ-312: Satellite visibility prediction by ray casting
// IAP-RQ-313: Canopy density κ along LOS
// IAP-RQ-314: σ_eff(κ, θ) noise model

#include <iap/gnss/visibility_predictor.hpp>
#include <cmath>
#include <algorithm>
#include <utility>
#include <spdlog/spdlog.h>

namespace iap {

VisibilityPredictor::VisibilityPredictor() : params_(Params{}) {}
VisibilityPredictor::VisibilityPredictor(const Params& p) : params_(p) {}

void VisibilityPredictor::set_occupancy_query(OccupancyQuery query, double resolution_m) {
  if (!std::isfinite(resolution_m) || resolution_m <= 0.0)
    throw std::invalid_argument("occupancy query requires positive resolution");
  grid_ = nullptr;
  occupancy_query_ = std::move(query);
  occupancy_resolution_m_ = resolution_m;
}

void VisibilityPredictor::set_occupancy(const LocalOccupancyGrid* grid) {
  occupancy_query_ = {};
  grid_ = grid;
}

void VisibilityPredictor::set_observation_predicate(
    ObservationPredicate predicate) {
  observation_predicate_ = std::move(predicate);
}

void VisibilityPredictor::set_support_query(SupportQuery query) {
  support_query_ = std::move(query);
}

// ---------------------------------------------------------------------------
Eigen::Vector3d VisibilityPredictor::enu_dir(double elevation, double azimuth) {
  // ENU: East=X, North=Y, Up=Z
  const double ce = std::cos(elevation);
  return Eigen::Vector3d(ce * std::sin(azimuth),
                         ce * std::cos(azimuth),
                         std::sin(elevation));
}

// ---------------------------------------------------------------------------
VisibilityResult VisibilityPredictor::predict(const Eigen::Vector3d& pos_world,
                                              const GnssEpoch& epoch,
                                              const bool measured_epoch_support,
                                              const bool retain_unknown_support,
                                              const double query_time_s,
                                              const double evaluation_time_s,
                                              const bool unknown_as_open_bound) const {
  const Eigen::Vector3d antenna_position = pos_world + epoch.antenna_offset_query;
  VisibilityResult res;
  const std::size_t N = epoch.sats.size();
  res.vis_flags.resize(N, false);
  res.kappas.resize(N, 0.0);
  res.known_occupancy_kappas.resize(N, 0.0);
  res.unknown_kappa_upper_bounds.resize(N, 0.0);
  res.combined_conservative_kappas.resize(N, 0.0);
  res.sigma_effs.resize(N, params_.canopy.sigma_c);
  res.unknown_flags.resize(N, false);
  res.known_flags.resize(N, false);
  res.blocked_flags.resize(N, false);
  res.support_sample_counts.resize(N, 0u);
  res.support_covered_sample_counts.resize(N, 0u);
  res.unknown_support_fractions.resize(N, 0.0);
  res.first_missing_support_distances_m.resize(
      N, std::numeric_limits<double>::quiet_NaN());
  res.first_missing_support_statuses.resize(
      N, LocalMapSupportStatus::MODEL_COMPLETE);

  double kappa_sum  = 0.0;
  int    n_above_el = 0;
  if (support_query_) {
    res.support_authority = LocalMapSupportAuthority::TRUSTED_LOCAL_MAP;
    res.support_status = LocalMapSupportStatus::MODEL_COMPLETE;
  } else if (observation_predicate_) {
    res.support_status = LocalMapSupportStatus::MODEL_COMPLETE;
  }

  for (std::size_t i = 0; i < N; ++i) {
    const SatObs& sat = epoch.sats[i];
    if (sat.excluded) continue;

    // Elevation mask
    if (sat.elevation < params_.min_elevation) {
      res.vis_flags[i] = false;
      continue;
    }
    ++n_above_el;

    const Eigen::Vector3d dir = epoch.R_query_enu * enu_dir(sat.elevation, sat.azimuth);

    if (measured_epoch_support &&
        (!std::isfinite(sat.pr_sigma) || sat.pr_sigma <= 0.0)) {
      res.unknown_flags[i] = true;
      ++res.n_unknown;
      continue;
    }

    // Away from the measured receiver voxel, the planning-side visibility
    // model is allowed to use only observed online space. A point-cloud miss
    // is not evidence of free space. Receiver-local measured support only
    // proves signal reception; NLOS quality remains represented by the
    // measurement sigma and integrity exclusions in the epoch.
    bool unknown_support = false;
    if ((support_query_ || observation_predicate_) &&
        !measured_epoch_support) {
      const double start_offset = std::max(0.0, params_.ray_start_offset);
      const double support_length = std::max(
          0.0, params_.hard_occlusion
                   ? params_.occ_range
                   : start_offset + params_.occ_L);
      constexpr double kSupportStepM = 0.5;
      for (double distance = start_offset;
           distance <= support_length + 1.0e-9;
           distance += kSupportStepM) {
        ++res.support_sample_counts[i];
        const Eigen::Vector3d support_point = antenna_position + distance * dir;
        bool complete = false;
        LocalMapSupportStatus sample_status =
            LocalMapSupportStatus::MODEL_COMPLETE;
        if (support_query_) {
          const double effective_query_time_s = std::isfinite(query_time_s)
              ? query_time_s : epoch.stamp;
          const double effective_evaluation_time_s =
              std::isfinite(evaluation_time_s)
                  ? evaluation_time_s : effective_query_time_s;
          const auto support = support_query_(support_point,
                                              effective_evaluation_time_s,
                                              effective_query_time_s);
          complete = support.complete();
          sample_status = support.status;
          if (!complete && res.support_status ==
                               LocalMapSupportStatus::MODEL_COMPLETE) {
            res.support_status = support.status;
          }
        } else {
          complete = observation_predicate_(support_point);
          if (!complete) {
            sample_status = LocalMapSupportStatus::OBSERVATION_INCOMPLETE;
            res.support_status =
                LocalMapSupportStatus::OBSERVATION_INCOMPLETE;
          }
        }
        if (complete) {
          ++res.support_covered_sample_counts[i];
        } else {
          unknown_support = true;
          if (!std::isfinite(res.first_missing_support_distances_m[i])) {
            res.first_missing_support_distances_m[i] = distance;
            res.first_missing_support_statuses[i] = sample_status;
          }
        }
      }
      if (res.support_sample_counts[i] > 0u) {
        res.unknown_support_fractions[i] = 1.0 -
            static_cast<double>(res.support_covered_sample_counts[i]) /
            static_cast<double>(res.support_sample_counts[i]);
      }
      res.unknown_flags[i] = unknown_support;
      if (unknown_support) {
        ++res.n_unknown;
        if (!retain_unknown_support) {
          res.vis_flags[i] = false;
          continue;
        }
      }
    }
    res.known_flags[i] = !unknown_support;
    if (!unknown_support) {
      ++res.n_known;
    }

    // κ and occlusion
    double known_occupancy_kappa = 0.0;
    bool blocked = false;
    if (occupancy_query_) {
      const double offset = std::max(0.0, params_.ray_start_offset);
      const Eigen::Vector3d origin = antenna_position + offset * dir;
      const double step = 0.5 * occupancy_resolution_m_;
      const int samples = std::max(1, static_cast<int>(std::ceil(params_.occ_L / step)));
      for (int k = 0; k < samples; ++k)
        known_occupancy_kappa += occupancy_query_(
            origin + ((k + 0.5) * params_.occ_L / samples) * dir) ? 1.0 : 0.0;
      known_occupancy_kappa /= samples;
      if (params_.hard_occlusion) {
        const double length = std::max(0.0, params_.occ_range - offset);
        for (double distance = 0.0; distance <= length; distance += step) {
          if (occupancy_query_(origin + distance * dir)) { blocked = true; break; }
        }
      }
    } else if (grid_ != nullptr) {
      const double start_offset = std::max(0.0, params_.ray_start_offset);
      const Eigen::Vector3d ray_origin = antenna_position + start_offset * dir;
      const double occ_range = std::max(0.0, params_.occ_range - start_offset);
      if (params_.clearance_transition_m > 0.0) {
        // Evaluate the bounded union per LOS sample. The clearance proximity
        // is exactly one for an occupied sample, so union(binary, proximity)
        // reduces to proximity without retaining the binary boundary jump.
        // Do this once: aggregating first and unioning the two aggregate
        // ratios would both double-count canopy and remain discontinuous.
        known_occupancy_kappa = grid_->clearance_proximity_ratio(
            ray_origin, dir, params_.occ_L);
      } else {
        known_occupancy_kappa =
            grid_->occupancy_ratio(ray_origin, dir, params_.occ_L);
      }
      blocked = params_.hard_occlusion &&
                grid_->ray_occluded(ray_origin, dir, occ_range);
    }

    known_occupancy_kappa =
        std::clamp(known_occupancy_kappa, 0.0, 1.0);
    const double unknown_kappa_upper_bound = unknown_support
        ? std::clamp(res.unknown_support_fractions[i], 0.0, 1.0)
        : 0.0;
    const double combined_conservative_kappa =
        1.0 - (1.0 - known_occupancy_kappa) *
                  (1.0 - unknown_kappa_upper_bound);
    res.known_occupancy_kappas[i] = known_occupancy_kappa;
    res.unknown_kappa_upper_bounds[i] = unknown_kappa_upper_bound;
    res.combined_conservative_kappas[i] = combined_conservative_kappa;
    res.kappas[i] = retain_unknown_support && !unknown_as_open_bound
        ? combined_conservative_kappa : known_occupancy_kappa;
    res.blocked_flags[i] = blocked;
    if (blocked) {
      ++res.n_blocked;
    }
    res.vis_flags[i] = !blocked;
    if (!blocked) {
      ++res.n_vis;
      kappa_sum += res.kappas[i];
    }

    // σ_eff (RQ-314)
    const double canopy_sigma =
        sigma_eff_canopy(params_.canopy, res.kappas[i], sat.elevation);
    // The same epoch S_i and measurement-noise floor are used on both sides
    // of the receiver measured-support radius. That radius changes support
    // admission only; it must not create a discontinuous drop in sigma/PL.
    res.sigma_effs[i] = std::isfinite(sat.pr_sigma) && sat.pr_sigma > 0.0
        ? std::max(sat.pr_sigma, canopy_sigma)
        : canopy_sigma;
  }

  res.mean_kappa = (res.n_vis > 0) ? (kappa_sum / res.n_vis) : 0.0;
  if (support_query_ && res.n_unknown == 0) {
    res.support_status = LocalMapSupportStatus::MODEL_COMPLETE;
  }

  spdlog::trace("[VisibilityPredictor] pos=({:.1f},{:.1f},{:.1f}) "
                "n_sats={} n_above_el={} n_vis={} n_unknown={} "
                "mean_kappa={:.3f}",
                antenna_position.x(), antenna_position.y(), antenna_position.z(),
                N, n_above_el, res.n_vis, res.n_unknown, res.mean_kappa);

  return res;
}

}  // namespace iap
