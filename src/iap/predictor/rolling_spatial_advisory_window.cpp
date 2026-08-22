#include <iap/predictor/rolling_spatial_advisory_window.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <utility>

namespace iap {
namespace {

using WorldKey = Eigen::Matrix<std::int64_t, 3, 1>;

template <typename T>
bool sameOwner(const std::shared_ptr<const T>& lhs,
               const std::shared_ptr<const T>& rhs) {
  return !lhs.owner_before(rhs) && !rhs.owner_before(lhs);
}

bool exactDouble(const double lhs, const double rhs) {
  std::uint64_t lhs_bits = 0;
  std::uint64_t rhs_bits = 0;
  static_assert(sizeof(lhs_bits) == sizeof(lhs));
  std::memcpy(&lhs_bits, &lhs, sizeof(lhs));
  std::memcpy(&rhs_bits, &rhs, sizeof(rhs));
  return lhs_bits == rhs_bits;
}

bool exactVector(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
  return exactDouble(lhs.x(), rhs.x()) && exactDouble(lhs.y(), rhs.y()) &&
         exactDouble(lhs.z(), rhs.z());
}

bool exactCanopy(const CanopyNoiseParams& lhs, const CanopyNoiseParams& rhs) {
  return exactDouble(lhs.sigma_0, rhs.sigma_0) &&
         exactDouble(lhs.sigma_mp, rhs.sigma_mp) &&
         exactDouble(lhs.sigma_c, rhs.sigma_c) &&
         exactDouble(lhs.alpha, rhs.alpha);
}

bool exactVisibility(const VisibilityPredictor::Params& lhs,
                     const VisibilityPredictor::Params& rhs) {
  return exactDouble(lhs.min_elevation, rhs.min_elevation) &&
         exactDouble(lhs.occ_range, rhs.occ_range) &&
         exactDouble(lhs.occ_L, rhs.occ_L) &&
         exactDouble(lhs.ray_start_offset, rhs.ray_start_offset) &&
         lhs.hard_occlusion == rhs.hard_occlusion &&
         exactCanopy(lhs.canopy, rhs.canopy);
}

bool exactGeometry(const GnssGeometryPlPredictorParams& lhs,
                   const GnssGeometryPlPredictorParams& rhs) {
  return exactDouble(lhs.P_HMI_req, rhs.P_HMI_req) &&
         exactDouble(lhs.P_FA_req, rhs.P_FA_req) &&
         lhs.dynamic_budget == rhs.dynamic_budget &&
         exactDouble(lhs.K_fa, rhs.K_fa) &&
         exactDouble(lhs.K_md, rhs.K_md) &&
         exactDouble(lhs.K_ff, rhs.K_ff) &&
         exactDouble(lhs.p_sat_default, rhs.p_sat_default) &&
         exactDouble(lhs.eps_degen, rhs.eps_degen) &&
         lhs.min_sats == rhs.min_sats &&
         lhs.parallel_hypotheses == rhs.parallel_hypotheses &&
         lhs.hypothesis_threads == rhs.hypothesis_threads;
}

bool exactLidarFim(const LidarObservabilityFim::Params& lhs,
                   const LidarObservabilityFim::Params& rhs) {
  return exactDouble(lhs.search_radius_m, rhs.search_radius_m) &&
         lhs.min_points == rhs.min_points && lhs.good_points == rhs.good_points &&
         exactDouble(lhs.sigma_lidar_m, rhs.sigma_lidar_m) &&
         exactDouble(lhs.alpha_min, rhs.alpha_min) &&
         exactDouble(lhs.alpha_max, rhs.alpha_max) &&
         exactDouble(lhs.condition_ref, rhs.condition_ref) &&
         exactDouble(lhs.condition_max, rhs.condition_max) &&
         exactDouble(lhs.tdop_ref, rhs.tdop_ref) &&
         exactDouble(lhs.tdop_max, rhs.tdop_max) &&
         exactDouble(lhs.bias_h_m, rhs.bias_h_m) &&
         exactDouble(lhs.bias_v_m, rhs.bias_v_m) &&
         exactDouble(lhs.fim_radius_m, rhs.fim_radius_m) &&
         lhs.fim_min_voxels == rhs.fim_min_voxels &&
         exactDouble(lhs.fim_range_sigma_base, rhs.fim_range_sigma_base) &&
         exactDouble(lhs.fim_condition_max, rhs.fim_condition_max) &&
         exactDouble(lhs.fim_weight_scale, rhs.fim_weight_scale);
}

bool exactParams(const PredictorParams& lhs, const PredictorParams& rhs) {
  return exactGeometry(lhs.gnss.geometry_params, rhs.gnss.geometry_params) &&
         exactVisibility(lhs.gnss.visibility_params,
                         rhs.gnss.visibility_params) &&
         exactDouble(lhs.gnss.fallback_pl, rhs.gnss.fallback_pl) &&
         exactDouble(lhs.gnss.fim_clock_epsilon,
                     rhs.gnss.fim_clock_epsilon) &&
         exactDouble(lhs.gnss.fim_psd_epsilon,
                     rhs.gnss.fim_psd_epsilon) &&
         exactLidarFim(lhs.lidar.fim_params, rhs.lidar.fim_params) &&
         lhs.lidar.enable_legacy_observability ==
             rhs.lidar.enable_legacy_observability &&
         exactDouble(lhs.fusion.fim_epsilon, rhs.fusion.fim_epsilon) &&
         exactDouble(lhs.fusion.K_H_adv, rhs.fusion.K_H_adv) &&
         exactDouble(lhs.fusion.K_V_adv, rhs.fusion.K_V_adv) &&
         exactDouble(lhs.fusion.b_H_pred, rhs.fusion.b_H_pred) &&
         exactDouble(lhs.fusion.b_V_pred, rhs.fusion.b_V_pred) &&
         exactDouble(lhs.fusion.s_H_pred, rhs.fusion.s_H_pred) &&
         exactDouble(lhs.fusion.s_V_pred, rhs.fusion.s_V_pred) &&
         lhs.fusion.conservative_max_with_gnss ==
             rhs.fusion.conservative_max_with_gnss &&
         lhs.freshness.enabled == rhs.freshness.enabled &&
         exactDouble(lhs.freshness.max_odom_age_s,
                     rhs.freshness.max_odom_age_s) &&
         exactDouble(lhs.freshness.max_integrity_age_s,
                     rhs.freshness.max_integrity_age_s) &&
         exactDouble(lhs.freshness.max_gnss_age_s,
                     rhs.freshness.max_gnss_age_s) &&
         exactDouble(lhs.freshness.max_snapshot_age_s,
                     rhs.freshness.max_snapshot_age_s) &&
         exactDouble(lhs.covariance_growth.sigma_grow_m_sqrt_s,
                     rhs.covariance_growth.sigma_grow_m_sqrt_s) &&
         lhs.source_mode == rhs.source_mode &&
         lhs.gnss_epoch_policy == rhs.gnss_epoch_policy;
}

bool exactSat(const SatObs& lhs, const SatObs& rhs) {
  return lhs.sat_id == rhs.sat_id && lhs.excluded == rhs.excluded &&
         exactDouble(lhs.elevation, rhs.elevation) &&
         exactDouble(lhs.azimuth, rhs.azimuth) &&
         exactDouble(lhs.pr_sigma, rhs.pr_sigma);
}

bool exactEpoch(const IntegritySnapshot& lhs, const IntegritySnapshot& rhs) {
  if (lhs.has_epoch != rhs.has_epoch) return false;
  if (!lhs.has_epoch) return true;
  const auto& a = lhs.gnss_epoch;
  const auto& b = rhs.gnss_epoch;
  if (!exactDouble(a.stamp, b.stamp) || a.sats.size() != b.sats.size()) {
    return false;
  }
  for (std::size_t index = 0; index < a.sats.size(); ++index) {
    if (!exactSat(a.sats[index], b.sats[index])) return false;
  }
  return true;
}

bool exactEpochDiscrete(const IntegritySnapshot& lhs,
                        const IntegritySnapshot& rhs) {
  if (lhs.has_epoch != rhs.has_epoch) return false;
  if (!lhs.has_epoch) return true;
  const auto& a = lhs.gnss_epoch;
  const auto& b = rhs.gnss_epoch;
  if (a.sats.size() != b.sats.size()) return false;
  for (std::size_t index = 0; index < a.sats.size(); ++index) {
    const SatObs& left = a.sats[index];
    const SatObs& right = b.sats[index];
    if (left.sat_id != right.sat_id || left.excluded != right.excluded ||
        !exactDouble(left.pr_sigma, right.pr_sigma)) {
      return false;
    }
  }
  return true;
}

bool exactCurrentSpatial(const CurrentIntegrityState& lhs,
                         const CurrentIntegrityState& rhs) {
  return lhs.n_trunks_observed == rhs.n_trunks_observed &&
         exactDouble(lhs.tdop, rhs.tdop) &&
         lhs.excluded_trunk_ids == rhs.excluded_trunk_ids;
}

bool exactCurrentDiscrete(const CurrentIntegrityState& lhs,
                          const CurrentIntegrityState& rhs) {
  return lhs.n_trunks_observed == rhs.n_trunks_observed &&
         lhs.excluded_trunk_ids == rhs.excluded_trunk_ids;
}

bool policyEnabled(const double value) {
  return std::isfinite(value) && value >= 0.0;
}

bool exactPolicy(const RollingSpatialRetentionPolicy& lhs,
                 const RollingSpatialRetentionPolicy& rhs) {
  return exactDouble(lhs.gnss_spatial_ttl_s, rhs.gnss_spatial_ttl_s) &&
         exactDouble(lhs.legacy_current_spatial_ttl_s,
                     rhs.legacy_current_spatial_ttl_s) &&
         exactDouble(lhs.full_refresh_watchdog_s,
                     rhs.full_refresh_watchdog_s);
}

bool exactGeometry(const RollingSpatialWindowGeometry& lhs,
                   const RollingSpatialWindowGeometry& rhs) {
  return lhs.frame_id == rhs.frame_id &&
         exactVector(lhs.lattice_anchor_w, rhs.lattice_anchor_w) &&
         exactDouble(lhs.resolution_m, rhs.resolution_m) &&
         lhs.shape == rhs.shape;
}

std::size_t positiveModulo(const std::int64_t value, const int modulus) {
  const std::int64_t remainder = value % static_cast<std::int64_t>(modulus);
  return static_cast<std::size_t>(remainder < 0 ? remainder + modulus
                                                : remainder);
}

}  // namespace

struct RollingSpatialAdvisoryWindow::Impl {
  struct Identity {
    RollingSpatialWindowGeometry geometry;
    PredictorParams params;
    IntegritySnapshot snapshot;
    RollingSpatialRetentionPolicy policy;
    RollingSpatialSourceProvenance provenance;
    std::shared_ptr<const LocalOccupancyGrid> occupancy_owner;
    std::shared_ptr<const std::vector<Eigen::Vector3d>> lidar_points_owner;
    std::shared_ptr<const std::vector<LidarFimPrimitive>> lidar_fim_owner;
    std::uint64_t validity_generation = 0;
  };

  struct Slot {
    bool valid = false;
    WorldKey world_key = WorldKey::Zero();
    std::uint64_t validity_generation = 0;
    PredictorModule::SpatialAdvisory advisory;
    IntegritySnapshot source_snapshot;
    RollingSpatialSourceProvenance provenance;
  };

  struct Candidate {
    Identity identity;
    PredictorModule module;
    std::vector<Slot> slots;
    std::vector<unsigned char> touched;
    std::atomic<std::size_t> retained{0};
    std::atomic<std::size_t> entered{0};
    std::atomic<std::size_t> evicted{0};
    std::atomic<std::size_t> exact_retained{0};
    std::atomic<std::size_t> ttl_retained{0};
    std::atomic<std::size_t> gnss_ttl_expired{0};
    std::atomic<std::size_t> legacy_current_ttl_expired{0};
    std::atomic<std::size_t> invalid_source_provenance{0};
    std::atomic<bool> source_identity_valid{true};
    std::size_t full_invalidations = 0;
    RollingSpatialInvalidationReason invalidation_reason =
        RollingSpatialInvalidationReason::None;
    bool full_rebuild = false;
    bool watchdog_forced = false;
  };

  std::optional<Identity> active_identity;
  std::vector<Slot> active_slots;
  std::unique_ptr<Candidate> candidate;
  RollingSpatialRefreshDiagnostics last_begin_diagnostics;
  std::uint64_t next_validity_generation = 1;
  double last_successful_full_refresh_reference_time_s =
      std::numeric_limits<double>::quiet_NaN();

  static RollingSpatialInvalidationReason compareIdentity(
      const Identity& active, const Identity& incoming) {
    if (!exactGeometry(active.geometry, incoming.geometry)) {
      return RollingSpatialInvalidationReason::GeometryChanged;
    }
    if (active.params.source_mode != incoming.params.source_mode ||
        active.params.gnss_epoch_policy != incoming.params.gnss_epoch_policy) {
      return RollingSpatialInvalidationReason::SourcePolicyChanged;
    }
    if (!exactParams(active.params, incoming.params)) {
      return RollingSpatialInvalidationReason::PredictorParametersChanged;
    }
    if (!exactPolicy(active.policy, incoming.policy)) {
      return RollingSpatialInvalidationReason::PredictorParametersChanged;
    }
    if (incoming.provenance.refresh_reference_time_s <
        active.provenance.refresh_reference_time_s) {
      return RollingSpatialInvalidationReason::SourceProvenanceInvalid;
    }
    const PredictorSpatialSourceUsage projection =
        predictorSpatialSourceUsage(incoming.params);
    if (projection.lidar &&
        (incoming.provenance.lidar_generation == 0u ||
         !std::isfinite(incoming.provenance.lidar_stamp) ||
         !incoming.lidar_fim_owner ||
         (projection.legacy_lidar && !incoming.lidar_points_owner))) {
      return RollingSpatialInvalidationReason::SourceProvenanceInvalid;
    }
    const bool active_gnss = projection.gnss && active.snapshot.has_epoch;
    const bool incoming_gnss =
        projection.gnss && incoming.snapshot.has_epoch;
    if (active_gnss != incoming_gnss) {
      return RollingSpatialInvalidationReason::GnssEpochChanged;
    }
    if (incoming_gnss) {
      const auto& old_source = active.provenance;
      const auto& new_source = incoming.provenance;
      if (new_source.occupancy_generation <
              old_source.occupancy_generation ||
          new_source.gnss_epoch_generation <
              old_source.gnss_epoch_generation) {
        return RollingSpatialInvalidationReason::SourceProvenanceInvalid;
      }
      if (old_source.occupancy_generation !=
              new_source.occupancy_generation ||
          !sameOwner(active.occupancy_owner, incoming.occupancy_owner)) {
        return RollingSpatialInvalidationReason::OccupancySourceChanged;
      }
      if (!exactDouble(old_source.occupancy_stamp,
                       new_source.occupancy_stamp)) {
        return RollingSpatialInvalidationReason::SourceProvenanceInvalid;
      }
      if (old_source.gnss_epoch_generation ==
          new_source.gnss_epoch_generation) {
        if (!exactDouble(old_source.gnss_epoch_stamp,
                         new_source.gnss_epoch_stamp) ||
            !exactEpoch(active.snapshot, incoming.snapshot)) {
          return RollingSpatialInvalidationReason::SourceProvenanceInvalid;
        }
      } else if (!exactEpochDiscrete(active.snapshot, incoming.snapshot) ||
                 !policyEnabled(incoming.policy.gnss_spatial_ttl_s)) {
        return RollingSpatialInvalidationReason::GnssEpochChanged;
      }
    }
    if (projection.lidar) {
      if (incoming.provenance.lidar_generation <
          active.provenance.lidar_generation) {
        return RollingSpatialInvalidationReason::SourceProvenanceInvalid;
      }
      if (incoming.provenance.lidar_generation ==
              active.provenance.lidar_generation &&
          (!sameOwner(active.lidar_fim_owner, incoming.lidar_fim_owner) ||
           !exactDouble(active.provenance.lidar_stamp,
                        incoming.provenance.lidar_stamp))) {
        return RollingSpatialInvalidationReason::SourceProvenanceInvalid;
      }
      if (incoming.provenance.lidar_generation !=
              active.provenance.lidar_generation ||
          !sameOwner(active.lidar_fim_owner, incoming.lidar_fim_owner)) {
        return RollingSpatialInvalidationReason::LidarSourceChanged;
      }
    }
    if (projection.legacy_lidar) {
      if (!sameOwner(active.lidar_points_owner, incoming.lidar_points_owner)) {
        return incoming.provenance.lidar_generation ==
                       active.provenance.lidar_generation
                   ? RollingSpatialInvalidationReason::SourceProvenanceInvalid
                   : RollingSpatialInvalidationReason::LidarSourceChanged;
      }
      if (incoming.provenance.current_generation <
          active.provenance.current_generation) {
        return RollingSpatialInvalidationReason::SourceProvenanceInvalid;
      }
      if (incoming.provenance.current_generation ==
          active.provenance.current_generation) {
        if ((policyEnabled(
                 incoming.policy.legacy_current_spatial_ttl_s) &&
             !exactDouble(active.provenance.current_stamp,
                          incoming.provenance.current_stamp)) ||
            !exactCurrentSpatial(active.snapshot.current,
                                 incoming.snapshot.current)) {
          return RollingSpatialInvalidationReason::SourceProvenanceInvalid;
        }
      } else if (!exactCurrentSpatial(active.snapshot.current,
                                      incoming.snapshot.current)) {
        if (!exactCurrentDiscrete(active.snapshot.current,
                                  incoming.snapshot.current) ||
            !policyEnabled(
                incoming.policy.legacy_current_spatial_ttl_s)) {
          return RollingSpatialInvalidationReason::CurrentIntegrityChanged;
        }
      }
      if (!std::isfinite(active.snapshot.current.tdop) ||
          !std::isfinite(incoming.snapshot.current.tdop)) {
        return RollingSpatialInvalidationReason::CurrentIntegrityChanged;
      }
    }
    return RollingSpatialInvalidationReason::None;
  }

  struct SlotRetention {
    bool reusable = false;
    bool ttl = false;
    bool gnss_expired = false;
    bool legacy_current_expired = false;
    bool invalid_provenance = false;
  };

  SlotRetention evaluateSlotRetention(const Slot& slot) const {
    SlotRetention out;
    if (!candidate || !slot.valid) return out;
    const Identity& incoming = candidate->identity;
    const PredictorSpatialSourceUsage usage =
        predictorSpatialSourceUsage(incoming.params);
    bool exact = true;
    if (usage.gnss && incoming.snapshot.has_epoch) {
      const bool gnss_exact =
          slot.provenance.gnss_epoch_generation ==
              incoming.provenance.gnss_epoch_generation &&
          exactDouble(slot.provenance.gnss_epoch_stamp,
                      incoming.provenance.gnss_epoch_stamp) &&
          exactEpoch(slot.source_snapshot, incoming.snapshot);
      const bool gnss_ttl_enabled =
          policyEnabled(incoming.policy.gnss_spatial_ttl_s);
      if (!gnss_exact) {
        exact = false;
        if (!exactEpochDiscrete(slot.source_snapshot, incoming.snapshot) ||
            !gnss_ttl_enabled) {
          out.gnss_expired = true;
          return out;
        }
      }
      if (gnss_ttl_enabled) {
        const double age_s =
            incoming.provenance.refresh_reference_time_s -
            slot.provenance.gnss_epoch_stamp;
        if (!std::isfinite(age_s) || age_s < 0.0) {
          out.invalid_provenance = true;
          out.gnss_expired = true;
          return out;
        }
        if (age_s > incoming.policy.gnss_spatial_ttl_s) {
          out.gnss_expired = true;
          return out;
        }
      }
    }
    if (usage.legacy_lidar) {
      const bool current_spatial_exact =
          exactCurrentSpatial(slot.source_snapshot.current,
                              incoming.snapshot.current);
      const bool current_version_exact =
          slot.provenance.current_generation ==
              incoming.provenance.current_generation &&
          exactDouble(slot.provenance.current_stamp,
                      incoming.provenance.current_stamp);
      const bool current_ttl_enabled = policyEnabled(
          incoming.policy.legacy_current_spatial_ttl_s);
      if (!current_spatial_exact) {
        exact = false;
        if (!exactCurrentDiscrete(slot.source_snapshot.current,
                                  incoming.snapshot.current) ||
            !current_ttl_enabled) {
          out.legacy_current_expired = true;
          return out;
        }
      } else if (current_ttl_enabled && !current_version_exact) {
        exact = false;
      }
      if (current_ttl_enabled) {
        const double age_s =
            incoming.provenance.refresh_reference_time_s -
            slot.provenance.current_stamp;
        if (!std::isfinite(age_s) || age_s < 0.0) {
          out.invalid_provenance = true;
          out.legacy_current_expired = true;
          return out;
        }
        if (age_s > incoming.policy.legacy_current_spatial_ttl_s) {
          out.legacy_current_expired = true;
          return out;
        }
      }
    }
    out.reusable = true;
    out.ttl = !exact;
    return out;
  }

  bool queryMatchesCandidateIdentity(
      const PredictorQueryInput& input,
      const Eigen::Vector3d& expected_position) const {
    if (!candidate || input.frame_id != candidate->identity.geometry.frame_id ||
        !exactVector(input.query_position_map, expected_position)) {
      return false;
    }
    const PredictorParams& params = candidate->identity.params;
    const PredictorSpatialSourceUsage projection =
        predictorSpatialSourceUsage(params);
    if (projection.gnss &&
        !exactEpoch(input.snapshot, candidate->identity.snapshot)) {
      return false;
    }
    return !projection.legacy_lidar ||
           exactCurrentSpatial(input.snapshot.current,
                               candidate->identity.snapshot.current);
  }

  std::size_t address(const WorldKey& key) const {
    const Eigen::Vector3i& shape = candidate->identity.geometry.shape;
    const std::size_t x = positiveModulo(key.x(), shape.x());
    const std::size_t y = positiveModulo(key.y(), shape.y());
    const std::size_t z = positiveModulo(key.z(), shape.z());
    return (x * static_cast<std::size_t>(shape.y()) + y) *
               static_cast<std::size_t>(shape.z()) +
           z;
  }

  bool worldKey(const Eigen::Vector3d& position, WorldKey* key) const {
    if (!key || !position.allFinite()) return false;
    constexpr double kMaxExactInteger = 9007199254740991.0;
    const auto& geometry = candidate->identity.geometry;
    for (int axis = 0; axis < 3; ++axis) {
      const double value = std::floor(
          (position(axis) - geometry.lattice_anchor_w(axis)) /
          geometry.resolution_m);
      if (!std::isfinite(value) || std::abs(value) > kMaxExactInteger) {
        return false;
      }
      (*key)(axis) = static_cast<std::int64_t>(value);
    }
    return true;
  }
};

const char* rollingSpatialInvalidationReasonName(
    const RollingSpatialInvalidationReason reason) {
  switch (reason) {
    case RollingSpatialInvalidationReason::None:
      return "none";
    case RollingSpatialInvalidationReason::Uninitialized:
      return "uninitialized";
    case RollingSpatialInvalidationReason::GeometryChanged:
      return "geometry_changed";
    case RollingSpatialInvalidationReason::PredictorParametersChanged:
      return "predictor_parameters_changed";
    case RollingSpatialInvalidationReason::GnssEpochChanged:
      return "gnss_epoch_changed";
    case RollingSpatialInvalidationReason::OccupancySourceChanged:
      return "occupancy_source_changed";
    case RollingSpatialInvalidationReason::LidarSourceChanged:
      return "lidar_source_changed";
    case RollingSpatialInvalidationReason::CurrentIntegrityChanged:
      return "current_integrity_changed";
    case RollingSpatialInvalidationReason::SourcePolicyChanged:
      return "source_policy_changed";
    case RollingSpatialInvalidationReason::WindowDisjoint:
      return "window_disjoint";
    case RollingSpatialInvalidationReason::SourceProvenanceInvalid:
      return "source_provenance_invalid";
    case RollingSpatialInvalidationReason::WatchdogForced:
      return "watchdog_forced";
  }
  return "invalid";
}

RollingSpatialAdvisoryWindow::RollingSpatialAdvisoryWindow()
    : impl_(std::make_unique<Impl>()) {}
RollingSpatialAdvisoryWindow::~RollingSpatialAdvisoryWindow() = default;
RollingSpatialAdvisoryWindow::RollingSpatialAdvisoryWindow(
    RollingSpatialAdvisoryWindow&&) noexcept = default;
RollingSpatialAdvisoryWindow& RollingSpatialAdvisoryWindow::operator=(
    RollingSpatialAdvisoryWindow&&) noexcept = default;

bool RollingSpatialAdvisoryWindow::beginRefresh(
    RollingSpatialRefreshInput input, std::string* reason) {
  if (impl_->candidate) {
    if (reason) *reason = "refresh_already_active";
    return false;
  }
  impl_->last_begin_diagnostics = {};
  const auto reject_provenance = [&](const char* detail) {
    impl_->last_begin_diagnostics.invalid_source_provenance_count = 1u;
    impl_->last_begin_diagnostics.invalidation_reason =
        RollingSpatialInvalidationReason::SourceProvenanceInvalid;
    if (reason) *reason = detail;
    return false;
  };
  const auto& geometry = input.geometry;
  if (geometry.frame_id.empty() || !geometry.lattice_anchor_w.allFinite() ||
      !std::isfinite(geometry.resolution_m) || geometry.resolution_m <= 0.0 ||
      (geometry.shape.array() <= 0).any()) {
    if (reason) *reason = "invalid_geometry";
    return false;
  }
  std::size_t capacity = 1;
  for (int axis = 0; axis < 3; ++axis) {
    const std::size_t extent =
        static_cast<std::size_t>(geometry.shape(axis));
    if (extent > static_cast<std::size_t>(
                     std::numeric_limits<int>::max()) /
                     capacity) {
      if (reason) *reason = "invalid_capacity";
      return false;
    }
    capacity *= extent;
  }
  const PredictorParams& params = input.module.params();
  const PredictorSpatialSourceUsage projection =
      predictorSpatialSourceUsage(params);
  const auto& source = input.provenance;
  if (!std::isfinite(source.refresh_reference_time_s)) {
    return reject_provenance("invalid_refresh_reference_time");
  }
  if (source.current_generation == 0u ||
      !std::isfinite(source.current_stamp) ||
      !exactDouble(source.current_stamp, input.snapshot.current.stamp)) {
    return reject_provenance("invalid_current_provenance");
  }
  if (projection.gnss && input.snapshot.has_epoch &&
      (!input.occupancy_owner || source.occupancy_generation == 0u ||
       !std::isfinite(source.occupancy_stamp))) {
    return reject_provenance("missing_occupancy_identity");
  }
  if (projection.gnss &&
      params.gnss_epoch_policy == PredictorGnssEpochPolicy::Required &&
      !input.snapshot.has_epoch) {
    return reject_provenance("missing_required_gnss_epoch_identity");
  }
  if (projection.gnss && input.snapshot.has_epoch) {
    const GnssEpoch& epoch = input.snapshot.gnss_epoch;
    if (source.gnss_epoch_generation == 0u ||
        !std::isfinite(source.gnss_epoch_stamp) ||
        !std::isfinite(epoch.stamp) ||
        !exactDouble(source.gnss_epoch_stamp, epoch.stamp)) {
      return reject_provenance("invalid_gnss_epoch_identity");
    }
    for (const SatObs& satellite : epoch.sats) {
      if (!std::isfinite(satellite.elevation) ||
          !std::isfinite(satellite.azimuth) ||
          !std::isfinite(satellite.pr_sigma)) {
        return reject_provenance("invalid_gnss_satellite_identity");
      }
    }
  }
  if (projection.lidar &&
      (!input.lidar_fim_primitives_owner || source.lidar_generation == 0u ||
       !std::isfinite(source.lidar_stamp) ||
       (projection.legacy_lidar && !input.lidar_map_points_owner))) {
    return reject_provenance("invalid_lidar_provenance");
  }
  if (projection.legacy_lidar &&
      !std::isfinite(input.snapshot.current.tdop)) {
    return reject_provenance("invalid_legacy_lidar_provenance");
  }
  auto candidate = std::make_unique<Impl::Candidate>();
  candidate->identity.geometry = std::move(input.geometry);
  candidate->identity.params = input.module.params();
  candidate->identity.snapshot = input.snapshot;
  candidate->identity.policy = input.policy;
  candidate->identity.provenance = input.provenance;
  candidate->identity.occupancy_owner = std::move(input.occupancy_owner);
  candidate->identity.lidar_points_owner =
      std::move(input.lidar_map_points_owner);
  candidate->identity.lidar_fim_owner =
      std::move(input.lidar_fim_primitives_owner);
  candidate->module = std::move(input.module);
  candidate->module.set_local_occupancy(
      candidate->identity.occupancy_owner.get());
  candidate->module.set_lidar_map_points(
      candidate->identity.lidar_points_owner);
  candidate->module.set_lidar_fim_primitives(
      candidate->identity.lidar_fim_owner);

  RollingSpatialInvalidationReason invalidation =
      RollingSpatialInvalidationReason::Uninitialized;
  if (impl_->active_identity) {
    invalidation = Impl::compareIdentity(*impl_->active_identity,
                                         candidate->identity);
    if (invalidation == RollingSpatialInvalidationReason::None &&
        policyEnabled(candidate->identity.policy.full_refresh_watchdog_s)) {
      const double elapsed =
          candidate->identity.provenance.refresh_reference_time_s -
          impl_->last_successful_full_refresh_reference_time_s;
      if (!std::isfinite(elapsed) || elapsed < 0.0) {
        invalidation =
            RollingSpatialInvalidationReason::SourceProvenanceInvalid;
      } else if (elapsed >=
                 candidate->identity.policy.full_refresh_watchdog_s) {
        invalidation = RollingSpatialInvalidationReason::WatchdogForced;
        candidate->watchdog_forced = true;
      }
    }
  }
  candidate->invalidation_reason = invalidation;
  if (impl_->active_identity &&
      invalidation == RollingSpatialInvalidationReason::None) {
    candidate->identity.validity_generation =
        impl_->active_identity->validity_generation;
    candidate->slots = impl_->active_slots;
  } else {
    candidate->full_rebuild = true;
    candidate->identity.validity_generation = impl_->next_validity_generation++;
    candidate->slots.resize(capacity);
    if (impl_->active_identity) {
      candidate->full_invalidations = 1;
      candidate->evicted.store(static_cast<std::size_t>(std::count_if(
          impl_->active_slots.begin(), impl_->active_slots.end(),
          [](const Impl::Slot& slot) { return slot.valid; })));
    }
  }
  if (candidate->slots.size() != capacity) {
    candidate->slots.assign(capacity, Impl::Slot{});
  }
  candidate->touched.assign(capacity, 0u);
  impl_->candidate = std::move(candidate);
  if (reason) *reason = "ok";
  return true;
}

std::vector<PredictorQueryResult>
RollingSpatialAdvisoryWindow::queryPositionHorizons(
    const std::vector<PredictorQueryInput>& inputs,
    PredictorBatchDiagnostics* diagnostics) {
  PredictorBatchDiagnostics local;
  local.collect_component_timing =
      diagnostics && diagnostics->collect_component_timing;
  local.query_count = inputs.size();
  std::vector<PredictorQueryResult> outputs;
  outputs.reserve(inputs.size());
  if (!impl_->candidate || inputs.empty()) {
    if (diagnostics) *diagnostics = local;
    return outputs;
  }

  const Eigen::Vector3d& expected_position = inputs.front().query_position_map;
  for (const auto& input : inputs) {
    if (!impl_->queryMatchesCandidateIdentity(input, expected_position)) {
      impl_->candidate->source_identity_valid.store(
          false, std::memory_order_relaxed);
      if (diagnostics) *diagnostics = local;
      return outputs;
    }
  }

  WorldKey key;
  const bool key_valid = impl_->worldKey(inputs.front().query_position_map, &key);
  if (!key_valid) {
    for (const auto& input : inputs) {
      outputs.push_back(impl_->candidate->module.queryWithSpatialAdvisory(
          input, nullptr, nullptr, &local));
    }
    if (diagnostics) *diagnostics = local;
    return outputs;
  }
  const std::size_t address = impl_->address(key);
  Impl::Slot& slot = impl_->candidate->slots[address];
  const bool slot_matches =
      slot.valid && slot.world_key == key &&
      slot.validity_generation ==
          impl_->candidate->identity.validity_generation;
  const Impl::SlotRetention retention =
      slot_matches ? impl_->evaluateSlotRetention(slot)
                   : Impl::SlotRetention{};
  const bool hit = slot_matches && retention.reusable;
  if (!impl_->candidate->touched[address]) {
    impl_->candidate->touched[address] = 1u;
    if (hit) {
      impl_->candidate->retained.fetch_add(1, std::memory_order_relaxed);
      if (retention.ttl) {
        impl_->candidate->ttl_retained.fetch_add(1,
                                                 std::memory_order_relaxed);
      } else {
        impl_->candidate->exact_retained.fetch_add(
            1, std::memory_order_relaxed);
      }
    } else {
      impl_->candidate->entered.fetch_add(1, std::memory_order_relaxed);
      if (slot.valid) {
        impl_->candidate->evicted.fetch_add(1, std::memory_order_relaxed);
      }
      if (retention.gnss_expired) {
        impl_->candidate->gnss_ttl_expired.fetch_add(
            1, std::memory_order_relaxed);
      }
      if (retention.legacy_current_expired) {
        impl_->candidate->legacy_current_ttl_expired.fetch_add(
            1, std::memory_order_relaxed);
      }
      if (retention.invalid_provenance) {
        impl_->candidate->invalid_source_provenance.fetch_add(
            1, std::memory_order_relaxed);
      }
    }
  }

  const PredictorModule::SpatialAdvisory* cached = hit ? &slot.advisory : nullptr;
  bool populated_lidar_this_call = false;
  for (const auto& input : inputs) {
    PredictorQueryInput evaluated_input = input;
    if (cached && predictorSpatialSourceUsage(
                      impl_->candidate->identity.params).gnss) {
      evaluated_input.snapshot.gnss_epoch.stamp =
          slot.provenance.gnss_epoch_stamp;
    }
    PredictorModule::SpatialAdvisory evaluated;
    const std::size_t recomputes_before =
        local.spatial_advisory_recompute_count;
    const std::size_t reuses_before = local.spatial_advisory_reuse_count;
    outputs.push_back(impl_->candidate->module.queryWithSpatialAdvisory(
        evaluated_input, cached, &evaluated, &local));
    if (populated_lidar_this_call &&
        local.spatial_advisory_reuse_count > reuses_before) {
      ++local.lidar_cache_hits;
    }
    if (!cached && local.spatial_advisory_recompute_count > recomputes_before) {
      slot.valid = true;
      slot.world_key = key;
      slot.validity_generation =
          impl_->candidate->identity.validity_generation;
      slot.advisory = std::move(evaluated);
      slot.source_snapshot = impl_->candidate->identity.snapshot;
      slot.provenance = impl_->candidate->identity.provenance;
      cached = &slot.advisory;
      if (local.lidar_advisory_invocations > 0) {
        populated_lidar_this_call = true;
        ++local.unique_positions;
        ++local.lidar_evaluations;
      }
    }
  }
  if (diagnostics) *diagnostics = local;
  return outputs;
}

void RollingSpatialAdvisoryWindow::commitRefresh() {
  if (!impl_->candidate) return;
  if (!impl_->candidate->source_identity_valid.load(
          std::memory_order_relaxed)) {
    impl_->candidate.reset();
    return;
  }
  if (impl_->candidate->full_rebuild) {
    impl_->last_successful_full_refresh_reference_time_s =
        impl_->candidate->identity.provenance.refresh_reference_time_s;
  }
  impl_->active_identity = std::move(impl_->candidate->identity);
  impl_->active_slots = std::move(impl_->candidate->slots);
  impl_->candidate.reset();
}

void RollingSpatialAdvisoryWindow::abortRefresh() {
  impl_->candidate.reset();
}

RollingSpatialRefreshDiagnostics RollingSpatialAdvisoryWindow::diagnostics()
    const {
  RollingSpatialRefreshDiagnostics out;
  if (!impl_->candidate) return impl_->last_begin_diagnostics;
  out.retained_position_count =
      impl_->candidate->retained.load(std::memory_order_relaxed);
  out.entered_position_count =
      impl_->candidate->entered.load(std::memory_order_relaxed);
  out.evicted_position_count =
      impl_->candidate->evicted.load(std::memory_order_relaxed);
  out.full_invalidation_count = impl_->candidate->full_invalidations;
  out.exact_retained_position_count =
      impl_->candidate->exact_retained.load(std::memory_order_relaxed);
  out.ttl_retained_position_count =
      impl_->candidate->ttl_retained.load(std::memory_order_relaxed);
  out.gnss_ttl_expired_position_count =
      impl_->candidate->gnss_ttl_expired.load(std::memory_order_relaxed);
  out.legacy_current_ttl_expired_position_count =
      impl_->candidate->legacy_current_ttl_expired.load(
          std::memory_order_relaxed);
  out.watchdog_forced_full_rebuild_count =
      impl_->candidate->watchdog_forced ? 1u : 0u;
  out.invalid_source_provenance_count =
      impl_->candidate->invalidation_reason ==
              RollingSpatialInvalidationReason::SourceProvenanceInvalid
          ? 1u
          : impl_->candidate->invalid_source_provenance.load(
                std::memory_order_relaxed);
  out.invalidation_reason = impl_->candidate->invalidation_reason;
  if (impl_->active_identity && out.full_invalidation_count == 0 &&
      out.retained_position_count == 0 &&
      out.entered_position_count == impl_->candidate->slots.size() &&
      out.evicted_position_count == impl_->candidate->slots.size()) {
    out.full_invalidation_count = 1;
    out.invalidation_reason = RollingSpatialInvalidationReason::WindowDisjoint;
  }
  return out;
}

}  // namespace iap
