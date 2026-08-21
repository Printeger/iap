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
  return lhs.sat_id == rhs.sat_id && lhs.constellation == rhs.constellation &&
         exactDouble(lhs.pr_meas, rhs.pr_meas) &&
         exactDouble(lhs.dop_meas, rhs.dop_meas) &&
         exactDouble(lhs.pr_sigma, rhs.pr_sigma) &&
         exactDouble(lhs.dop_sigma, rhs.dop_sigma) &&
         exactVector(lhs.sat_pos, rhs.sat_pos) &&
         exactVector(lhs.sat_vel, rhs.sat_vel) &&
         exactDouble(lhs.tgd, rhs.tgd) &&
         exactDouble(lhs.svddt, rhs.svddt) &&
         exactDouble(lhs.elevation, rhs.elevation) &&
         exactDouble(lhs.azimuth, rhs.azimuth) &&
         exactDouble(lhs.kappa, rhs.kappa) &&
         exactDouble(lhs.pr_residual, rhs.pr_residual) &&
         exactDouble(lhs.nis_pr, rhs.nis_pr) &&
         exactDouble(lhs.nis_dop, rhs.nis_dop) &&
         lhs.excluded == rhs.excluded;
}

bool exactDoubles(const std::vector<double>& lhs,
                  const std::vector<double>& rhs) {
  if (lhs.size() != rhs.size()) return false;
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    if (!exactDouble(lhs[index], rhs[index])) return false;
  }
  return true;
}

bool exactEpoch(const IntegritySnapshot& lhs, const IntegritySnapshot& rhs) {
  if (lhs.has_epoch != rhs.has_epoch) return false;
  if (!lhs.has_epoch) return true;
  const auto& a = lhs.gnss_epoch;
  const auto& b = rhs.gnss_epoch;
  if (!exactDouble(a.stamp, b.stamp) ||
      !exactDouble(a.gps_sec, b.gps_sec) ||
      !exactDoubles(a.iono_params, b.iono_params) ||
      a.sats.size() != b.sats.size()) {
    return false;
  }
  for (std::size_t index = 0; index < a.sats.size(); ++index) {
    if (!exactSat(a.sats[index], b.sats[index])) return false;
  }
  return true;
}

bool exactCurrentSpatial(const CurrentIntegrityState& lhs,
                         const CurrentIntegrityState& rhs) {
  return exactDouble(lhs.stamp, rhs.stamp) && lhs.valid == rhs.valid &&
         lhs.n_trunks_observed == rhs.n_trunks_observed &&
         exactDouble(lhs.tdop, rhs.tdop) &&
         lhs.excluded_trunk_ids == rhs.excluded_trunk_ids;
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
    std::shared_ptr<const LocalOccupancyGrid> occupancy_owner;
    std::uint64_t occupancy_generation = 0;
    std::shared_ptr<const std::vector<Eigen::Vector3d>> lidar_points_owner;
    std::shared_ptr<const std::vector<LidarFimPrimitive>> lidar_fim_owner;
    std::uint64_t validity_generation = 0;
  };

  struct Slot {
    bool valid = false;
    WorldKey world_key = WorldKey::Zero();
    std::uint64_t validity_generation = 0;
    PredictorModule::SpatialAdvisory advisory;
  };

  struct Candidate {
    Identity identity;
    PredictorModule module;
    std::vector<Slot> slots;
    std::vector<unsigned char> touched;
    std::atomic<std::size_t> retained{0};
    std::atomic<std::size_t> entered{0};
    std::atomic<std::size_t> evicted{0};
    std::atomic<bool> source_identity_valid{true};
    std::size_t full_invalidations = 0;
    RollingSpatialInvalidationReason invalidation_reason =
        RollingSpatialInvalidationReason::None;
  };

  std::optional<Identity> active_identity;
  std::vector<Slot> active_slots;
  std::unique_ptr<Candidate> candidate;
  std::uint64_t next_validity_generation = 1;

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
    const bool gnss_spatial_enabled =
        incoming.params.source_mode != PredictorSourceMode::LidarOnly &&
        incoming.params.gnss_epoch_policy !=
            PredictorGnssEpochPolicy::Disabled;
    if (gnss_spatial_enabled &&
        (!active.snapshot.has_epoch || !incoming.snapshot.has_epoch)) {
      return RollingSpatialInvalidationReason::GnssEpochChanged;
    }
    if (!exactEpoch(active.snapshot, incoming.snapshot)) {
      return RollingSpatialInvalidationReason::GnssEpochChanged;
    }
    if (active.occupancy_generation != incoming.occupancy_generation ||
        !sameOwner(active.occupancy_owner, incoming.occupancy_owner)) {
      return RollingSpatialInvalidationReason::OccupancySourceChanged;
    }
    const bool lidar_spatial_enabled =
        incoming.params.source_mode != PredictorSourceMode::GnssOnly;
    if (lidar_spatial_enabled &&
        (!std::isfinite(active.snapshot.current.stamp) ||
         !std::isfinite(incoming.snapshot.current.stamp) ||
         !std::isfinite(active.snapshot.current.tdop) ||
         !std::isfinite(incoming.snapshot.current.tdop))) {
      return RollingSpatialInvalidationReason::CurrentIntegrityChanged;
    }
    if (lidar_spatial_enabled &&
        (!active.lidar_points_owner || !incoming.lidar_points_owner ||
         !active.lidar_fim_owner || !incoming.lidar_fim_owner)) {
      return RollingSpatialInvalidationReason::LidarSourceChanged;
    }
    if (!sameOwner(active.lidar_points_owner, incoming.lidar_points_owner) ||
        !sameOwner(active.lidar_fim_owner, incoming.lidar_fim_owner)) {
      return RollingSpatialInvalidationReason::LidarSourceChanged;
    }
    if (!exactCurrentSpatial(active.snapshot.current,
                             incoming.snapshot.current)) {
      return RollingSpatialInvalidationReason::CurrentIntegrityChanged;
    }
    return RollingSpatialInvalidationReason::None;
  }

  bool queryMatchesCandidateIdentity(
      const PredictorQueryInput& input,
      const Eigen::Vector3d& expected_position) const {
    if (!candidate || input.frame_id != candidate->identity.geometry.frame_id ||
        !exactVector(input.query_position_map, expected_position)) {
      return false;
    }
    const PredictorParams& params = candidate->identity.params;
    const bool gnss_spatial_enabled =
        params.source_mode != PredictorSourceMode::LidarOnly &&
        params.gnss_epoch_policy != PredictorGnssEpochPolicy::Disabled;
    if (gnss_spatial_enabled &&
        !exactEpoch(input.snapshot, candidate->identity.snapshot)) {
      return false;
    }
    const bool lidar_spatial_enabled =
        params.source_mode != PredictorSourceMode::GnssOnly;
    return !lidar_spatial_enabled ||
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
  const bool gnss_spatial_enabled =
      params.source_mode != PredictorSourceMode::LidarOnly &&
      params.gnss_epoch_policy != PredictorGnssEpochPolicy::Disabled;
  if (gnss_spatial_enabled &&
      (!input.occupancy_owner || input.occupancy_generation == 0u)) {
    if (reason) *reason = "missing_occupancy_identity";
    return false;
  }
  if (gnss_spatial_enabled &&
      params.gnss_epoch_policy == PredictorGnssEpochPolicy::Required &&
      !input.snapshot.has_epoch) {
    if (reason) *reason = "missing_required_gnss_epoch_identity";
    return false;
  }
  if (gnss_spatial_enabled && input.snapshot.has_epoch) {
    const GnssEpoch& epoch = input.snapshot.gnss_epoch;
    if (!std::isfinite(epoch.stamp)) {
      if (reason) *reason = "invalid_gnss_epoch_identity";
      return false;
    }
    for (const SatObs& satellite : epoch.sats) {
      if (!std::isfinite(satellite.elevation) ||
          !std::isfinite(satellite.azimuth) ||
          !std::isfinite(satellite.pr_sigma)) {
        if (reason) *reason = "invalid_gnss_satellite_identity";
        return false;
      }
    }
  }
  auto candidate = std::make_unique<Impl::Candidate>();
  candidate->identity.geometry = std::move(input.geometry);
  candidate->identity.params = input.module.params();
  candidate->identity.snapshot = input.snapshot;
  candidate->identity.occupancy_owner = std::move(input.occupancy_owner);
  candidate->identity.occupancy_generation = input.occupancy_generation;
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
  }
  candidate->invalidation_reason = invalidation;
  if (impl_->active_identity &&
      invalidation == RollingSpatialInvalidationReason::None) {
    candidate->identity.validity_generation =
        impl_->active_identity->validity_generation;
    candidate->slots = impl_->active_slots;
  } else {
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
  const bool hit = slot.valid && slot.world_key == key &&
                   slot.validity_generation ==
                       impl_->candidate->identity.validity_generation;
  if (!impl_->candidate->touched[address]) {
    impl_->candidate->touched[address] = 1u;
    if (hit) {
      impl_->candidate->retained.fetch_add(1, std::memory_order_relaxed);
    } else {
      impl_->candidate->entered.fetch_add(1, std::memory_order_relaxed);
      if (slot.valid) {
        impl_->candidate->evicted.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }

  const PredictorModule::SpatialAdvisory* cached = hit ? &slot.advisory : nullptr;
  for (const auto& input : inputs) {
    PredictorModule::SpatialAdvisory evaluated;
    const std::size_t recomputes_before =
        local.spatial_advisory_recompute_count;
    const bool used_cached = cached != nullptr;
    outputs.push_back(impl_->candidate->module.queryWithSpatialAdvisory(
        input, cached, &evaluated, &local));
    if (used_cached &&
        impl_->candidate->identity.params.source_mode !=
            PredictorSourceMode::GnssOnly) {
      ++local.lidar_cache_hits;
    }
    if (!cached && local.spatial_advisory_recompute_count > recomputes_before) {
      slot.valid = true;
      slot.world_key = key;
      slot.validity_generation =
          impl_->candidate->identity.validity_generation;
      slot.advisory = std::move(evaluated);
      cached = &slot.advisory;
      if (local.lidar_advisory_invocations > 0) {
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
  if (!impl_->candidate) return out;
  out.retained_position_count =
      impl_->candidate->retained.load(std::memory_order_relaxed);
  out.entered_position_count =
      impl_->candidate->entered.load(std::memory_order_relaxed);
  out.evicted_position_count =
      impl_->candidate->evicted.load(std::memory_order_relaxed);
  out.full_invalidation_count = impl_->candidate->full_invalidations;
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
