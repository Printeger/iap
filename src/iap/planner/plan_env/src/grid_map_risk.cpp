#include <plan_env/grid_map.h>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <stdexcept>

uint64_t GridMap::bindRiskContext(GridRiskContext context)
{
  // Geometry is fixed after initMap. Lock order is occupancy then risk.
  std::lock_guard<std::mutex> map_lock(occupancy_epoch_mutex_);
  std::lock_guard<std::mutex> lock(risk_mutex_);
  risk_context_ = std::move(context);
  risk_occupancy_sequence_ = occupancy_update_sequence_.load();
  if (++risk_version_ == 0) ++risk_version_;
  md_.risk_buffer_.resize(md_.occupancy_buffer_.size());
  return risk_version_;
}

void GridMap::invalidateRiskContext()
{
  std::lock_guard<std::mutex> lock(risk_mutex_);
  risk_context_ = {};
  if (++risk_version_ == 0) ++risk_version_;
}

GridRiskVoxel GridMap::queryRisk(const Eigen::Vector3d& position,
                               const uint64_t version, const double now)
{
  GridRiskVoxel result;
  result.version = version;
  if (!position.allFinite() || !std::isfinite(now)) {
    result.status = GridRiskStatus::INVALID_QUERY;
    return result;
  }
  if (!isInMap(position)) {
    result.status = GridRiskStatus::OUT_OF_MAP;
    return result;
  }
  Eigen::Vector3i index;
  posToIndex(position, index);
  const auto address = static_cast<size_t>(toAddress(index));
  Eigen::Vector3d center;
  indexToPos(index, center);
  GridRiskContext context;
  const auto check = [&]() {
    if (version == 0 || version != risk_version_ ||
        (risk_occupancy_sequence_ & 1u) ||
        occupancy_update_sequence_.load() != risk_occupancy_sequence_ ||
        risk_context_.occupancy_generation != risk_occupancy_sequence_ / 2u)
      return GridRiskStatus::VERSION_CHANGED;
    if (risk_context_.frame_id != mp_.frame_id_)
      return GridRiskStatus::FRAME_MISMATCH;
    if (!std::isfinite(risk_context_.reference_time_s) ||
        !std::isfinite(risk_context_.valid_until_s))
      return GridRiskStatus::INVALID;
    if (now < risk_context_.reference_time_s || now > risk_context_.valid_until_s)
      return GridRiskStatus::STALE;
    if (!risk_context_.predict) return GridRiskStatus::UNCOMPUTED;
    return GridRiskStatus::VALID;
  };
  {
    std::lock_guard<std::mutex> lock(risk_mutex_);
    result.status = check();
    if (result.status != GridRiskStatus::VALID) return result;
    if (address >= md_.risk_buffer_.size()) {
      result.status = GridRiskStatus::INVALID;
      return result;
    }
    const auto& cached = md_.risk_buffer_[address];
    if (cached.version == version && cached.status != GridRiskStatus::UNCOMPUTED)
      return cached;
    context = risk_context_;
  }
  // Never hold map/cache locks during prediction. Input callbacks may revoke
  // this version, and the post-check then rejects the result without storing it.
  try {
    result = context.predict(center);
  } catch (const std::exception&) {
    result = {};
    result.status = GridRiskStatus::INVALID;
  }
  result.version = version;
  if (result.status == GridRiskStatus::VALID &&
      (!std::isfinite(result.hpl) || !std::isfinite(result.vpl) ||
       result.hpl < 0.0 || result.vpl < 0.0))
    result.status = GridRiskStatus::INVALID;
  if (result.status != GridRiskStatus::VALID) {
    result.hpl = result.vpl = std::numeric_limits<double>::quiet_NaN();
  }
  std::lock_guard<std::mutex> lock(risk_mutex_);
  const auto state = check();
  if (state != GridRiskStatus::VALID) {
    result = {};
    result.version = version;
    result.status = state;
    return result;
  }
  md_.risk_buffer_[address] = result;
  if (result.status == GridRiskStatus::VALID) {
    risk_history_[address] = {result.hpl, result.vpl,
        context.reference_time_s, context.valid_until_s,
        context.reference_position,
        context.frame_id};
    // History is a bounded soft preference, never another occupancy truth.
    if (risk_history_.size() > 4096) {
      for (auto it = risk_history_.begin(); it != risk_history_.end();) {
        if (now - it->second.reference_time_s > 2.0)
          it = risk_history_.erase(it);
        else ++it;
      }
      if (risk_history_.size() > 4096) risk_history_.clear();
    }
  }
  return result;
}

GridPlanningRisk GridMap::queryPlanningRisk(const Eigen::Vector3d& position,
                                            const uint64_t version,
                                            const double now,
                                            const GridPlanningRiskPolicy& policy)
{
  GridPlanningRisk planning;
  planning.version = version;
  planning.cost_multiplier = std::max(1.0, policy.unknown_multiplier);
  const auto live = queryRisk(position, version, now);
  planning.query_status = live.status;
  if (live.status == GridRiskStatus::VALID) {
    planning.hpl = live.hpl;
    planning.vpl = live.vpl;
    planning.classification = live.hpl >= policy.hpl_budget_m ||
        live.vpl >= policy.vpl_budget_m
        ? GridAdvisoryClass::PREDICTED_DEGRADED
        : (live.hpl >= policy.hpl_budget_m - policy.reserve_h_m ||
           live.vpl >= policy.vpl_budget_m - policy.reserve_v_m
            ? GridAdvisoryClass::AVOID : GridAdvisoryClass::VALID);
    planning.cost_multiplier = 1.0;
    return planning;
  }
  if (live.status == GridRiskStatus::PREDICTED_DEGRADED) {
    planning.classification = GridAdvisoryClass::PREDICTED_DEGRADED;
    return planning;
  }
  // Geometry and frame errors revoke all historical evidence immediately.
  if (live.status == GridRiskStatus::OUT_OF_MAP ||
      live.status == GridRiskStatus::FRAME_MISMATCH ||
      live.status == GridRiskStatus::VERSION_CHANGED ||
      live.status == GridRiskStatus::INVALID_QUERY || !isInMap(position) ||
      policy.stale_soft_seconds <= 0.0)
    return planning;
  Eigen::Vector3i index;
  posToIndex(position, index);
  std::lock_guard<std::mutex> lock(risk_mutex_);
  const auto it = risk_history_.find(static_cast<size_t>(toAddress(index)));
  if (it == risk_history_.end() ||
      it->second.frame_id != mp_.frame_id_ ||
      !risk_context_.reference_position.allFinite() ||
      (it->second.reference_position - risk_context_.reference_position).norm() >
          policy.stale_max_motion_m)
    return planning;
  const double age_after_expiry = now - it->second.valid_until_s;
  if (age_after_expiry < 0.0 ||
      age_after_expiry > policy.stale_soft_seconds ||
      !std::isfinite(age_after_expiry))
    return planning;
  planning.classification = GridAdvisoryClass::STALE_REFERENCE;
  planning.hpl = it->second.hpl;
  planning.vpl = it->second.vpl;
  const double strength = 1.0 - age_after_expiry / policy.stale_soft_seconds;
  if (planning.hpl >= policy.hpl_budget_m - policy.reserve_h_m ||
      planning.vpl >= policy.vpl_budget_m - policy.reserve_v_m)
    planning.cost_multiplier += std::max(0.0, strength);
  return planning;
}

GridPlanningContext GridMap::preparePlanningQuery(
    const double now, const GridMotionContext& motion,
    std::shared_ptr<const FrozenOccupancyEpoch> epoch) const {
  GridPlanningContext context;
  context.epoch = std::move(epoch);
  context.generation = context.epoch ? context.epoch->generation : occupancyGeneration();
  const double stamp = context.epoch ? context.epoch->cloud_stamp_s : occupancy_cloud_stamp_s_.load();
  if (!std::isfinite(stamp) || !std::isfinite(now) || now < stamp ||
      now - stamp > motion.max_environment_age_s)
    context.environment_reason = GridExecutionReason::ENVIRONMENT_STALE;
  context.required_clearance_m = motion.body_radius_m +
      motion.tracking_reserve_m + motion.error_proxy_m +
      std::sqrt(3.0) * (context.epoch ? context.epoch->resolution_m : mp_.resolution_) / 2.0;
  if (motion.quality == 0 || (motion.quality == 2 && !motion.allow_bridged) ||
      !std::isfinite(motion.error_proxy_m) ||
      !std::isfinite(context.required_clearance_m) ||
      motion.body_radius_m < 0 || motion.tracking_reserve_m < 0 ||
      motion.error_proxy_m < 0)
    context.motion_reason = GridExecutionReason::CURRENT_MOTION_UNAVAILABLE;
  else if (!std::isfinite(motion.stamp_s) || now < motion.stamp_s ||
           now - motion.stamp_s > motion.max_motion_age_s)
    context.motion_reason = GridExecutionReason::CURRENT_MOTION_STALE;
  else if (motion.error_proxy_m >= motion.motion_budget_m)
    context.motion_reason = GridExecutionReason::CURRENT_MOTION_BUDGET;
  return context;
}

double GridMap::measureRawClearance(const Eigen::Vector3d& position,
    const Eigen::Vector3i& index, const double required,
    const bool decision_only, Eigen::Vector3d* nearest,
    const FrozenOccupancyEpoch* epoch) {
  const double resolution = epoch ? epoch->resolution_m : mp_.resolution_;
  const int radius_cells = static_cast<int>(std::ceil(required / resolution)) + 1;
  double closest = std::numeric_limits<double>::infinity();
  std::unique_lock<std::mutex> lock(occupancy_epoch_mutex_, std::defer_lock);
  if (!epoch) lock.lock();
  const auto dims = epoch ? epoch->voxel_dimensions : mp_.map_voxel_num_;
  const auto& addresses = epoch ? epoch->cells->raw_addresses : frozen_raw_addresses_;
  const auto& offsets = epoch ? epoch->cells->raw_row_offsets : frozen_raw_row_offsets_;
  const bool indexed = epoch || (frozen_raw_index_generation_ == occupancyGeneration() && !offsets.empty());
  const auto examine_raw = [&](const Eigen::Vector3i& index) {
    Eigen::Vector3d center;
    if (epoch) center = epoch->lattice_origin + (index.cast<double>().array() + .5).matrix() * resolution;
    else indexToPos(index, center);
    const double distance = (position - center).norm();
    if (distance < closest) {
      closest = distance;
      if (nearest) *nearest = center;
    }
  };
  const int first_z = std::max(0, index.z() - radius_cells);
  const int last_z = std::min(dims.z() - 1, index.z() + radius_cells);
  for (int x = std::max(0, index.x() - radius_cells);
       x <= std::min(dims.x() - 1, index.x() + radius_cells); ++x)
    for (int y = std::max(0, index.y() - radius_cells);
         y <= std::min(dims.y() - 1, index.y() + radius_cells); ++y) {
      if (indexed) {
        const size_t row = static_cast<size_t>(x) * dims.y() + y;
        const int base = static_cast<int>(row) * dims.z();
        const auto begin = addresses.begin() + offsets[row];
        const auto end = addresses.begin() + offsets[row + 1];
        for (auto it = std::lower_bound(begin, end, base + first_z);
             it != end && *it <= base + last_z; ++it) {
          examine_raw(Eigen::Vector3i(x, y, *it - base));
          if (decision_only && closest < required) return closest;
        }
        continue;
      }
      for (int z = first_z; z <= last_z; ++z) {
        const Eigen::Vector3i index(x, y, z);
        const auto address = static_cast<size_t>(toAddress(index));
        const bool raw_cloud = address < md_.occupancy_buffer_raw_cloud_.size() &&
            md_.occupancy_buffer_raw_cloud_[address] != 0;
        const bool raw_fused = address < md_.occupancy_buffer_.size() &&
            md_.occupancy_buffer_[address] > mp_.min_occupancy_log_;
        if (raw_cloud || raw_fused) {
          examine_raw(index);
          if (decision_only && closest < required) return closest;
        }
      }
    }
  return closest;
}

bool GridMap::hasRequiredClearance(const Eigen::Vector3d& position,
    const Eigen::Vector3i& index, const double required,
    const FrozenOccupancyEpoch* epoch) {
  if (!epoch && (frozen_raw_index_generation_ != occupancyGeneration() ||
      frozen_raw_row_offsets_.empty()))
    return measureRawClearance(position, index, required, true, nullptr, epoch) >= required;
  const auto generation = epoch ? epoch->generation : occupancyGeneration();
  const double resolution = epoch ? epoch->resolution_m : mp_.resolution_;
  if (frozen_clearance_radius_m_ != required || clearance_bounds_generation_ != generation) {
    clearance_bounds_generation_ = generation;
    frozen_clearance_bounds_.clear();
    frozen_clearance_radius_m_ = required;
  }
  const int address = epoch ? (index.x()*epoch->voxel_dimensions.y()+index.y())*epoch->voxel_dimensions.z()+index.z() : toAddress(index);
  auto found = frozen_clearance_bounds_.find(address);
  Eigen::Vector3d center;
  if (epoch) center = epoch->lattice_origin + (index.cast<double>().array() + .5).matrix() * resolution;
  else indexToPos(index, center);
  if (found == frozen_clearance_bounds_.end()) {
    ++planning_query_stats_.bounds_misses;
    const double nearest = measureRawClearance(center, index, required, false, nullptr, epoch);
    const int radius = static_cast<int>(std::ceil(required / resolution)) + 1;
    // All unscanned raw centers lie at least this far from the voxel center.
    // No hit in the finite cube gives a finite lower bound, never infinity.
    const double outside = (radius + 0.5) * resolution;
    found = frozen_clearance_bounds_.emplace(address,
        ClearanceBounds{std::min(nearest, outside), nearest}).first;
  }
  else ++planning_query_stats_.bounds_hits;
  const double offset = (position - center).norm();
  // Near floating point equality use the original exact comparison.
  if (found->second.lower_m - offset > required + 1e-12) {
    ++planning_query_stats_.fast_pass;
    return true;
  }
  if (found->second.upper_m + offset < required - 1e-12) {
    ++planning_query_stats_.fast_reject;
    return false;
  }
  ++planning_query_stats_.exact_decisions;
  return measureRawClearance(position, index, required, true, nullptr, epoch) >= required;
}

GridPlanningQueryStats GridMap::planningQueryStats() const {
  auto stats = planning_query_stats_;
  stats.bounds_entries = frozen_clearance_bounds_.size();
  stats.bounds_bytes_estimate = frozen_clearance_bounds_.size() *
      (sizeof(int) + sizeof(ClearanceBounds) + 2 * sizeof(void*)) +
      frozen_clearance_bounds_.bucket_count() * sizeof(void*);
  return stats;
}

GridPlanningCell GridMap::queryPlanningCell(
    const Eigen::Vector3d& position, const uint64_t version,
    const double now, const GridPlanningRiskPolicy& risk_policy,
    const GridMotionContext& motion, const bool include_rejected_clearance,
    const GridPlanningContext* context, const bool performance_diagnostics)
{
  GridPlanningCell cell;
  // The prepared path is confined to the serialized frozen PlanningView.
  // Live trajectory checks retain their independent map locking and no shared
  // mutable query counters.
  struct Accumulate {
    GridPlanningQueryStats* stats;
    const GridPlanningCell& cell;
    ~Accumulate() {
      if (!stats) return;
      stats->occupancy_s += cell.occupancy_query_s;
      stats->clearance_s += cell.clearance_query_s;
      stats->advisory_s += cell.advisory_query_s;
    }
  } accumulate{context && performance_diagnostics ? &planning_query_stats_ : nullptr, cell};
  if (context) {
    ++planning_query_stats_.queries;
    if (include_rejected_clearance) ++planning_query_stats_.detailed_queries;
  }
  const auto occupancy_started = performance_diagnostics ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
  const auto* epoch = context && context->epoch ? context->epoch.get() : nullptr;
  const auto observed = epoch ? queryFrozenOccupancy(*epoch, position, include_rejected_clearance) :
      queryOccupancyDiagnostic(position, !context || include_rejected_clearance);
  if (performance_diagnostics) cell.occupancy_query_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - occupancy_started).count();
  cell.voxel_index = observed.voxel_index;
  cell.occupancy_generation = observed.generation;
  cell.cloud_stamp_s = observed.cloud_stamp_s;
  cell.observed = observed.observed;
  if (!context && std::isfinite(motion.error_proxy_m) &&
      std::isfinite(observed.resolution_m))
    cell.required_clearance_m = motion.body_radius_m +
        motion.tracking_reserve_m + motion.error_proxy_m +
        std::sqrt(3.0) * observed.resolution_m / 2.0;
  if (context) cell.required_clearance_m = context->required_clearance_m;
  const auto measure_clearance = [&]() {
    const auto clearance_started = performance_diagnostics ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    if (!observed.available || !std::isfinite(cell.required_clearance_m) ||
        !std::isfinite(observed.resolution_m) || observed.resolution_m <= 0.0)
      return;
    const double closest = measureRawClearance(position, observed.voxel_index,
        cell.required_clearance_m, false, &cell.nearest_raw_center, epoch);
    if (!epoch && occupancyGeneration() != observed.generation) return;
    cell.raw_center_clearance_m = closest;
    if (performance_diagnostics) cell.clearance_query_s += std::chrono::duration<double>(
        std::chrono::steady_clock::now() - clearance_started).count();
  };
  if (!observed.available) return cell;
  if (!observed.observed) {
    cell.execution_reason = GridExecutionReason::ENVIRONMENT_UNOBSERVED;
    if (include_rejected_clearance) measure_clearance();
    return cell;
  }
  if (context ? context->environment_reason != GridExecutionReason::OK ||
          context->generation != observed.generation :
      !std::isfinite(observed.cloud_stamp_s) ||
      !std::isfinite(now) || now < observed.cloud_stamp_s ||
      now - observed.cloud_stamp_s > motion.max_environment_age_s) {
    cell.execution_reason = GridExecutionReason::ENVIRONMENT_STALE;
    if (include_rejected_clearance) measure_clearance();
    return cell;
  }
  if (observed.raw_occupied || observed.inflated_occupied) {
    cell.execution_reason = GridExecutionReason::PHYSICAL_OBSTACLE;
    if (include_rejected_clearance) measure_clearance();
    return cell;
  }
  if (context) {
    if (context->motion_reason != GridExecutionReason::OK) {
      cell.execution_reason = context->motion_reason;
      return cell;
    }
  } else {
    if (motion.quality == 0 || (motion.quality == 2 && !motion.allow_bridged) ||
        !std::isfinite(motion.error_proxy_m)) {
      cell.execution_reason = GridExecutionReason::CURRENT_MOTION_UNAVAILABLE;
      return cell;
    }
    if (!std::isfinite(motion.stamp_s) || now < motion.stamp_s ||
        now - motion.stamp_s > motion.max_motion_age_s) {
      cell.execution_reason = GridExecutionReason::CURRENT_MOTION_STALE;
      return cell;
    }
    if (motion.error_proxy_m >= motion.motion_budget_m) {
      cell.execution_reason = GridExecutionReason::CURRENT_MOTION_BUDGET;
      return cell;
    }
  }
  // The raw voxel test accounts for the body's radius, tracking reserve,
  // posterior error proxy, and voxel-center uncertainty exactly once. The
  // inflated occupancy above remains a separate fast physical guard.
  bool clearance_ok = true;
  if (context && !include_rejected_clearance) {
    const auto started = performance_diagnostics ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    clearance_ok = hasRequiredClearance(position, observed.voxel_index, cell.required_clearance_m, epoch);
    if (performance_diagnostics) cell.clearance_query_s = std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
  } else {
    measure_clearance();
    clearance_ok = !(cell.raw_center_clearance_m < cell.required_clearance_m);
  }
  if (!epoch && occupancyGeneration() != observed.generation) {
    cell.execution_reason = GridExecutionReason::ENVIRONMENT_STALE;
    return cell;
  }
  if (!clearance_ok) {
    cell.execution_reason = GridExecutionReason::INSUFFICIENT_CLEARANCE;
    return cell;
  }
  cell.execution_reason = GridExecutionReason::OK;
  if (context && version == 0) {
    cell.advisory.query_status = GridRiskStatus::VERSION_CHANGED;
    cell.advisory.cost_multiplier = std::max(1.0, risk_policy.unknown_multiplier);
    return cell;
  }
  const auto advisory_started = performance_diagnostics ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
  cell.advisory = queryPlanningRisk(position, version, now, risk_policy);
  if (performance_diagnostics) cell.advisory_query_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - advisory_started).count();
  return cell;
}

std::optional<GridMapFailureSnapshot> GridMap::captureFailureSnapshot(
    const bool include_observation_evidence) const
{
  GridMapFailureSnapshot snapshot;
  std::lock_guard<std::mutex> map_lock(occupancy_epoch_mutex_);
  const auto sequence = occupancy_update_sequence_.load(std::memory_order_acquire);
  if (sequence == 0 || (sequence & 1u) != 0u) return std::nullopt;
  snapshot.origin = mp_.map_origin_;
  snapshot.max_boundary = mp_.map_max_boundary_;
  snapshot.dimensions = mp_.map_voxel_num_;
  snapshot.resolution_m = mp_.resolution_;
  snapshot.cloud_stamp_s = occupancy_cloud_stamp_s_.load(
      std::memory_order_acquire);
  snapshot.generation = sequence / 2u;
  snapshot.frame_id = mp_.frame_id_;
  if (include_observation_evidence && registered_lidar_window_) {
    snapshot.observation_evidence_available = failure_evidence_capture_;
    snapshot.current_frame = registered_lidar_window_->currentFrameSource();
    snapshot.active_window_generation = registered_lidar_window_->activeGeneration();
    snapshot.sensor_position = md_.camera_pos_;
    snapshot.vehicle_observed_radius_m = current_vehicle_clearance_radius_m_;
    snapshot.observation_sources = registered_lidar_window_->observationSourceFlags();
    if (failure_evidence_capture_)
      for (size_t i = 0; i < snapshot.observation_sources.size(); ++i)
        if (md_.observed_buffer_[i] == 0)
          snapshot.observation_sources[i] |= observation_loss_producer_[i] << 4;
  }
  const auto count = static_cast<size_t>(snapshot.dimensions.x()) *
      snapshot.dimensions.y() * snapshot.dimensions.z();
  if (count == 0 || count > md_.occupancy_buffer_.size() ||
      count > md_.occupancy_buffer_inflate_.size() ||
      count > md_.occupancy_buffer_raw_cloud_.size() ||
      count > md_.observed_buffer_.size()) return std::nullopt;
  snapshot.cell_flags.resize(count);
  for (size_t i = 0; i < count; ++i) {
    const bool raw = md_.occupancy_buffer_raw_cloud_[i] != 0 ||
        md_.occupancy_buffer_[i] > mp_.min_occupancy_log_;
    snapshot.cell_flags[i] = static_cast<uint8_t>((raw ? 1 : 0) |
        (md_.occupancy_buffer_inflate_[i] != 0 ? 2 : 0) |
        (md_.observed_buffer_[i] != 0 ? 4 : 0));
  }
  std::lock_guard<std::mutex> risk_lock(risk_mutex_);
  snapshot.risk_version = risk_version_;
  snapshot.risk_context_matches_map = risk_occupancy_sequence_ == sequence &&
      risk_context_.occupancy_generation == snapshot.generation;
  snapshot.risk_reference_time_s = risk_context_.reference_time_s;
  snapshot.risk_valid_until_s = risk_context_.valid_until_s;
  if (!snapshot.risk_context_matches_map) return snapshot;
  for (size_t i = 0; i < std::min(count, md_.risk_buffer_.size()); ++i) {
    const auto& risk = md_.risk_buffer_[i];
    if (risk.version == risk_version_ &&
        risk.status != GridRiskStatus::UNCOMPUTED)
      snapshot.queried_risk.push_back(
          {static_cast<uint32_t>(i), risk});
  }
  return snapshot;
}

GridMap::Ptr GridMap::fromFailureSnapshot(const GridMapFailureSnapshot& snapshot)
{
  const auto dims = snapshot.dimensions;
  if ((dims.array() <= 0).any() || !std::isfinite(snapshot.resolution_m) ||
      snapshot.resolution_m <= 0.0 || !snapshot.origin.allFinite() ||
      !snapshot.max_boundary.allFinite() || snapshot.generation == 0 ||
      snapshot.generation > std::numeric_limits<uint64_t>::max() / 2u)
    throw std::invalid_argument("invalid failure map geometry or generation");
  const size_t count = static_cast<size_t>(dims.x()) * dims.y() * dims.z();
  if (snapshot.cell_flags.size() != count)
    throw std::invalid_argument("failure map flag count differs from dimensions");
  auto map = std::make_shared<GridMap>();
  map->mp_.map_origin_ = snapshot.origin;
  map->mp_.map_min_boundary_ = snapshot.origin;
  map->mp_.map_max_boundary_ = snapshot.max_boundary;
  map->mp_.map_size_ = snapshot.max_boundary - snapshot.origin;
  map->mp_.map_voxel_num_ = dims;
  map->mp_.resolution_ = snapshot.resolution_m;
  map->mp_.resolution_inv_ = 1.0 / snapshot.resolution_m;
  map->mp_.frame_id_ = snapshot.frame_id;
  map->mp_.min_occupancy_log_ = 0.5;
  map->md_.occupancy_buffer_.assign(count, 0.0);
  map->md_.occupancy_buffer_raw_cloud_.resize(count);
  map->md_.occupancy_buffer_inflate_.resize(count);
  map->md_.observed_buffer_.resize(count);
  map->frozen_raw_row_offsets_.resize(static_cast<size_t>(dims.x()) * dims.y() + 1);
  for (size_t i = 0; i < count; ++i) {
    const auto flags = snapshot.cell_flags[i];
    if (flags & ~static_cast<uint8_t>(7))
      throw std::invalid_argument("failure map contains unknown flag bits");
    map->md_.occupancy_buffer_raw_cloud_[i] = flags & 1;
    map->md_.occupancy_buffer_inflate_[i] = flags & 2;
    map->md_.observed_buffer_[i] = flags & 4;
    if (i % static_cast<size_t>(dims.z()) == 0)
      map->frozen_raw_row_offsets_[i / dims.z()] = map->frozen_raw_addresses_.size();
    if (flags & 1) map->frozen_raw_addresses_.push_back(static_cast<int>(i));
  }
  map->frozen_raw_row_offsets_.back() = map->frozen_raw_addresses_.size();
  map->frozen_raw_index_generation_ = snapshot.generation;
  map->occupancy_update_sequence_.store(snapshot.generation * 2u);
  map->occupancy_cloud_stamp_s_.store(snapshot.cloud_stamp_s);
  return map;
}

GridMapOccupancyDiagnostic GridMap::queryFrozenOccupancy(
    const FrozenOccupancyEpoch& epoch, const Eigen::Vector3d& position,
    const bool detailed) {
  GridMapOccupancyDiagnostic out;
  out.resolution_m = epoch.resolution_m; out.inflation_m = epoch.map_inflation_m;
  out.cloud_stamp_s = epoch.cloud_stamp_s; out.generation = epoch.generation;
  if (detailed) out.frame_id = epoch.frame_id;
  if (!epoch.cells || !position.allFinite() || !(epoch.resolution_m > 0)) return out;
  out.voxel_index = ((position - epoch.lattice_origin) / epoch.resolution_m).array().floor().cast<int>();
  if ((out.voxel_index.array() < 0).any() ||
      (out.voxel_index.array() >= epoch.voxel_dimensions.array()).any()) return out;
  const int address = (out.voxel_index.x() * epoch.voxel_dimensions.y() + out.voxel_index.y()) *
      epoch.voxel_dimensions.z() + out.voxel_index.z();
  const auto flags = epoch.cells->at(address);
  out.raw_occupied = flags & 1; out.inflated_occupied = flags & 2;
  out.observed = (flags & 4) || out.raw_occupied || out.inflated_occupied;
  out.available = true;
  out.state = out.raw_occupied || out.inflated_occupied ? GridMapObservationState::OCCUPIED :
      out.observed ? GridMapObservationState::OBSERVED_FREE : GridMapObservationState::UNKNOWN;
  if (detailed) {
    out.voxel_center = epoch.lattice_origin + (out.voxel_index.cast<double>().array()+.5).matrix() * epoch.resolution_m;
    out.source = out.raw_occupied ? ((flags & 8) ? "fused_depth" : "raw_cloud") : out.inflated_occupied ? "inflated_neighbor" :
        out.observed ? "observed_free" : "unknown";
  }
  return out;
}

bool GridMap::geometryMatches(const FrozenOccupancyEpoch& epoch) const {
  std::lock_guard<std::mutex> lock(occupancy_epoch_mutex_);
  return epoch.frame_id == mp_.frame_id_ && epoch.lattice_origin == mp_.map_origin_ &&
      epoch.voxel_dimensions == mp_.map_voxel_num_ && epoch.resolution_m == mp_.resolution_;
}

std::shared_ptr<const FrozenOccupancyEpoch> GridMap::captureFrozenCorridor(
    const std::vector<Eigen::Vector3d>& positions, const double required) const {
  if (!std::isfinite(required) || required < 0) return {};
  std::lock_guard<std::mutex> lock(occupancy_epoch_mutex_);
  const auto sequence = occupancy_update_sequence_.load();
  if (!sequence || (sequence & 1u) ||
      (registered_lidar_window_enabled_ && (!registered_active_window_healthy_ || !registered_current_frame_healthy_))) return {};
  auto epoch = std::make_shared<FrozenOccupancyEpoch>();
  epoch->lattice_origin = mp_.map_origin_; epoch->voxel_dimensions = mp_.map_voxel_num_;
  epoch->resolution_m = mp_.resolution_; epoch->extent_m = mp_.map_size_;
  epoch->frame_id = mp_.frame_id_; epoch->map_inflation_m = mp_.obstacles_inflation_;
  epoch->generation = sequence/2u; epoch->cloud_stamp_s = occupancy_cloud_stamp_s_.load();
  auto cells = std::make_shared<FrozenOccupancyCells>();
  std::unordered_map<int,uint8_t> masks;
  const int radius = std::ceil(required / mp_.resolution_) + 1;
  const auto dims = mp_.map_voxel_num_;
  for (const auto& p : positions) {
    if (!p.allFinite()) continue;
    const Eigen::Vector3i index = ((p-mp_.map_origin_)/mp_.resolution_).array().floor().cast<int>();
    if ((index.array()<0).any() || (index.array()>=dims.array()).any()) continue;
    masks[(index.x()*dims.y()+index.y())*dims.z()+index.z()] |= 7;
    for (int x=std::max(0,index.x()-radius); x<=std::min(dims.x()-1,index.x()+radius); ++x)
      for (int y=std::max(0,index.y()-radius); y<=std::min(dims.y()-1,index.y()+radius); ++y)
        for (int z=std::max(0,index.z()-radius); z<=std::min(dims.z()-1,index.z()+radius); ++z)
          masks[(x*dims.y()+y)*dims.z()+z] |= 1;
  }
  cells->addresses.reserve(masks.size());
  for (const auto& [address,mask] : masks) cells->addresses.push_back(address);
  std::sort(cells->addresses.begin(),cells->addresses.end());
  cells->raw_row_offsets.resize(static_cast<size_t>(dims.x())*dims.y()+1,0);
  for (const auto address : cells->addresses) {
    const bool raw = md_.occupancy_buffer_raw_cloud_[address] || md_.occupancy_buffer_[address] > mp_.min_occupancy_log_;
    cells->flags.push_back((raw?1:0) | (md_.occupancy_buffer_inflate_[address]?2:0) | (md_.observed_buffer_[address]?4:0));
    cells->comparison_masks.push_back(masks[address]);
    if (raw) { cells->raw_addresses.push_back(address); ++cells->raw_row_offsets[address/dims.z()+1]; }
  }
  for (size_t i=1;i<cells->raw_row_offsets.size();++i) cells->raw_row_offsets[i]+=cells->raw_row_offsets[i-1];
  epoch->cells = cells;
  return epoch;
}

GridMap::CorridorCommit GridMap::commitFrozenCorridor(
    const FrozenOccupancyEpoch& epoch, const double now, const double max_age,
    const std::function<bool()>& commit) {
  std::lock_guard<std::mutex> lock(occupancy_epoch_mutex_);
  const auto sequence = occupancy_update_sequence_.load();
  const double stamp = occupancy_cloud_stamp_s_.load();
  if (!epoch.cells || epoch.cells->addresses.empty() || !sequence || (sequence&1u) ||
      epoch.frame_id != mp_.frame_id_ || epoch.lattice_origin != mp_.map_origin_ ||
      epoch.voxel_dimensions != mp_.map_voxel_num_ || epoch.resolution_m != mp_.resolution_ ||
      !std::isfinite(now) || !std::isfinite(stamp) || now < stamp || now-stamp > max_age ||
      !std::isfinite(epoch.cloud_stamp_s) || now < epoch.cloud_stamp_s || now-epoch.cloud_stamp_s > max_age ||
      (registered_lidar_window_enabled_ && (!registered_active_window_healthy_ || !registered_current_frame_healthy_)))
    return CorridorCommit::Invalid;
  for (size_t i=0;i<epoch.cells->addresses.size();++i) {
    const int address = epoch.cells->addresses[i];
    if (address<0 || static_cast<size_t>(address)>=md_.observed_buffer_.size()) return CorridorCommit::Invalid;
    const bool raw = md_.occupancy_buffer_raw_cloud_[address] || md_.occupancy_buffer_[address] > mp_.min_occupancy_log_;
    const uint8_t flags=(raw?1:0) | (md_.occupancy_buffer_inflate_[address]?2:0) | (md_.observed_buffer_[address]?4:0);
    if ((flags ^ epoch.cells->flags[i]) & epoch.cells->comparison_masks[i]) return CorridorCommit::Changed;
  }
  return commit() ? CorridorCommit::Committed : CorridorCommit::Invalid;
}
