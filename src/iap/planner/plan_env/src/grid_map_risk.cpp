#include <plan_env/grid_map.h>

#include <algorithm>
#include <cmath>

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

GridPlanningCell GridMap::queryPlanningCell(
    const Eigen::Vector3d& position, const uint64_t version,
    const double now, const GridPlanningRiskPolicy& risk_policy,
    const GridMotionContext& motion)
{
  GridPlanningCell cell;
  const auto observed = queryOccupancyDiagnostic(position);
  if (!observed.available) return cell;
  if (!observed.observed) {
    cell.execution_reason = GridExecutionReason::ENVIRONMENT_UNOBSERVED;
    return cell;
  }
  if (!std::isfinite(observed.cloud_stamp_s) ||
      !std::isfinite(now) || now < observed.cloud_stamp_s ||
      now - observed.cloud_stamp_s > motion.max_environment_age_s) {
    cell.execution_reason = GridExecutionReason::ENVIRONMENT_STALE;
    return cell;
  }
  if (observed.raw_occupied || observed.inflated_occupied) {
    cell.execution_reason = GridExecutionReason::PHYSICAL_OBSTACLE;
    return cell;
  }
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
  // The raw voxel test accounts for the body's radius, tracking reserve,
  // posterior error proxy, and voxel-center uncertainty exactly once. The
  // inflated occupancy above remains a separate fast physical guard.
  const double required = motion.body_radius_m + motion.tracking_reserve_m +
      motion.error_proxy_m + std::sqrt(3.0) * observed.resolution_m / 2.0;
  const int radius_cells = static_cast<int>(std::ceil(required / observed.resolution_m)) + 1;
  double closest = std::numeric_limits<double>::infinity();
  {
    std::lock_guard<std::mutex> lock(occupancy_epoch_mutex_);
    const auto dims = mp_.map_voxel_num_;
    for (int x = std::max(0, observed.voxel_index.x() - radius_cells);
         x <= std::min(dims.x() - 1, observed.voxel_index.x() + radius_cells); ++x)
      for (int y = std::max(0, observed.voxel_index.y() - radius_cells);
           y <= std::min(dims.y() - 1, observed.voxel_index.y() + radius_cells); ++y)
        for (int z = std::max(0, observed.voxel_index.z() - radius_cells);
             z <= std::min(dims.z() - 1, observed.voxel_index.z() + radius_cells); ++z) {
          const Eigen::Vector3i index(x, y, z);
          const auto address = static_cast<size_t>(toAddress(index));
          const bool raw_cloud = address < md_.occupancy_buffer_raw_cloud_.size() &&
              md_.occupancy_buffer_raw_cloud_[address] != 0;
          const bool raw_fused = address < md_.occupancy_buffer_.size() &&
              md_.occupancy_buffer_[address] > mp_.min_occupancy_log_;
          if (raw_cloud || raw_fused) {
            Eigen::Vector3d center;
            indexToPos(index, center);
            closest = std::min(closest, (position - center).norm());
          }
        }
  }
  if (occupancyGeneration() != observed.generation) {
    cell.execution_reason = GridExecutionReason::ENVIRONMENT_STALE;
    return cell;
  }
  cell.raw_center_clearance_m = closest;
  if (closest < required) {
    cell.execution_reason = GridExecutionReason::INSUFFICIENT_CLEARANCE;
    return cell;
  }
  cell.execution_reason = GridExecutionReason::OK;
  cell.advisory = queryPlanningRisk(position, version, now, risk_policy);
  return cell;
}
