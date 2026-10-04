#include <plan_env/grid_map.h>

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
  return result;
}
