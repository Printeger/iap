#include <iap/planner/risk_grid_map.hpp>
#include <iap/predictor/predictor_types.hpp>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <utility>

namespace iap {
namespace {

constexpr double kBoundaryEps = 1.0e-4;

bool finite_positive(const double v) {
  return std::isfinite(v) && v > 0.0;
}

bool finite_nonnegative(const double v) {
  return std::isfinite(v) && v >= 0.0;
}

double clamp_cost(const double value, const double max_value) {
  if (!std::isfinite(value)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::clamp(value, 0.0, max_value);
}

bool finite_pl(const RiskPredictionResult& result) {
  return std::isfinite(result.hpl_pred) && std::isfinite(result.vpl_pred);
}

std::string join_reason(const std::string& prefix, const std::string& reason) {
  if (reason.empty()) {
    return prefix;
  }
  return prefix + ":" + reason;
}

}  // namespace

struct RiskGridSnapshot::Generation {
  RiskGridMapParams params;
  RiskGridHealth health;
  Eigen::Vector3i voxel_num = Eigen::Vector3i::Zero();
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  double stamp_s = std::numeric_limits<double>::quiet_NaN();
  uint64_t generation_id = 0;
  std::vector<RiskVoxel> voxels;

  int layerVoxelCount() const {
    return voxel_num.x() * voxel_num.y() * voxel_num.z();
  }
};

RiskGridSnapshot::RiskGridSnapshot(
    std::shared_ptr<const Generation> generation)
    : generation_(std::move(generation)) {}

RiskGridHealth RiskGridSnapshot::health() const {
  return generation_ ? generation_->health : RiskGridHealth{};
}

double RiskGridSnapshot::stamp_s() const {
  return generation_ ? generation_->stamp_s
                     : std::numeric_limits<double>::quiet_NaN();
}

uint64_t RiskGridSnapshot::generation_id() const {
  return generation_ ? generation_->generation_id : 0;
}

int RiskGridSnapshot::horizonCount() const {
  return generation_ ? static_cast<int>(generation_->params.horizons_s.size())
                     : 0;
}

int RiskGridSnapshot::layerVoxelCount() const {
  return generation_ ? generation_->layerVoxelCount() : 0;
}

const RiskGridMapParams& RiskGridSnapshot::params() const {
  static const RiskGridMapParams kDefaultParams;
  return generation_ ? generation_->params : kDefaultParams;
}

const Eigen::Vector3d& RiskGridSnapshot::origin() const {
  static const Eigen::Vector3d kZero = Eigen::Vector3d::Zero();
  return generation_ ? generation_->origin : kZero;
}

const Eigen::Vector3i& RiskGridSnapshot::voxelNum() const {
  static const Eigen::Vector3i kZero = Eigen::Vector3i::Zero();
  return generation_ ? generation_->voxel_num : kZero;
}

bool RiskGridSnapshot::posToIndex(const Eigen::Vector3d& pos,
                                  Eigen::Vector3i* id) const {
  if (!generation_ || id == nullptr || !pos.allFinite()) {
    return false;
  }
  for (int i = 0; i < 3; ++i) {
    (*id)(i) = static_cast<int>(
        std::floor((pos(i) - generation_->origin(i)) /
                   generation_->params.resolution_m));
  }
  return true;
}

Eigen::Vector3d RiskGridSnapshot::indexToPos(const Eigen::Vector3i& id) const {
  if (!generation_) {
    return Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
  }
  return (id.cast<double>() + Eigen::Vector3d::Constant(0.5)) *
             generation_->params.resolution_m +
         generation_->origin;
}

int RiskGridSnapshot::toAddress(const Eigen::Vector3i& id) const {
  if (!generation_) {
    return -1;
  }
  return id.x() * generation_->voxel_num.y() * generation_->voxel_num.z() +
         id.y() * generation_->voxel_num.z() + id.z();
}

bool RiskGridSnapshot::isInMap(const Eigen::Vector3d& pos) const {
  if (!generation_ || !pos.allFinite()) {
    return false;
  }
  const Eigen::Vector3d max =
      generation_->origin +
      generation_->voxel_num.cast<double>() * generation_->params.resolution_m;
  for (int i = 0; i < 3; ++i) {
    if (pos(i) < generation_->origin(i) + kBoundaryEps ||
        pos(i) > max(i) - kBoundaryEps) {
      return false;
    }
  }
  return true;
}

bool RiskGridSnapshot::isInMap(const Eigen::Vector3i& idx) const {
  if (!generation_) {
    return false;
  }
  for (int i = 0; i < 3; ++i) {
    if (idx(i) < 0 || idx(i) >= generation_->voxel_num(i)) {
      return false;
    }
  }
  return true;
}

namespace {

struct HorizonBracket {
  int lower = -1;
  int upper = -1;
  double weight_upper = 0.0;
};

bool find_horizon_bracket(const std::vector<double>& horizons,
                          const double tau,
                          HorizonBracket* out,
                          std::string* reason) {
  if (out == nullptr || horizons.empty() || !std::isfinite(tau)) {
    if (reason) {
      *reason = "invalid_query_time";
    }
    return false;
  }
  if (tau < horizons.front() || tau > horizons.back()) {
    if (reason) {
      *reason = "time_out_of_horizon";
    }
    return false;
  }
  auto upper_it = std::lower_bound(horizons.begin(), horizons.end(), tau);
  if (upper_it == horizons.end()) {
    out->lower = static_cast<int>(horizons.size()) - 1;
    out->upper = out->lower;
    out->weight_upper = 0.0;
    return true;
  }
  const int upper = static_cast<int>(upper_it - horizons.begin());
  if (std::abs(*upper_it - tau) <= 1.0e-12 || upper == 0) {
    out->lower = upper;
    out->upper = upper;
    out->weight_upper = 0.0;
    return true;
  }
  out->lower = upper - 1;
  out->upper = upper;
  const double dt = horizons[out->upper] - horizons[out->lower];
  out->weight_upper = dt > 0.0 ? (tau - horizons[out->lower]) / dt : 0.0;
  return true;
}

struct SpatialCostInterp {
  double value = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
};

bool validate_corner(const RiskVoxel& voxel,
                     const double query_time_s,
                     const double stale_timeout_s,
                     const bool require_pl,
                     const bool require_cost,
                     std::string* reason) {
  if (voxel.unknown) {
    if (reason) {
      *reason =
          (voxel.source_flags & RISK_GRID_SOURCE_OCCUPIED_SKIP) != 0u
              ? "occupied"
              : "unknown_voxel";
    }
    return false;
  }
  if (!voxel.valid) {
    if (reason) {
      *reason = "invalid_voxel";
    }
    return false;
  }
  const bool stale =
      voxel.stale ||
      !std::isfinite(voxel.stamp_s) ||
      (std::isfinite(query_time_s) && stale_timeout_s >= 0.0 &&
       query_time_s - voxel.stamp_s > stale_timeout_s);
  if (stale) {
    if (reason) {
      *reason = "stale_voxel";
    }
    return false;
  }
  if (require_cost && !std::isfinite(voxel.c_pi)) {
    if (reason) {
      *reason = "invalid_cost";
    }
    return false;
  }
  if (require_pl &&
      (!std::isfinite(voxel.hpl_pred) || !std::isfinite(voxel.vpl_pred))) {
    if (reason) {
      *reason = "invalid_predicted_pl";
    }
    return false;
  }
  return true;
}

const RiskVoxel& voxel_at(const RiskGridSnapshot::Generation& generation,
                          const int horizon_id,
                          const Eigen::Vector3i& id) {
  const int layer_size = generation.layerVoxelCount();
  const int address =
      id.x() * generation.voxel_num.y() * generation.voxel_num.z() +
      id.y() * generation.voxel_num.z() + id.z();
  return generation.voxels[static_cast<std::size_t>(
      horizon_id * layer_size + address)];
}

bool interpolate_cost_layer(const RiskGridSnapshot::Generation& generation,
                            const int horizon_id,
                            const Eigen::Vector3i& base_id,
                            const Eigen::Vector3d& frac,
                            const double query_time_s,
                            SpatialCostInterp* out,
                            std::string* reason) {
  double c[2][2][2]{};
  for (int dx = 0; dx <= 1; ++dx) {
    for (int dy = 0; dy <= 1; ++dy) {
      for (int dz = 0; dz <= 1; ++dz) {
        const Eigen::Vector3i id = base_id + Eigen::Vector3i(dx, dy, dz);
        const RiskVoxel& voxel = voxel_at(generation, horizon_id, id);
        if (!validate_corner(voxel, query_time_s,
                             generation.params.stale_timeout_s,
                             false, true, reason)) {
          return false;
        }
        c[dx][dy][dz] = voxel.c_pi;
      }
    }
  }

  const double fx = frac.x();
  const double fy = frac.y();
  const double fz = frac.z();
  const double wx[2] = {1.0 - fx, fx};
  const double wy[2] = {1.0 - fy, fy};
  const double wz[2] = {1.0 - fz, fz};
  double value = 0.0;
  for (int dx = 0; dx <= 1; ++dx) {
    for (int dy = 0; dy <= 1; ++dy) {
      for (int dz = 0; dz <= 1; ++dz) {
        value += wx[dx] * wy[dy] * wz[dz] * c[dx][dy][dz];
      }
    }
  }

  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  for (int dy = 0; dy <= 1; ++dy) {
    for (int dz = 0; dz <= 1; ++dz) {
      grad.x() += wy[dy] * wz[dz] * (c[1][dy][dz] - c[0][dy][dz]);
    }
  }
  for (int dx = 0; dx <= 1; ++dx) {
    for (int dz = 0; dz <= 1; ++dz) {
      grad.y() += wx[dx] * wz[dz] * (c[dx][1][dz] - c[dx][0][dz]);
    }
  }
  for (int dx = 0; dx <= 1; ++dx) {
    for (int dy = 0; dy <= 1; ++dy) {
      grad.z() += wx[dx] * wy[dy] * (c[dx][dy][1] - c[dx][dy][0]);
    }
  }
  grad /= generation.params.resolution_m;

  out->value = value;
  out->grad = grad;
  return true;
}

struct SpatialPLInterp {
  double hpl = std::numeric_limits<double>::quiet_NaN();
  double vpl = std::numeric_limits<double>::quiet_NaN();
};

bool interpolate_pl_layer(const RiskGridSnapshot::Generation& generation,
                          const int horizon_id,
                          const Eigen::Vector3i& base_id,
                          const Eigen::Vector3d& frac,
                          const double query_time_s,
                          SpatialPLInterp* out,
                          std::string* reason) {
  double h[2][2][2]{};
  double v[2][2][2]{};
  for (int dx = 0; dx <= 1; ++dx) {
    for (int dy = 0; dy <= 1; ++dy) {
      for (int dz = 0; dz <= 1; ++dz) {
        const Eigen::Vector3i id = base_id + Eigen::Vector3i(dx, dy, dz);
        const RiskVoxel& voxel = voxel_at(generation, horizon_id, id);
        if (!validate_corner(voxel, query_time_s,
                             generation.params.stale_timeout_s,
                             true, false, reason)) {
          return false;
        }
        h[dx][dy][dz] = voxel.hpl_pred;
        v[dx][dy][dz] = voxel.vpl_pred;
      }
    }
  }

  const double wx[2] = {1.0 - frac.x(), frac.x()};
  const double wy[2] = {1.0 - frac.y(), frac.y()};
  const double wz[2] = {1.0 - frac.z(), frac.z()};
  double hpl = 0.0;
  double vpl = 0.0;
  for (int dx = 0; dx <= 1; ++dx) {
    for (int dy = 0; dy <= 1; ++dy) {
      for (int dz = 0; dz <= 1; ++dz) {
        const double weight = wx[dx] * wy[dy] * wz[dz];
        hpl += weight * h[dx][dy][dz];
        vpl += weight * v[dx][dy][dz];
      }
    }
  }
  out->hpl = hpl;
  out->vpl = vpl;
  return true;
}

bool trilinear_base(const RiskGridSnapshot& snapshot,
                    const Eigen::Vector3d& p_w,
                    Eigen::Vector3i* base_id,
                    Eigen::Vector3d* frac,
                    std::string* reason) {
  if (!snapshot.isInMap(p_w)) {
    if (reason) {
      *reason = "position_out_of_map";
    }
    return false;
  }
  Eigen::Vector3d grid =
      (p_w - snapshot.origin()) / snapshot.params().resolution_m -
      Eigen::Vector3d::Constant(0.5);
  for (int i = 0; i < 3; ++i) {
    (*base_id)(i) = static_cast<int>(std::floor(grid(i)));
    (*frac)(i) = grid(i) - static_cast<double>((*base_id)(i));
  }
  const Eigen::Vector3i upper_id = *base_id + Eigen::Vector3i::Ones();
  if (!snapshot.isInMap(*base_id) || !snapshot.isInMap(upper_id)) {
    if (reason) {
      *reason = "position_out_of_interpolation_bounds";
    }
    return false;
  }
  return true;
}

}  // namespace

bool RiskGridSnapshot::queryCost(const Eigen::Vector3d& p_w,
                                 const double query_time_s,
                                 RiskCostSample* out) const {
  if (out == nullptr) {
    return false;
  }
  *out = RiskCostSample{};
  out->cost = params().unknown_cost;
  out->generation_id = generation_id();
  if (!generation_) {
    out->reason = "snapshot_not_ready";
    return false;
  }
  if (!p_w.allFinite() || !std::isfinite(query_time_s)) {
    out->reason = "invalid_query";
    return false;
  }
  const double tau = query_time_s - generation_->stamp_s;
  HorizonBracket bracket;
  std::string reason;
  if (!find_horizon_bracket(generation_->params.horizons_s, tau,
                            &bracket, &reason)) {
    out->reason = reason;
    return false;
  }
  Eigen::Vector3i base_id;
  Eigen::Vector3d frac;
  if (!trilinear_base(*this, p_w, &base_id, &frac, &reason)) {
    out->reason = reason;
    return false;
  }

  SpatialCostInterp lower;
  if (!interpolate_cost_layer(*generation_, bracket.lower, base_id, frac,
                              query_time_s, &lower, &reason)) {
    out->reason = reason;
    return false;
  }
  SpatialCostInterp upper = lower;
  if (bracket.upper != bracket.lower) {
    if (!interpolate_cost_layer(*generation_, bracket.upper, base_id, frac,
                                query_time_s, &upper, &reason)) {
      out->reason = reason;
      return false;
    }
  }

  const double w = bracket.weight_upper;
  out->cost = (1.0 - w) * lower.value + w * upper.value;
  out->grad = (1.0 - w) * lower.grad + w * upper.grad;
  out->valid = true;
  out->stale = false;
  out->reason = "ok";
  return true;
}

bool RiskGridSnapshot::queryPredictedPL(const Eigen::Vector3d& p_w,
                                        const double query_time_s,
                                        PredictedPLSample* out) const {
  if (out == nullptr) {
    return false;
  }
  *out = PredictedPLSample{};
  out->query_time_s = query_time_s;
  out->generation_id = generation_id();
  if (!generation_) {
    out->reason = "snapshot_not_ready";
    return false;
  }
  if (!p_w.allFinite() || !std::isfinite(query_time_s)) {
    out->reason = "invalid_query";
    return false;
  }
  const double tau = query_time_s - generation_->stamp_s;
  HorizonBracket bracket;
  std::string reason;
  if (!find_horizon_bracket(generation_->params.horizons_s, tau,
                            &bracket, &reason)) {
    out->reason = reason;
    return false;
  }
  Eigen::Vector3i base_id;
  Eigen::Vector3d frac;
  if (!trilinear_base(*this, p_w, &base_id, &frac, &reason)) {
    out->reason = reason;
    return false;
  }

  SpatialPLInterp lower;
  if (!interpolate_pl_layer(*generation_, bracket.lower, base_id, frac,
                            query_time_s, &lower, &reason)) {
    out->reason = reason;
    return false;
  }
  SpatialPLInterp upper = lower;
  if (bracket.upper != bracket.lower) {
    if (!interpolate_pl_layer(*generation_, bracket.upper, base_id, frac,
                              query_time_s, &upper, &reason)) {
      out->reason = reason;
      return false;
    }
  }

  const double w = bracket.weight_upper;
  out->hpl_pred = (1.0 - w) * lower.hpl + w * upper.hpl;
  out->vpl_pred = (1.0 - w) * lower.vpl + w * upper.vpl;
  out->available = true;
  out->valid = true;
  out->stale = false;
  out->reason = "ok";
  return true;
}

bool RiskGridSnapshot::voxelAt(const int horizon_id,
                               const Eigen::Vector3i& id,
                               RiskVoxel* out) const {
  if (out == nullptr) {
    return false;
  }
  *out = RiskVoxel{};
  if (!generation_ || horizon_id < 0 ||
      horizon_id >= static_cast<int>(generation_->params.horizons_s.size()) ||
      !isInMap(id)) {
    return false;
  }
  const int address = toAddress(id);
  const int layer_size = generation_->layerVoxelCount();
  const std::size_t index =
      static_cast<std::size_t>(horizon_id * layer_size + address);
  if (index >= generation_->voxels.size()) {
    return false;
  }
  *out = generation_->voxels[index];
  return true;
}

RiskGridMap::RiskGridMap() {
  std::string ignored;
  configure(params_, &ignored);
}

RiskGridMap::RiskGridMap(RiskGridMapParams params) {
  std::string ignored;
  configure(std::move(params), &ignored);
}

bool RiskGridMap::configure(RiskGridMapParams params, std::string* reason) {
  std::string local_reason;
  if (!validateParams(params, &local_reason)) {
    if (reason) {
      *reason = local_reason;
    }
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  params_ = std::move(params);
  for (int i = 0; i < 3; ++i) {
    const double size = i == 0 ? params_.size_x_m
                        : i == 1 ? params_.size_y_m
                                 : params_.size_z_m;
    voxel_num_(i) = static_cast<int>(std::ceil(size / params_.resolution_m));
  }
  origin_ = Eigen::Vector3d(-0.5 * params_.size_x_m,
                            -0.5 * params_.size_y_m,
                            -0.5 * params_.size_z_m);
  next_generation_id_ = 1;
  active_.reset();
  health_ = RiskGridHealth{};
  if (reason) {
    *reason = "ok";
  }
  return true;
}

RiskGridHealth RiskGridMap::health() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return health_;
}

RiskGridHealth RiskGridMap::health(const double now_s) const {
  std::lock_guard<std::mutex> lock(mutex_);
  RiskGridHealth out = health_;
  if (active_ && std::isfinite(now_s) && std::isfinite(active_->stamp_s)) {
    out.age_s = std::max(0.0, now_s - active_->stamp_s);
    out.stale = out.age_s > params_.stale_timeout_s;
    if (out.stale && out.reason == "ok") {
      out.reason = "stale_snapshot";
    }
  }
  return out;
}

std::shared_ptr<const RiskGridSnapshot> RiskGridMap::acquireSnapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) {
    return nullptr;
  }
  return std::shared_ptr<const RiskGridSnapshot>(
      new RiskGridSnapshot(active_));
}

bool RiskGridMap::posToIndex(const Eigen::Vector3d& pos,
                             Eigen::Vector3i* id) const {
  if (id == nullptr || !pos.allFinite()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (int i = 0; i < 3; ++i) {
    (*id)(i) =
        static_cast<int>(std::floor((pos(i) - origin_(i)) /
                                    params_.resolution_m));
  }
  return true;
}

Eigen::Vector3d RiskGridMap::indexToPos(const Eigen::Vector3i& id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return (id.cast<double>() + Eigen::Vector3d::Constant(0.5)) *
             params_.resolution_m +
         origin_;
}

int RiskGridMap::toAddress(const Eigen::Vector3i& id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return id.x() * voxel_num_.y() * voxel_num_.z() +
         id.y() * voxel_num_.z() + id.z();
}

bool RiskGridMap::isInMap(const Eigen::Vector3d& pos) const {
  if (!pos.allFinite()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const Eigen::Vector3d max =
      origin_ + voxel_num_.cast<double>() * params_.resolution_m;
  for (int i = 0; i < 3; ++i) {
    if (pos(i) < origin_(i) + kBoundaryEps ||
        pos(i) > max(i) - kBoundaryEps) {
      return false;
    }
  }
  return true;
}

bool RiskGridMap::isInMap(const Eigen::Vector3i& idx) const {
  std::lock_guard<std::mutex> lock(mutex_);
  for (int i = 0; i < 3; ++i) {
    if (idx(i) < 0 || idx(i) >= voxel_num_(i)) {
      return false;
    }
  }
  return true;
}

bool RiskGridMap::refreshFromProvider(const Eigen::Vector3d& uav_position_w,
                                      const double now_s,
                                      RiskPredictionProvider& provider,
                                      std::string* reason) {
  return refreshFromProvider(uav_position_w, now_s, provider,
                             OccupancyPredicate{}, reason);
}

bool RiskGridMap::refreshFromProvider(const Eigen::Vector3d& uav_position_w,
                                      const double now_s,
                                      RiskPredictionProvider& provider,
                                      const OccupancyPredicate& is_occupied,
                                      std::string* reason) {
  if (!uav_position_w.allFinite() || !std::isfinite(now_s)) {
    if (reason) {
      *reason = "invalid_refresh_input";
    }
    markRefreshFailure(now_s, "invalid_refresh_input");
    return false;
  }

  RiskGridMapParams params_copy;
  Eigen::Vector3i voxel_num_copy;
  Eigen::Vector3d origin_copy;
  uint64_t generation_id = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    updateGeometry(uav_position_w);
    params_copy = params_;
    voxel_num_copy = voxel_num_;
    origin_copy = origin_;
    generation_id = next_generation_id_;
  }

  const int layer_size =
      voxel_num_copy.x() * voxel_num_copy.y() * voxel_num_copy.z();
  const int horizon_count = static_cast<int>(params_copy.horizons_s.size());
  const int total_voxel_count = layer_size * horizon_count;
  std::vector<RiskPredictionQuery> queries;
  std::vector<std::size_t> query_voxel_indices;
  std::vector<RiskPredictionQuery> voxel_queries(
      static_cast<std::size_t>(total_voxel_count));
  std::vector<bool> occupied_skip(static_cast<std::size_t>(total_voxel_count),
                                  false);
  uint64_t occupied_skip_count = 0;
  queries.reserve(static_cast<std::size_t>(total_voxel_count));
  query_voxel_indices.reserve(static_cast<std::size_t>(total_voxel_count));
  for (int h = 0; h < horizon_count; ++h) {
    for (int x = 0; x < voxel_num_copy.x(); ++x) {
      for (int y = 0; y < voxel_num_copy.y(); ++y) {
        for (int z = 0; z < voxel_num_copy.z(); ++z) {
          const Eigen::Vector3i id(x, y, z);
          const int address =
              x * voxel_num_copy.y() * voxel_num_copy.z() +
              y * voxel_num_copy.z() + z;
          const std::size_t voxel_index =
              static_cast<std::size_t>(h * layer_size + address);
          RiskPredictionQuery query;
          query.position_w =
              (id.cast<double>() + Eigen::Vector3d::Constant(0.5)) *
                  params_copy.resolution_m +
              origin_copy;
          query.horizon_s = params_copy.horizons_s[static_cast<std::size_t>(h)];
          query.query_time_s = now_s + query.horizon_s;
          voxel_queries[voxel_index] = query;
          if (params_copy.skip_occupied_voxels && is_occupied &&
              is_occupied(query.position_w)) {
            occupied_skip[voxel_index] = true;
            ++occupied_skip_count;
            continue;
          }
          queries.push_back(query);
          query_voxel_indices.push_back(voxel_index);
        }
      }
    }
  }

  std::vector<RiskPredictionResult> results;
  if (!queries.empty()) {
    if (!provider.batchQuery(queries, &results) ||
        results.size() != queries.size()) {
      const std::string failure = "provider_refresh_failed";
      if (reason) {
        *reason = failure;
      }
      markRefreshFailure(now_s, failure);
      return false;
    }
  }

  auto next = std::make_shared<RiskGridSnapshot::Generation>();
  next->params = params_copy;
  next->voxel_num = voxel_num_copy;
  next->origin = origin_copy;
  next->stamp_s = now_s;
  next->generation_id = generation_id;
  next->voxels.resize(static_cast<std::size_t>(total_voxel_count));

  std::vector<RiskPredictionResult> indexed_results(
      static_cast<std::size_t>(total_voxel_count));
  std::vector<bool> has_provider_result(
      static_cast<std::size_t>(total_voxel_count), false);
  for (std::size_t i = 0; i < results.size(); ++i) {
    const std::size_t voxel_index = query_voxel_indices[i];
    indexed_results[voxel_index] = results[i];
    has_provider_result[voxel_index] = true;
  }

  uint64_t valid_count = 0;
  uint64_t unknown_count = 0;
  uint64_t provider_stale_count = 0;
  uint64_t provider_invalid_count = 0;
  uint64_t predictor_gnss_used_count = 0;
  uint64_t predictor_lidar_used_count = 0;
  uint64_t predictor_prior_used_count = 0;
  uint64_t predictor_regularized_count = 0;
  uint64_t predictor_conservative_max_count = 0;
  std::unordered_map<std::string, uint64_t> unknown_reason_counts;
  const auto record_unknown_reason = [&unknown_reason_counts](
                                         const std::string& reason) {
    ++unknown_reason_counts[reason.empty() ? "provider_invalid" : reason];
  };
  for (std::size_t i = 0; i < next->voxels.size(); ++i) {
    RiskVoxel voxel;
    voxel.stamp_s = voxel_queries[i].query_time_s;
    if (occupied_skip[i]) {
      voxel.source_flags = RISK_GRID_SOURCE_OCCUPIED_SKIP;
      voxel.valid = false;
      voxel.stale = false;
      voxel.unknown = true;
      voxel.c_pi = params_copy.unknown_cost;
      ++unknown_count;
      record_unknown_reason("occupied_skip");
      next->voxels[i] = voxel;
      continue;
    }

    const RiskPredictionResult& result = indexed_results[i];
    voxel.source_flags = result.source_flags;
    if ((voxel.source_flags & PREDICTOR_RESULT_GNSS_USED) != 0u) {
      ++predictor_gnss_used_count;
    }
    if ((voxel.source_flags & PREDICTOR_RESULT_LIDAR_USED) != 0u) {
      ++predictor_lidar_used_count;
    }
    if ((voxel.source_flags & PREDICTOR_RESULT_PRIOR_VALID) != 0u) {
      ++predictor_prior_used_count;
    }
    if ((voxel.source_flags & PREDICTOR_RESULT_REGULARIZED) != 0u) {
      ++predictor_regularized_count;
    }
    if ((voxel.source_flags & PREDICTOR_RESULT_CONSERVATIVE_MAX) != 0u) {
      ++predictor_conservative_max_count;
    }
    voxel.valid = has_provider_result[i] &&
        result.available && result.valid && !result.stale && finite_pl(result);
    voxel.stale = result.stale;
    voxel.unknown = !voxel.valid;
    if (voxel.valid) {
      voxel.hpl_pred = result.hpl_pred;
      voxel.vpl_pred = result.vpl_pred;
      voxel.c_pi = clamp_cost(std::max(result.hpl_pred, result.vpl_pred),
                              params_copy.cost_max);
      ++valid_count;
    } else {
      if (has_provider_result[i] && result.stale) {
        ++provider_stale_count;
        record_unknown_reason(result.reason.empty() ? "provider_stale"
                                                    : result.reason);
      } else {
        ++provider_invalid_count;
        record_unknown_reason("provider_invalid");
      }
      voxel.c_pi = params_copy.unknown_cost;
      ++unknown_count;
    }
    next->voxels[i] = voxel;
  }

  RiskGridHealth new_health;
  new_health.ready = true;
  new_health.stale = false;
  new_health.age_s = 0.0;
  const double health_denominator = total_voxel_count > 0
      ? static_cast<double>(total_voxel_count)
      : 1.0;
  new_health.valid_ratio =
      static_cast<double>(valid_count) / health_denominator;
  new_health.unknown_ratio =
      static_cast<double>(unknown_count) / health_denominator;
  new_health.generation_id = generation_id;
  new_health.provider_query_count = static_cast<uint64_t>(queries.size());
  new_health.occupied_skip_count = occupied_skip_count;
  new_health.provider_stale_count = provider_stale_count;
  new_health.provider_invalid_count = provider_invalid_count;
  new_health.predictor_gnss_used_count = predictor_gnss_used_count;
  new_health.predictor_lidar_used_count = predictor_lidar_used_count;
  new_health.predictor_prior_used_count = predictor_prior_used_count;
  new_health.predictor_regularized_count = predictor_regularized_count;
  new_health.predictor_conservative_max_count =
      predictor_conservative_max_count;
  new_health.reason = "ok";
  if (valid_count == 0u &&
      unknown_count == static_cast<uint64_t>(total_voxel_count)) {
    uint64_t dominant_count = 0;
    std::string dominant_reason = "provider_invalid";
    for (const auto& entry : unknown_reason_counts) {
      if (entry.second > dominant_count) {
        dominant_reason = entry.first;
        dominant_count = entry.second;
      }
    }
    new_health.reason = dominant_reason;
  }
  next->health = new_health;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = next;
    health_ = new_health;
    next_generation_id_ = generation_id + 1;
  }
  if (reason) {
    *reason = "ok";
  }
  return true;
}

void RiskGridMap::markRefreshFailure(const double now_s,
                                     const std::string& reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) {
    health_ = RiskGridHealth{};
    health_.reason = reason.empty() ? "refresh_failed" : reason;
    return;
  }
  health_ = active_->health;
  health_.age_s =
      std::isfinite(now_s) && std::isfinite(active_->stamp_s)
          ? std::max(0.0, now_s - active_->stamp_s)
          : std::numeric_limits<double>::infinity();
  health_.stale = health_.age_s > params_.stale_timeout_s;
  health_.reason = reason.empty() ? "refresh_failed" : reason;
}

bool RiskGridMap::validateParams(const RiskGridMapParams& params,
                                 std::string* reason) const {
  if (params.frame_id.empty()) {
    if (reason) *reason = "empty_frame_id";
    return false;
  }
  if (!finite_positive(params.resolution_m) ||
      !finite_positive(params.size_x_m) ||
      !finite_positive(params.size_y_m) ||
      !finite_positive(params.size_z_m)) {
    if (reason) *reason = "invalid_geometry";
    return false;
  }
  if (params.horizons_s.empty()) {
    if (reason) *reason = "empty_horizons";
    return false;
  }
  for (std::size_t i = 0; i < params.horizons_s.size(); ++i) {
    if (!finite_nonnegative(params.horizons_s[i])) {
      if (reason) *reason = "invalid_horizon";
      return false;
    }
    if (i > 0 && params.horizons_s[i] <= params.horizons_s[i - 1]) {
      if (reason) *reason = "unsorted_horizons";
      return false;
    }
  }
  if (!finite_nonnegative(params.refresh_period_s) ||
      !std::isfinite(params.stale_timeout_s) ||
      !std::isfinite(params.unknown_cost) ||
      !finite_positive(params.cost_max)) {
    if (reason) *reason = "invalid_timing_or_cost";
    return false;
  }
  if (reason) {
    *reason = "ok";
  }
  return true;
}

void RiskGridMap::updateGeometry(const Eigen::Vector3d& center_w) {
  origin_ = center_w - Eigen::Vector3d(0.5 * params_.size_x_m,
                                      0.5 * params_.size_y_m,
                                      0.5 * params_.size_z_m);
}

}  // namespace iap
