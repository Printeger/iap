#include <iap/planner/unified_risk_grid.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace iap {
namespace {

constexpr uint32_t kValidityFlags =
    VALID_ESDF | VALID_OCCUPANCY | VALID_AL | VALID_ADVISORY_PL | VALID_PI |
    GNSS_FIM_VALID | LIDAR_FIM_VALID | PI_INPUT_VALID;

constexpr uint32_t kProblemFlags =
    STALE_PL | UNKNOWN_RISK | OCCUPIED | OUT_OF_RANGE | FIM_ADD_USED;

int axis_count(const double extent_m, const double res) {
  return std::max(1, static_cast<int>(std::floor(2.0 * extent_m / res)) + 1);
}

float nanf() {
  return std::numeric_limits<float>::quiet_NaN();
}

double lerp(const double a, const double b, const double t) {
  return a * (1.0 - t) + b * t;
}

bool finite(const float value) {
  return std::isfinite(static_cast<double>(value));
}

bool all_have(const std::vector<const UnifiedRiskVoxel*>& cells,
              const uint32_t flag) {
  return std::all_of(cells.begin(), cells.end(), [flag](const auto* cell) {
    return (cell->flags & flag) != 0u;
  });
}

float weighted(const std::vector<const UnifiedRiskVoxel*>& cells,
               const std::vector<double>& weights,
               float UnifiedRiskVoxel::*field) {
  double out = 0.0;
  for (std::size_t i = 0; i < cells.size(); ++i) {
    const float value = cells[i]->*field;
    if (!finite(value)) {
      return nanf();
    }
    out += weights[i] * static_cast<double>(value);
  }
  return static_cast<float>(out);
}

double oldest_finite_timestamp(const std::vector<const UnifiedRiskVoxel*>& cells) {
  double oldest = std::numeric_limits<double>::infinity();
  for (const auto* cell : cells) {
    const double value = cell->updated_time_s;
    if (std::isfinite(value)) {
      oldest = std::min(oldest, value);
    }
  }
  return std::isfinite(oldest)
             ? oldest
             : std::numeric_limits<double>::quiet_NaN();
}

void set_unknown(UnifiedRiskQueryResult* out, const Eigen::Vector3d& p) {
  out->valid = false;
  out->grid_hit = false;
  out->grid_miss = true;
  out->direct_query_used = false;
  out->position = p;
  out->voxel.flags |= UNKNOWN_RISK | OUT_OF_RANGE;
  out->flags = out->voxel.flags;
  out->query_source = "miss";
}

}  // namespace

bool UnifiedRiskGrid::reset(const Eigen::Vector3d& center,
                            const double half_extent_x_m,
                            const double half_extent_y_m,
                            const int z_slices,
                            const double resolution_m) {
  if (!center.allFinite() || half_extent_x_m <= 0.0 || half_extent_y_m <= 0.0 ||
      z_slices <= 0 || resolution_m <= 0.0 ||
      !std::isfinite(half_extent_x_m) ||
      !std::isfinite(half_extent_y_m) ||
      !std::isfinite(resolution_m)) {
    valid_ = false;
    cells_.clear();
    return false;
  }

  center_ = center;
  resolution_ = resolution_m;
  nx_ = axis_count(half_extent_x_m, resolution_m);
  ny_ = axis_count(half_extent_y_m, resolution_m);
  nz_ = std::max(1, z_slices);
  min_corner_ = Eigen::Vector3d(center.x() - 0.5 * resolution_m * (nx_ - 1),
                                center.y() - 0.5 * resolution_m * (ny_ - 1),
                                center.z() - 0.5 * resolution_m * (nz_ - 1));
  cells_.assign(static_cast<std::size_t>(nx_ * ny_ * nz_), UnifiedRiskVoxel{});
  generation_ = -1;
  stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  build_time_ms_ = std::numeric_limits<double>::quiet_NaN();
  valid_ = true;
  stats_.active = true;
  return true;
}

bool UnifiedRiskGrid::contains(const Eigen::Vector3d& p) const {
  if (!valid_ || !p.allFinite()) {
    return false;
  }
  const Eigen::Vector3d max_corner =
      min_corner_ + resolution_ * Eigen::Vector3d(nx_ - 1, ny_ - 1, nz_ - 1);
  const double z_pad = nz_ == 1 ? 0.5 * resolution_ : 0.0;
  return p.x() >= min_corner_.x() && p.y() >= min_corner_.y() &&
         p.z() >= min_corner_.z() - z_pad && p.x() <= max_corner.x() &&
         p.y() <= max_corner.y() && p.z() <= max_corner.z() + z_pad;
}

bool UnifiedRiskGrid::cell_index(const Eigen::Vector3d& p,
                                 int* ix,
                                 int* iy,
                                 int* iz) const {
  if (!contains(p)) {
    return false;
  }
  const Eigen::Vector3d rel = (p - min_corner_) / resolution_;
  *ix = std::clamp(static_cast<int>(std::floor(rel.x())), 0, nx_ - 1);
  *iy = std::clamp(static_cast<int>(std::floor(rel.y())), 0, ny_ - 1);
  *iz = nz_ == 1 ? 0 : std::clamp(static_cast<int>(std::floor(rel.z())), 0, nz_ - 1);
  return true;
}

UnifiedRiskVoxel& UnifiedRiskGrid::at(const int ix, const int iy, const int iz) {
  if (!in_bounds(ix, iy, iz)) {
    throw std::out_of_range("UnifiedRiskGrid index out of range");
  }
  return cells_.at(flat_index(ix, iy, iz));
}

const UnifiedRiskVoxel& UnifiedRiskGrid::at(const int ix,
                                            const int iy,
                                            const int iz) const {
  if (!in_bounds(ix, iy, iz)) {
    throw std::out_of_range("UnifiedRiskGrid index out of range");
  }
  return cells_.at(flat_index(ix, iy, iz));
}

Eigen::Vector3d UnifiedRiskGrid::position(const int ix,
                                          const int iy,
                                          const int iz) const {
  return min_corner_ + resolution_ * Eigen::Vector3d(ix, iy, iz);
}

UnifiedRiskQueryResult UnifiedRiskGrid::interpolate(const Eigen::Vector3d& p) const {
  UnifiedRiskQueryResult out;
  out.position = p;
  if (!contains(p)) {
    set_unknown(&out, p);
    return out;
  }

  int ix = 0;
  int iy = 0;
  int iz = 0;
  if (!cell_index(p, &ix, &iy, &iz)) {
    set_unknown(&out, p);
    return out;
  }

  const int ix1 = nx_ == 1 ? ix : std::min(ix + 1, nx_ - 1);
  const int iy1 = ny_ == 1 ? iy : std::min(iy + 1, ny_ - 1);
  const int iz1 = nz_ == 1 ? iz : std::min(iz + 1, nz_ - 1);
  const Eigen::Vector3d rel = (p - min_corner_) / resolution_;
  const double tx = nx_ == 1 ? 0.0 : std::clamp(rel.x() - std::floor(rel.x()), 0.0, 1.0);
  const double ty = ny_ == 1 ? 0.0 : std::clamp(rel.y() - std::floor(rel.y()), 0.0, 1.0);
  const double tz = nz_ == 1 ? 0.0 : std::clamp(rel.z() - std::floor(rel.z()), 0.0, 1.0);

  std::vector<const UnifiedRiskVoxel*> cells;
  std::vector<double> weights;
  const auto add = [&](const int x, const int y, const int z, const double w) {
    if (w <= 0.0) {
      return;
    }
    const auto* cell = &at(x, y, z);
    const auto duplicate = std::find(cells.begin(), cells.end(), cell);
    if (duplicate == cells.end()) {
      cells.push_back(cell);
      weights.push_back(w);
    } else {
      weights[static_cast<std::size_t>(duplicate - cells.begin())] += w;
    }
  };
  add(ix, iy, iz, (1.0 - tx) * (1.0 - ty) * (1.0 - tz));
  add(ix1, iy, iz, tx * (1.0 - ty) * (1.0 - tz));
  add(ix, iy1, iz, (1.0 - tx) * ty * (1.0 - tz));
  add(ix1, iy1, iz, tx * ty * (1.0 - tz));
  add(ix, iy, iz1, (1.0 - tx) * (1.0 - ty) * tz);
  add(ix1, iy, iz1, tx * (1.0 - ty) * tz);
  add(ix, iy1, iz1, (1.0 - tx) * ty * tz);
  add(ix1, iy1, iz1, tx * ty * tz);

  if (cells.empty()) {
    set_unknown(&out, p);
    return out;
  }

  uint32_t valid_flags = kValidityFlags;
  uint32_t problem_flags = 0;
  for (const auto* cell : cells) {
    valid_flags &= cell->flags;
    problem_flags |= cell->flags & kProblemFlags;
  }
  out.voxel.flags = valid_flags | problem_flags;
  out.flags = out.voxel.flags;
  out.voxel.updated_time_s = oldest_finite_timestamp(cells);
  out.voxel.age_s = weighted(cells, weights, &UnifiedRiskVoxel::age_s);

  if (all_have(cells, VALID_ESDF)) {
    out.voxel.esdf_m = weighted(cells, weights, &UnifiedRiskVoxel::esdf_m);
  }
  if (all_have(cells, VALID_OCCUPANCY)) {
    out.voxel.occ_prob = weighted(cells, weights, &UnifiedRiskVoxel::occ_prob);
  }
  if (all_have(cells, VALID_AL)) {
    out.voxel.al_h_m = weighted(cells, weights, &UnifiedRiskVoxel::al_h_m);
    out.voxel.al_v_m = weighted(cells, weights, &UnifiedRiskVoxel::al_v_m);
    out.voxel.hal_m = weighted(cells, weights, &UnifiedRiskVoxel::hal_m);
    out.voxel.val_m = weighted(cells, weights, &UnifiedRiskVoxel::val_m);
  }
  if (all_have(cells, VALID_ADVISORY_PL)) {
    out.voxel.hpl_adv_m = weighted(cells, weights, &UnifiedRiskVoxel::hpl_adv_m);
    out.voxel.vpl_adv_m = weighted(cells, weights, &UnifiedRiskVoxel::vpl_adv_m);
    out.voxel.pl_adv_m = weighted(cells, weights, &UnifiedRiskVoxel::pl_adv_m);
    out.voxel.im_h_adv_m = weighted(cells, weights, &UnifiedRiskVoxel::im_h_adv_m);
    out.voxel.im_v_adv_m = weighted(cells, weights, &UnifiedRiskVoxel::im_v_adv_m);
    out.voxel.im_min_adv_m = weighted(cells, weights, &UnifiedRiskVoxel::im_min_adv_m);
  }
  if (all_have(cells, VALID_PI)) {
    out.voxel.pi_cost = weighted(cells, weights, &UnifiedRiskVoxel::pi_cost);
    out.voxel.pi_grad_x = weighted(cells, weights, &UnifiedRiskVoxel::pi_grad_x);
    out.voxel.pi_grad_y = weighted(cells, weights, &UnifiedRiskVoxel::pi_grad_y);
    out.voxel.pi_grad_z = weighted(cells, weights, &UnifiedRiskVoxel::pi_grad_z);
  } else {
    out.voxel.flags |= UNKNOWN_RISK;
    out.voxel.pi_cost = nanf();
  }

  out.flags = out.voxel.flags;
  out.valid = (out.flags & VALID_PI) != 0u && finite(out.voxel.pi_cost);
  out.grid_hit = true;
  out.grid_miss = false;
  out.grid_generation = generation_;
  out.query_source = "grid";
  return out;
}

UnifiedRiskQueryResult UnifiedRiskGrid::queryRisk(
    const Eigen::Vector3d& p,
    const UnifiedRiskQueryOptions& options) {
  ++stats_.query_count;
  auto out = interpolate(p);
  if (out.grid_hit) {
    ++stats_.grid_hit_count;
    apply_unified_risk_stale_policy(&out, options, stamp_s_);
  } else {
    ++stats_.grid_miss_count;
    if (options.direct_query_on_miss && options.direct_query) {
      UnifiedRiskVoxel direct;
      if (options.direct_query(p, &direct)) {
        out.valid = (direct.flags & VALID_PI) != 0u && finite(direct.pi_cost);
        out.grid_hit = false;
        out.grid_miss = true;
        out.direct_query_used = true;
        out.position = p;
        out.voxel = direct;
        out.flags = direct.flags;
        out.grid_generation = generation_;
        out.query_source = "direct";
        ++stats_.direct_query_count;
        apply_unified_risk_stale_policy(&out, options, options.now_s);
      }
    }
    if (!out.direct_query_used) {
      out.voxel.flags |= UNKNOWN_RISK | OUT_OF_RANGE;
      out.flags = out.voxel.flags;
      out.voxel.pi_cost = static_cast<float>(
          std::max(0.0, options.unknown_penalty));
      out.unknown_penalty = out.voxel.pi_cost;
      out.query_source = "unknown";
    }
  }

  if ((out.flags & STALE_PL) != 0u) {
    ++stats_.stale_count;
  }
  if ((out.flags & UNKNOWN_RISK) != 0u) {
    ++stats_.unknown_count;
  }
  if ((out.flags & VALID_PI) != 0u) {
    ++stats_.valid_pi_count;
  }
  if (out.unknown_penalty > 0.0f) {
    ++stats_.unknown_penalty_count;
  }
  if (finite(out.voxel.age_s)) {
    const double age = static_cast<double>(out.voxel.age_s);
    stats_.max_age_s = std::isfinite(stats_.max_age_s)
                           ? std::max(stats_.max_age_s, age)
                           : age;
  }
  ++stats_.flags_histogram[out.flags];
  return out;
}

void UnifiedRiskGrid::compute_gradients() {
  if (!valid_) {
    return;
  }
  const auto cost_at = [&](const int ix, const int iy, const int iz) {
    const auto& voxel = at(ix, iy, iz);
    return ((voxel.flags & VALID_PI) != 0u && finite(voxel.pi_cost))
               ? static_cast<double>(voxel.pi_cost)
               : std::numeric_limits<double>::quiet_NaN();
  };
  for (int iz = 0; iz < nz_; ++iz) {
    for (int iy = 0; iy < ny_; ++iy) {
      for (int ix = 0; ix < nx_; ++ix) {
        auto& voxel = at(ix, iy, iz);
        if ((voxel.flags & VALID_PI) == 0u) {
          continue;
        }
        const int xm = std::max(0, ix - 1);
        const int xp = std::min(nx_ - 1, ix + 1);
        const int ym = std::max(0, iy - 1);
        const int yp = std::min(ny_ - 1, iy + 1);
        const int zm = std::max(0, iz - 1);
        const int zp = std::min(nz_ - 1, iz + 1);
        const double cxm = cost_at(xm, iy, iz);
        const double cxp = cost_at(xp, iy, iz);
        const double cym = cost_at(ix, ym, iz);
        const double cyp = cost_at(ix, yp, iz);
        const double czm = cost_at(ix, iy, zm);
        const double czp = cost_at(ix, iy, zp);
        voxel.pi_grad_x = std::isfinite(cxm) && std::isfinite(cxp) && xp != xm
                              ? static_cast<float>((cxp - cxm) /
                                                   ((xp - xm) * resolution_))
                              : 0.0f;
        voxel.pi_grad_y = std::isfinite(cym) && std::isfinite(cyp) && yp != ym
                              ? static_cast<float>((cyp - cym) /
                                                   ((yp - ym) * resolution_))
                              : 0.0f;
        voxel.pi_grad_z = std::isfinite(czm) && std::isfinite(czp) && zp != zm
                              ? static_cast<float>((czp - czm) /
                                                   ((zp - zm) * resolution_))
                              : 0.0f;
      }
    }
  }
}

void UnifiedRiskGrid::zero_gradients() {
  if (!valid_) {
    return;
  }
  for (auto& voxel : cells_) {
    if ((voxel.flags & VALID_PI) != 0u) {
      voxel.pi_grad_x = 0.0f;
      voxel.pi_grad_y = 0.0f;
      voxel.pi_grad_z = 0.0f;
    }
  }
}

void UnifiedRiskGrid::note_update(const double build_time_ms,
                                  const UpdateTiming& timing) {
  build_time_ms_ = build_time_ms;
  ++stats_.update_count;
  stats_.active = valid_;
  stats_.generation = generation_;
  stats_.last_timing = timing;
  update_times_ms_.push_back(build_time_ms);
  double sum = 0.0;
  for (const double value : update_times_ms_) {
    sum += value;
  }
  stats_.mean_update_ms =
      update_times_ms_.empty() ? std::numeric_limits<double>::quiet_NaN()
                               : sum / static_cast<double>(update_times_ms_.size());
  auto sorted = update_times_ms_;
  std::sort(sorted.begin(), sorted.end());
  const std::size_t idx =
      sorted.empty() ? 0 : std::min(sorted.size() - 1,
                                    static_cast<std::size_t>(
                                        std::ceil(0.95 * sorted.size()) - 1));
  stats_.p95_update_ms =
      sorted.empty() ? std::numeric_limits<double>::quiet_NaN() : sorted[idx];
}

std::size_t UnifiedRiskGrid::flat_index(const int ix,
                                        const int iy,
                                        const int iz) const {
  return static_cast<std::size_t>((ix * ny_ + iy) * nz_ + iz);
}

bool UnifiedRiskGrid::in_bounds(const int ix, const int iy, const int iz) const {
  return ix >= 0 && ix < nx_ && iy >= 0 && iy < ny_ && iz >= 0 && iz < nz_;
}

double unified_risk_unknown_penalty(const double age_s,
                                    const double unknown_penalty,
                                    const double unknown_tau_s) {
  const double c_unknown =
      std::isfinite(unknown_penalty) ? std::max(0.0, unknown_penalty) : 3000.0;
  const double tau =
      std::isfinite(unknown_tau_s) && unknown_tau_s > 1.0e-6
          ? unknown_tau_s
          : 2.0;
  const double age = std::isfinite(age_s) ? std::max(0.0, age_s) : tau;
  return c_unknown * (1.0 - std::exp(-age / tau));
}

void apply_unified_risk_stale_policy(UnifiedRiskQueryResult* result,
                                     const UnifiedRiskQueryOptions& options,
                                     const double grid_stamp_s) {
  if (!result) {
    return;
  }
  double age = std::numeric_limits<double>::quiet_NaN();
  if (std::isfinite(options.now_s) &&
      std::isfinite(result->voxel.updated_time_s)) {
    age = std::max(
        0.0, options.now_s - result->voxel.updated_time_s);
    result->voxel.age_s = static_cast<float>(age);
  } else if (finite(result->voxel.age_s)) {
    age = result->voxel.age_s;
  } else if (std::isfinite(options.now_s) && std::isfinite(grid_stamp_s)) {
    age = std::max(0.0, options.now_s - grid_stamp_s);
    result->voxel.age_s = static_cast<float>(age);
  }

  const double fresh =
      std::isfinite(options.fresh_timeout_s) ? std::max(0.0, options.fresh_timeout_s) : 1.0;
  const double stale =
      std::isfinite(options.stale_timeout_s) ? std::max(fresh, options.stale_timeout_s) : 5.0;
  const bool penalty_enabled =
      std::isfinite(options.unknown_penalty) && options.unknown_penalty > 0.0;
  const bool invalid_pi =
      (result->voxel.flags & VALID_PI) == 0u || !finite(result->voxel.pi_cost);
  const bool preexisting_unknown =
      (result->voxel.flags & UNKNOWN_RISK) != 0u;
  const bool age_unknown = !std::isfinite(age);
  const bool stale_risk = std::isfinite(age) && age > fresh;
  const bool timed_out = age_unknown || (std::isfinite(age) && age > stale);

  if (stale_risk || timed_out) {
    result->voxel.flags |= STALE_PL;
  }
  if (preexisting_unknown || invalid_pi || timed_out) {
    result->voxel.flags |= UNKNOWN_RISK;
  }

  if (penalty_enabled &&
      (stale_risk || timed_out || (result->voxel.flags & UNKNOWN_RISK) != 0u)) {
    const bool full_unknown_penalty =
        (result->voxel.flags & UNKNOWN_RISK) != 0u;
    const double penalty = full_unknown_penalty
                               ? options.unknown_penalty
                               : unified_risk_unknown_penalty(
                                     age, options.unknown_penalty,
                                     options.unknown_tau_s);
    const double base =
        finite(result->voxel.pi_cost) ? static_cast<double>(result->voxel.pi_cost) : 0.0;
    result->voxel.pi_cost = static_cast<float>(base + penalty);
    result->unknown_penalty = static_cast<float>(penalty);
  }
  result->flags = result->voxel.flags;
  result->valid = (result->flags & VALID_PI) != 0u && finite(result->voxel.pi_cost);
}

}  // namespace iap
