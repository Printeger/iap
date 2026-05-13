#include <iap/planner/pl_grid.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <stdexcept>

namespace iap {

namespace {

int axis_count(const double size, const double res) {
  return static_cast<int>(std::ceil(size / res)) + 1;
}

double lerp(const double a, const double b, const double t) {
  return a * (1.0 - t) + b * t;
}

std::string first_lidar_fallback_reason(
    const std::array<const FuturePLQueryResult*, 8>& cells) {
  for (const auto* value : cells) {
    if (!value->lidar_valid && !value->lidar_fallback_reason.empty()) {
      return value->lidar_fallback_reason;
    }
  }
  return "mixed_lidar_grid_fallback";
}

bool finite_result(const FuturePLQueryResult& value) {
  return value.valid && std::isfinite(value.hpl) &&
         std::isfinite(value.vpl) && std::isfinite(value.pl_scalar);
}

}  // namespace

bool PLGrid::reset(const Eigen::Vector3d& center,
                   const double sx,
                   const double sy,
                   const double sz,
                   const double res) {
  if (!center.allFinite() || sx <= 0.0 || sy <= 0.0 || sz <= 0.0 ||
      res <= 0.0 || !std::isfinite(sx) || !std::isfinite(sy) ||
      !std::isfinite(sz) || !std::isfinite(res)) {
    valid_ = false;
    cells_.clear();
    return false;
  }
  center_ = center;
  size_ = Eigen::Vector3d(sx, sy, sz);
  resolution_ = res;
  min_corner_ = center_ - 0.5 * size_;
  nx_ = std::max(2, axis_count(sx, res));
  ny_ = std::max(2, axis_count(sy, res));
  nz_ = std::max(2, axis_count(sz, res));
  cells_.assign(static_cast<std::size_t>(nx_ * ny_ * nz_), PLGridCell{});
  generation_ = -1;
  stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  build_time_ms_ = std::numeric_limits<double>::quiet_NaN();
  valid_ = true;
  return true;
}

bool PLGrid::contains(const Eigen::Vector3d& p) const {
  if (!valid_ || !p.allFinite()) {
    return false;
  }
  const Eigen::Vector3d max_corner =
      min_corner_ + resolution_ * Eigen::Vector3d(nx_ - 1, ny_ - 1, nz_ - 1);
  return p.x() >= min_corner_.x() && p.y() >= min_corner_.y() &&
         p.z() >= min_corner_.z() && p.x() <= max_corner.x() &&
         p.y() <= max_corner.y() && p.z() <= max_corner.z();
}

bool PLGrid::cell_index(const Eigen::Vector3d& p,
                        int* ix,
                        int* iy,
                        int* iz) const {
  if (!contains(p)) {
    return false;
  }
  const Eigen::Vector3d rel = (p - min_corner_) / resolution_;
  *ix = std::clamp(static_cast<int>(std::floor(rel.x())), 0, nx_ - 1);
  *iy = std::clamp(static_cast<int>(std::floor(rel.y())), 0, ny_ - 1);
  *iz = std::clamp(static_cast<int>(std::floor(rel.z())), 0, nz_ - 1);
  return true;
}

PLGridCell& PLGrid::at(const int ix, const int iy, const int iz) {
  if (!in_bounds(ix, iy, iz)) {
    throw std::out_of_range("PLGrid index out of range");
  }
  return cells_.at(flat_index(ix, iy, iz));
}

const PLGridCell& PLGrid::at(const int ix, const int iy, const int iz) const {
  if (!in_bounds(ix, iy, iz)) {
    throw std::out_of_range("PLGrid index out of range");
  }
  return cells_.at(flat_index(ix, iy, iz));
}

Eigen::Vector3d PLGrid::position(const int ix, const int iy, const int iz) const {
  return min_corner_ + resolution_ * Eigen::Vector3d(ix, iy, iz);
}

FuturePLQueryResult PLGrid::interpolate(const Eigen::Vector3d& p) const {
  FuturePLQueryResult out;
  out.valid = false;
  out.fallback = true;
  out.fallback_reason = "grid_miss";
  out.query_source = "direct";
  if (!contains(p)) {
    return out;
  }

  int ix = 0;
  int iy = 0;
  int iz = 0;
  if (!cell_index(p, &ix, &iy, &iz)) {
    return out;
  }
  if (ix >= nx_ - 1 || iy >= ny_ - 1 || iz >= nz_ - 1) {
    return out;
  }

  const Eigen::Vector3d rel = (p - min_corner_) / resolution_;
  const double tx = rel.x() - std::floor(rel.x());
  const double ty = rel.y() - std::floor(rel.y());
  const double tz = rel.z() - std::floor(rel.z());

  std::array<const FuturePLQueryResult*, 8> c = {
      &at(ix, iy, iz).value,
      &at(ix + 1, iy, iz).value,
      &at(ix, iy + 1, iz).value,
      &at(ix + 1, iy + 1, iz).value,
      &at(ix, iy, iz + 1).value,
      &at(ix + 1, iy, iz + 1).value,
      &at(ix, iy + 1, iz + 1).value,
      &at(ix + 1, iy + 1, iz + 1).value,
  };
  if (!std::all_of(c.begin(), c.end(),
                   [](const auto* value) { return finite_result(*value); })) {
    return out;
  }

  const auto interp = [&](auto getter) {
    const double x00 = lerp(getter(*c[0]), getter(*c[1]), tx);
    const double x10 = lerp(getter(*c[2]), getter(*c[3]), tx);
    const double x01 = lerp(getter(*c[4]), getter(*c[5]), tx);
    const double x11 = lerp(getter(*c[6]), getter(*c[7]), tx);
    return lerp(lerp(x00, x10, ty), lerp(x01, x11, ty), tz);
  };

  out = *c[0];
  out.valid = true;
  out.fallback = false;
  out.fallback_reason.clear();
  out.query_source = "grid";
  out.hpl = interp([](const FuturePLQueryResult& v) { return v.hpl; });
  out.vpl = interp([](const FuturePLQueryResult& v) { return v.vpl; });
  out.pl_scalar =
      interp([](const FuturePLQueryResult& v) { return v.pl_scalar; });
  out.pl_e = interp([](const FuturePLQueryResult& v) { return v.pl_e; });
  out.pl_n = interp([](const FuturePLQueryResult& v) { return v.pl_n; });
  out.pl_u = interp([](const FuturePLQueryResult& v) { return v.pl_u; });
  out.pl_ff_h = interp([](const FuturePLQueryResult& v) { return v.pl_ff_h; });
  out.pl_ff_v = interp([](const FuturePLQueryResult& v) { return v.pl_ff_v; });
  out.sigma_h = interp([](const FuturePLQueryResult& v) { return v.sigma_h; });
  out.sigma_v = interp([](const FuturePLQueryResult& v) { return v.sigma_v; });
  out.pdop = interp([](const FuturePLQueryResult& v) { return v.pdop; });
  out.gnss_hpl =
      interp([](const FuturePLQueryResult& v) { return v.gnss_hpl; });
  out.gnss_vpl =
      interp([](const FuturePLQueryResult& v) { return v.gnss_vpl; });
  out.fused_hpl =
      interp([](const FuturePLQueryResult& v) { return v.fused_hpl; });
  out.fused_vpl =
      interp([](const FuturePLQueryResult& v) { return v.fused_vpl; });
  out.lidar_alpha =
      interp([](const FuturePLQueryResult& v) { return v.lidar_alpha; });
  out.lidar_tdop =
      interp([](const FuturePLQueryResult& v) { return v.lidar_tdop; });
  out.lidar_condition =
      interp([](const FuturePLQueryResult& v) { return v.lidar_condition; });
  out.lidar_bias_h =
      interp([](const FuturePLQueryResult& v) { return v.lidar_bias_h; });
  out.lidar_bias_v =
      interp([](const FuturePLQueryResult& v) { return v.lidar_bias_v; });
  out.lambda_prior_trace =
      interp([](const FuturePLQueryResult& v) { return v.lambda_prior_trace; });
  out.lambda_gnss_trace =
      interp([](const FuturePLQueryResult& v) { return v.lambda_gnss_trace; });
  out.lambda_lidar_trace =
      interp([](const FuturePLQueryResult& v) { return v.lambda_lidar_trace; });
  out.lambda_adv_trace =
      interp([](const FuturePLQueryResult& v) { return v.lambda_adv_trace; });
  out.lambda_adv_min_eig =
      interp([](const FuturePLQueryResult& v) { return v.lambda_adv_min_eig; });
  out.lambda_adv_condition =
      interp([](const FuturePLQueryResult& v) { return v.lambda_adv_condition; });
  out.hpl_adv = interp([](const FuturePLQueryResult& v) { return v.hpl_adv; });
  out.vpl_adv = interp([](const FuturePLQueryResult& v) { return v.vpl_adv; });
  out.lidar_valid =
      std::all_of(c.begin(), c.end(),
                  [](const auto* value) { return value->lidar_valid; });
  out.lidar_fim_valid =
      std::all_of(c.begin(), c.end(),
                  [](const auto* value) { return value->lidar_fim_valid; });
  out.gnss_fim_valid =
      std::all_of(c.begin(), c.end(),
                  [](const auto* value) { return value->gnss_fim_valid; });
  out.fim_regularized =
      std::any_of(c.begin(), c.end(),
                  [](const auto* value) { return value->fim_regularized; });
  out.advisory_fusion_mode = c[0]->advisory_fusion_mode;
  out.lidar_fallback_reason =
      out.lidar_valid ? std::string{} : first_lidar_fallback_reason(c);
  out.n_vis = static_cast<int>(std::lround(
      interp([](const FuturePLQueryResult& v) {
        return static_cast<double>(v.n_vis);
      })));
  out.n_hypotheses = static_cast<int>(std::lround(
      interp([](const FuturePLQueryResult& v) {
        return static_cast<double>(v.n_hypotheses);
      })));
  out.lidar_n_primitives = static_cast<int>(std::lround(
      interp([](const FuturePLQueryResult& v) {
        return static_cast<double>(v.lidar_n_primitives);
      })));
  out.grad_hpl =
      (c[0]->grad_hpl + c[1]->grad_hpl + c[2]->grad_hpl + c[3]->grad_hpl +
       c[4]->grad_hpl + c[5]->grad_hpl + c[6]->grad_hpl + c[7]->grad_hpl) /
      8.0;
  out.grad_vpl =
      (c[0]->grad_vpl + c[1]->grad_vpl + c[2]->grad_vpl + c[3]->grad_vpl +
       c[4]->grad_vpl + c[5]->grad_vpl + c[6]->grad_vpl + c[7]->grad_vpl) /
      8.0;
  out.grad_pl_scalar =
      (c[0]->grad_pl_scalar + c[1]->grad_pl_scalar +
       c[2]->grad_pl_scalar + c[3]->grad_pl_scalar +
       c[4]->grad_pl_scalar + c[5]->grad_pl_scalar +
       c[6]->grad_pl_scalar + c[7]->grad_pl_scalar) /
      8.0;
  out.grid_generation = generation_;
  out.grid_build_time_ms = build_time_ms_;
  return out;
}

void PLGrid::compute_gradients() {
  if (!valid_) {
    return;
  }
  const auto gradient = [&](int ix, int iy, int iz, auto getter) {
    const int xm = std::max(0, ix - 1);
    const int xp = std::min(nx_ - 1, ix + 1);
    const int ym = std::max(0, iy - 1);
    const int yp = std::min(ny_ - 1, iy + 1);
    const int zm = std::max(0, iz - 1);
    const int zp = std::min(nz_ - 1, iz + 1);
    const double dx = std::max(1, xp - xm) * resolution_;
    const double dy = std::max(1, yp - ym) * resolution_;
    const double dz = std::max(1, zp - zm) * resolution_;
    return Eigen::Vector3d(
        (getter(at(xp, iy, iz).value) - getter(at(xm, iy, iz).value)) / dx,
        (getter(at(ix, yp, iz).value) - getter(at(ix, ym, iz).value)) / dy,
        (getter(at(ix, iy, zp).value) - getter(at(ix, iy, zm).value)) / dz);
  };

  for (int iz = 0; iz < nz_; ++iz) {
    for (int iy = 0; iy < ny_; ++iy) {
      for (int ix = 0; ix < nx_; ++ix) {
        auto& v = at(ix, iy, iz).value;
        if (!v.valid) {
          continue;
        }
        v.grad_hpl = gradient(
            ix, iy, iz, [](const FuturePLQueryResult& r) { return r.hpl; });
        v.grad_vpl = gradient(
            ix, iy, iz, [](const FuturePLQueryResult& r) { return r.vpl; });
        v.grad_pl_scalar = gradient(
            ix, iy, iz,
            [](const FuturePLQueryResult& r) { return r.pl_scalar; });
      }
    }
  }
}

std::size_t PLGrid::flat_index(const int ix, const int iy, const int iz) const {
  return static_cast<std::size_t>((iz * ny_ + iy) * nx_ + ix);
}

bool PLGrid::in_bounds(const int ix, const int iy, const int iz) const {
  return valid_ && ix >= 0 && iy >= 0 && iz >= 0 && ix < nx_ && iy < ny_ &&
         iz < nz_;
}

}  // namespace iap
