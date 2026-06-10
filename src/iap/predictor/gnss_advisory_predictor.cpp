#include <iap/predictor/gnss_advisory_predictor.hpp>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

namespace iap {
namespace {

void copy_fim_diagnostics(const FimDiagnostic& diag,
                          GnssAdvisoryResult& out) {
  out.fim_valid = diag.valid;
  out.fim_regularized = diag.regularized;
  out.lambda_trace = diag.trace;
  out.lambda_min_eig = diag.min_eig;
  out.lambda_max_eig = diag.max_eig;
  out.lambda_condition = diag.condition;
  out.fim_fallback_reason = diag.fallback_reason;
}

std::vector<GnssGeometrySat> visible_geometry(
    const GnssEpoch& epoch,
    const VisibilityResult& visibility) {
  std::vector<GnssGeometrySat> geom;
  geom.reserve(static_cast<std::size_t>(visibility.n_vis));
  for (std::size_t i = 0; i < epoch.sats.size(); ++i) {
    if (i >= visibility.vis_flags.size() || !visibility.vis_flags[i]) {
      continue;
    }
    if (epoch.sats[i].excluded) {
      continue;
    }
    GnssGeometrySat sat;
    sat.elevation = epoch.sats[i].elevation;
    sat.azimuth = epoch.sats[i].azimuth;
    sat.pr_sigma =
        (i < visibility.sigma_effs.size() && visibility.sigma_effs[i] > 0.0)
            ? visibility.sigma_effs[i]
            : epoch.sats[i].pr_sigma;
    sat.sat_id = epoch.sats[i].sat_id;
    geom.push_back(sat);
  }
  return geom;
}

bool eliminate_clock_by_schur_complement(const Eigen::Matrix4d& lambda_gnss_4d,
                                         const double clock_epsilon,
                                         Eigen::Matrix3d* lambda_position) {
  if (!lambda_position || !lambda_gnss_4d.allFinite()) {
    return false;
  }
  const double lambda_cc = lambda_gnss_4d(3, 3);
  const double eps =
      std::isfinite(clock_epsilon) && clock_epsilon > 0.0 ? clock_epsilon
                                                          : 1.0e-6;
  if (!std::isfinite(lambda_cc) || lambda_cc + eps <= 0.0) {
    return false;
  }

  const Eigen::Matrix3d lambda_pp = lambda_gnss_4d.block<3, 3>(0, 0);
  const Eigen::Matrix<double, 3, 1> lambda_pc =
      lambda_gnss_4d.block<3, 1>(0, 3);
  const Eigen::Matrix<double, 1, 3> lambda_cp =
      lambda_gnss_4d.block<1, 3>(3, 0);
  *lambda_position = lambda_pp - (lambda_pc * lambda_cp) / (lambda_cc + eps);
  *lambda_position =
      0.5 * (*lambda_position + lambda_position->transpose());
  return lambda_position->allFinite();
}

}  // namespace

GnssAdvisoryPredictor::GnssAdvisoryPredictor()
    : GnssAdvisoryPredictor(GnssAdvisoryPredictorParams{}) {}

GnssAdvisoryPredictor::GnssAdvisoryPredictor(
    const GnssAdvisoryPredictorParams& params)
    : params_(params),
      geometry_predictor_(params.geometry_params),
      visibility_predictor_(params.visibility_params) {}

void GnssAdvisoryPredictor::set_params(
    const GnssAdvisoryPredictorParams& params) {
  params_ = params;
  geometry_predictor_ = GnssGeometryPlPredictor(params_.geometry_params);
  visibility_predictor_ = VisibilityPredictor(params_.visibility_params);
}

void GnssAdvisoryPredictor::set_local_occupancy(
    const LocalOccupancyGrid* occupancy) {
  visibility_predictor_.set_occupancy(occupancy);
}

GnssAdvisoryResult GnssAdvisoryPredictor::fallback(
    const std::string& reason) const {
  GnssAdvisoryResult out;
  out.available = false;
  out.valid = false;
  out.fallback = true;
  out.fallback_reason = reason.empty() ? "gnss_unavailable" : reason;
  out.fim_valid = false;
  out.fim_fallback_reason = out.fallback_reason;
  return out;
}

GnssAdvisoryResult GnssAdvisoryPredictor::compute_advisory_fim(
    const Eigen::Vector3d& query_position,
    const GnssEpoch& epoch,
    const VisibilityResult& visibility,
    const GnssAdvisoryResult& base) const {
  (void)query_position;
  GnssAdvisoryResult out = base;
  const auto geom = visible_geometry(epoch, visibility);
  out.n_used = static_cast<int>(geom.size());
  if (out.n_used < params_.geometry_params.min_sats) {
    out.fim_valid = false;
    out.fim_fallback_reason = "too_few_sats";
    return out;
  }

  Eigen::Matrix4d h_full = Eigen::Matrix4d::Zero();
  for (const auto& sat : geom) {
    Eigen::Vector4d g;
    g << std::cos(sat.elevation) * std::sin(sat.azimuth),
         std::cos(sat.elevation) * std::cos(sat.azimuth),
         std::sin(sat.elevation),
         1.0;
    const double sigma = std::max(sat.pr_sigma, 0.01);
    h_full += (1.0 / (sigma * sigma)) * (g * g.transpose());
  }

  const double clock_eps =
      std::isfinite(params_.fim_clock_epsilon) &&
              params_.fim_clock_epsilon > 0.0
          ? params_.fim_clock_epsilon
          : 1.0e-6;
  if (!eliminate_clock_by_schur_complement(h_full, clock_eps,
                                           &out.lambda_gnss)) {
    out.fim_valid = false;
    out.fim_fallback_reason = "degenerate_clock_information";
    return out;
  }

  FimDiagnostic diag;
  diag.lambda = out.lambda_gnss;
  fill_fim_diagnostics(diag);
  const double psd_eps =
      std::isfinite(params_.fim_psd_epsilon) && params_.fim_psd_epsilon > 0.0
          ? params_.fim_psd_epsilon
          : 1.0e-9;
  if (!std::isfinite(diag.min_eig) || diag.min_eig < -psd_eps ||
      diag.max_eig <= 0.0) {
    diag.valid = false;
    diag.fallback_reason = "gnss_fim_not_psd";
    copy_fim_diagnostics(diag, out);
    return out;
  }
  if (diag.min_eig < 0.0) {
    out.lambda_gnss += Eigen::Matrix3d::Identity() * (-diag.min_eig + psd_eps);
    diag.lambda = out.lambda_gnss;
    diag.regularized = true;
    fill_fim_diagnostics(diag);
  }
  diag.valid = true;
  diag.fallback_reason.clear();
  copy_fim_diagnostics(diag, out);
  return out;
}

GnssAdvisoryResult GnssAdvisoryPredictor::query(
    const Eigen::Vector3d& query_position,
    const IntegritySnapshot& snapshot) const {
  if (!query_position.allFinite()) {
    return fallback("invalid_position");
  }
  if (!snapshot.has_epoch) {
    return fallback("no_gnss_epoch");
  }

  const VisibilityResult visibility =
      visibility_predictor_.predict(query_position, snapshot.gnss_epoch);
  auto geom = visible_geometry(snapshot.gnss_epoch, visibility);
  if (static_cast<int>(geom.size()) < params_.geometry_params.min_sats) {
    auto out = fallback("too_few_sats");
    out.n_visible = visibility.n_vis;
    out.n_used = static_cast<int>(geom.size());
    return out;
  }

  const GnssGeometryPlResult pl = geometry_predictor_.predict(geom);
  if (!pl.valid) {
    auto out = fallback("singular_geometry");
    out.n_visible = visibility.n_vis;
    out.n_used = static_cast<int>(geom.size());
    out.n_hypotheses = pl.n_hypotheses;
    return out;
  }

  GnssAdvisoryResult out;
  out.available = true;
  out.valid = true;
  out.fallback = false;
  out.fallback_reason.clear();
  out.hpl = pl.HPL;
  out.vpl = pl.VPL;
  out.pl_scalar = std::max(out.hpl, out.vpl);
  out.pl_e = pl.PL_E;
  out.pl_n = pl.PL_N;
  out.pl_u = pl.PL_U;
  out.pl_ff_h = pl.pl_ff;
  out.pl_ff_v = pl.pl_ff_V;
  out.sigma_h = std::sqrt(std::max(
      0.0, pl.sigma_ff_E * pl.sigma_ff_E + pl.sigma_ff_N * pl.sigma_ff_N));
  out.sigma_v = pl.sigma_ff_U;
  if (pl.S0(0, 0) > 0.0 && pl.S0(1, 1) > 0.0 && pl.S0(2, 2) > 0.0) {
    out.pdop = std::sqrt(pl.S0(0, 0) + pl.S0(1, 1) + pl.S0(2, 2));
  }
  out.n_visible = visibility.n_vis;
  out.n_used = static_cast<int>(geom.size());
  out.n_hypotheses = pl.n_hypotheses;
  return compute_advisory_fim(query_position, snapshot.gnss_epoch, visibility,
                              out);
}

}  // namespace iap
