#include <iap/planner/future_pl_field_predictor.hpp>
#include <iap/util/timing_csv.hpp>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace iap {

namespace {

PredictedAraimComputer::Params with_fallback(
    PredictedAraimComputer::Params params,
    const double fallback_pl) {
  params.fallback_pl = fallback_pl;
  return params;
}

LidarObservabilityFim::Params lidar_params_from(
    const FuturePLFieldPredictor::Params& params) {
  LidarObservabilityFim::Params out;
  out.search_radius_m = params.lidar_search_radius_m;
  out.min_points = params.lidar_min_points;
  out.good_points = params.lidar_good_points;
  out.sigma_lidar_m = params.lidar_sigma_m;
  out.alpha_min = params.lidar_alpha_min;
  out.alpha_max = params.lidar_alpha_max;
  out.condition_ref = params.lidar_condition_ref;
  out.condition_max = params.lidar_condition_max;
  out.tdop_ref = params.lidar_tdop_ref;
  out.tdop_max = params.lidar_tdop_max;
  out.bias_h_m = params.lidar_bias_h_m;
  out.bias_v_m = params.lidar_bias_v_m;
  out.fim_radius_m = params.lidar_fim_radius_m;
  out.fim_min_voxels = params.lidar_fim_min_voxels;
  out.fim_range_sigma_base = params.lidar_fim_range_sigma_base;
  out.fim_condition_max = params.lidar_fim_condition_max;
  out.fim_weight_scale = params.lidar_fim_weight_scale;
  return out;
}

double k_ff_or_default(const PredictedAraimComputer::Params& params) {
  return params.araim_params.K_ff > 0.0 ? params.araim_params.K_ff : 5.42;
}

void keep_gnss_only(FuturePLQueryResult& out) {
  // Advisory compatibility mode: fused advisory output falls back to the GNSS
  // advisory proxy. This is not certified monitor fusion.
  out.fused_hpl = out.gnss_hpl;
  out.fused_vpl = out.gnss_vpl;
  out.hpl = out.gnss_hpl;
  out.vpl = out.gnss_vpl;
  out.pl_scalar = std::max(out.hpl, out.vpl);
}

void mark_lidar_fallback(FuturePLQueryResult& out,
                         const std::string& reason) {
  out.lidar_valid = false;
  out.lidar_alpha = 0.0;
  out.lidar_fallback_reason = reason.empty() ? "unknown" : reason;
}

Eigen::Matrix3d gnss_base_information(const FuturePLQueryResult& gnss,
                                      const IntegritySnapshot& snapshot) {
  if (snapshot.has_lambda_base && snapshot.lambda_base_pos.allFinite()) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(
        snapshot.lambda_base_pos, Eigen::EigenvaluesOnly);
    if (eig.info() == Eigen::Success && eig.eigenvalues().minCoeff() > 1.0e-12) {
      return snapshot.lambda_base_pos;
    }
  }

  double sigma_h = gnss.sigma_h;
  double sigma_v = gnss.sigma_v;
  if (!std::isfinite(sigma_h) || sigma_h <= 0.0) {
    sigma_h = std::max(gnss.hpl / 5.42, 0.1);
  }
  if (!std::isfinite(sigma_v) || sigma_v <= 0.0) {
    sigma_v = std::max(gnss.vpl / 5.42, 0.1);
  }
  const double sigma_e = std::max(sigma_h / std::sqrt(2.0), 0.05);
  const double sigma_n = sigma_e;
  const double sigma_u = std::max(sigma_v, 0.05);
  Eigen::Matrix3d lambda = Eigen::Matrix3d::Zero();
  lambda(0, 0) = 1.0 / (sigma_e * sigma_e);
  lambda(1, 1) = 1.0 / (sigma_n * sigma_n);
  lambda(2, 2) = 1.0 / (sigma_u * sigma_u);
  return lambda;
}

FimDiagnostic prior_information(const IntegritySnapshot& snapshot) {
  FimDiagnostic out;
  if (!snapshot.has_lambda_base || !snapshot.lambda_base_pos.allFinite()) {
    out.fallback_reason = "missing_prior";
    fill_fim_diagnostics(out);
    return out;
  }

  out.lambda =
      0.5 * (snapshot.lambda_base_pos + snapshot.lambda_base_pos.transpose());
  fill_fim_diagnostics(out);
  if (out.min_eig <= 1.0e-12 || !std::isfinite(out.condition)) {
    out.lambda.setZero();
    out.valid = false;
    out.fallback_reason = "invalid_prior";
    fill_fim_diagnostics(out);
    return out;
  }
  out.valid = true;
  out.fallback_reason.clear();
  return out;
}

void copy_fim_debug(const FusedAdvisoryFimResult& fim,
                    FuturePLQueryResult& out) {
  out.lambda_prior_trace = fim.prior.trace;
  out.lambda_gnss_trace = fim.gnss.trace;
  out.lambda_lidar_trace = fim.lidar.trace;
  out.lambda_adv_trace = fim.trace;
  out.lambda_adv_min_eig = fim.min_eig;
  out.lambda_adv_condition = fim.condition;
  out.hpl_adv = fim.hpl_adv;
  out.vpl_adv = fim.vpl_adv;
  out.lidar_fim_valid = fim.lidar.valid;
  out.gnss_fim_valid = fim.gnss.valid;
  out.fim_regularized = fim.regularized || fim.prior.regularized ||
                        fim.gnss.regularized || fim.lidar.regularized;
  out.advisory_fusion_mode = fim.fusion_mode;
}

}  // namespace

FuturePLFieldPredictor::FuturePLFieldPredictor()
    : FuturePLFieldPredictor(Params{}) {}

FuturePLFieldPredictor::FuturePLFieldPredictor(const Params& params)
    : params_(params) {
  refresh_stats_params();
}

void FuturePLFieldPredictor::set_occupancy(const LocalOccupancyGrid* grid) {
  occupancy_ = grid;
}

void FuturePLFieldPredictor::set_lidar_map_points(
    std::shared_ptr<const std::vector<Eigen::Vector3d>> points) {
  std::lock_guard<std::mutex> lock(lidar_map_mutex_);
  lidar_map_points_ = std::move(points);
}

void FuturePLFieldPredictor::set_lidar_fim_primitives(
    std::shared_ptr<const std::vector<LidarFimPrimitive>> primitives) {
  std::lock_guard<std::mutex> lock(lidar_map_mutex_);
  lidar_fim_primitives_ = std::move(primitives);
}

void FuturePLFieldPredictor::update_snapshot(const IntegritySnapshot& snapshot) {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_ = snapshot;
}

void FuturePLFieldPredictor::set_params(const Params& params) {
  params_ = params;
  refresh_stats_params();
}

FuturePLQueryResult FuturePLFieldPredictor::evaluate_point_direct(
    const Eigen::Vector3d& p_w) const {
  IntegritySnapshot snapshot;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot = snapshot_;
  }
  std::shared_ptr<const std::vector<Eigen::Vector3d>> points;
  std::shared_ptr<const std::vector<LidarFimPrimitive>> primitives;
  {
    std::lock_guard<std::mutex> lock(lidar_map_mutex_);
    points = lidar_map_points_;
    primitives = lidar_fim_primitives_;
  }
  auto out = evaluate_point(p_w, snapshot, points, primitives, "direct");
  out.grid_generation = -1;
  return out;
}

FuturePLQueryResult FuturePLFieldPredictor::query(const Eigen::Vector3d& p_w,
                                                  const double now_s) const {
  if (params_.use_grid) {
    auto grid_result = query_grid(p_w, now_s);
    if (grid_result.valid) {
      record_query(grid_result);
      return grid_result;
    }
  }
  auto direct = evaluate_point_direct(p_w);
  direct.query_source = direct.fallback ? "fallback" : "direct";
  record_query(direct);
  return direct;
}

bool FuturePLFieldPredictor::rebuild_grid(const double now_s) {
  if (!params_.use_grid) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++stats_.skip_count;
    return false;
  }

  IntegritySnapshot snapshot;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot = snapshot_;
  }
  if (!snapshot.valid || !snapshot.has_pose || !snapshot.p_wb.allFinite()) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++stats_.skip_count;
    return false;
  }

  iap::timing_csv::ScopedTimer timing_scope(now_s, "pl_grid_build");
  const auto t0 = std::chrono::steady_clock::now();
  auto grid = std::make_shared<PLGrid>();
  if (!grid->reset(snapshot.p_wb,
                   params_.grid_size_x_m,
                   params_.grid_size_y_m,
                   params_.grid_size_z_m,
                   params_.grid_resolution_m)) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++stats_.skip_count;
    return false;
  }

  std::shared_ptr<const std::vector<Eigen::Vector3d>> points;
  std::shared_ptr<const std::vector<LidarFimPrimitive>> primitives;
  {
    std::lock_guard<std::mutex> lock(lidar_map_mutex_);
    points = lidar_map_points_;
    primitives = lidar_fim_primitives_;
  }

  for (int iz = 0; iz < grid->nz(); ++iz) {
    for (int iy = 0; iy < grid->ny(); ++iy) {
      for (int ix = 0; ix < grid->nx(); ++ix) {
        const Eigen::Vector3d p = grid->position(ix, iy, iz);
        auto value = evaluate_point(p, snapshot, points, primitives, "grid");
        value.grid_generation = next_generation_;
        grid->at(ix, iy, iz).value = std::move(value);
      }
    }
  }
  grid->compute_gradients();

  const auto t1 = std::chrono::steady_clock::now();
  const double build_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
  grid->set_generation(next_generation_++);
  grid->set_stamp_s(now_s);
  grid->set_build_time_ms(build_ms);

  auto self_grid = grid->interpolate(snapshot.p_wb);
  auto self_direct =
      evaluate_point(snapshot.p_wb, snapshot, points, primitives, "direct");
  double self_ratio = std::numeric_limits<double>::quiet_NaN();
  if (std::isfinite(self_grid.pl_scalar) &&
      std::isfinite(self_direct.pl_scalar)) {
    self_ratio =
        std::abs(self_grid.pl_scalar - self_direct.pl_scalar) /
        std::max(std::abs(self_direct.pl_scalar), 1.0e-9);
  }

  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    active_grid_ = grid;
  }
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.active = true;
    stats_.generation = grid->generation();
    ++stats_.update_count;
    stats_.last_build_time_ms = build_ms;
    build_time_sum_ms_ += build_ms;
    stats_.mean_build_time_ms =
        build_time_sum_ms_ / std::max(1, stats_.update_count);
    stats_.max_build_time_ms =
        std::isfinite(stats_.max_build_time_ms)
            ? std::max(stats_.max_build_time_ms, build_ms)
            : build_ms;
    stats_.last_self_check_pl_ratio = self_ratio;
  }
  return true;
}

FuturePLFieldPredictor::GridStats FuturePLFieldPredictor::stats() const {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return stats_;
}

std::shared_ptr<const PLGrid> FuturePLFieldPredictor::active_grid() const {
  std::lock_guard<std::mutex> lock(grid_mutex_);
  return active_grid_;
}

FuturePLQueryResult FuturePLFieldPredictor::evaluate_point(
    const Eigen::Vector3d& p_w,
    const IntegritySnapshot& snapshot,
    const std::shared_ptr<const std::vector<Eigen::Vector3d>>& points,
    const std::shared_ptr<const std::vector<LidarFimPrimitive>>& primitives,
    const std::string& query_source) const {
  PredictedAraimComputer predictor(
      with_fallback(params_.araim_params, params_.araim_params.fallback_pl));
  predictor.set_occupancy(occupancy_);
  predictor.set_epoch(snapshot.has_epoch ? &snapshot.gnss_epoch : nullptr);
  auto out = make_future_pl_query_result(
      predictor.predict_araim_result(p_w), query_source);

  // Legacy field names retained for compatibility. Semantically these are
  // GNSS advisory PL proxy values and advisory predicted fused values.
  out.gnss_hpl = out.hpl;
  out.gnss_vpl = out.vpl;
  out.fused_hpl = out.hpl;
  out.fused_vpl = out.vpl;
  out.lidar_bias_h = params_.lidar_bias_h_m;
  out.lidar_bias_v = params_.lidar_bias_v_m;
  out.lidar_tdop = params_.lidar_tdop_max;
  out.lidar_condition = params_.lidar_condition_max;
  out.lidar_fallback_reason =
      params_.use_lidar_observability ? "not_evaluated" : "lidar_disabled";

  if (params_.use_advisory_fim_add) {
    FusedAdvisoryFimResult fim;
    fim.fusion_mode = "fim_add";
    fim.prior = prior_information(snapshot);
    fim.gnss = predictor.predict_advisory_fim(p_w);
    fim.lidar.fallback_reason = params_.use_lidar_advisory_fim
                                     ? "not_evaluated"
                                     : "lidar_fim_disabled";

    if (params_.use_lidar_advisory_fim) {
      LidarObservabilityFim estimator(lidar_params_from(params_));
      fim.lidar =
          estimator.evaluate_advisory_fim(p_w, primitives.get(), snapshot.current);
      out.lidar_valid = fim.lidar.valid;
      out.lidar_fim_valid = fim.lidar.valid;
      out.lidar_alpha = fim.lidar.valid ? 1.0 : 0.0;
      out.lidar_n_primitives = fim.lidar.n_primitives;
      out.lidar_condition = fim.lidar.condition;
      out.lidar_fallback_reason = fim.lidar.fallback_reason;
    } else {
      out.lidar_fallback_reason = "lidar_fim_disabled";
    }

    if (fim.prior.valid) {
      fim.lambda += fim.prior.lambda;
    }
    if (fim.gnss.valid) {
      fim.lambda += fim.gnss.lambda;
    }
    if (fim.lidar.valid) {
      fim.lambda += fim.lidar.lambda;
    }
    fim.lambda = 0.5 * (fim.lambda + fim.lambda.transpose());
    fill_fim_diagnostics(fim);

    const double eps =
        std::isfinite(params_.fim_epsilon) && params_.fim_epsilon > 0.0
            ? params_.fim_epsilon
            : 1.0e-6;
    const Eigen::Matrix3d regularized_lambda =
        fim.lambda + eps * Eigen::Matrix3d::Identity();
    Eigen::LDLT<Eigen::Matrix3d> ldlt(regularized_lambda);
    if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
      fim.valid = false;
      fim.fallback_reason = "singular_advisory_fim";
      fim.fusion_mode = "fim_add_fallback";
      copy_fim_debug(fim, out);
      keep_gnss_only(out);
      return out;
    }
    fim.sigma_pos = ldlt.solve(Eigen::Matrix3d::Identity());
    if (!fim.sigma_pos.allFinite()) {
      fim.valid = false;
      fim.fallback_reason = "invalid_advisory_covariance";
      fim.fusion_mode = "fim_add_fallback";
      copy_fim_debug(fim, out);
      keep_gnss_only(out);
      return out;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig_h(
        fim.sigma_pos.block<2, 2>(0, 0), Eigen::EigenvaluesOnly);
    if (eig_h.info() != Eigen::Success || fim.sigma_pos(2, 2) < 0.0) {
      fim.valid = false;
      fim.fallback_reason = "invalid_advisory_covariance";
      fim.fusion_mode = "fim_add_fallback";
      copy_fim_debug(fim, out);
      keep_gnss_only(out);
      return out;
    }

    fim.regularized = true;
    fim.valid = true;
    fim.fallback_reason.clear();
    const double k_h =
        std::isfinite(params_.K_H_adv) && params_.K_H_adv > 0.0
            ? params_.K_H_adv
            : 5.0;
    const double k_v =
        std::isfinite(params_.K_V_adv) && params_.K_V_adv > 0.0
            ? params_.K_V_adv
            : 5.0;
    fim.hpl_adv =
        k_h * std::sqrt(std::max(0.0, eig_h.eigenvalues().maxCoeff())) +
        params_.b_H_pred + params_.s_H_pred;
    fim.vpl_adv = k_v * std::sqrt(std::max(0.0, fim.sigma_pos(2, 2))) +
                  params_.b_V_pred + params_.s_V_pred;

    out.valid = std::isfinite(fim.hpl_adv) && std::isfinite(fim.vpl_adv);
    out.fallback = !out.valid;
    out.fallback_reason = out.valid ? std::string{} : "invalid_advisory_pl";
    out.hpl = fim.hpl_adv;
    out.vpl = fim.vpl_adv;
    out.pl_scalar = std::max(out.hpl, out.vpl);
    out.fused_hpl = out.hpl;
    out.fused_vpl = out.vpl;
    out.sigma_h = std::sqrt(std::max(0.0, eig_h.eigenvalues().maxCoeff()));
    out.sigma_v = std::sqrt(std::max(0.0, fim.sigma_pos(2, 2)));
    copy_fim_debug(fim, out);
    return out;
  }

  if (!params_.use_fused_fim_grid || !out.valid) {
    return out;
  }

  // LiDAR observability is an advisory LOI-style proxy for planning only. It
  // must not be reported as certified LiDAR ARAIM.
  LidarObservabilityResult lidar;
  if (params_.use_lidar_observability) {
    LidarObservabilityFim estimator(lidar_params_from(params_));
    lidar = estimator.evaluate(p_w, points.get(), snapshot.current);
    out.lidar_valid = lidar.valid;
    out.lidar_alpha = lidar.lidar_alpha;
    out.lidar_tdop = lidar.tdop_proxy;
    out.lidar_condition = lidar.condition;
    out.lidar_n_primitives = lidar.n_primitives;
    out.lidar_bias_h = lidar.bias_h;
    out.lidar_bias_v = lidar.bias_v;
    out.lidar_fallback_reason = lidar.fallback_reason;
  }

  if (!lidar.valid || lidar.lidar_alpha <= 0.0) {
    keep_gnss_only(out);
    return out;
  }

  // Advisory predicted fused PL proxy. Stage 2 may replace this with the
  // formal FIM-add predictor, but Stage 1 does not alter the math here.
  const Eigen::Matrix3d lambda =
      gnss_base_information(out, snapshot) +
      params_.lidar_info_scale * lidar.lidar_alpha * lidar.delta_lambda;
  Eigen::LDLT<Eigen::Matrix3d> ldlt(lambda);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    mark_lidar_fallback(out, "singular_fused_information");
    keep_gnss_only(out);
    return out;
  }
  const Eigen::Matrix3d sigma = ldlt.solve(Eigen::Matrix3d::Identity());
  if (!sigma.allFinite()) {
    mark_lidar_fallback(out, "singular_fused_information");
    keep_gnss_only(out);
    return out;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig_h(
      sigma.block<2, 2>(0, 0), Eigen::EigenvaluesOnly);
  if (eig_h.info() != Eigen::Success || sigma(2, 2) <= 0.0) {
    mark_lidar_fallback(out, "singular_fused_information");
    keep_gnss_only(out);
    return out;
  }
  const double k_ff = k_ff_or_default(params_.araim_params);
  const double sigma_h =
      std::sqrt(std::max(0.0, eig_h.eigenvalues().maxCoeff()));
  const double sigma_v = std::sqrt(std::max(0.0, sigma(2, 2)));
  out.fused_hpl = k_ff * sigma_h + lidar.bias_h;
  out.fused_vpl = k_ff * sigma_v + lidar.bias_v;
  out.hpl = std::max(out.gnss_hpl, out.fused_hpl);
  out.vpl = std::max(out.gnss_vpl, out.fused_vpl);
  out.pl_scalar = std::max(out.hpl, out.vpl);
  return out;
}

FuturePLQueryResult FuturePLFieldPredictor::query_grid(
    const Eigen::Vector3d& p_w,
    const double now_s) const {
  std::shared_ptr<PLGrid> grid;
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    grid = active_grid_;
  }
  FuturePLQueryResult miss;
  miss.valid = false;
  miss.fallback = true;
  miss.fallback_reason = "grid_miss";
  miss.query_source = "direct";
  if (!grid || !grid->valid() || !grid->contains(p_w)) {
    return miss;
  }
  const double age = std::isfinite(now_s) && std::isfinite(grid->stamp_s())
                         ? now_s - grid->stamp_s()
                         : std::numeric_limits<double>::quiet_NaN();
  if (std::isfinite(age) && age > params_.grid_max_age_s) {
    return miss;
  }
  auto out = grid->interpolate(p_w);
  if (out.valid) {
    out.grid_generation = grid->generation();
    out.grid_age_s = age;
    out.grid_build_time_ms = grid->build_time_ms();
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.last_grid_age_s = age;
  }
  return out;
}

void FuturePLFieldPredictor::record_query(
    const FuturePLQueryResult& result) const {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  if (result.query_source == "grid") {
    ++stats_.query_grid_count;
  } else if (result.query_source == "fallback") {
    ++stats_.query_fallback_count;
  } else {
    ++stats_.query_direct_count;
  }
  if (params_.use_advisory_fim_add) {
    ++stats_.fim_query_count;
    if (result.fim_regularized) {
      ++stats_.fim_regularized_count;
    }
    if (result.gnss_fim_valid) {
      ++stats_.gnss_fim_valid_count;
    }
    if (result.lidar_fim_valid) {
      ++stats_.lidar_fim_valid_count;
    }
  }
  if (params_.use_fused_fim_grid || params_.use_lidar_observability) {
    ++stats_.lidar_query_count;
    if (result.lidar_valid) {
      ++stats_.lidar_valid_count;
      lidar_alpha_sum_ += result.lidar_alpha;
      if (std::isfinite(result.lidar_tdop)) {
        lidar_tdop_sum_ += result.lidar_tdop;
      }
      if (std::isfinite(result.lidar_condition)) {
        lidar_condition_sum_ += result.lidar_condition;
      }
      stats_.mean_lidar_alpha =
          lidar_alpha_sum_ / std::max(1, stats_.lidar_valid_count);
      stats_.max_lidar_alpha =
          std::isfinite(stats_.max_lidar_alpha)
              ? std::max(stats_.max_lidar_alpha, result.lidar_alpha)
              : result.lidar_alpha;
      stats_.mean_lidar_tdop =
          lidar_tdop_sum_ / std::max(1, stats_.lidar_valid_count);
      stats_.mean_lidar_condition =
          lidar_condition_sum_ / std::max(1, stats_.lidar_valid_count);
    } else {
      ++stats_.lidar_fallback_count;
      ++stats_.lidar_fallback_reason_histogram
            [result.lidar_fallback_reason.empty()
                 ? std::string("unknown")
                 : result.lidar_fallback_reason];
    }
    if (!std::isfinite(result.lidar_alpha) ||
        !std::isfinite(result.lidar_tdop) ||
        !std::isfinite(result.lidar_condition)) {
      ++stats_.lidar_nonfinite_debug_count;
    }
    if (result.valid && std::isfinite(result.gnss_hpl) &&
        std::isfinite(result.gnss_vpl)) {
      ++stats_.lidar_conservative_check_count;
      if (result.hpl + 1.0e-9 < result.gnss_hpl ||
          result.vpl + 1.0e-9 < result.gnss_vpl) {
        ++stats_.lidar_conservative_violation_count;
      }
    }
  }
}

void FuturePLFieldPredictor::refresh_stats_params() {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  stats_.enabled = params_.use_grid;
  stats_.resolution_m = params_.grid_resolution_m;
  stats_.size_x_m = params_.grid_size_x_m;
  stats_.size_y_m = params_.grid_size_y_m;
  stats_.size_z_m = params_.grid_size_z_m;
  stats_.lidar_enabled =
      params_.use_fused_fim_grid || params_.use_lidar_observability ||
      params_.use_lidar_advisory_fim;
}

}  // namespace iap
