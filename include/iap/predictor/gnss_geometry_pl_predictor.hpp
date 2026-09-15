#pragma once
// Predictor-side GNSS geometry-only advisory PL component.
// This class does NOT include current ARAIM solver headers.

#include <Eigen/Core>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace iap {

enum class GnssGeometryStatus {
  NOT_EVALUATED = 0,
  VALID,
  TOO_FEW_SATELLITES,
  FULL_GEOMETRY_DEGENERATE,
  SUBSET_DEGENERATE,
  NUMERICAL_FAILURE,
};

const char* gnssGeometryStatusName(GnssGeometryStatus status);

struct GnssGeometrySat {
  double elevation = 0.0;
  double azimuth   = 0.0;
  double pr_sigma  = 5.0;
  int    sat_id    = -1;
};

struct GnssGeometryPlPredictorParams {
  double P_HMI_req      = 1e-7;
  double P_FA_req       = 1e-5;
  bool   dynamic_budget = true;
  double K_fa = 4.50;
  double K_md = 5.50;
  double K_ff = 5.42;
  double p_sat_default = 1e-5;
  double eps_degen     = 1e-10;
  int    min_sats      = 4;
  bool parallel_hypotheses = true;
  int  hypothesis_threads  = 0;
  // Exact (bit-identical input) geometry results may be shared within one
  // predictor snapshot. Zero disables caching.
  std::size_t exact_cache_capacity = 4096;
};

struct GnssGeometryCacheStats {
  std::uint64_t hits = 0;
  std::uint64_t misses = 0;
  std::uint64_t fallback_factorizations = 0;
  std::size_t entries = 0;
};

struct GnssGeometryPlResult {
  bool   valid       = false;
  GnssGeometryStatus status = GnssGeometryStatus::NOT_EVALUATED;
  double HPL         = std::numeric_limits<double>::quiet_NaN();
  double VPL         = std::numeric_limits<double>::quiet_NaN();
  double PL_E        = std::numeric_limits<double>::quiet_NaN();
  double PL_N        = std::numeric_limits<double>::quiet_NaN();
  double PL_U        = std::numeric_limits<double>::quiet_NaN();
  double pl_ff       = std::numeric_limits<double>::quiet_NaN();
  double pl_ff_V     = std::numeric_limits<double>::quiet_NaN();
  double sigma_ff_E  = 0.0;
  double sigma_ff_N  = 0.0;
  double sigma_ff_U  = 0.0;
  double K_ff_used   = 0.0;
  double K_fa_used   = 0.0;
  int    n_hypotheses = 0;
  int    worst_hyp   = -1;
  int    worst_hyp_h = -1;
  int    worst_hyp_v = -1;
  std::vector<int> degenerate_satellite_ids;
  double weighted_normal_condition =
      std::numeric_limits<double>::quiet_NaN();
  Eigen::Matrix4d S0 = Eigen::Matrix4d::Identity();
};

class GnssGeometryPlPredictor {
 public:
  explicit GnssGeometryPlPredictor(
      const GnssGeometryPlPredictorParams& params = {});
  GnssGeometryPlResult predict(
      const std::vector<GnssGeometrySat>& visible_sats) const;
  const GnssGeometryPlPredictorParams& params() const { return params_; }
  GnssGeometryCacheStats cacheStats() const;
  void clearCache() const;
 private:
  struct CacheState;
  GnssGeometryPlResult predictUncached(
      const std::vector<GnssGeometrySat>& visible_sats) const;
  GnssGeometryPlPredictorParams params_;
  std::shared_ptr<CacheState> cache_state_;
};

}  // namespace iap
