#pragma once
// Predictor-side GNSS geometry-only advisory PL component.
// This class does NOT include current ARAIM solver headers.

#include <Eigen/Core>
#include <vector>

namespace iap {

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
};

struct GnssGeometryPlResult {
  bool   valid       = false;
  double HPL         = 1e9;
  double VPL         = 1e9;
  double PL_E        = 1e9;
  double PL_N        = 1e9;
  double PL_U        = 1e9;
  double pl_ff       = 1e9;
  double pl_ff_V     = 1e9;
  double sigma_ff_E  = 0.0;
  double sigma_ff_N  = 0.0;
  double sigma_ff_U  = 0.0;
  double K_ff_used   = 0.0;
  double K_fa_used   = 0.0;
  int    n_hypotheses = 0;
  int    worst_hyp   = -1;
  Eigen::Matrix4d S0 = Eigen::Matrix4d::Identity();
};

class GnssGeometryPlPredictor {
 public:
  explicit GnssGeometryPlPredictor(
      const GnssGeometryPlPredictorParams& params = {});
  GnssGeometryPlResult predict(
      const std::vector<GnssGeometrySat>& visible_sats) const;
  const GnssGeometryPlPredictorParams& params() const { return params_; }
 private:
  GnssGeometryPlPredictorParams params_;
};

}  // namespace iap
