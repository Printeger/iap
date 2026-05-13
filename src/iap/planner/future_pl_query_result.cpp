#include <iap/planner/future_pl_query_result.hpp>

namespace iap {

FuturePLQueryResult make_future_pl_query_result(
    const PredictedAraimResult& pred,
    const std::string& query_source) {
  FuturePLQueryResult out;
  out.valid = pred.valid;
  out.fallback = pred.fallback;
  out.fallback_reason = pred.fallback_reason;
  out.query_source = query_source;
  out.hpl = pred.hpl;
  out.vpl = pred.vpl;
  out.pl_scalar = pred.pl_scalar;
  out.pl_e = pred.pl_e;
  out.pl_n = pred.pl_n;
  out.pl_u = pred.pl_u;
  out.pl_ff_h = pred.pl_ff_h;
  out.pl_ff_v = pred.pl_ff_v;
  out.sigma_h = pred.sigma_h;
  out.sigma_v = pred.sigma_v;
  out.pdop = pred.pdop;
  out.n_vis = pred.n_vis;
  out.n_hypotheses = pred.n_hypotheses;
  // Legacy storage names retained; semantically these are advisory proxy
  // outputs, not current certified monitor fields.
  out.gnss_hpl = pred.hpl;
  out.gnss_vpl = pred.vpl;
  out.fused_hpl = pred.hpl;
  out.fused_vpl = pred.vpl;
  out.hpl_adv = pred.hpl;
  out.vpl_adv = pred.vpl;
  out.fim_fallback_reason = "fim_disabled";
  out.advisory_fusion_mode = AdvisoryFusionMode::Legacy;
  return out;
}

}  // namespace iap
