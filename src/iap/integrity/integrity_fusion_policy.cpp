// IAP Step 4: IntegrityFusionPolicy implementation.

#include <iap/integrity/integrity_fusion_policy.hpp>
#include <iap/integrity/numerical_guard.hpp>
#include <algorithm>
#include <cmath>

namespace iap {

IntegrityFusionPolicy::IntegrityFusionPolicy(
    const IntegrityFusionPolicyParams& params)
    : params_(params) {}

IntegrityFusionResult IntegrityFusionPolicy::conservative(
    const std::string& reason) const {
  IntegrityFusionResult r;
  r.HPL = params_.conservative_hpl_m;
  r.VPL = params_.conservative_vpl_m;
  r.PL_E = params_.conservative_hpl_m;
  r.PL_N = params_.conservative_hpl_m;
  r.PL_U = params_.conservative_vpl_m;
  r.final_HPL_source = "CONSERVATIVE";
  r.final_VPL_source = "CONSERVATIVE";
  r.final_PL_source  = "CONSERVATIVE";
  r.failure_reason = reason;
  r.any_source_valid = false;
  return r;
}

IntegrityFusionResult IntegrityFusionPolicy::fuse_max_pl(
    const IntegritySourceResult& fallback,
    const IntegritySourceResult& gnss,
    const IntegritySourceResult& lidar) const {

  if (params_.require_valid_gnss && !gnss.valid) {
    return conservative("required GNSS source missing or invalid");
  }
  if (params_.require_valid_lidar && !lidar.valid) {
    return conservative("required LiDAR source missing or invalid");
  }

  auto use_source = [&](const IntegritySourceResult& src) {
    return src.enabled && src.valid;
  };

  bool have_fallback = use_source(fallback);
  bool have_gnss     = use_source(gnss);
  bool have_lidar    = use_source(lidar);

  if (!have_fallback && !have_gnss && !have_lidar) {
    return conservative("no valid integrity source available");
  }

  IntegrityFusionResult r;
  r.any_source_valid = true;
  r.PL_E = 0.0;
  r.PL_N = 0.0;
  r.PL_U = 0.0;

  auto max_into = [&](const IntegritySourceResult& src,
                       const std::string& tag) {
    if (src.PL_E > r.PL_E) { r.PL_E = src.PL_E; }
    if (src.PL_N > r.PL_N) { r.PL_N = src.PL_N; }
    if (src.PL_U > r.PL_U) { r.PL_U = src.PL_U; }
  };

  if (have_fallback) max_into(fallback, "FALLBACK");
  if (have_gnss)     max_into(gnss,     "GNSS");
  if (have_lidar)    max_into(lidar,    "LIDAR");

  r.HPL = std::max(r.PL_E, r.PL_N);
  r.VPL = r.PL_U;

  // Determine dominant source per axis
  double best_h_e = 0.0, best_h_n = 0.0, best_v = 0.0;
  auto track = [&](const IntegritySourceResult& src,
                    const std::string& tag) {
    if (src.enabled && src.valid) {
      if (src.PL_E > best_h_e || src.PL_N > best_h_n) {
        r.final_HPL_source = tag;
        best_h_e = std::max(best_h_e, src.PL_E);
        best_h_n = std::max(best_h_n, src.PL_N);
      }
      if (src.PL_U > best_v) {
        r.final_VPL_source = tag;
        best_v = src.PL_U;
      }
    }
  };
  if (have_fallback) track(fallback, "FALLBACK");
  if (have_gnss)     track(gnss,     "GNSS");
  if (have_lidar)    track(lidar,    "LIDAR");

  r.final_PL_source = (r.HPL > r.VPL) ? r.final_HPL_source
                                        : r.final_VPL_source;

  return r;
}

IntegrityFusionResult IntegrityFusionPolicy::fuse(
    const IntegritySourceResult& fallback,
    const IntegritySourceResult& gnss,
    const IntegritySourceResult& lidar) const {
  switch (params_.mode) {
    case IntegrityFusionMode::GNSS_ONLY:
      if (gnss.enabled && gnss.valid) {
        auto r = fuse_max_pl(
            IntegritySourceResult::make_disabled("FALLBACK"),
            gnss,
            IntegritySourceResult::make_disabled("LIDAR"));
        r.PL_E = gnss.PL_E; r.PL_N = gnss.PL_N; r.PL_U = gnss.PL_U;
        r.HPL = gnss.HPL; r.VPL = gnss.VPL;
        r.final_HPL_source = "GNSS";
        r.final_VPL_source = "GNSS";
        r.final_PL_source  = "GNSS";
        r.any_source_valid = true;
        return r;
      }
      return conservative("GNSS source unavailable or invalid (gnss_only mode)");

    case IntegrityFusionMode::LIDAR_ONLY:
      if (lidar.enabled && lidar.valid) {
        auto r = fuse_max_pl(
            IntegritySourceResult::make_disabled("FALLBACK"),
            IntegritySourceResult::make_disabled("GNSS"),
            lidar);
        r.PL_E = lidar.PL_E; r.PL_N = lidar.PL_N; r.PL_U = lidar.PL_U;
        r.HPL = lidar.HPL; r.VPL = lidar.VPL;
        r.final_HPL_source = "LIDAR";
        r.final_VPL_source = "LIDAR";
        r.final_PL_source  = "LIDAR";
        r.any_source_valid = true;
        return r;
      }
      return conservative("LiDAR source unavailable or invalid (lidar_only mode)");

    case IntegrityFusionMode::FALLBACK_ONLY:
      if (fallback.enabled && fallback.valid) {
        auto r = fuse_max_pl(
            fallback,
            IntegritySourceResult::make_disabled("GNSS"),
            IntegritySourceResult::make_disabled("LIDAR"));
        r.PL_E = fallback.PL_E; r.PL_N = fallback.PL_N; r.PL_U = fallback.PL_U;
        r.HPL = fallback.HPL; r.VPL = fallback.VPL;
        r.final_HPL_source = "FALLBACK";
        r.final_VPL_source = "FALLBACK";
        r.final_PL_source  = "FALLBACK";
        r.any_source_valid = true;
        return r;
      }
      return conservative("Fallback source unavailable (fallback_only mode)");

    case IntegrityFusionMode::MAX_PL:
      return fuse_max_pl(fallback, gnss, lidar);

    case IntegrityFusionMode::WEIGHTED_DEBUG_ONLY:
      return fuse_max_pl(fallback, gnss, lidar);
  }
  return conservative("unknown fusion mode");
}

}  // namespace iap
