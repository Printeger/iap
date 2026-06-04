#pragma once
// IAP Step 4: Explicit integrity fusion policy.

#include <iap/integrity/integrity_types.hpp>
#include <iap/integrity/integrity_source_result.hpp>
#include <string>

namespace iap {

struct IntegrityFusionPolicyParams {
  IntegrityFusionMode mode = IntegrityFusionMode::MAX_PL;
  bool require_valid_gnss  = false;
  bool require_valid_lidar = false;
  double conservative_hpl_m = 999.0;
  double conservative_vpl_m = 999.0;
};

struct IntegrityFusionResult {
  double HPL = 1e9;
  double VPL = 1e9;
  double PL_E = 1e9;
  double PL_N = 1e9;
  double PL_U = 1e9;
  std::string final_HPL_source = "UNKNOWN";
  std::string final_VPL_source = "UNKNOWN";
  std::string final_PL_source  = "UNKNOWN";
  std::string failure_reason;
  bool   any_source_valid = false;
};

class IntegrityFusionPolicy {
 public:
  explicit IntegrityFusionPolicy(const IntegrityFusionPolicyParams& params = {});
  const IntegrityFusionPolicyParams& params() const { return params_; }

  IntegrityFusionResult fuse(const IntegritySourceResult& fallback,
                              const IntegritySourceResult& gnss,
                              const IntegritySourceResult& lidar) const;

 private:
  IntegrityFusionResult conservative(const std::string& reason) const;
  IntegrityFusionResult fuse_max_pl(const IntegritySourceResult& fallback,
                                     const IntegritySourceResult& gnss,
                                     const IntegritySourceResult& lidar) const;

  IntegrityFusionPolicyParams params_;
};

}  // namespace iap
