#pragma once
// IAP Step 4: Explicit integrity source result for fusion policy.

#include <string>

namespace iap {

struct IntegritySourceResult {
  bool   enabled      = true;
  bool   valid        = false;
  std::string source_name = "UNKNOWN";
  double HPL          = 1e9;
  double VPL          = 1e9;
  double PL_E         = 1e9;
  double PL_N         = 1e9;
  double PL_U         = 1e9;
  std::string failure_reason;

  static IntegritySourceResult make_disabled(const std::string& name) {
    IntegritySourceResult r;
    r.enabled = false;
    r.valid = false;
    r.source_name = name;
    r.failure_reason = name + " disabled in config";
    return r;
  }

  static IntegritySourceResult make_invalid(const std::string& name,
                                        const std::string& reason = "") {
    IntegritySourceResult r;
    r.enabled = true;
    r.valid = false;
    r.source_name = name;
    r.failure_reason = reason.empty() ? name + " source invalid" : reason;
    return r;
  }

  static IntegritySourceResult make_valid(const std::string& name,
                                      double hpl, double vpl,
                                      double pl_e, double pl_n, double pl_u) {
    IntegritySourceResult r;
    r.enabled = true;
    r.valid = true;
    r.source_name = name;
    r.HPL = hpl;
    r.VPL = vpl;
    r.PL_E = pl_e;
    r.PL_N = pl_n;
    r.PL_U = pl_u;
    return r;
  }
};

}  // namespace iap
