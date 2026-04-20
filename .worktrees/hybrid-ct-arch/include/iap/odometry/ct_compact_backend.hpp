#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Compact backend interface for the planned hybrid CT architecture.
// This boundary consumes only compact frontend summary state while keeping
// GNSS/shared-state/mapping ownership on the backend side.

#include <iap/odometry/ct_backend_summary.hpp>

#include <cstddef>
#include <optional>

namespace iap {

struct CTCompactBackendResult {
  std::size_t gnss_factor_count{0};
  std::size_t carried_prior_count{0};
  std::optional<CTBackendSummary> consumed_frontend_summary;
};

class CTCompactBackend {
 public:
  struct Input {
    std::optional<CTBackendSummary> frontend_summary;
  };

  CTCompactBackendResult run(const Input& input) const;
};

}  // namespace iap
