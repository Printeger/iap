#include <iap/odometry/ct_compact_backend.hpp>

namespace iap {

CTCompactBackendResult CTCompactBackend::run(const Input& input) const {
  CTCompactBackendResult result;
  result.gnss_factor_count = 0;
  result.carried_prior_count = 0;
  result.consumed_frontend_summary = input.frontend_summary;
  return result;
}

}  // namespace iap
