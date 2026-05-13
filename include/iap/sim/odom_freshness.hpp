#pragma once

#include <cmath>
#include <cstddef>
#include <limits>

namespace iap::sim {

enum class OdomFreshnessReference {
  kTruthOdomStamp,
  kNodeNowNoTruth,
};

enum class OdomFreshnessRejectReason {
  kAccepted,
  kStale,
  kNonIncreasing,
  kZeroStamp,
};

struct OdomFreshnessDecision {
  bool stamp_increasing = false;
  bool fresh_enough = false;
  bool valid_sample = false;
  double age_sec = std::numeric_limits<double>::quiet_NaN();
  double reference_stamp_sec = std::numeric_limits<double>::quiet_NaN();
  OdomFreshnessReference reference = OdomFreshnessReference::kNodeNowNoTruth;
  OdomFreshnessRejectReason reason = OdomFreshnessRejectReason::kStale;
};

inline const char* to_string(const OdomFreshnessReference reference) {
  switch (reference) {
    case OdomFreshnessReference::kTruthOdomStamp:
      return "truth_odom_stamp";
    case OdomFreshnessReference::kNodeNowNoTruth:
      return "node_now_no_truth";
  }
  return "unknown";
}

inline const char* to_string(const OdomFreshnessRejectReason reason) {
  switch (reason) {
    case OdomFreshnessRejectReason::kAccepted:
      return "accepted";
    case OdomFreshnessRejectReason::kStale:
      return "stale";
    case OdomFreshnessRejectReason::kNonIncreasing:
      return "non_increasing";
    case OdomFreshnessRejectReason::kZeroStamp:
      return "zero_stamp";
  }
  return "unknown";
}

inline std::size_t histogram_index(const OdomFreshnessRejectReason reason) {
  switch (reason) {
    case OdomFreshnessRejectReason::kAccepted:
      return 0;
    case OdomFreshnessRejectReason::kStale:
      return 1;
    case OdomFreshnessRejectReason::kNonIncreasing:
      return 2;
    case OdomFreshnessRejectReason::kZeroStamp:
      return 3;
  }
  return 1;
}

inline OdomFreshnessDecision evaluate_odom_freshness(
    const double stamp_sec,
    const bool have_last_stamp,
    const double last_stamp_sec,
    const bool have_truth_stamp,
    const double truth_stamp_sec,
    const double node_now_sec,
    const double freshness_sec) {
  OdomFreshnessDecision decision;
  decision.stamp_increasing = !have_last_stamp || stamp_sec > last_stamp_sec;
  decision.reference = have_truth_stamp ? OdomFreshnessReference::kTruthOdomStamp
                                        : OdomFreshnessReference::kNodeNowNoTruth;
  decision.reference_stamp_sec = have_truth_stamp ? truth_stamp_sec : node_now_sec;
  decision.age_sec = std::abs(decision.reference_stamp_sec - stamp_sec);
  decision.fresh_enough = std::isfinite(decision.age_sec) && decision.age_sec <= freshness_sec;
  decision.valid_sample = decision.stamp_increasing && decision.fresh_enough && stamp_sec > 0.0;

  if (decision.valid_sample) {
    decision.reason = OdomFreshnessRejectReason::kAccepted;
  } else if (stamp_sec <= 0.0) {
    decision.reason = OdomFreshnessRejectReason::kZeroStamp;
  } else if (!decision.stamp_increasing) {
    decision.reason = OdomFreshnessRejectReason::kNonIncreasing;
  } else {
    decision.reason = OdomFreshnessRejectReason::kStale;
  }
  return decision;
}

}  // namespace iap::sim
