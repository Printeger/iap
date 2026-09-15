#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <iap/gnss/gnss_types.hpp>

namespace iap {

// State is local to the advisory input stream. It never delays disappearance,
// hard occlusion, or a certified exclusion; it only debounces additions.
class GnssSatelliteAdmissionHysteresis {
 public:
  explicit GnssSatelliteAdmissionHysteresis(
      int required_epochs = 3, double maximum_epoch_gap_s = 2.0)
      : required_epochs_(std::max(1, required_epochs)),
        maximum_epoch_gap_s_(maximum_epoch_gap_s) {}

  std::vector<int> apply(GnssEpoch* epoch) {
    std::vector<int> pending;
    if (!epoch) return pending;
    const bool stamp_valid = std::isfinite(epoch->stamp);
    const bool duplicate_epoch = stamp_valid &&
        std::isfinite(last_epoch_stamp_s_) &&
        epoch->stamp == last_epoch_stamp_s_ &&
        epoch->source_identity == last_source_identity_;
    const bool stream_gap = stamp_valid &&
        std::isfinite(last_epoch_stamp_s_) && epoch->stamp > last_epoch_stamp_s_ &&
        maximum_epoch_gap_s_ >= 0.0 &&
        epoch->stamp - last_epoch_stamp_s_ > maximum_epoch_gap_s_;
    if (stream_gap) consecutive_epochs_.clear();
    std::unordered_set<int> present;
    present.reserve(epoch->sats.size());
    for (const auto& sat : epoch->sats) present.insert(sat.sat_id);
    for (auto it = consecutive_epochs_.begin();
         it != consecutive_epochs_.end();) {
      if (present.count(it->first) == 0u) {
        it = consecutive_epochs_.erase(it);
      } else {
        ++it;
      }
    }
    for (auto& sat : epoch->sats) {
      int& count = consecutive_epochs_[sat.sat_id];
      if (!duplicate_epoch)
        count = std::min(required_epochs_, count + 1);
      sat.admission_hysteresis_pending = count < required_epochs_;
      if (sat.admission_hysteresis_pending) {
        pending.push_back(sat.sat_id);
      }
    }
    if (!duplicate_epoch && stamp_valid) {
      last_epoch_stamp_s_ = epoch->stamp;
      last_source_identity_ = epoch->source_identity;
    }
    std::sort(pending.begin(), pending.end());
    return pending;
  }

  void reset() {
    consecutive_epochs_.clear();
    last_epoch_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
    last_source_identity_ = 0;
  }

 private:
  int required_epochs_ = 3;
  double maximum_epoch_gap_s_ = 2.0;
  double last_epoch_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t last_source_identity_ = 0;
  std::unordered_map<int, int> consecutive_epochs_;
};

}  // namespace iap
