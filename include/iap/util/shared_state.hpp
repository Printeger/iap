#pragma once
// IapSharedState — lightweight cross-extension shared data bus.
//
// Purpose:
//   Extension modules run as independent shared libraries loaded by GLIM.
//   There is no built-in mechanism for an extension to query data from
//   another.  This singleton provides a thread-safe mailbox where:
//     - gnss_extension writes the latest GnssEpoch after building each epoch.
//     - trunk_extension writes the confirmed landmark count after each
//       smoother update.
//     - integrity_extension reads both in on_smoother_update_finish to run
//       IntegrityMonitor::compute().
//
// Usage:
//   Writer (gnss_extension):
//     IapSharedState::instance().set_gnss_epoch(epoch);
//
//   Writer (trunk_extension):
//     IapSharedState::instance().set_n_confirmed_trunks(n);
//
//   Reader (integrity_extension):
//     auto epoch = IapSharedState::instance().get_gnss_epoch();  // optional<GnssEpoch>
//     int  n     = IapSharedState::instance().get_n_confirmed_trunks();

#include <iap/gnss/gnss_types.hpp>
#include <iap/trunk/trunk_types.hpp>

#include <mutex>
#include <optional>

namespace iap {

class IapSharedState {
 public:
  /// Return the process-wide singleton.
  static IapSharedState& instance() {
    static IapSharedState inst;
    return inst;
  }

  // ── GNSS epoch ──────────────────────────────────────────────────────────

  /// Called by gnss_extension after each epoch is built and enqueued.
  void set_gnss_epoch(const GnssEpoch& epoch) {
    std::lock_guard<std::mutex> lk(gnss_mutex_);
    latest_gnss_epoch_ = epoch;
  }

  /// Returns a copy of the latest GnssEpoch, or nullopt if none yet.
  std::optional<GnssEpoch> get_gnss_epoch() const {
    std::lock_guard<std::mutex> lk(gnss_mutex_);
    return latest_gnss_epoch_;
  }

  // ── Confirmed trunk count ─────────────────────────────────────────────

  /// Called by trunk_extension after each smoother update.
  void set_n_confirmed_trunks(int n) {
    n_confirmed_trunks_.store(n, std::memory_order_relaxed);
  }

  /// Returns the number of confirmed trunk landmarks in the factor graph.
  int get_n_confirmed_trunks() const {
    return n_confirmed_trunks_.load(std::memory_order_relaxed);
  }

  // ── Latest trunk detection (world-frame, from on_new_frame) ──────────

  /// Called by trunk_extension::on_new_frame_ after transforming detections
  /// to world frame.  Provides trunk geometry for HAL computation.
  void set_trunk_detection(const TrunkDetectionResult& det) {
    std::lock_guard<std::mutex> lk(trunk_mutex_);
    latest_trunk_det_ = det;
  }

  /// Returns the latest TrunkDetectionResult, or nullopt if none yet.
  std::optional<TrunkDetectionResult> get_trunk_detection() const {
    std::lock_guard<std::mutex> lk(trunk_mutex_);
    return latest_trunk_det_;
  }

 private:
  IapSharedState() = default;

  mutable std::mutex gnss_mutex_;
  std::optional<GnssEpoch> latest_gnss_epoch_;

  std::atomic<int> n_confirmed_trunks_{0};

  mutable std::mutex trunk_mutex_;
  std::optional<TrunkDetectionResult> latest_trunk_det_;
};

}  // namespace iap
