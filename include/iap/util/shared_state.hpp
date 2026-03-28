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
#include <iap/planner/continuous_trajectory_view.hpp>
#include <iap/trunk/trunk_types.hpp>

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

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
    gnss_epoch_queue_.push_back(epoch);
    while (gnss_epoch_queue_.size() > gnss_epoch_queue_limit_) {
      gnss_epoch_queue_.pop_front();
    }
  }

  /// Returns a copy of the latest GnssEpoch, or nullopt if none yet.
  std::optional<GnssEpoch> get_gnss_epoch() const {
    std::lock_guard<std::mutex> lk(gnss_mutex_);
    return latest_gnss_epoch_;
  }

  /// Drains GNSS epochs that align with the requested stamp within tolerance.
  std::vector<GnssEpoch> consume_gnss_epochs_near(double stamp, double tolerance) {
    std::lock_guard<std::mutex> lk(gnss_mutex_);

    std::vector<GnssEpoch> matched;
    auto it = gnss_epoch_queue_.begin();
    while (it != gnss_epoch_queue_.end()) {
      if (std::abs(it->stamp - stamp) <= tolerance) {
        matched.push_back(std::move(*it));
        it = gnss_epoch_queue_.erase(it);
      } else if (it->stamp < stamp - tolerance) {
        it = gnss_epoch_queue_.erase(it);
      } else {
        ++it;
      }
    }

    return matched;
  }

  // ── GNSS anchor ────────────────────────────────────────────────────────

  void set_gnss_anchor(const GnssAnchorState& anchor) {
    std::lock_guard<std::mutex> lk(gnss_mutex_);
    latest_gnss_anchor_ = anchor;
  }

  std::optional<GnssAnchorState> get_gnss_anchor() const {
    std::lock_guard<std::mutex> lk(gnss_mutex_);
    return latest_gnss_anchor_;
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

  // ── Clock readiness marker (GNSS owner mode) ─────────────────────────

  /// Mark that C(frame_id) has been successfully prepared by GNSS injection.
  void set_clock_ready(long frame_id, double stamp) {
    clock_ready_frame_id_.store(frame_id, std::memory_order_relaxed);
    clock_ready_stamp_.store(stamp, std::memory_order_relaxed);
  }

  /// Clear readiness marker (used during recovery/corruption reset).
  void clear_clock_ready() {
    clock_ready_frame_id_.store(-1, std::memory_order_relaxed);
    clock_ready_stamp_.store(0.0, std::memory_order_relaxed);
  }

  /// True when GNSS has explicitly prepared clock state for this frame.
  bool is_clock_ready(long frame_id) const {
    return clock_ready_frame_id_.load(std::memory_order_relaxed) == frame_id;
  }

  double get_clock_ready_stamp() const {
    return clock_ready_stamp_.load(std::memory_order_relaxed);
  }

  // ── Continuous trajectory publication (dev_ct foundation) ─────────────

  void set_continuous_trajectory_view(std::shared_ptr<const ContinuousTrajectoryView> view) {
    std::lock_guard<std::mutex> lk(trajectory_mutex_);
    latest_trajectory_view_ = std::move(view);
  }

  std::shared_ptr<const ContinuousTrajectoryView> get_continuous_trajectory_view() const {
    std::lock_guard<std::mutex> lk(trajectory_mutex_);
    return latest_trajectory_view_;
  }

  void set_spline_control_access(std::shared_ptr<const SplineControlAccess> access) {
    std::lock_guard<std::mutex> lk(trajectory_mutex_);
    latest_control_access_ = std::move(access);
  }

  std::shared_ptr<const SplineControlAccess> get_spline_control_access() const {
    std::lock_guard<std::mutex> lk(trajectory_mutex_);
    return latest_control_access_;
  }

 private:
  IapSharedState() = default;

  mutable std::mutex gnss_mutex_;
  std::optional<GnssEpoch> latest_gnss_epoch_;
  std::optional<GnssAnchorState> latest_gnss_anchor_;
  std::deque<GnssEpoch> gnss_epoch_queue_;
  std::size_t gnss_epoch_queue_limit_ = 256;

  std::atomic<int> n_confirmed_trunks_{0};

  mutable std::mutex trunk_mutex_;
  std::optional<TrunkDetectionResult> latest_trunk_det_;

  std::atomic<long> clock_ready_frame_id_{-1};
  std::atomic<double> clock_ready_stamp_{0.0};

  mutable std::mutex trajectory_mutex_;
  std::shared_ptr<const ContinuousTrajectoryView> latest_trajectory_view_;
  std::shared_ptr<const SplineControlAccess> latest_control_access_;
};

}  // namespace iap
