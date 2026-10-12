#pragma once

#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <iap/util/run_log_manager.hpp>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <set>
#include <nlohmann/json.hpp>

namespace iap {
struct NativeResponsePacket;
// Diagnostic native response coefficients, NOT a physical joint error law.
// No nonlinear evaluation, GPU synchronization, marginal covariance or solve.
class NativeResponseObserver : public std::enable_shared_from_this<NativeResponseObserver> {
 public:
  static constexpr std::size_t kMaxFrames=31, kMaxKeys=512, kMaxFactors=4096;
  static constexpr std::size_t kMaxIndexEntries=32768, kMaxRetainedGpuFactors=2048;
  static constexpr std::size_t kMaxFactorDim=1536, kMaxBytes=8*1024*1024, kQueueLimit=2;
  static constexpr std::size_t kMaxCachedBytes=2*kMaxBytes;
  static constexpr double kCopyBudgetMs=5.;
  NativeResponseObserver(const glim::RunLogManager& logs, const std::vector<long>& frames,
                         bool first_source_freeze=false);
  ~NativeResponseObserver();
  void prepare(gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
               const gtsam::NonlinearFactorGraph& new_factors, long frame, double stamp);
  // Also used by focused native-solver contract tests.
  static nlohmann::json snapshot(const gtsam_points::IncrementalFixedLagSmootherExt& native,
                                 const gtsam::KeyVector& discarded, long frame, double stamp);
 private:
  void observe(const gtsam_points::IncrementalFixedLagSmootherExt& native,
               const gtsam::KeyVector& discarded, long frame, double stamp);
  void drain();
  static std::unique_ptr<NativeResponsePacket> capture(
      const gtsam_points::IncrementalFixedLagSmootherExt& native,
      const gtsam::KeyVector& discarded, long frame, double stamp);
  static nlohmann::json serialize(const NativeResponsePacket& packet);
  std::set<long> frames_, seen_;
  bool initial_seen_=false, stopping_=false;
  bool source_freeze_enabled_=false, source_selection_done_=false;
  long source_id_=-1, source_birth_frame_=-1, source_freeze_frame_=-1;
  long source_previous_frame_=-1, source_retirement_frame_=-1;
  long observed_frame_=-1;
  std::size_t update_in_frame_=0;
  std::set<long> boundary_frames_seen_;
  std::string source_selection_failure_;
  double max_observer_ms_=0.;
  std::size_t observer_budget_overruns_=0, rolling_copies_=0, rolling_failures_=0;
  std::unique_ptr<NativeResponsePacket> previous_;
  std::filesystem::path raw_path_, manifest_path_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<std::unique_ptr<NativeResponsePacket>> queue_;
  std::vector<nlohmann::json> ledger_;
  std::vector<std::weak_ptr<gtsam::NonlinearFactor>> retained_gpu_factors_;
  std::thread worker_;
  void* raw_=nullptr;
  std::size_t written_=0, write_failures_=0;
};
}
