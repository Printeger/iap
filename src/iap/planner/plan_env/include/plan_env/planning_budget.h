#ifndef IAP_PLANNING_BUDGET_H
#define IAP_PLANNING_BUDGET_H
#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
// One round owns this budget; nested optimizer/search calls share it.
class PlanningBudget {
 public:
  enum class Repair { TargetShortening, Reinitialize, Search, AdvisoryFallback,
                      BackendRestart, PublicationRecheck, CurveCorrection, Count };
  using Clock = std::chrono::steady_clock;
  explicit PlanningBudget(double seconds = 1.5, unsigned repairs = 3)
      : deadline_(started_ + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(seconds))), limit_(repairs) {}
  double remaining() const {
    return std::max(0.0, std::chrono::duration<double>(deadline_ - Clock::now()).count());
  }
  // Measured successful forest post-search work reaches 0.387 s. Retain
  // 0.5 s for backend/check/commit, and 0.4 s from optional quality work.
  double searchRemaining() const { return std::max(0.,std::min(1.-searches.seconds,remaining()-.5)); }
  double optionalRemaining() const { return std::max(0.,remaining()-.4); }
  void beginOptionalWork() { optional_work_=true; }
  void endOptionalWork() { optional_work_=false; }
  bool workExpired() const { return expired() || (optional_work_ && optionalRemaining()<=0.); }
  bool expired() const { return Clock::now() >= deadline_; }
  bool tryRepair(Repair reason) {
    // Optional quality (including its optimizer restarts) cannot consume the
    // last original allowance needed by current corridor publication. A soft
    // refusal keeps the already checked working curve; it is not a hard
    // exhausted-budget disposition. No allowance is added or replenished.
    if (optional_work_ && (used_ >= limit_ || limit_ - used_ <= 1)) return false;
    if (expired() || used_ >= limit_) { denied_ = true; return false; }
    ++used_; ++counts_[static_cast<size_t>(reason)]; return true;
  }
  bool denied() const { return denied_; }
  unsigned used() const { return used_; }
  unsigned count(Repair reason) const { return counts_[static_cast<size_t>(reason)]; }
  double elapsed() const { return std::chrono::duration<double>(Clock::now()-started_).count(); }
  struct SearchStatistics {
    double seconds=0;
    size_t calls=0, queries=0, expanded=0, pushes=0, pops=0;
    std::array<size_t,3> hits{}, misses{}, peak_cache_bytes{};
  } searches;
  using Ptr = std::shared_ptr<PlanningBudget>;
 private:
  Clock::time_point started_=Clock::now();
  Clock::time_point deadline_;
  unsigned limit_, used_ = 0;
  bool denied_ = false;
  bool optional_work_ = false;
  std::array<unsigned, static_cast<size_t>(Repair::Count)> counts_{};
};
#endif
