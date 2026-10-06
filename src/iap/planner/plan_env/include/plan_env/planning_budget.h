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
                      BackendRestart, PublicationRecheck, Count };
  using Clock = std::chrono::steady_clock;
  explicit PlanningBudget(double seconds = 1.5, unsigned repairs = 3)
      : deadline_(Clock::now() + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(seconds))), limit_(repairs) {}
  double remaining() const {
    return std::max(0.0, std::chrono::duration<double>(deadline_ - Clock::now()).count());
  }
  bool expired() const { return Clock::now() >= deadline_; }
  bool tryRepair(Repair reason) {
    if (expired() || used_ >= limit_) { denied_ = true; return false; }
    ++used_; ++counts_[static_cast<size_t>(reason)]; return true;
  }
  bool denied() const { return denied_; }
  unsigned used() const { return used_; }
  unsigned count(Repair reason) const { return counts_[static_cast<size_t>(reason)]; }
  using Ptr = std::shared_ptr<PlanningBudget>;
 private:
  Clock::time_point deadline_;
  unsigned limit_, used_ = 0;
  bool denied_ = false;
  std::array<unsigned, static_cast<size_t>(Repair::Count)> counts_{};
};
#endif
