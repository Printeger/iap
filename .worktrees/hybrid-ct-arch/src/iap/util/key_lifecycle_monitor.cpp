#include <iap/util/key_lifecycle_monitor.hpp>

#include <algorithm>
#include <map>
#include <mutex>
#include <sstream>

#include <spdlog/spdlog.h>

namespace glim {

namespace {

struct SymbolStats {
  std::map<std::string, std::uint64_t> writes_by_owner;
  std::map<std::string, std::uint64_t> missing_by_consumer;
  std::string expected_owner;
  std::uint64_t owner_conflicts = 0;
  std::uint64_t owner_policy_violations = 0;
};

struct MonitorState {
  std::mutex mutex;
  std::map<char, SymbolStats> stats;
  std::uint64_t events = 0;
  double last_log_stamp = 0.0;
};

MonitorState& state() {
  static MonitorState monitor_state;
  return monitor_state;
}

std::string summary_unsafe(const std::map<char, SymbolStats>& stats) {
  std::ostringstream oss;
  oss << "{";

  bool first_symbol = true;
  for (const auto& [symbol, symbol_stats] : stats) {
    if (!first_symbol) {
      oss << ", ";
    }
    first_symbol = false;

    oss << symbol << ":{";
    oss << "writes=";
    bool first_owner = true;
    for (const auto& [owner, count] : symbol_stats.writes_by_owner) {
      if (!first_owner) {
        oss << "|";
      }
      first_owner = false;
      oss << owner << "=" << count;
    }

    oss << ";missing=";
    bool first_consumer = true;
    for (const auto& [consumer, count] : symbol_stats.missing_by_consumer) {
      if (!first_consumer) {
        oss << "|";
      }
      first_consumer = false;
      oss << consumer << "=" << count;
    }

    oss << ";expected=" << (symbol_stats.expected_owner.empty() ? "-" : symbol_stats.expected_owner);
    oss << ";conflicts=" << symbol_stats.owner_conflicts;
    oss << ";violations=" << symbol_stats.owner_policy_violations;
    oss << "}";
  }

  oss << "}";
  return oss.str();
}

}  // namespace

KeyLifecycleMonitor& KeyLifecycleMonitor::instance() {
  static KeyLifecycleMonitor monitor;
  return monitor;
}

void KeyLifecycleMonitor::set_expected_owner(char symbol, const std::string& owner) {
  auto& st = state();
  std::lock_guard<std::mutex> lk(st.mutex);

  auto& symbol_stats = st.stats[symbol];
  if (!symbol_stats.expected_owner.empty() && symbol_stats.expected_owner != owner) {
    symbol_stats.owner_policy_violations++;
  }
  symbol_stats.expected_owner = owner;
  st.events++;
}

void KeyLifecycleMonitor::record_write(char symbol, const std::string& owner) {
  auto& st = state();
  std::lock_guard<std::mutex> lk(st.mutex);

  auto& symbol_stats = st.stats[symbol];
  if (!symbol_stats.expected_owner.empty() && symbol_stats.expected_owner != owner) {
    symbol_stats.owner_policy_violations++;
  }
  if (!symbol_stats.writes_by_owner.empty() && symbol_stats.writes_by_owner.find(owner) == symbol_stats.writes_by_owner.end()) {
    symbol_stats.owner_conflicts++;
  }

  symbol_stats.writes_by_owner[owner]++;
  st.events++;
}

void KeyLifecycleMonitor::record_missing(char symbol, const std::string& consumer) {
  auto& st = state();
  std::lock_guard<std::mutex> lk(st.mutex);

  auto& symbol_stats = st.stats[symbol];
  symbol_stats.missing_by_consumer[consumer]++;
  st.events++;
}

void KeyLifecycleMonitor::maybe_log(const std::shared_ptr<spdlog::logger>& logger, double stamp, std::uint64_t event_period, double min_interval_sec) {
  if (!logger) {
    return;
  }

  std::string summary;
  std::uint64_t events_snapshot = 0;
  {
    auto& st = state();
    std::lock_guard<std::mutex> lk(st.mutex);

    if (st.events == 0) {
      return;
    }

    const bool period_hit = (st.events % std::max<std::uint64_t>(1, event_period) == 0);
    const bool interval_hit = (st.last_log_stamp <= 0.0) || ((stamp - st.last_log_stamp) >= min_interval_sec);
    if (!period_hit && !interval_hit) {
      return;
    }

    summary = summary_unsafe(st.stats);
    events_snapshot = st.events;
    st.last_log_stamp = stamp;
  }

  logger->info("[lifecycle] events={} stamp={:.3f} {}", events_snapshot, stamp, summary);
}

}  // namespace glim
