#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace spdlog {
class logger;
}

namespace glim {

class KeyLifecycleMonitor {
public:
  static KeyLifecycleMonitor& instance();

  void set_expected_owner(char symbol, const std::string& owner);

  void record_write(char symbol, const std::string& owner);
  void record_missing(char symbol, const std::string& consumer);

  void maybe_log(const std::shared_ptr<spdlog::logger>& logger, double stamp, std::uint64_t event_period = 500, double min_interval_sec = 5.0);

private:
  KeyLifecycleMonitor() = default;
  KeyLifecycleMonitor(const KeyLifecycleMonitor&) = delete;
  KeyLifecycleMonitor& operator=(const KeyLifecycleMonitor&) = delete;
};

}  // namespace glim
