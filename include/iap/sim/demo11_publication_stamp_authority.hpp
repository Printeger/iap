#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace iap::sim {

enum class StampUpdateResult {
  kAccepted,
  kNonPositive,
  kMalformed,
  kRegressed,
};

class Demo11PublicationStampAuthority {
 public:
  StampUpdateResult update(const builtin_interfaces::msg::Time& stamp) {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {
      return StampUpdateResult::kMalformed;
    }
    if (stamp.sec == 0 && stamp.nanosec == 0U) {
      return StampUpdateResult::kNonPositive;
    }
    if (accepted_ && less_than(stamp, *accepted_)) {
      return StampUpdateResult::kRegressed;
    }
    accepted_ = stamp;
    return StampUpdateResult::kAccepted;
  }

  std::optional<builtin_interfaces::msg::Time> snapshot() const {
    return accepted_;
  }

 private:
  static bool less_than(const builtin_interfaces::msg::Time& lhs,
                        const builtin_interfaces::msg::Time& rhs) {
    return lhs.sec < rhs.sec ||
           (lhs.sec == rhs.sec && lhs.nanosec < rhs.nanosec);
  }

  std::optional<builtin_interfaces::msg::Time> accepted_;
};

template <std::size_t N>
bool stamp_demo11_publication(
    const Demo11PublicationStampAuthority& authority,
    std::array<sensor_msgs::msg::PointCloud2, N>& clouds) {
  const auto stamp = authority.snapshot();
  if (!stamp) {
    return false;
  }
  for (auto& cloud : clouds) {
    cloud.header.stamp = *stamp;
  }
  return true;
}

template <typename... Clouds>
bool stamp_demo11_publication(
    const Demo11PublicationStampAuthority& authority,
    sensor_msgs::msg::PointCloud2& first,
    Clouds&... remaining) {
  const auto stamp = authority.snapshot();
  if (!stamp) {
    return false;
  }
  first.header.stamp = *stamp;
  ((remaining.header.stamp = *stamp), ...);
  return true;
}

}  // namespace iap::sim
