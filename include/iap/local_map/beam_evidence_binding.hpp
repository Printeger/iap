#pragma once

#include <iap/msg/lidar_beam_evidence.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace iap::local_map {

inline double beamEvidenceStampSeconds(
    const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) +
      static_cast<double>(stamp.nanosec) * 1.0e-9;
}

inline std::string beamEvidenceContentHash(
    const iap::msg::LidarBeamEvidence& evidence) {
  std::uint64_t hash = 1469598103934665603ULL;
  const auto mix = [&hash](const void* data, const std::size_t size) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t index = 0; index < size; ++index) {
      hash ^= static_cast<std::uint64_t>(bytes[index]);
      hash *= 1099511628211ULL;
    }
  };
  for (std::size_t index = 0; index < evidence.outcomes.size(); ++index) {
    mix(&evidence.outcomes[index], sizeof(evidence.outcomes[index]));
    mix(&evidence.direction_x[index], sizeof(evidence.direction_x[index]));
    mix(&evidence.direction_y[index], sizeof(evidence.direction_y[index]));
    mix(&evidence.direction_z[index], sizeof(evidence.direction_z[index]));
    mix(&evidence.ranges_m[index], sizeof(evidence.ranges_m[index]));
  }
  std::ostringstream out;
  out << std::hex << std::setfill('0') << std::setw(16) << hash;
  return out.str();
}

inline bool validBeamEvidenceMessage(
    const iap::msg::LidarBeamEvidence& evidence) {
  const std::size_t count = evidence.outcomes.size();
  const std::size_t expected =
      static_cast<std::size_t>(evidence.horizontal_samples) *
      static_cast<std::size_t>(evidence.vertical_samples);
  return evidence.complete && !evidence.sensor_model_id.empty() &&
      evidence.horizontal_samples > 0U && evidence.vertical_samples > 0U &&
      count == expected && evidence.direction_x.size() == count &&
      evidence.direction_y.size() == count &&
      evidence.direction_z.size() == count &&
      evidence.ranges_m.size() == count &&
      std::isfinite(evidence.scan_end_stamp_s) &&
      std::isfinite(evidence.horizontal_fov_rad) &&
      std::isfinite(evidence.vertical_min_rad) &&
      std::isfinite(evidence.vertical_max_rad) &&
      evidence.vertical_min_rad <= evidence.vertical_max_rad &&
      std::isfinite(evidence.min_range_m) && evidence.min_range_m >= 0.0 &&
      std::isfinite(evidence.max_range_m) &&
      evidence.max_range_m > evidence.min_range_m &&
      !evidence.content_hash.empty() &&
      evidence.content_hash == beamEvidenceContentHash(evidence);
}

// Matching an already validated history entry must not hash all beam rows
// again, especially for every nonmatching scan in the bounded history.
inline bool beamEvidenceScanTimesMatch(
    const iap::msg::LidarBeamEvidence& evidence,
    const double scan_stamp_s, const double scan_end_stamp_s,
    const double tolerance_s = 1.0e-6) {
  return std::isfinite(scan_stamp_s) && std::isfinite(scan_end_stamp_s) &&
      std::isfinite(tolerance_s) && tolerance_s >= 0.0 &&
      std::abs(beamEvidenceStampSeconds(evidence.header.stamp) -
               scan_stamp_s) <= tolerance_s &&
      std::abs(evidence.scan_end_stamp_s - scan_end_stamp_s) <= tolerance_s;
}

inline bool beamEvidenceMatchesRegisteredScan(
    const iap::msg::LidarBeamEvidence& evidence,
    const double scan_stamp_s, const double scan_end_stamp_s,
    const double tolerance_s = 1.0e-6) {
  return beamEvidenceScanTimesMatch(
      evidence, scan_stamp_s, scan_end_stamp_s, tolerance_s) &&
      validBeamEvidenceMessage(evidence);
}

}  // namespace iap::local_map
