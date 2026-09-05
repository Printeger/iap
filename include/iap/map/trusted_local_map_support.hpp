#pragma once

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace iap {

enum class LocalMapSupportAuthority : std::uint8_t {
  STRICT_OBSERVATION = 0,
  TRUSTED_LOCAL_MAP,
};

enum class LocalMapSupportStatus : std::uint8_t {
  MODEL_COMPLETE = 0,
  OBSERVATION_INCOMPLETE,
  OUTSIDE_ENVELOPE,
  EXPIRED,
  FRAME_INVALID,
};

struct LocalMapSupportQuery {
  LocalMapSupportAuthority authority =
      LocalMapSupportAuthority::STRICT_OBSERVATION;
  LocalMapSupportStatus status = LocalMapSupportStatus::FRAME_INVALID;

  bool complete() const {
    return status == LocalMapSupportStatus::MODEL_COMPLETE;
  }
};

// Immutable description of the region in which a hit-only local point cloud
// may be interpreted with the configured "no hit means no detected obstacle"
// model. It never changes OBSERVED_FREE evidence in the occupancy map.
struct TrustedLocalMapSupport {
  Eigen::Isometry3d T_map_sensor = Eigen::Isometry3d::Identity();
  Eigen::Vector3d retained_min_map = Eigen::Vector3d::Constant(
      -std::numeric_limits<double>::infinity());
  Eigen::Vector3d retained_max_map = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  double min_range_m = 0.0;
  double max_range_m = std::numeric_limits<double>::quiet_NaN();
  double horizontal_fov_rad = 2.0 * 3.14159265358979323846;
  double vertical_min_rad = -0.5 * 3.14159265358979323846;
  double vertical_max_rad = 0.5 * 3.14159265358979323846;
  double stamp_s = std::numeric_limits<double>::quiet_NaN();
  double valid_until_s = std::numeric_limits<double>::quiet_NaN();
  std::string frame_id;
  std::string model_version = "trusted_local_map_v1";

  bool valid() const {
    return T_map_sensor.matrix().allFinite() && retained_min_map.allFinite() &&
        retained_max_map.allFinite() &&
        (retained_max_map.array() > retained_min_map.array()).all() &&
        std::isfinite(min_range_m) && min_range_m >= 0.0 &&
        std::isfinite(max_range_m) && max_range_m > min_range_m &&
        std::isfinite(horizontal_fov_rad) && horizontal_fov_rad > 0.0 &&
        horizontal_fov_rad <= 2.0 * 3.14159265358979323846 + 1.0e-9 &&
        std::isfinite(vertical_min_rad) && std::isfinite(vertical_max_rad) &&
        vertical_min_rad < vertical_max_rad &&
        std::isfinite(stamp_s) && std::isfinite(valid_until_s) &&
        valid_until_s >= stamp_s && !frame_id.empty() &&
        !model_version.empty();
  }

  LocalMapSupportQuery query(const Eigen::Vector3d& point_map,
                             const double now_s) const {
    LocalMapSupportQuery out;
    out.authority = LocalMapSupportAuthority::TRUSTED_LOCAL_MAP;
    if (!valid() || !point_map.allFinite() || !std::isfinite(now_s)) {
      out.status = LocalMapSupportStatus::FRAME_INVALID;
      return out;
    }
    if (now_s < stamp_s || now_s > valid_until_s) {
      out.status = LocalMapSupportStatus::EXPIRED;
      return out;
    }
    if ((point_map.array() < retained_min_map.array()).any() ||
        (point_map.array() > retained_max_map.array()).any()) {
      out.status = LocalMapSupportStatus::OUTSIDE_ENVELOPE;
      return out;
    }
    // T_map_sensor is an isometry, so applying the transposed rotation avoids
    // rebuilding an inverse for every LOS support sample.
    const Eigen::Vector3d point_sensor =
        T_map_sensor.linear().transpose() *
        (point_map - T_map_sensor.translation());
    const double range = point_sensor.norm();
    if (!std::isfinite(range) || range < min_range_m || range > max_range_m) {
      out.status = LocalMapSupportStatus::OUTSIDE_ENVELOPE;
      return out;
    }
    const double horizontal = std::atan2(point_sensor.y(), point_sensor.x());
    const double elevation = std::atan2(
        point_sensor.z(), std::hypot(point_sensor.x(), point_sensor.y()));
    if (std::abs(horizontal) > 0.5 * horizontal_fov_rad + 1.0e-9 ||
        elevation < vertical_min_rad - 1.0e-9 ||
        elevation > vertical_max_rad + 1.0e-9) {
      out.status = LocalMapSupportStatus::OUTSIDE_ENVELOPE;
      return out;
    }
    out.status = LocalMapSupportStatus::MODEL_COMPLETE;
    return out;
  }

  std::string identity() const {
    if (!valid()) return {};
    std::ostringstream stream;
    stream << model_version << '|' << frame_id << '|' << std::setprecision(17)
           << stamp_s << '|' << valid_until_s << '|' << min_range_m << '|'
           << max_range_m << '|' << horizontal_fov_rad << '|'
           << vertical_min_rad << '|' << vertical_max_rad << '|';
    for (int row = 0; row < 4; ++row)
      for (int column = 0; column < 4; ++column)
        stream << std::hexfloat << T_map_sensor.matrix()(row, column) << '|';
    for (int axis = 0; axis < 3; ++axis)
      stream << std::hexfloat << retained_min_map(axis) << '|'
             << retained_max_map(axis) << '|';
    return stream.str();
  }
};

inline const char* localMapSupportAuthorityName(
    const LocalMapSupportAuthority authority) {
  return authority == LocalMapSupportAuthority::TRUSTED_LOCAL_MAP
      ? "TRUSTED_LOCAL_MAP" : "STRICT_OBSERVATION";
}

inline const char* localMapSupportStatusName(
    const LocalMapSupportStatus status) {
  switch (status) {
    case LocalMapSupportStatus::MODEL_COMPLETE: return "MODEL_COMPLETE";
    case LocalMapSupportStatus::OBSERVATION_INCOMPLETE:
      return "OBSERVATION_INCOMPLETE";
    case LocalMapSupportStatus::OUTSIDE_ENVELOPE: return "OUTSIDE_ENVELOPE";
    case LocalMapSupportStatus::EXPIRED: return "EXPIRED";
    case LocalMapSupportStatus::FRAME_INVALID: return "FRAME_INVALID";
  }
  return "FRAME_INVALID";
}

}  // namespace iap
