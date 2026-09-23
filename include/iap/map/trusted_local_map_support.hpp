#pragma once

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

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
  double observation_stamp_s = std::numeric_limits<double>::quiet_NaN();
  double observation_age_s = std::numeric_limits<double>::quiet_NaN();

  bool complete() const {
    return status == LocalMapSupportStatus::MODEL_COMPLETE;
  }
};

// One immutable sensor envelope contributing recent model-support coverage.
// It records no ray-derived free-space claim; OBSERVED_FREE remains owned by
// the occupancy layer.
struct TrustedLocalMapSupportObservation {
  Eigen::Isometry3d T_map_sensor = Eigen::Isometry3d::Identity();
  Eigen::Vector3d retained_min_map = Eigen::Vector3d::Constant(
      -std::numeric_limits<double>::infinity());
  Eigen::Vector3d retained_max_map = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  double stamp_s = std::numeric_limits<double>::quiet_NaN();
  double valid_until_s = std::numeric_limits<double>::quiet_NaN();
  double sensor_receipt_steady_s =
      std::numeric_limits<double>::quiet_NaN();

  bool valid() const {
    return T_map_sensor.matrix().allFinite() && retained_min_map.allFinite() &&
        retained_max_map.allFinite() &&
        (retained_max_map.array() > retained_min_map.array()).all() &&
        std::isfinite(stamp_s) && std::isfinite(valid_until_s) &&
        valid_until_s >= stamp_s;
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
  double sensor_receipt_steady_s =
      std::numeric_limits<double>::quiet_NaN();
  std::string frame_id;
  std::string model_version = "trusted_local_map_v1";
  // Older current-frame envelopes retained only inside the same hard
  // freshness window. The fields above remain the newest observation and
  // preserve the single-envelope wire/test contract.
  std::vector<TrustedLocalMapSupportObservation> observations;

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
        !model_version.empty() &&
        std::all_of(observations.begin(), observations.end(),
                    [](const auto& observation) {
                      return observation.valid();
                    });
  }

  bool freshAt(const double evaluation_time_s) const {
    if (!valid() || !std::isfinite(evaluation_time_s)) return false;
    if (evaluation_time_s >= stamp_s && evaluation_time_s <= valid_until_s)
      return true;
    return std::any_of(observations.begin(), observations.end(),
                       [evaluation_time_s](const auto& observation) {
                         return evaluation_time_s >= observation.stamp_s &&
                             evaluation_time_s <= observation.valid_until_s;
                       });
  }

  LocalMapSupportQuery query(const Eigen::Vector3d& point_map,
                             const double evaluation_time_s) const {
    LocalMapSupportQuery out;
    out.authority = LocalMapSupportAuthority::TRUSTED_LOCAL_MAP;
    if (!valid() || !point_map.allFinite() ||
        !std::isfinite(evaluation_time_s)) {
      out.status = LocalMapSupportStatus::FRAME_INVALID;
      return out;
    }
    const auto spatially_covers = [this, &point_map](
        const Eigen::Isometry3d& pose, const Eigen::Vector3d& retained_min,
        const Eigen::Vector3d& retained_max) {
      if ((point_map.array() < retained_min.array()).any() ||
          (point_map.array() > retained_max.array()).any())
        return false;
      // T_map_sensor is an isometry, so applying the transposed rotation
      // avoids rebuilding an inverse for every support sample.
      const Eigen::Vector3d point_sensor = pose.linear().transpose() *
          (point_map - pose.translation());
      const double range = point_sensor.norm();
      if (!std::isfinite(range) || range < min_range_m || range > max_range_m)
        return false;
      const double horizontal =
          std::atan2(point_sensor.y(), point_sensor.x());
      const double elevation = std::atan2(
          point_sensor.z(), std::hypot(point_sensor.x(), point_sensor.y()));
      return std::abs(horizontal) <= 0.5 * horizontal_fov_rad + 1.0e-9 &&
          elevation >= vertical_min_rad - 1.0e-9 &&
          elevation <= vertical_max_rad + 1.0e-9;
    };
    bool covered_by_any_observation = false;
    double newest_covering_stamp = -std::numeric_limits<double>::infinity();
    const auto consider = [&](const Eigen::Isometry3d& pose,
                              const Eigen::Vector3d& retained_min,
                              const Eigen::Vector3d& retained_max,
                              const double observation_stamp,
                              const double valid_until) {
      if (!spatially_covers(pose, retained_min, retained_max)) return false;
      covered_by_any_observation = true;
      if (evaluation_time_s >= observation_stamp &&
          evaluation_time_s <= valid_until &&
          observation_stamp > newest_covering_stamp) {
        newest_covering_stamp = observation_stamp;
        return true;
      }
      return false;
    };

    // Production histories are appended in source-stamp order and the
    // primary envelope is the newest frame. Preserve exact generic behavior
    // for hand-built/legacy unordered inputs, but exploit that invariant in
    // the hot LOS support path: the first covering entry in newest-to-oldest
    // order is necessarily the answer.
    const bool ordered_history =
        std::is_sorted(observations.begin(), observations.end(),
                       [](const auto& lhs, const auto& rhs) {
                         return lhs.stamp_s < rhs.stamp_s;
                       }) &&
        (observations.empty() || observations.back().stamp_s <= stamp_s);
    if (ordered_history) {
      if (consider(T_map_sensor, retained_min_map, retained_max_map, stamp_s,
                   valid_until_s)) {
        out.status = LocalMapSupportStatus::MODEL_COMPLETE;
        out.observation_stamp_s = newest_covering_stamp;
        out.observation_age_s = evaluation_time_s - newest_covering_stamp;
        return out;
      }
      for (auto observation = observations.rbegin();
           observation != observations.rend(); ++observation) {
        if (consider(observation->T_map_sensor,
                     observation->retained_min_map,
                     observation->retained_max_map, observation->stamp_s,
                     observation->valid_until_s)) {
          out.status = LocalMapSupportStatus::MODEL_COMPLETE;
          out.observation_stamp_s = newest_covering_stamp;
          out.observation_age_s = evaluation_time_s - newest_covering_stamp;
          return out;
        }
      }
    } else {
      consider(T_map_sensor, retained_min_map, retained_max_map, stamp_s,
               valid_until_s);
      for (const auto& observation : observations)
        consider(observation.T_map_sensor, observation.retained_min_map,
                 observation.retained_max_map, observation.stamp_s,
                 observation.valid_until_s);
    }
    if (std::isfinite(newest_covering_stamp)) {
      out.status = LocalMapSupportStatus::MODEL_COMPLETE;
      out.observation_stamp_s = newest_covering_stamp;
      out.observation_age_s = evaluation_time_s - newest_covering_stamp;
    } else {
      out.status = covered_by_any_observation
          ? LocalMapSupportStatus::EXPIRED
          : LocalMapSupportStatus::OUTSIDE_ENVELOPE;
    }
    return out;
  }

  // Candidate arrival time is deliberately not part of local-map freshness.
  // It remains explicit at the call boundary so a future prediction cannot
  // accidentally be substituted for this round's evaluation clock again.
  LocalMapSupportQuery query(const Eigen::Vector3d& point_map,
                             const double evaluation_time_s,
                             const double query_time_s) const {
    if (!std::isfinite(query_time_s)) {
      LocalMapSupportQuery out;
      out.authority = LocalMapSupportAuthority::TRUSTED_LOCAL_MAP;
      out.status = LocalMapSupportStatus::FRAME_INVALID;
      return out;
    }
    return query(point_map, evaluation_time_s);
  }

  // Return the smallest deterministic correction into one of the sensor
  // envelopes that is valid at evaluation_time_s.  This is a geometry hint,
  // not new authority: callers must regenerate their curve and run query()
  // (plus collision/dynamics/risk gates) over the complete result.  In
  // particular, expired observations are never projected or refreshed.
  bool projectIntoFreshEnvelope(const Eigen::Vector3d& point_map,
                                const double evaluation_time_s,
                                const double interior_margin_m,
                                Eigen::Vector3d* projected_map) const {
    if (!projected_map || !valid() || !point_map.allFinite() ||
        !std::isfinite(evaluation_time_s) ||
        !std::isfinite(interior_margin_m) || interior_margin_m < 0.0) {
      return false;
    }
    if (query(point_map, evaluation_time_s).complete()) {
      *projected_map = point_map;
      return true;
    }

    Eigen::Vector3d best = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    double best_distance = std::numeric_limits<double>::infinity();
    const auto consider = [&](const Eigen::Isometry3d& pose,
                              const Eigen::Vector3d& retained_min,
                              const Eigen::Vector3d& retained_max,
                              const double observation_stamp,
                              const double valid_until) {
      if (evaluation_time_s < observation_stamp ||
          evaluation_time_s > valid_until) {
        return;
      }
      Eigen::Vector3d candidate = point_map.cwiseMax(
          retained_min + Eigen::Vector3d::Constant(interior_margin_m));
      candidate = candidate.cwiseMin(
          retained_max - Eigen::Vector3d::Constant(interior_margin_m));
      Eigen::Vector3d sensor = pose.linear().transpose() *
          (candidate - pose.translation());
      double planar = std::hypot(sensor.x(), sensor.y());
      double azimuth = std::atan2(sensor.y(), sensor.x());

      constexpr double kPi = 3.14159265358979323846;
      if (horizontal_fov_rad < 2.0 * kPi - 1.0e-9) {
        const double angular_margin = std::atan2(
            interior_margin_m, std::max(planar, 1.0e-9));
        const double half_width = 0.5 * horizontal_fov_rad;
        if (angular_margin >= half_width) return;
        azimuth = std::clamp(
            azimuth, -half_width + angular_margin,
            half_width - angular_margin);
        sensor.x() = planar * std::cos(azimuth);
        sensor.y() = planar * std::sin(azimuth);
      }

      planar = std::hypot(sensor.x(), sensor.y());
      const double lower = planar * std::tan(vertical_min_rad) +
          interior_margin_m;
      const double upper = planar * std::tan(vertical_max_rad) -
          interior_margin_m;
      if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper)
        return;
      sensor.z() = std::clamp(sensor.z(), lower, upper);

      double range = sensor.norm();
      const double minimum = min_range_m + interior_margin_m;
      const double maximum = max_range_m - interior_margin_m;
      if (!std::isfinite(range) || maximum <= minimum) return;
      if (range <= 1.0e-12) {
        sensor = Eigen::Vector3d(minimum, 0.0, 0.0);
      } else if (range < minimum || range > maximum) {
        sensor *= std::clamp(range, minimum, maximum) / range;
      }
      candidate = pose * sensor;
      if (!query(candidate, evaluation_time_s).complete()) return;
      const double distance = (candidate - point_map).squaredNorm();
      if (distance < best_distance) {
        best_distance = distance;
        best = candidate;
      }
    };

    consider(T_map_sensor, retained_min_map, retained_max_map, stamp_s,
             valid_until_s);
    for (const auto& observation : observations) {
      consider(observation.T_map_sensor, observation.retained_min_map,
               observation.retained_max_map, observation.stamp_s,
               observation.valid_until_s);
    }
    if (!best.allFinite()) return false;
    *projected_map = best;
    return true;
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
    stream << "observations=" << observations.size() << '|';
    for (const auto& observation : observations) {
      stream << std::defaultfloat << std::setprecision(17)
             << observation.stamp_s << '|' << observation.valid_until_s << '|';
      for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column)
          stream << std::hexfloat
                 << observation.T_map_sensor.matrix()(row, column) << '|';
      for (int axis = 0; axis < 3; ++axis)
        stream << std::hexfloat << observation.retained_min_map(axis) << '|'
               << observation.retained_max_map(axis) << '|';
    }
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
