#pragma once
// IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410:
// Unified fixed-lag lifecycle/state registry for the continuous-time B-spline
// odometry path. Owns the active control-buffer timeline together with the
// segment-level auxiliary state lifecycle used by LiDAR/IMU/GNSS factors.

#include <iap/odometry/bspline_control_window.hpp>
#include <iap/odometry/bspline_marginalization.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <utility>
#include <vector>

namespace iap {

struct BSplineFixedLagSegmentState {
  double stamp = 0.0;
  double scan_end = 0.0;
  int span_begin_idx = -1;
  int span_end_idx = -1;
  std::vector<std::size_t> active_control_indices;
  std::array<std::size_t, kBSplineControlPointCount> control_indices{};
  std::size_t auxiliary_index = 0;
};

struct BSplineFixedLagSharedState {
  gtsam::Vector3 gyro_bias = gtsam::Vector3::Zero();
  gtsam::Vector3 accel_bias = gtsam::Vector3::Zero();
  gtsam::Vector3 gravity = gtsam::Vector3(0.0, 0.0, 9.80665);
  bool gnss_anchor_initialized = false;
  gtsam::Vector3 ecef_origin = gtsam::Vector3::Zero();
  gtsam::Rot3 ecef_rot = gtsam::Rot3::Identity();
};

enum class BSplineFixedLagLifecycleState {
  Empty,
  WindowSeeded,
  TrackingLidar,
  TrackingLidarGnss,
};

inline const char* to_string(BSplineFixedLagLifecycleState state) {
  switch (state) {
    case BSplineFixedLagLifecycleState::Empty:
      return "empty";
    case BSplineFixedLagLifecycleState::WindowSeeded:
      return "window_seeded";
    case BSplineFixedLagLifecycleState::TrackingLidar:
      return "tracking_lidar";
    case BSplineFixedLagLifecycleState::TrackingLidarGnss:
      return "tracking_lidar_gnss";
  }
  return "unknown";
}

struct BSplineFixedLagTelemetry {
  double lag_start_stamp = 0.0;
  double lag_end_stamp = 0.0;
  double latest_segment_stamp = 0.0;
  double latest_segment_end = 0.0;
  std::size_t control_point_count = 0;
  std::size_t segment_count = 0;
  std::size_t active_auxiliary_count = 0;
  std::size_t auxiliary_value_count = 0;
  std::size_t active_shared_state_count = 0;
  std::size_t newest_control_index = 0;
  std::size_t newest_auxiliary_index = 0;
  bool has_active_segment = false;
  bool gnss_anchor_initialized = false;
  BSplineFixedLagLifecycleState lifecycle_state = BSplineFixedLagLifecycleState::Empty;
};

struct BSplineFixedLagControlReference {
  std::size_t control_index = 0;
  std::size_t reference_count = 0;
  std::vector<int> active_span_indices;
  std::vector<std::size_t> auxiliary_indices;
};

template <typename SegmentT = BSplineFixedLagSegmentState>
class BSplineFixedLagStateRegistryT {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void clear() {
    control_buffer_.clear();
    segments_.clear();
    auxiliary_values_.clear();
    announced_control_indices_.clear();
    announced_auxiliary_indices_.clear();
    announced_persistent_keys_.clear();
  }

  void reset_from_window(const BSplineControlWindow& window) {
    control_buffer_.reset_from_window(window);
    segments_.clear();
    announced_control_indices_.clear();
    announced_auxiliary_indices_.clear();
    announced_persistent_keys_.clear();
  }

  void append_window(const BSplineControlWindow& window) {
    control_buffer_.append_window(window);
  }

  SegmentT& append_segment(const SegmentT& segment) {
    segments_.push_back(segment);
    sort_segments();
    return *std::find_if(segments_.begin(), segments_.end(), [&](const auto& candidate) {
      return candidate.stamp == segment.stamp && candidate.auxiliary_index == segment.auxiliary_index;
    });
  }

  SegmentT& append_segment(SegmentT&& segment) {
    const double stamp = segment.stamp;
    const std::size_t auxiliary_index = segment.auxiliary_index;
    segments_.push_back(std::move(segment));
    sort_segments();
    return *std::find_if(segments_.begin(), segments_.end(), [&](const auto& candidate) {
      return candidate.stamp == stamp && candidate.auxiliary_index == auxiliary_index;
    });
  }

  void prune_before(double min_stamp) {
    control_buffer_.prune_before(min_stamp);

    const auto keep_begin = std::find_if(segments_.begin(), segments_.end(), [&](const auto& segment) {
      return segment.scan_end >= min_stamp;
    });

    if (keep_begin != segments_.begin()) {
      segments_.erase(segments_.begin(), keep_begin);
    }
  }

  std::vector<std::size_t> active_auxiliary_indices() const {
    std::vector<std::size_t> indices;
    indices.reserve(segments_.size());

    for (const auto& segment : segments_) {
      if (std::find(indices.begin(), indices.end(), segment.auxiliary_index) == indices.end()) {
        indices.push_back(segment.auxiliary_index);
      }
    }

    return indices;
  }

  bool contains_auxiliary_index(std::size_t index) const {
    return std::any_of(segments_.begin(), segments_.end(), [&](const auto& segment) {
      return segment.auxiliary_index == index;
    });
  }

  std::vector<int> active_span_indices(double min_active_stamp) const {
    std::vector<int> indices;
    for (const auto& segment : segments_) {
      if (segment.scan_end < min_active_stamp) {
        continue;
      }
      for (int span = segment.span_begin_idx; span >= 0 && span <= segment.span_end_idx; ++span) {
        if (std::find(indices.begin(), indices.end(), span) == indices.end()) {
          indices.push_back(span);
        }
      }
    }
    return indices;
  }

  std::vector<BSplineFixedLagControlReference> active_control_references(double min_active_stamp) const {
    std::vector<BSplineFixedLagControlReference> references;

    for (const auto& segment : segments_) {
      if (segment.scan_end < min_active_stamp) {
        continue;
      }

      const auto referenced_controls = [&]() {
        if (!segment.active_control_indices.empty()) {
          return segment.active_control_indices;
        }
        return std::vector<std::size_t>(segment.control_indices.begin(), segment.control_indices.end());
      }();

      for (const auto control_index : referenced_controls) {
        auto it = std::find_if(references.begin(), references.end(), [&](const auto& reference) {
          return reference.control_index == control_index;
        });
        if (it == references.end()) {
          references.push_back(BSplineFixedLagControlReference{control_index});
          it = std::prev(references.end());
        }

        it->reference_count++;
        if (std::find(it->auxiliary_indices.begin(), it->auxiliary_indices.end(), segment.auxiliary_index) == it->auxiliary_indices.end()) {
          it->auxiliary_indices.push_back(segment.auxiliary_index);
        }
        for (int span = segment.span_begin_idx; span >= 0 && span <= segment.span_end_idx; ++span) {
          if (std::find(it->active_span_indices.begin(), it->active_span_indices.end(), span) == it->active_span_indices.end()) {
            it->active_span_indices.push_back(span);
          }
        }
      }
    }

    return references;
  }

  SplineActiveStateSet active_state_set(
    const gtsam::Values& values,
    double min_active_stamp,
    bool include_clock) const {
    return build_spline_active_state_set(
      control_buffer_.states(),
      marginalization_segment_states(),
      values,
      min_active_stamp,
      include_clock);
  }

  std::vector<BSplineMarginalizationSegmentState> marginalization_segment_states() const {
    std::vector<BSplineMarginalizationSegmentState> states;
    states.reserve(segments_.size());

    for (const auto& segment : segments_) {
      BSplineMarginalizationSegmentState state;
      state.scan_end = segment.scan_end;
      state.span_begin_idx = segment.span_begin_idx;
      state.span_end_idx = segment.span_end_idx;
      state.active_control_indices = segment.active_control_indices;
      state.control_indices = segment.control_indices;
      state.auxiliary_index = segment.auxiliary_index;
      states.push_back(state);
    }

    return states;
  }

  void prune_to_active_state_set(
    double min_stamp,
    const SplineActiveStateSet& active_state_set,
    bool include_clock = true) {
    if (active_state_set.active_control_indices.empty()) {
      control_buffer_.prune_before(min_stamp);
    } else {
      control_buffer_.retain_control_indices(active_state_set.active_control_indices);
    }

    const auto keep_begin = std::find_if(segments_.begin(), segments_.end(), [&](const auto& segment) {
      return segment.scan_end >= min_stamp;
    });

    if (keep_begin != segments_.begin()) {
      segments_.erase(segments_.begin(), keep_begin);
    }

    retain_active_auxiliary_values(active_state_set, include_clock);
  }

  gtsam::Values filter_aux_values(const gtsam::Values& values, bool include_clock = true) const {
    gtsam::Values filtered;

    for (const auto& segment : segments_) {
      const auto velocity_key = bspline_velocity_key(segment.auxiliary_index);
      if (values.exists(velocity_key) && !filtered.exists(velocity_key)) {
        filtered.insert(velocity_key, values.at<gtsam::Vector3>(velocity_key));
      }

      const auto clock_key = bspline_clock_key(segment.auxiliary_index);
      if (include_clock && values.exists(clock_key) && !filtered.exists(clock_key)) {
        filtered.insert(clock_key, values.at<gtsam::Vector2>(clock_key));
      }
    }

    return filtered;
  }

  gtsam::Values filter_aux_values(
    const gtsam::Values& values,
    const SplineActiveStateSet& active_state_set,
    bool include_clock = true) const {
    gtsam::Values filtered;

    for (const auto aux_index : active_state_set.active_auxiliary_indices) {
      const auto velocity_key = bspline_velocity_key(aux_index);
      if (values.exists(velocity_key) && !filtered.exists(velocity_key)) {
        filtered.insert(velocity_key, values.at<gtsam::Vector3>(velocity_key));
      }

      const auto clock_key = bspline_clock_key(aux_index);
      if (include_clock && values.exists(clock_key) && !filtered.exists(clock_key)) {
        filtered.insert(clock_key, values.at<gtsam::Vector2>(clock_key));
      }
    }

    return filtered;
  }

  void retain_active_auxiliary_values(bool include_clock = true) {
    auxiliary_values_ = filter_aux_values(auxiliary_values_, include_clock);
  }

  void retain_active_auxiliary_values(
    const SplineActiveStateSet& active_state_set,
    bool include_clock = true) {
    auxiliary_values_ = filter_aux_values(auxiliary_values_, active_state_set, include_clock);
  }

  void clear_auxiliary_values() {
    auxiliary_values_.clear();
  }

  bool control_index_announced(std::size_t index) const {
    return std::find(announced_control_indices_.begin(), announced_control_indices_.end(), index) !=
           announced_control_indices_.end();
  }

  bool auxiliary_index_announced(std::size_t index) const {
    return std::find(announced_auxiliary_indices_.begin(), announced_auxiliary_indices_.end(), index) !=
           announced_auxiliary_indices_.end();
  }

  bool persistent_key_announced(gtsam::Key key) const {
    return std::find(announced_persistent_keys_.begin(), announced_persistent_keys_.end(), key) !=
           announced_persistent_keys_.end();
  }

  void mark_control_index_announced(std::size_t index) {
    if (!control_index_announced(index)) {
      announced_control_indices_.push_back(index);
    }
  }

  void mark_auxiliary_index_announced(std::size_t index) {
    if (!auxiliary_index_announced(index)) {
      announced_auxiliary_indices_.push_back(index);
    }
  }

  void mark_persistent_key_announced(gtsam::Key key) {
    if (!persistent_key_announced(key)) {
      announced_persistent_keys_.push_back(key);
    }
  }

  void retire_announced_keys(const gtsam::KeyVector& keys) {
    auto retire_control = [&](std::size_t index) {
      announced_control_indices_.erase(
        std::remove(announced_control_indices_.begin(), announced_control_indices_.end(), index),
        announced_control_indices_.end());
    };
    auto retire_aux = [&](std::size_t index) {
      announced_auxiliary_indices_.erase(
        std::remove(announced_auxiliary_indices_.begin(), announced_auxiliary_indices_.end(), index),
        announced_auxiliary_indices_.end());
    };
    auto retire_persistent = [&](gtsam::Key key) {
      announced_persistent_keys_.erase(
        std::remove(announced_persistent_keys_.begin(), announced_persistent_keys_.end(), key),
        announced_persistent_keys_.end());
    };

    for (const auto key : keys) {
      const auto symbol = gtsam::Symbol(key);
      switch (symbol.chr()) {
        case 's':
          retire_control(symbol.index());
          break;
        case 'u':
        case 'c':
          retire_aux(symbol.index());
          break;
        default:
          retire_persistent(key);
          break;
      }
    }
  }

  const gtsam::Values& auxiliary_values() const { return auxiliary_values_; }
  gtsam::Values& auxiliary_values() { return auxiliary_values_; }

  void set_shared_imu_state(
    const gtsam::Vector3& gyro_bias,
    const gtsam::Vector3& accel_bias,
    const gtsam::Vector3& gravity) {
    shared_state_.gyro_bias = gyro_bias;
    shared_state_.accel_bias = accel_bias;
    shared_state_.gravity = gravity;
  }

  void set_shared_gnss_anchor(
    const gtsam::Vector3& ecef_origin,
    const gtsam::Rot3& ecef_rot,
    bool initialized = true) {
    shared_state_.ecef_origin = ecef_origin;
    shared_state_.ecef_rot = ecef_rot;
    shared_state_.gnss_anchor_initialized = initialized;
  }

  const BSplineFixedLagSharedState& shared_state() const { return shared_state_; }
  BSplineFixedLagSharedState& shared_state() { return shared_state_; }

  void seed_shared_values(gtsam::Values& values, bool include_gnss_anchor) const {
    if (!values.exists(gtsam::symbol('j', 0))) {
      values.insert(gtsam::symbol('j', 0), shared_state_.gyro_bias);
    }
    if (!values.exists(gtsam::symbol('k', 0))) {
      values.insert(gtsam::symbol('k', 0), shared_state_.accel_bias);
    }
    if (!values.exists(gtsam::symbol('g', 0))) {
      values.insert(gtsam::symbol('g', 0), shared_state_.gravity);
    }
    if (include_gnss_anchor && shared_state_.gnss_anchor_initialized) {
      if (!values.exists(bspline_ecef_origin_key())) {
        values.insert(bspline_ecef_origin_key(), shared_state_.ecef_origin);
      }
      if (!values.exists(bspline_ecef_rot_key())) {
        values.insert(bspline_ecef_rot_key(), shared_state_.ecef_rot);
      }
    }
  }

  void update_shared_state_from_values(const gtsam::Values& values) {
    if (values.exists(gtsam::symbol('j', 0))) {
      shared_state_.gyro_bias = values.at<gtsam::Vector3>(gtsam::symbol('j', 0));
    }
    if (values.exists(gtsam::symbol('k', 0))) {
      shared_state_.accel_bias = values.at<gtsam::Vector3>(gtsam::symbol('k', 0));
    }
    if (values.exists(gtsam::symbol('g', 0))) {
      shared_state_.gravity = values.at<gtsam::Vector3>(gtsam::symbol('g', 0));
    }
    if (values.exists(bspline_ecef_origin_key())) {
      shared_state_.ecef_origin = values.at<gtsam::Vector3>(bspline_ecef_origin_key());
      shared_state_.gnss_anchor_initialized = true;
    }
    if (values.exists(bspline_ecef_rot_key())) {
      shared_state_.ecef_rot = values.at<gtsam::Rot3>(bspline_ecef_rot_key());
      shared_state_.gnss_anchor_initialized = true;
    }
  }

  std::vector<gtsam::Key> active_shared_keys(bool include_gnss_anchor) const {
    std::vector<gtsam::Key> keys = {
      gtsam::symbol('j', 0),
      gtsam::symbol('k', 0),
      gtsam::symbol('g', 0),
    };
    if (include_gnss_anchor && shared_state_.gnss_anchor_initialized) {
      keys.push_back(bspline_ecef_origin_key());
      keys.push_back(bspline_ecef_rot_key());
    }
    return keys;
  }

  BSplineFixedLagTelemetry telemetry() const {
    BSplineFixedLagTelemetry telemetry;
    telemetry.control_point_count = control_buffer_.size();
    telemetry.segment_count = segments_.size();
    telemetry.active_auxiliary_count = active_auxiliary_indices().size();
    telemetry.auxiliary_value_count = auxiliary_values_.size();
    telemetry.gnss_anchor_initialized = shared_state_.gnss_anchor_initialized;
    telemetry.active_shared_state_count =
      active_shared_keys(shared_state_.gnss_anchor_initialized).size();

    if (control_buffer_.empty()) {
      telemetry.lifecycle_state = BSplineFixedLagLifecycleState::Empty;
      return telemetry;
    }

    telemetry.lag_start_stamp = control_buffer_.states().front().stamp;
    telemetry.lag_end_stamp = control_buffer_.states().back().stamp;
    telemetry.newest_control_index = control_buffer_.states().back().index;

    if (segments_.empty()) {
      telemetry.lifecycle_state = BSplineFixedLagLifecycleState::WindowSeeded;
      return telemetry;
    }

    const auto& latest_segment = segments_.back();
    telemetry.has_active_segment = true;
    telemetry.latest_segment_stamp = latest_segment.stamp;
    telemetry.latest_segment_end = latest_segment.scan_end;
    telemetry.newest_auxiliary_index = latest_segment.auxiliary_index;
    telemetry.lifecycle_state = shared_state_.gnss_anchor_initialized
      ? BSplineFixedLagLifecycleState::TrackingLidarGnss
      : BSplineFixedLagLifecycleState::TrackingLidar;
    return telemetry;
  }

  BSplineControlWindowBuffer& control_buffer() { return control_buffer_; }
  const BSplineControlWindowBuffer& control_buffer() const { return control_buffer_; }

  std::vector<SegmentT>& segments() { return segments_; }
  const std::vector<SegmentT>& segments() const { return segments_; }

 private:
  void sort_segments() {
    std::sort(segments_.begin(), segments_.end(), [](const auto& lhs, const auto& rhs) {
      if (lhs.stamp == rhs.stamp) {
        return lhs.auxiliary_index < rhs.auxiliary_index;
      }
      return lhs.stamp < rhs.stamp;
    });
  }

  BSplineControlWindowBuffer control_buffer_;
  std::vector<SegmentT> segments_;
  gtsam::Values auxiliary_values_;
  BSplineFixedLagSharedState shared_state_;
  std::vector<std::size_t> announced_control_indices_;
  std::vector<std::size_t> announced_auxiliary_indices_;
  gtsam::KeyVector announced_persistent_keys_;
};

using BSplineFixedLagStateRegistry = BSplineFixedLagStateRegistryT<BSplineFixedLagSegmentState>;

}  // namespace iap
