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
#include <gtsam/nonlinear/Values.h>
#include <utility>
#include <vector>

namespace iap {

struct BSplineFixedLagSegmentState {
  double stamp = 0.0;
  double scan_end = 0.0;
  std::array<std::size_t, kBSplineControlPointCount> control_indices{};
  std::size_t auxiliary_index = 0;
};

template <typename SegmentT = BSplineFixedLagSegmentState>
class BSplineFixedLagStateRegistryT {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void clear() {
    control_buffer_.clear();
    segments_.clear();
  }

  void reset_from_window(const BSplineControlWindow& window) {
    control_buffer_.reset_from_window(window);
    segments_.clear();
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

  std::vector<BSplineMarginalizationSegmentState> marginalization_segment_states() const {
    std::vector<BSplineMarginalizationSegmentState> states;
    states.reserve(segments_.size());

    for (const auto& segment : segments_) {
      BSplineMarginalizationSegmentState state;
      state.scan_end = segment.scan_end;
      state.control_indices = segment.control_indices;
      state.auxiliary_index = segment.auxiliary_index;
      states.push_back(state);
    }

    return states;
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
};

using BSplineFixedLagStateRegistry = BSplineFixedLagStateRegistryT<BSplineFixedLagSegmentState>;

}  // namespace iap
