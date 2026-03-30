#pragma once
// IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410:
// Local continuous-time solve-domain abstraction used to move BSpline odometry
// away from full active-window LiDAR graph rebuilds.

#include <iap/odometry/bspline_control_window.hpp>
#include <iap/odometry/bspline_fixed_lag_registry.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <vector>

namespace iap {

struct CTSolveDomainSegment {
  std::size_t ordinal = 0;
  double stamp = 0.0;
  double scan_end = 0.0;
  std::array<std::size_t, kBSplineControlPointCount> control_indices{};
  std::size_t auxiliary_index = 0;
};

class ICTSolveDomain {
 public:
  virtual ~ICTSolveDomain() = default;

  virtual bool empty() const = 0;
  virtual double start_time() const = 0;
  virtual double end_time() const = 0;
  virtual std::vector<gtsam::Key> active_control_keys() const = 0;
  virtual std::vector<gtsam::Key> retired_control_keys() const = 0;
  virtual std::vector<std::size_t> active_segment_ordinals() const = 0;
  virtual std::vector<std::size_t> retired_segment_ordinals() const = 0;
  virtual bool supports_time(double stamp) const = 0;
  virtual const std::vector<CTSolveDomainSegment>& active_segments() const = 0;
};

class BSplineSolveDomain final : public ICTSolveDomain {
 public:
  BSplineSolveDomain() = default;

  template <typename SegmentT>
  static BSplineSolveDomain from_segments(const std::vector<SegmentT>& segments, std::size_t recent_overlap_segments) {
    BSplineSolveDomain domain;
    if (segments.empty()) {
      return domain;
    }

    const std::size_t keep_count = std::min(segments.size(), std::max<std::size_t>(1, recent_overlap_segments + 1));
    const std::size_t first_active = segments.size() - keep_count;

    domain.start_time_ = segments[first_active].stamp;
    domain.end_time_ = segments.back().scan_end;

    for (std::size_t i = 0; i < first_active; ++i) {
      domain.retired_segment_ordinals_.push_back(i);
      for (const auto control_index : segments[i].control_indices) {
        const gtsam::Key key = bspline_control_point_key(control_index);
        if (std::find(domain.retired_control_keys_.begin(), domain.retired_control_keys_.end(), key) ==
            domain.retired_control_keys_.end()) {
          domain.retired_control_keys_.push_back(key);
        }
      }
    }

    for (std::size_t i = first_active; i < segments.size(); ++i) {
      domain.active_segment_ordinals_.push_back(i);
      CTSolveDomainSegment segment;
      segment.ordinal = i;
      segment.stamp = segments[i].stamp;
      segment.scan_end = segments[i].scan_end;
      segment.control_indices = segments[i].control_indices;
      segment.auxiliary_index = segments[i].auxiliary_index;
      domain.active_segments_.push_back(segment);

      for (const auto control_index : segments[i].control_indices) {
        const gtsam::Key key = bspline_control_point_key(control_index);
        if (std::find(domain.active_control_keys_.begin(), domain.active_control_keys_.end(), key) ==
            domain.active_control_keys_.end()) {
          domain.active_control_keys_.push_back(key);
        }
      }
    }

    domain.retired_control_keys_.erase(
      std::remove_if(
        domain.retired_control_keys_.begin(),
        domain.retired_control_keys_.end(),
        [&](const gtsam::Key key) {
          return std::find(domain.active_control_keys_.begin(), domain.active_control_keys_.end(), key) !=
                 domain.active_control_keys_.end();
        }),
      domain.retired_control_keys_.end());

    return domain;
  }

  bool empty() const override { return active_segments_.empty(); }
  double start_time() const override { return start_time_; }
  double end_time() const override { return end_time_; }
  std::vector<gtsam::Key> active_control_keys() const override { return active_control_keys_; }
  std::vector<gtsam::Key> retired_control_keys() const override { return retired_control_keys_; }
  std::vector<std::size_t> active_segment_ordinals() const override { return active_segment_ordinals_; }
  std::vector<std::size_t> retired_segment_ordinals() const override { return retired_segment_ordinals_; }
  bool supports_time(double stamp) const override { return !empty() && stamp >= start_time_ && stamp <= end_time_; }
  const std::vector<CTSolveDomainSegment>& active_segments() const override { return active_segments_; }

 private:
  double start_time_ = 0.0;
  double end_time_ = 0.0;
  std::vector<gtsam::Key> active_control_keys_;
  std::vector<gtsam::Key> retired_control_keys_;
  std::vector<std::size_t> active_segment_ordinals_;
  std::vector<std::size_t> retired_segment_ordinals_;
  std::vector<CTSolveDomainSegment> active_segments_;
};

}  // namespace iap

