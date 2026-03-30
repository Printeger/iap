#include <iap/odometry/ct_incremental_solver_skeleton.hpp>

#include <algorithm>
#include <stdexcept>

#include <gtsam/inference/Symbol.h>

namespace iap {

namespace {

template <typename T>
bool contains_value(const std::vector<T>& values, const T& value) {
  return std::find(values.begin(), values.end(), value) != values.end();
}

template <typename T>
void append_unique(std::vector<T>& values, const T& value) {
  if (!contains_value(values, value)) {
    values.push_back(value);
  }
}

double key_timestamp(
  gtsam::Key key,
  const std::vector<BSplineControlPointState>& control_states,
  const std::vector<CTSolveDomainSegment>& active_segments,
  double current_stamp) {
  const char c = gtsam::Symbol(key).chr();
  const std::size_t index = gtsam::Symbol(key).index();

  switch (c) {
    case 's':
      for (const auto& state : control_states) {
        if (state.index == index) {
          return state.stamp;
        }
      }
      break;
    case 'u':
    case 'c':
      for (const auto& segment : active_segments) {
        if (segment.auxiliary_index == index) {
          return segment.stamp;
        }
      }
      break;
    case 'j':
    case 'k':
    case 'g':
    case 'e':
    case 'r':
      return current_stamp;
  }

  return current_stamp;
}

void insert_bspline_value(gtsam::Values& dst, const gtsam::Values& src, gtsam::Key key) {
  const char c = gtsam::Symbol(key).chr();
  switch (c) {
    case 's':
      dst.insert(key, src.at<gtsam::Pose3>(key));
      return;
    case 'u':
    case 'g':
    case 'j':
    case 'k':
    case 'e':
      dst.insert(key, src.at<gtsam::Vector3>(key));
      return;
    case 'c':
      dst.insert(key, src.at<gtsam::Vector2>(key));
      return;
    case 'r':
      dst.insert(key, src.at<gtsam::Rot3>(key));
      return;
    default:
      throw std::runtime_error(std::string("unsupported CT incremental solver key '") + c + "'");
  }
}

std::vector<gtsam::Key> active_shared_keys(
  const BSplineFixedLagSharedState& shared_state,
  const gtsam::Values& values) {
  std::vector<gtsam::Key> keys;
  const std::array<gtsam::Key, 3> always_shared = {
    gtsam::symbol('j', 0),
    gtsam::symbol('k', 0),
    gtsam::symbol('g', 0),
  };

  for (const auto key : always_shared) {
    if (values.exists(key)) {
      keys.push_back(key);
    }
  }

  if (shared_state.gnss_anchor_initialized) {
    if (values.exists(bspline_ecef_origin_key())) {
      keys.push_back(bspline_ecef_origin_key());
    }
    if (values.exists(bspline_ecef_rot_key())) {
      keys.push_back(bspline_ecef_rot_key());
    }
  }

  return keys;
}

}  // namespace

void BSplineIncrementalSolverSkeleton::reset() {
  last_active_segment_ordinals_.clear();
  known_keys_.clear();
  last_delta_ = CTSolverLifecycleDelta();
}

CTSolverLifecycleDelta BSplineIncrementalSolverSkeleton::prepare_update(
  const BSplineSolveDomain& domain,
  const std::vector<BSplineControlPointState>& control_states,
  const BSplineFixedLagSharedState& shared_state,
  const gtsam::Values& authoritative_values,
  double current_stamp) {
  CTSolverLifecycleDelta delta;
  delta.active_segment_ordinals = domain.active_segment_ordinals();
  delta.active_control_keys = domain.active_control_keys();

  for (const auto& segment : domain.active_segments()) {
    append_unique(delta.active_auxiliary_keys, bspline_velocity_key(segment.auxiliary_index));
    if (authoritative_values.exists(bspline_clock_key(segment.auxiliary_index))) {
      append_unique(delta.active_auxiliary_keys, bspline_clock_key(segment.auxiliary_index));
    }
  }

  delta.active_shared_keys = active_shared_keys(shared_state, authoritative_values);

  for (const auto segment_ordinal : delta.active_segment_ordinals) {
    if (!contains_value(last_active_segment_ordinals_, segment_ordinal)) {
      delta.newly_active_segment_ordinals.push_back(segment_ordinal);
    }
  }
  for (const auto segment_ordinal : last_active_segment_ordinals_) {
    if (!contains_value(delta.active_segment_ordinals, segment_ordinal)) {
      delta.retired_segment_ordinals.push_back(segment_ordinal);
    }
  }

  delta.active_keys = delta.active_control_keys;
  for (const auto key : delta.active_auxiliary_keys) {
    append_unique(delta.active_keys, key);
  }
  for (const auto key : delta.active_shared_keys) {
    append_unique(delta.active_keys, key);
  }

  for (const auto key : delta.active_keys) {
    if (!contains_value(known_keys_, key)) {
      delta.new_keys.push_back(key);
      if (authoritative_values.exists(key)) {
        insert_bspline_value(delta.new_values, authoritative_values, key);
      }
      delta.new_stamps[key] = key_timestamp(key, control_states, domain.active_segments(), current_stamp);
      continue;
    }

    const char c = gtsam::Symbol(key).chr();
    if (c == 'j' || c == 'k' || c == 'g' || c == 'e' || c == 'r') {
      delta.new_stamps[key] = current_stamp;
    }
  }

  for (const auto key : known_keys_) {
    if (!contains_value(delta.active_keys, key)) {
      delta.retired_keys.push_back(key);
    }
  }

  known_keys_ = delta.active_keys;
  last_active_segment_ordinals_ = delta.active_segment_ordinals;
  last_delta_ = delta;
  return last_delta_;
}

}  // namespace iap
