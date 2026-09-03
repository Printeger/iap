#include <plan_env/registered_lidar_window.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <unordered_set>

namespace {

void rememberBefore(
    const int address, const RegisteredVoxelState state,
    std::unordered_map<int, RegisteredVoxelState>* before) {
  before->emplace(address, state);
}

}  // namespace

RegisteredLidarWindow::RegisteredLidarWindow(Geometry geometry)
    : geometry_(std::move(geometry)) {
  const auto count = validGeometry()
                         ? static_cast<std::size_t>(geometry_.dimensions.x()) *
                               static_cast<std::size_t>(geometry_.dimensions.y()) *
                               static_cast<std::size_t>(geometry_.dimensions.z())
                         : 0U;
  active_hit_count_.assign(count, 0U);
  active_free_count_.assign(count, 0U);
  current_hit_.assign(count, 0U);
  current_free_.assign(count, 0U);
}

bool RegisteredLidarWindow::validGeometry() const {
  return geometry_.origin.allFinite() &&
         (geometry_.dimensions.array() > 0).all() &&
         std::isfinite(geometry_.resolution_m) &&
         geometry_.resolution_m > 0.0 &&
         !geometry_.frame_contract_id.empty();
}

bool RegisteredLidarWindow::inBounds(const Eigen::Vector3i& index) const {
  return (index.array() >= 0).all() &&
         (index.array() < geometry_.dimensions.array()).all();
}

int RegisteredLidarWindow::address(const Eigen::Vector3i& index) const {
  return index.x() * geometry_.dimensions.y() * geometry_.dimensions.z() +
         index.y() * geometry_.dimensions.z() + index.z();
}

Eigen::Vector3i RegisteredLidarWindow::indexOf(
    const Eigen::Vector3d& point) const {
  return ((point - geometry_.origin) / geometry_.resolution_m)
      .array()
      .floor()
      .cast<int>();
}

Eigen::Vector3i RegisteredLidarWindow::indexFromAddress(
    const int linear_address) const {
  const int yz = geometry_.dimensions.y() * geometry_.dimensions.z();
  const int x = linear_address / yz;
  const int remainder = linear_address % yz;
  return Eigen::Vector3i(
      x, remainder / geometry_.dimensions.z(),
      remainder % geometry_.dimensions.z());
}

RegisteredLidarWindow::FrameContribution
RegisteredLidarWindow::buildContribution(
    const RegisteredLidarFrameData& frame) const {
  FrameContribution contribution;
  contribution.source = frame;
  const std::size_t word_count =
      (active_hit_count_.size() + 63U) / 64U;
  std::vector<std::uint64_t> hit_bits(word_count, 0U);
  std::vector<std::uint64_t> free_bits(word_count, 0U);
  std::vector<std::uint64_t> ray_end_bits(word_count, 0U);
  const auto mark = [](std::vector<std::uint64_t>* bits,
                       const int linear_address) {
    const auto address = static_cast<std::size_t>(linear_address);
    auto& word = (*bits)[address >> 6U];
    const std::uint64_t mask =
        std::uint64_t{1} << (address & 63U);
    const bool inserted = (word & mask) == 0U;
    word |= mask;
    return inserted;
  };
  const Eigen::Vector3d sensor = frame.T_map_lidar.translation();
  const Eigen::Vector3i start = indexOf(sensor);
  struct RayEnd {
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    bool occupied_endpoint = false;
  };
  std::vector<RayEnd> representative_rays;
  representative_rays.reserve(frame.hits_lidar.size());
  const Eigen::Vector3d map_max =
      geometry_.origin +
      geometry_.dimensions.cast<double>() * geometry_.resolution_m;
  const auto clip_to_map = [&](const Eigen::Vector3d& endpoint)
      -> std::optional<Eigen::Vector3d> {
    if (!inBounds(start)) {
      return std::nullopt;
    }
    const Eigen::Vector3d direction = endpoint - sensor;
    double exit_fraction = 1.0;
    for (int axis = 0; axis < 3; ++axis) {
      if (direction[axis] > 0.0) {
        exit_fraction = std::min(
            exit_fraction,
            (map_max[axis] - sensor[axis]) / direction[axis]);
      } else if (direction[axis] < 0.0) {
        exit_fraction = std::min(
            exit_fraction,
            (geometry_.origin[axis] - sensor[axis]) / direction[axis]);
      }
    }
    if (!std::isfinite(exit_fraction) || exit_fraction <= 0.0) {
      return std::nullopt;
    }
    const double inside_fraction = std::clamp(
        std::nextafter(exit_fraction, 0.0), 0.0, 1.0);
    return sensor + inside_fraction * direction;
  };

  for (const auto& hit_lidar : frame.hits_lidar) {
    if (!hit_lidar.allFinite()) {
      continue;
    }
    const Eigen::Vector3d hit = frame.T_map_lidar * hit_lidar;
    const Eigen::Vector3i finish = indexOf(hit);
    if (inBounds(finish)) {
      const int hit_address = address(finish);
      // Geometry uses every occupied endpoint voxel. For observed-free ray
      // evidence, one actual successful return per endpoint voxel is a
      // conservative 0.1 m voxel downsample: it never invents a no-return ray
      // and avoids retracing thousands of equivalent beams.
      if (mark(&hit_bits, hit_address)) {
        representative_rays.push_back({hit, true});
      }
      continue;
    }
    // A real return beyond the fixed planning geofence still proves the
    // in-map prefix free. Clip just inside the boundary and retain that
    // successful-return ray without inventing an occupied endpoint.
    const auto clipped = clip_to_map(hit);
    if (clipped) {
      const Eigen::Vector3i clipped_finish = indexOf(*clipped);
      if (inBounds(clipped_finish) &&
          mark(&ray_end_bits, address(clipped_finish))) {
        representative_rays.push_back({*clipped, false});
      }
    }
  }

  for (const auto& ray : representative_rays) {
    const Eigen::Vector3i finish = indexOf(ray.point);
    // Amanatides-Woo traversal in lattice coordinates. The successful return
    // voxel is excluded from free evidence and later wins over every ray.
    Eigen::Vector3i cell = start;
    if (!inBounds(cell) || cell == finish) {
      continue;
    }
    const Eigen::Vector3d start_grid =
        (sensor - geometry_.origin) / geometry_.resolution_m;
    const Eigen::Vector3d end_grid =
        (ray.point - geometry_.origin) / geometry_.resolution_m;
    const Eigen::Vector3d direction = end_grid - start_grid;
    Eigen::Vector3i step = Eigen::Vector3i::Zero();
    Eigen::Vector3d t_max = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
    Eigen::Vector3d t_delta = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
    for (int axis = 0; axis < 3; ++axis) {
      if (direction[axis] > 0.0) {
        step[axis] = 1;
        t_max[axis] =
            (static_cast<double>(cell[axis] + 1) - start_grid[axis]) /
            direction[axis];
        t_delta[axis] = 1.0 / direction[axis];
      } else if (direction[axis] < 0.0) {
        step[axis] = -1;
        t_max[axis] =
            (start_grid[axis] - static_cast<double>(cell[axis])) /
            -direction[axis];
        t_delta[axis] = 1.0 / -direction[axis];
      }
    }
    while (cell != finish) {
      if (inBounds(cell)) {
        mark(&free_bits, address(cell));
      }
      const double crossing = t_max.minCoeff();
      for (int axis = 0; axis < 3; ++axis) {
        if (t_max[axis] <= crossing + 1.0e-12) {
          cell[axis] += step[axis];
          t_max[axis] += t_delta[axis];
        }
      }
    }
    if (!ray.occupied_endpoint && inBounds(finish)) {
      mark(&free_bits, address(finish));
    }
  }

  contribution.hits.reserve(frame.hits_lidar.size());
  for (std::size_t word_index = 0; word_index < word_count; ++word_index) {
    free_bits[word_index] &= ~hit_bits[word_index];
    auto append_addresses = [word_index](
                                std::uint64_t word,
                                std::vector<int>* addresses) {
      while (word != 0U) {
        const unsigned bit = static_cast<unsigned>(__builtin_ctzll(word));
        addresses->push_back(static_cast<int>(word_index * 64U + bit));
        word &= word - 1U;
      }
    };
    append_addresses(hit_bits[word_index], &contribution.hits);
    append_addresses(free_bits[word_index], &contribution.observed_free);
  }
  return contribution;
}

RegisteredVoxelState RegisteredLidarWindow::stateAtAddress(
    const int linear_address) const {
  if (linear_address < 0 ||
      linear_address >= static_cast<int>(active_hit_count_.size())) {
    return RegisteredVoxelState::UNKNOWN;
  }
  const auto i = static_cast<std::size_t>(linear_address);
  if (active_hit_count_[i] != 0U || current_hit_[i] != 0U) {
    return RegisteredVoxelState::OCCUPIED;
  }
  if (active_free_count_[i] != 0U || current_free_[i] != 0U) {
    return RegisteredVoxelState::OBSERVED_FREE;
  }
  return RegisteredVoxelState::UNKNOWN;
}

RegisteredVoxelState RegisteredLidarWindow::stateAt(
    const Eigen::Vector3i& index) const {
  return inBounds(index) ? stateAtAddress(address(index))
                         : RegisteredVoxelState::UNKNOWN;
}

void RegisteredLidarWindow::removeActiveContribution(
    const FrameContribution& contribution) {
  for (const int address : contribution.hits) {
    auto& count = active_hit_count_[static_cast<std::size_t>(address)];
    if (count > 0U) {
      --count;
    }
  }
  for (const int address : contribution.observed_free) {
    auto& count = active_free_count_[static_cast<std::size_t>(address)];
    if (count > 0U) {
      --count;
    }
  }
}

void RegisteredLidarWindow::addActiveContribution(
    const FrameContribution& contribution) {
  for (const int address : contribution.hits) {
    auto& count = active_hit_count_[static_cast<std::size_t>(address)];
    if (count != std::numeric_limits<std::uint32_t>::max()) {
      ++count;
    }
  }
  for (const int address : contribution.observed_free) {
    auto& count = active_free_count_[static_cast<std::size_t>(address)];
    if (count != std::numeric_limits<std::uint32_t>::max()) {
      ++count;
    }
  }
}

void RegisteredLidarWindow::collectChanges(
    const std::unordered_map<int, RegisteredVoxelState>& before,
    RegisteredLidarWindowUpdate* update) const {
  for (const auto& [address, previous] : before) {
    const auto current = stateAtAddress(address);
    if (current != previous) {
      update->changes.push_back({indexFromAddress(address), current});
    }
  }
  std::sort(update->changes.begin(), update->changes.end(),
            [this](const auto& lhs, const auto& rhs) {
              return address(lhs.index) < address(rhs.index);
            });
}

RegisteredLidarWindowUpdate RegisteredLidarWindow::applyCurrentFrame(
    const RegisteredLidarFrameData& frame) {
  RegisteredLidarWindowUpdate update;
  update.active_generation = active_generation_;
  update.current_frame_id = current_frame_id_;
  if (!validGeometry() ||
      frame.frame_contract_id != geometry_.frame_contract_id ||
      frame.frame_id < 0 || !frame.T_map_lidar.matrix().allFinite() ||
      !std::isfinite(frame.stamp_s) ||
      !std::isfinite(frame.scan_end_stamp_s) ||
      frame.scan_end_stamp_s + 1.0e-9 < frame.stamp_s ||
      frame.sensor_receipt_steady_ns == 0U) {
    update.reason = frame.frame_contract_id != geometry_.frame_contract_id
                        ? "frame_contract_mismatch"
                        : "invalid_current_frame";
    return update;
  }

  const FrameContribution next = buildContribution(frame);

  // Contributions are sorted. Compare the old and new overlays with one
  // four-way merge instead of materializing a large unordered map. A MID-360
  // frame can traverse hundreds of thousands of free voxels, so hashing every
  // address here dominated the 10 Hz current-frame callback.
  const std::vector<int> empty;
  const auto& old_hits = has_current_frame_ ? current_frame_.hits : empty;
  const auto& old_free =
      has_current_frame_ ? current_frame_.observed_free : empty;
  const std::array<const std::vector<int>*, 4> lists = {
      &old_hits, &old_free, &next.hits, &next.observed_free};
  std::array<std::size_t, 4> cursor = {0U, 0U, 0U, 0U};
  while (true) {
    int next_address = std::numeric_limits<int>::max();
    for (std::size_t list = 0; list < lists.size(); ++list) {
      if (cursor[list] < lists[list]->size()) {
        next_address = std::min(
            next_address, (*lists[list])[cursor[list]]);
      }
    }
    if (next_address == std::numeric_limits<int>::max()) {
      break;
    }
    std::array<bool, 4> present = {false, false, false, false};
    for (std::size_t list = 0; list < lists.size(); ++list) {
      if (cursor[list] < lists[list]->size() &&
          (*lists[list])[cursor[list]] == next_address) {
        present[list] = true;
        ++cursor[list];
      }
    }
    const auto i = static_cast<std::size_t>(next_address);
    const auto state = [this, i](const bool overlay_hit,
                                 const bool overlay_free) {
      if (active_hit_count_[i] != 0U || overlay_hit) {
        return RegisteredVoxelState::OCCUPIED;
      }
      if (active_free_count_[i] != 0U || overlay_free) {
        return RegisteredVoxelState::OBSERVED_FREE;
      }
      return RegisteredVoxelState::UNKNOWN;
    };
    const auto before = state(present[0], present[1]);
    const auto after = state(present[2], present[3]);
    if (before != after) {
      update.changes.push_back({indexFromAddress(next_address), after});
    }
  }

  if (has_current_frame_) {
    for (const int address : current_frame_.hits) {
      current_hit_[static_cast<std::size_t>(address)] = 0U;
    }
    for (const int address : current_frame_.observed_free) {
      current_free_[static_cast<std::size_t>(address)] = 0U;
    }
  }
  for (const int address : next.hits) {
    current_hit_[static_cast<std::size_t>(address)] = 1U;
  }
  for (const int address : next.observed_free) {
    current_free_[static_cast<std::size_t>(address)] = 1U;
  }
  current_frame_ = next;
  has_current_frame_ = true;
  current_frame_id_ = frame.frame_id;
  current_stamp_s_ = frame.stamp_s;
  update.accepted = true;
  update.reason = "ok";
  update.current_frame_id = current_frame_id_;
  update.stamp_s = current_stamp_s_;
  return update;
}

RegisteredLidarWindowUpdate RegisteredLidarWindow::applyActiveDelta(
    const ActiveLidarWindowDeltaData& delta) {
  RegisteredLidarWindowUpdate update;
  update.active_generation = active_generation_;
  update.current_frame_id = current_frame_id_;
  if (delta.frame_contract_id != geometry_.frame_contract_id) {
    update.recovery_required = true;
    update.reason = "frame_contract_mismatch";
    return update;
  }
  if (!delta.complete) {
    update.recovery_required = true;
    update.reason = "incomplete_delta";
    return update;
  }
  if (delta.base_generation != active_generation_ ||
      delta.generation != delta.base_generation + 1U) {
    update.recovery_required = true;
    update.reason = "generation_gap";
    return update;
  }

  std::unordered_set<std::int64_t> removed_ids;
  for (const auto id : delta.removed_frame_ids) {
    if (!removed_ids.insert(id).second || active_frames_.count(id) == 0U) {
      update.recovery_required = true;
      update.reason = "missing_removed_frame";
      return update;
    }
  }
  std::unordered_set<std::int64_t> updated_ids;
  for (const auto& [id, pose] : delta.pose_updates) {
    if (!updated_ids.insert(id).second || removed_ids.count(id) != 0U ||
        active_frames_.count(id) == 0U || !pose.matrix().allFinite()) {
      update.recovery_required = true;
      update.reason = "missing_pose_update_frame";
      return update;
    }
  }
  std::unordered_set<std::int64_t> added_ids;
  for (const auto& source : delta.added) {
    if (source.frame_contract_id != geometry_.frame_contract_id ||
        source.frame_id < 0 || active_frames_.count(source.frame_id) != 0U ||
        !added_ids.insert(source.frame_id).second ||
        !source.T_map_lidar.matrix().allFinite() ||
        !std::isfinite(source.stamp_s) ||
        !std::isfinite(source.scan_end_stamp_s) ||
        source.scan_end_stamp_s + 1.0e-9 < source.stamp_s ||
        source.sensor_receipt_steady_ns == 0U) {
      update.recovery_required = true;
      update.reason = "invalid_added_frame";
      return update;
    }
  }

  std::unordered_map<int, RegisteredVoxelState> before;
  auto rememberContribution = [this, &before](const FrameContribution& value) {
    for (const int address : value.hits) {
      rememberBefore(address, stateAtAddress(address), &before);
    }
    for (const int address : value.observed_free) {
      rememberBefore(address, stateAtAddress(address), &before);
    }
  };

  for (const auto id : delta.removed_frame_ids) {
    const auto found = active_frames_.find(id);
    rememberContribution(found->second);
    removeActiveContribution(found->second);
    active_frames_.erase(found);
  }
  for (const auto& [id, pose] : delta.pose_updates) {
    const auto found = active_frames_.find(id);
    rememberContribution(found->second);
    removeActiveContribution(found->second);
    auto source = found->second.source;
    source.T_map_lidar = pose;
    auto rebuilt = buildContribution(source);
    rememberContribution(rebuilt);
    addActiveContribution(rebuilt);
    found->second = std::move(rebuilt);
  }
  for (const auto& source : delta.added) {
    auto contribution = buildContribution(source);
    rememberContribution(contribution);
    addActiveContribution(contribution);
    active_frames_.emplace(source.frame_id, std::move(contribution));
  }

  active_generation_ = delta.generation;
  update.accepted = true;
  update.reason = "ok";
  update.active_generation = active_generation_;
  update.stamp_s = current_stamp_s_;
  collectChanges(before, &update);
  return update;
}

RegisteredLidarWindowUpdate RegisteredLidarWindow::replaceActiveWindow(
    const std::uint64_t generation,
    const std::string& frame_contract_id,
    const std::vector<RegisteredLidarFrameData>& frames) {
  RegisteredLidarWindowUpdate update;
  update.active_generation = active_generation_;
  update.current_frame_id = current_frame_id_;
  if (frame_contract_id != geometry_.frame_contract_id) {
    update.recovery_required = true;
    update.reason = "frame_contract_mismatch";
    return update;
  }
  if (generation < active_generation_) {
    update.recovery_required = true;
    update.reason = "recovery_generation_regression";
    return update;
  }
  std::unordered_set<std::int64_t> frame_ids;
  for (const auto& source : frames) {
    if (source.frame_contract_id != geometry_.frame_contract_id ||
        source.frame_id < 0 || !frame_ids.insert(source.frame_id).second ||
        !source.T_map_lidar.matrix().allFinite() ||
        !std::isfinite(source.stamp_s) ||
        !std::isfinite(source.scan_end_stamp_s) ||
        source.scan_end_stamp_s + 1.0e-9 < source.stamp_s ||
        source.sensor_receipt_steady_ns == 0U) {
      update.recovery_required = true;
      update.reason = "invalid_recovery_window";
      return update;
    }
  }
  std::unordered_map<int, RegisteredVoxelState> before;
  for (int address = 0; address < static_cast<int>(active_hit_count_.size());
       ++address) {
    if (active_hit_count_[static_cast<std::size_t>(address)] != 0U ||
        active_free_count_[static_cast<std::size_t>(address)] != 0U) {
      rememberBefore(address, stateAtAddress(address), &before);
    }
  }
  std::fill(active_hit_count_.begin(), active_hit_count_.end(), 0U);
  std::fill(active_free_count_.begin(), active_free_count_.end(), 0U);
  active_frames_.clear();
  for (const auto& source : frames) {
    auto contribution = buildContribution(source);
    for (const int address : contribution.hits) {
      rememberBefore(address, stateAtAddress(address), &before);
    }
    for (const int address : contribution.observed_free) {
      rememberBefore(address, stateAtAddress(address), &before);
    }
    addActiveContribution(contribution);
    active_frames_.emplace(source.frame_id, std::move(contribution));
  }
  active_generation_ = generation;
  update.accepted = true;
  update.reason = "recovered";
  update.active_generation = active_generation_;
  update.stamp_s = current_stamp_s_;
  collectChanges(before, &update);
  return update;
}
