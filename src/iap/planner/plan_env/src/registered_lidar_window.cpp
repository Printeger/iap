#include <plan_env/registered_lidar_window.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace {

void rememberBefore(
    const int address, const RegisteredVoxelState state,
    std::unordered_map<int, RegisteredVoxelState>* before) {
  before->emplace(address, state);
}

std::string contributionContentHash(
    const RegisteredLidarFrameData& source,
    const std::vector<int>& occupied_addresses) {
  std::ostringstream canonical;
  canonical << source.frame_id << ';' << std::hexfloat << source.stamp_s
            << ';' << source.scan_end_stamp_s << ';'
            << source.frame_contract_id << ';'
            << source.source_is_map_reference << ';'
            << source.source_health_valid << ';'
            << source.source_health_stamp_s << ';'
            << source.source_icp_degenerate << ';'
            << source.source_icp_rmse << ';'
            << source.source_icp_condition << ';'
            << source.source_icp_gamma_lidar << ';';
  const Eigen::Matrix4d transform = source.T_map_lidar.matrix();
  for (int row = 0; row < transform.rows(); ++row) {
    for (int column = 0; column < transform.cols(); ++column) {
      canonical << transform(row, column) << ';';
    }
  }
  for (const int address : occupied_addresses) canonical << address << ',';
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char byte : canonical.str()) {
    hash ^= static_cast<std::uint64_t>(byte);
    hash *= 1099511628211ULL;
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << hash;
  return output.str();
}

bool validExplicitBeamEvidence(const RegisteredLidarFrameData& frame) {
  if (!frame.beam_evidence_complete) {
    return true;
  }
  const std::size_t expected =
      static_cast<std::size_t>(frame.horizontal_samples) *
      static_cast<std::size_t>(frame.vertical_samples);
  if (frame.sensor_model_id.empty() || frame.beam_content_hash.empty() ||
      frame.horizontal_samples == 0U || frame.vertical_samples == 0U ||
      expected != frame.beams.size() ||
      !std::isfinite(frame.horizontal_fov_rad) ||
      !std::isfinite(frame.vertical_min_rad) ||
      !std::isfinite(frame.vertical_max_rad) ||
      frame.vertical_min_rad > frame.vertical_max_rad ||
      !std::isfinite(frame.min_range_m) || frame.min_range_m < 0.0 ||
      !std::isfinite(frame.max_range_m) ||
      frame.max_range_m <= frame.min_range_m) {
    return false;
  }
  for (const auto& beam : frame.beams) {
    if (!beam.direction_lidar.allFinite() ||
        beam.direction_lidar.norm() <= 1.0e-12) {
      return false;
    }
    if (beam.outcome == RegisteredLidarBeamOutcome::HIT &&
        (!std::isfinite(beam.range_m) ||
         beam.range_m < frame.min_range_m ||
         beam.range_m > frame.max_range_m + 1.0e-6)) {
      return false;
    }
    if (beam.outcome != RegisteredLidarBeamOutcome::INVALID &&
        beam.outcome != RegisteredLidarBeamOutcome::HIT &&
        beam.outcome != RegisteredLidarBeamOutcome::NO_RETURN) {
      return false;
    }
  }
  return true;
}

std::string fnvIdentity(const std::string& canonical) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char byte : canonical) {
    hash ^= static_cast<std::uint64_t>(byte);
    hash *= 1099511628211ULL;
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << hash;
  return output.str();
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
    const RegisteredLidarFrameData& frame, const bool deduplicate) const {
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
  representative_rays.reserve(frame.beam_evidence_complete
                                  ? frame.beams.size()
                                  : frame.hits_lidar.size());
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

  std::vector<Eigen::Vector3d> explicit_hits;
  if (frame.beam_evidence_complete) {
    for (const auto& beam : frame.beams) {
      if (beam.outcome == RegisteredLidarBeamOutcome::INVALID) {
        continue;
      }
      const Eigen::Vector3d direction = beam.direction_lidar.normalized();
      const double range = beam.outcome == RegisteredLidarBeamOutcome::HIT
                               ? beam.range_m : frame.max_range_m;
      const Eigen::Vector3d endpoint_lidar = direction * range;
      if (beam.outcome == RegisteredLidarBeamOutcome::HIT) {
        explicit_hits.push_back(endpoint_lidar);
      } else {
        const Eigen::Vector3d endpoint = frame.T_map_lidar * endpoint_lidar;
        const auto clipped = inBounds(indexOf(endpoint))
            ? std::optional<Eigen::Vector3d>(endpoint)
            : clip_to_map(endpoint);
        if (clipped) {
          const Eigen::Vector3i finish = indexOf(*clipped);
          if (inBounds(finish) &&
              (mark(&ray_end_bits, address(finish)) || !deduplicate)) {
            representative_rays.push_back({*clipped, false});
          }
        }
      }
    }
  }
  const auto& hits = frame.beam_evidence_complete
      ? explicit_hits : frame.hits_lidar;
  for (const auto& hit_lidar : hits) {
    if (!hit_lidar.allFinite()) {
      continue;
    }
    const Eigen::Vector3d hit = frame.T_map_lidar * hit_lidar;
    const Eigen::Vector3i environment_index = indexOf(hit);
    contribution.environment_hit_keys.push_back({
        environment_index.x(), environment_index.y(), environment_index.z()});
    const Eigen::Vector3i finish = indexOf(hit);
    if (inBounds(finish)) {
      const int hit_address = address(finish);
      // Geometry uses every occupied endpoint voxel. For observed-free ray
      // evidence, one actual successful return per endpoint voxel is a
      // conservative 0.1 m voxel downsample: it never invents a no-return ray
      // and avoids retracing thousands of equivalent beams.
      if (mark(&hit_bits, hit_address) || !deduplicate) {
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
          (mark(&ray_end_bits, address(clipped_finish)) || !deduplicate)) {
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
      // The tie tolerance may conservatively advance several axes at one
      // crossing. Exclude axes that already reached their endpoint so a ray
      // landing within the tolerance of a voxel boundary cannot overshoot and
      // then walk forever away from `finish`.
      const auto before_finish = [&cell, &finish, &step](const int axis) {
        return step[axis] > 0
            ? cell[axis] < finish[axis]
            : step[axis] < 0 && cell[axis] > finish[axis];
      };
      double crossing = std::numeric_limits<double>::infinity();
      for (int axis = 0; axis < 3; ++axis) {
        if (before_finish(axis)) {
          crossing = std::min(crossing, t_max[axis]);
        }
      }
      for (int axis = 0; axis < 3; ++axis) {
        if (before_finish(axis) && t_max[axis] <= crossing + 1.0e-12) {
          cell[axis] += step[axis];
          t_max[axis] += t_delta[axis];
        }
      }
    }
    if (!ray.occupied_endpoint && inBounds(finish)) {
      mark(&free_bits, address(finish));
    }
  }

  contribution.hits.reserve(hits.size());
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
  std::sort(contribution.environment_hit_keys.begin(),
            contribution.environment_hit_keys.end());
  contribution.environment_hit_keys.erase(
      std::unique(contribution.environment_hit_keys.begin(),
                  contribution.environment_hit_keys.end()),
      contribution.environment_hit_keys.end());
  return contribution;
}

std::optional<RegisteredLidarFrameData>
RegisteredLidarWindow::currentFrameSource() const {
  if (!has_current_frame_) return std::nullopt;
  return current_frame_.source;
}

std::vector<uint8_t> RegisteredLidarWindow::observationSourceFlags() const {
  std::vector<uint8_t> flags(active_hit_count_.size());
  for (size_t i = 0; i < flags.size(); ++i)
    flags[i] = (current_hit_[i] ? 1 : 0) | (current_free_[i] ? 2 : 0) |
        (active_hit_count_[i] ? 4 : 0) | (active_free_count_[i] ? 8 : 0);
  return flags;
}

std::vector<uint8_t> RegisteredLidarWindow::unthinnedObservationMask(
    const RegisteredLidarFrameData& frame) const {
  if (!validGeometry() || frame.frame_contract_id != geometry_.frame_contract_id)
    throw std::invalid_argument("unthinned diagnostic requires matching registered geometry");
  const auto contribution = buildContribution(frame, false);
  std::vector<uint8_t> mask(active_hit_count_.size(), 0);
  for (int address : contribution.hits) mask[address] = 1;
  for (int address : contribution.observed_free) mask[address] = 1;
  return mask;
}

std::optional<RegisteredLidarFrameMetadata>
RegisteredLidarWindow::currentFrameMetadata() const {
  if (!has_current_frame_) {
    return std::nullopt;
  }
  const auto& source = current_frame_.source;
  RegisteredLidarFrameMetadata metadata;
  metadata.frame_id = source.frame_id;
  metadata.stamp_s = source.stamp_s;
  metadata.scan_end_stamp_s = source.scan_end_stamp_s;
  metadata.sensor_receipt_steady_ns = source.sensor_receipt_steady_ns;
  metadata.T_map_lidar = source.T_map_lidar;
  metadata.frame_contract_id = source.frame_contract_id;
  metadata.source_is_map_reference = source.source_is_map_reference;
  metadata.source_health_valid = source.source_health_valid;
  metadata.source_health_stamp_s = source.source_health_stamp_s;
  metadata.source_icp_degenerate = source.source_icp_degenerate;
  metadata.source_icp_rmse = source.source_icp_rmse;
  metadata.source_icp_condition = source.source_icp_condition;
  metadata.source_icp_gamma_lidar = source.source_icp_gamma_lidar;
  metadata.source_lidar_pl_enu_m = source.source_lidar_pl_enu_m;
  metadata.content_hash = contributionContentHash(source, current_frame_.hits);
  return metadata;
}

std::shared_ptr<const std::vector<Eigen::Vector3d>>
RegisteredLidarWindow::environmentOccupiedVoxelCenters() const {
  return environment_occupied_voxel_centers_;
}

std::shared_ptr<const std::vector<Eigen::Vector3d>>
RegisteredLidarWindow::currentOccupiedVoxelCenters() const {
  auto centers = std::make_shared<std::vector<Eigen::Vector3d>>();
  if (!has_current_frame_) {
    return centers;
  }
  centers->reserve(current_frame_.hits.size());
  for (const int address : current_frame_.hits) {
    const Eigen::Vector3i index = indexFromAddress(address);
    centers->push_back(
        geometry_.origin +
        (index.cast<double>() + Eigen::Vector3d::Constant(0.5)) *
            geometry_.resolution_m);
  }
  return centers;
}

std::shared_ptr<const std::vector<RegisteredLidarObstacleSource>>
RegisteredLidarWindow::activeObstacleSources() const {
  auto sources = std::make_shared<std::vector<RegisteredLidarObstacleSource>>();
  sources->reserve(active_frames_.size());
  for (const auto& [frame_id, contribution] : active_frames_) {
    (void)frame_id;
    auto centers = std::make_shared<std::vector<Eigen::Vector3d>>();
    centers->reserve(contribution.environment_hit_keys.size());
    for (const auto& key : contribution.environment_hit_keys) {
      centers->push_back(
          geometry_.origin +
          (Eigen::Vector3d(key[0], key[1], key[2]) +
           Eigen::Vector3d::Constant(0.5)) * geometry_.resolution_m);
    }
    RegisteredLidarObstacleSource source;
    const auto& frame = contribution.source;
    source.metadata.frame_id = frame.frame_id;
    source.metadata.stamp_s = frame.stamp_s;
    source.metadata.scan_end_stamp_s = frame.scan_end_stamp_s;
    source.metadata.sensor_receipt_steady_ns = frame.sensor_receipt_steady_ns;
    source.metadata.T_map_lidar = frame.T_map_lidar;
    source.metadata.frame_contract_id = frame.frame_contract_id;
    source.metadata.source_is_map_reference =
        frame.source_is_map_reference;
    source.metadata.source_health_valid = frame.source_health_valid;
    source.metadata.source_health_stamp_s = frame.source_health_stamp_s;
    source.metadata.source_icp_degenerate = frame.source_icp_degenerate;
    source.metadata.source_icp_rmse = frame.source_icp_rmse;
    source.metadata.source_icp_condition = frame.source_icp_condition;
    source.metadata.source_icp_gamma_lidar = frame.source_icp_gamma_lidar;
    source.metadata.source_lidar_pl_enu_m = frame.source_lidar_pl_enu_m;
    source.metadata.content_hash = contributionContentHash(frame,
                                                           contribution.hits);
    source.occupied_voxel_centers = std::move(centers);
    sources->push_back(std::move(source));
  }
  std::sort(sources->begin(), sources->end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.metadata.frame_id < rhs.metadata.frame_id;
            });
  return sources;
}

std::shared_ptr<const LocalEvidenceSnapshot>
RegisteredLidarWindow::captureLocalEvidenceSnapshot(
    const std::uint64_t occupancy_generation) const {
  if (!validGeometry()) {
    return nullptr;
  }
  std::vector<const FrameContribution*> contributions;
  contributions.reserve(active_frames_.size() + (has_current_frame_ ? 1U : 0U));
  for (const auto& entry : active_frames_) {
    if (entry.second.source.beam_evidence_complete &&
        validExplicitBeamEvidence(entry.second.source)) {
      contributions.push_back(&entry.second);
    }
  }
  if (has_current_frame_ && current_frame_.source.beam_evidence_complete &&
      validExplicitBeamEvidence(current_frame_.source)) {
    contributions.push_back(&current_frame_);
  }
  std::sort(contributions.begin(), contributions.end(),
            [](const auto* lhs, const auto* rhs) {
              if (lhs->source.scan_end_stamp_s != rhs->source.scan_end_stamp_s) {
                return lhs->source.scan_end_stamp_s <
                    rhs->source.scan_end_stamp_s;
              }
              return lhs->source.frame_id < rhs->source.frame_id;
            });
  if (contributions.empty()) {
    return nullptr;
  }
  const std::string sensor_model =
      contributions.front()->source.sensor_model_id;
  const auto &sensor_contract = contributions.front()->source;
  const auto same_sensor_contract = [&sensor_contract](
      const FrameContribution *contribution) {
        const auto &frame = contribution->source;
        constexpr double kContractTolerance = 1.0e-12;
        return frame.sensor_model_id == sensor_contract.sensor_model_id &&
            frame.horizontal_samples == sensor_contract.horizontal_samples &&
            frame.vertical_samples == sensor_contract.vertical_samples &&
            std::abs(frame.horizontal_fov_rad -
                sensor_contract.horizontal_fov_rad) <= kContractTolerance &&
            std::abs(frame.vertical_min_rad -
                sensor_contract.vertical_min_rad) <= kContractTolerance &&
            std::abs(frame.vertical_max_rad -
                sensor_contract.vertical_max_rad) <= kContractTolerance &&
            std::abs(frame.min_range_m - sensor_contract.min_range_m) <=
                kContractTolerance &&
            std::abs(frame.max_range_m - sensor_contract.max_range_m) <=
                kContractTolerance;
      };
  if (sensor_model.empty() ||
      std::any_of(contributions.begin(), contributions.end(),
                  [&same_sensor_contract](const auto* contribution) {
                    return !same_sensor_contract(contribution);
                  }) ||
      contributions.size() >=
          static_cast<std::size_t>(std::numeric_limits<std::uint16_t>::max())) {
    return nullptr;
  }

  auto storage = std::make_shared<LocalEvidenceSnapshot::Storage>();
  storage->identity.occupancy_generation = occupancy_generation;
  storage->identity.active_window_generation = active_generation_;
  storage->identity.coordinate_contract = geometry_.frame_contract_id;
  storage->identity.sensor_model_identity = sensor_model;
  storage->identity.horizontal_samples = sensor_contract.horizontal_samples;
  storage->identity.vertical_samples = sensor_contract.vertical_samples;
  storage->identity.horizontal_fov_rad = sensor_contract.horizontal_fov_rad;
  storage->identity.vertical_min_rad = sensor_contract.vertical_min_rad;
  storage->identity.vertical_max_rad = sensor_contract.vertical_max_rad;
  storage->identity.min_range_m = sensor_contract.min_range_m;
  storage->identity.max_range_m = sensor_contract.max_range_m;
  storage->geometry.origin = geometry_.origin;
  storage->geometry.dimensions = geometry_.dimensions;
  storage->geometry.resolution_m = geometry_.resolution_m;
  const std::size_t cell_count = active_hit_count_.size();
  storage->packed_states.assign((cell_count + 3U) / 4U, 0U);
  storage->source_indices.assign(cell_count, 0U);
  storage->sources.reserve(contributions.size());
  std::vector<EvidenceVoxelState> states(
      cell_count, EvidenceVoxelState::UNKNOWN);
  std::vector<double> source_stamps(
      cell_count, -std::numeric_limits<double>::infinity());
  std::ostringstream source_identity;
  for (std::size_t source = 0; source < contributions.size(); ++source) {
    const auto& contribution = *contributions[source];
    const auto& frame = contribution.source;
    storage->sources.push_back(LocalEvidenceSource{
        frame.frame_id, frame.scan_end_stamp_s, frame.sensor_model_id,
        frame.beam_content_hash});
    source_identity << frame.frame_id << ':' << std::hexfloat
                    << frame.scan_end_stamp_s << ':'
                    << frame.beam_content_hash << ';';
    const std::uint16_t source_index =
        static_cast<std::uint16_t>(source + 1U);
    for (const int address : contribution.observed_free) {
      const auto index = static_cast<std::size_t>(address);
      if (states[index] != EvidenceVoxelState::RAW_OCCUPIED &&
          frame.scan_end_stamp_s >= source_stamps[index]) {
        states[index] = EvidenceVoxelState::OBSERVED_FREE;
        storage->source_indices[index] = source_index;
        source_stamps[index] = frame.scan_end_stamp_s;
      }
    }
    for (const int address : contribution.hits) {
      const auto index = static_cast<std::size_t>(address);
      if (states[index] != EvidenceVoxelState::RAW_OCCUPIED ||
          frame.scan_end_stamp_s >= source_stamps[index]) {
        states[index] = EvidenceVoxelState::RAW_OCCUPIED;
        storage->source_indices[index] = source_index;
        source_stamps[index] = frame.scan_end_stamp_s;
      }
    }
  }
  std::ostringstream content_identity;
  content_identity << occupancy_generation << ':' << active_generation_ << ':'
                   << geometry_.frame_contract_id << ':' << sensor_model << ':'
                   << sensor_contract.horizontal_samples << ':'
                   << sensor_contract.vertical_samples << ':' << std::hexfloat
                   << sensor_contract.horizontal_fov_rad << ':'
                   << sensor_contract.vertical_min_rad << ':'
                   << sensor_contract.vertical_max_rad << ':'
                   << sensor_contract.min_range_m << ':'
                   << sensor_contract.max_range_m << ':';
  for (std::size_t address = 0; address < states.size(); ++address) {
    const auto raw = static_cast<std::uint8_t>(states[address]);
    storage->packed_states[address / 4U] |=
        static_cast<std::uint8_t>(raw << ((address % 4U) * 2U));
    if (raw != 0U) {
      content_identity << address << '=' << static_cast<unsigned>(raw) << '@'
                       << storage->source_indices[address] << ';';
    }
  }
  storage->identity.source_set_hash = fnvIdentity(source_identity.str());
  content_identity << storage->identity.source_set_hash;
  storage->identity.content_hash = fnvIdentity(content_identity.str());
  return std::shared_ptr<const LocalEvidenceSnapshot>(
      new LocalEvidenceSnapshot(std::move(storage)));
}

bool RegisteredLidarWindow::addEnvironmentContribution(
    const FrameContribution& contribution) {
  bool union_changed = false;
  for (const auto& key : contribution.environment_hit_keys) {
    auto [found, inserted] = environment_voxel_ref_count_.try_emplace(key, 0U);
    union_changed = union_changed || inserted;
    if (found->second != std::numeric_limits<std::uint32_t>::max()) {
      ++found->second;
    }
  }
  return union_changed;
}

bool RegisteredLidarWindow::removeEnvironmentContribution(
    const FrameContribution& contribution) {
  bool union_changed = false;
  for (const auto& key : contribution.environment_hit_keys) {
    const auto found = environment_voxel_ref_count_.find(key);
    if (found == environment_voxel_ref_count_.end()) {
      continue;
    }
    if (found->second > 1U) {
      --found->second;
    } else {
      environment_voxel_ref_count_.erase(found);
      union_changed = true;
    }
  }
  return union_changed;
}

void RegisteredLidarWindow::publishEnvironmentOccupiedVoxelCenters() {
  auto centers = std::make_shared<std::vector<Eigen::Vector3d>>();
  centers->reserve(environment_voxel_ref_count_.size());
  for (const auto& [key, count] : environment_voxel_ref_count_) {
    (void)count;
    const Eigen::Vector3i index(key[0], key[1], key[2]);
    centers->push_back(
        geometry_.origin +
        (index.cast<double>() + Eigen::Vector3d::Constant(0.5)) *
            geometry_.resolution_m);
  }
  std::sort(centers->begin(), centers->end(),
            [](const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
              if (lhs.x() != rhs.x()) return lhs.x() < rhs.x();
              if (lhs.y() != rhs.y()) return lhs.y() < rhs.y();
              return lhs.z() < rhs.z();
            });
  environment_occupied_voxel_centers_ = std::move(centers);
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
  removeEnvironmentContribution(contribution);
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
  addEnvironmentContribution(contribution);
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
  RegisteredLidarWindowUpdate update{};
  update.operation = RegisteredLidarWindowUpdate::Operation::CURRENT_REPLACE;
  update.active_generation = active_generation_;
  update.current_frame_id = current_frame_id_;
  if (!validGeometry() ||
      frame.frame_contract_id != geometry_.frame_contract_id ||
      frame.frame_id < 0 || !frame.T_map_lidar.matrix().allFinite() ||
      !std::isfinite(frame.stamp_s) ||
      !std::isfinite(frame.scan_end_stamp_s) ||
      frame.scan_end_stamp_s + 1.0e-9 < frame.stamp_s ||
      frame.sensor_receipt_steady_ns == 0U ||
      !validExplicitBeamEvidence(frame)) {
    update.reason = frame.frame_contract_id != geometry_.frame_contract_id
                        ? "frame_contract_mismatch"
                        : "invalid_current_frame";
    return update;
  }

  FrameContribution next = buildContribution(frame);

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

  bool environment_union_changed = false;
  if (has_current_frame_) {
    environment_union_changed = removeEnvironmentContribution(current_frame_);
    for (const int address : current_frame_.hits) {
      current_hit_[static_cast<std::size_t>(address)] = 0U;
    }
    for (const int address : current_frame_.observed_free) {
      current_free_[static_cast<std::size_t>(address)] = 0U;
    }
  }
  environment_union_changed =
      addEnvironmentContribution(next) || environment_union_changed;
  for (const int address : next.hits) {
    current_hit_[static_cast<std::size_t>(address)] = 1U;
  }
  for (const int address : next.observed_free) {
    current_free_[static_cast<std::size_t>(address)] = 1U;
  }
  current_frame_ = std::move(next);
  has_current_frame_ = true;
  current_frame_id_ = frame.frame_id;
  current_stamp_s_ = frame.stamp_s;
  if (environment_union_changed)
    publishEnvironmentOccupiedVoxelCenters();
  update.accepted = true;
  update.reason = "ok";
  update.current_frame_id = current_frame_id_;
  update.stamp_s = current_stamp_s_;
  return update;
}

RegisteredLidarWindowUpdate RegisteredLidarWindow::applyActiveDelta(
    const ActiveLidarWindowDeltaData& delta) {
  RegisteredLidarWindowUpdate update{};
  update.operation = RegisteredLidarWindowUpdate::Operation::ACTIVE_DELTA;
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
        source.frame_id < 0 ||
        (active_frames_.count(source.frame_id) != 0U &&
         removed_ids.count(source.frame_id) == 0U) ||
        !added_ids.insert(source.frame_id).second ||
        !source.T_map_lidar.matrix().allFinite() ||
        !std::isfinite(source.stamp_s) ||
        !std::isfinite(source.scan_end_stamp_s) ||
        source.scan_end_stamp_s + 1.0e-9 < source.stamp_s ||
        source.sensor_receipt_steady_ns == 0U ||
        !validExplicitBeamEvidence(source)) {
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
  publishEnvironmentOccupiedVoxelCenters();
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
  RegisteredLidarWindowUpdate update{};
  update.operation = RegisteredLidarWindowUpdate::Operation::ACTIVE_REPLACE;
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
        source.sensor_receipt_steady_ns == 0U ||
        !validExplicitBeamEvidence(source)) {
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
  for (const auto& [id, contribution] : active_frames_) {
    (void)id;
    removeEnvironmentContribution(contribution);
  }
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
  publishEnvironmentOccupiedVoxelCenters();
  update.accepted = true;
  update.reason = "recovered";
  update.active_generation = active_generation_;
  update.stamp_s = current_stamp_s_;
  collectChanges(before, &update);
  return update;
}
