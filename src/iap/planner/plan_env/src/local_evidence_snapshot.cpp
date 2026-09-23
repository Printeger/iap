#include <plan_env/local_evidence_snapshot.h>

#include <algorithm>
#include <cmath>
#include <utility>

LocalEvidenceSnapshot::LocalEvidenceSnapshot(
    std::shared_ptr<const Storage> storage)
    : storage_(std::move(storage)) {}

const LocalEvidenceIdentity& LocalEvidenceSnapshot::identity() const {
  return storage_->identity;
}

const LocalEvidenceSnapshot::Geometry& LocalEvidenceSnapshot::geometry()
    const {
  return storage_->geometry;
}

const std::vector<LocalEvidenceSource>& LocalEvidenceSnapshot::sources()
    const {
  return storage_->sources;
}

bool LocalEvidenceSnapshot::matches(
    const std::uint64_t occupancy_generation,
    const std::uint64_t active_window_generation,
    const std::string& coordinate_contract,
    const std::string& sensor_model_identity) const {
  return storage_ &&
      storage_->identity.occupancy_generation == occupancy_generation &&
      storage_->identity.active_window_generation ==
          active_window_generation &&
      storage_->identity.coordinate_contract == coordinate_contract &&
      storage_->identity.sensor_model_identity == sensor_model_identity;
}

LocalEvidenceQuery LocalEvidenceSnapshot::queryVoxel(
    const Eigen::Vector3d& point_map, const double evaluation_time_s) const {
  LocalEvidenceQuery out;
  if (!storage_ || !point_map.allFinite() ||
      !std::isfinite(evaluation_time_s)) {
    out.reason = LocalEvidenceReason::INVALID_QUERY;
    return out;
  }
  const auto& geometry = storage_->geometry;
  const Eigen::Array3d relative =
      ((point_map - geometry.origin) / geometry.resolution_m).array();
  const Eigen::Vector3i index = relative.floor().cast<int>();
  if ((index.array() < 0).any() ||
      (index.array() >= geometry.dimensions.array()).any()) {
    out.reason = LocalEvidenceReason::OUT_OF_RANGE;
    return out;
  }
  const std::size_t address =
      static_cast<std::size_t>(index.x()) *
          static_cast<std::size_t>(geometry.dimensions.y()) *
          static_cast<std::size_t>(geometry.dimensions.z()) +
      static_cast<std::size_t>(index.y()) *
          static_cast<std::size_t>(geometry.dimensions.z()) +
      static_cast<std::size_t>(index.z());
  const std::size_t byte = address / 4U;
  const unsigned shift = static_cast<unsigned>((address % 4U) * 2U);
  if (byte >= storage_->packed_states.size() ||
      address >= storage_->source_indices.size()) {
    out.reason = LocalEvidenceReason::INVALID_QUERY;
    return out;
  }
  out.state = static_cast<EvidenceVoxelState>(
      (storage_->packed_states[byte] >> shift) & 0x3U);
  const std::uint16_t source_index = storage_->source_indices[address];
  if (out.state == EvidenceVoxelState::UNKNOWN || source_index == 0U ||
      source_index > storage_->sources.size()) {
    out.state = EvidenceVoxelState::UNKNOWN;
    out.reason = LocalEvidenceReason::UNOBSERVED;
    return out;
  }
  const auto& source = storage_->sources[source_index - 1U];
  out.source_frame_id = source.frame_id;
  out.observation_timestamp_s = source.observation_stamp_s;
  out.age_s = evaluation_time_s - source.observation_stamp_s;
  if (out.age_s < -1.0e-6) {
    out.state = EvidenceVoxelState::UNKNOWN;
    out.reason = LocalEvidenceReason::EVALUATION_PRECEDES_OBSERVATION;
    return out;
  }
  if (out.state == EvidenceVoxelState::OBSERVED_FREE &&
      out.age_s > storage_->freshness_s + 1.0e-9) {
    out.state = EvidenceVoxelState::UNKNOWN;
    out.reason = LocalEvidenceReason::STALE_OBSERVATION;
    return out;
  }
  out.reason = LocalEvidenceReason::OK;
  return out;
}

LocalEvidenceQuery LocalEvidenceSnapshot::queryVoxel(
    const Eigen::Vector3d& point_map, const double evaluation_time_s,
    const std::uint64_t occupancy_generation,
    const std::uint64_t active_window_generation,
    const std::string& coordinate_contract,
    const std::string& sensor_model_identity) const {
  LocalEvidenceQuery out;
  if (!storage_) {
    out.reason = LocalEvidenceReason::INVALID_QUERY;
  } else if (storage_->identity.occupancy_generation !=
                 occupancy_generation ||
             storage_->identity.active_window_generation !=
                 active_window_generation) {
    out.reason = LocalEvidenceReason::GENERATION_MISMATCH;
  } else if (storage_->identity.coordinate_contract != coordinate_contract) {
    out.reason = LocalEvidenceReason::COORDINATE_CONTRACT_MISMATCH;
  } else if (storage_->identity.sensor_model_identity !=
             sensor_model_identity) {
    out.reason = LocalEvidenceReason::SENSOR_MODEL_MISMATCH;
  } else {
    return queryVoxel(point_map, evaluation_time_s);
  }
  return out;
}

LocalEvidenceLosTrace LocalEvidenceSnapshot::traceLos(
    const Eigen::Vector3d& origin_map,
    const Eigen::Vector3d& direction_map,
    const double length_m, const double step_m,
    const double evaluation_time_s) const {
  LocalEvidenceLosTrace out;
  if (!origin_map.allFinite() || !direction_map.allFinite() ||
      !std::isfinite(length_m) || length_m < 0.0 ||
      !std::isfinite(step_m) || step_m <= 0.0 ||
      direction_map.norm() <= 1.0e-12) {
    out.unknown_count = 1;
    out.first_failure_reason = LocalEvidenceReason::INVALID_QUERY;
    return out;
  }
  const Eigen::Vector3d direction = direction_map.normalized();
  for (double distance = 0.0;;) {
    const auto query = queryVoxel(
        origin_map + distance * direction, evaluation_time_s);
    ++out.sample_count;
    if (query.state == EvidenceVoxelState::OBSERVED_FREE) {
      ++out.observed_free_count;
    } else if (query.state == EvidenceVoxelState::RAW_OCCUPIED) {
      ++out.raw_occupied_count;
      if (!std::isfinite(out.first_occupied_distance_m)) {
        out.first_occupied_distance_m = distance;
      }
    } else {
      ++out.unknown_count;
      if (!std::isfinite(out.first_unknown_distance_m)) {
        out.first_unknown_distance_m = distance;
        out.first_failure_reason = query.reason;
      }
    }
    if (distance >= length_m - 1.0e-9) {
      break;
    }
    distance = std::min(length_m, distance + step_m);
  }
  out.support_complete = out.unknown_count == 0U;
  return out;
}

LocalEvidenceCoverage LocalEvidenceSnapshot::coverage(
    const double evaluation_time_s) const {
  LocalEvidenceCoverage out;
  if (!storage_ || !std::isfinite(evaluation_time_s)) {
    return out;
  }
  out.valid = true;
  out.voxel_count = storage_->source_indices.size();
  for (std::size_t address = 0; address < out.voxel_count; ++address) {
    const std::size_t byte = address / 4U;
    const unsigned shift = static_cast<unsigned>((address % 4U) * 2U);
    if (byte >= storage_->packed_states.size()) {
      ++out.unknown_count;
      continue;
    }
    const auto state = static_cast<EvidenceVoxelState>(
        (storage_->packed_states[byte] >> shift) & 0x3U);
    const std::uint16_t source_index = storage_->source_indices[address];
    if (state == EvidenceVoxelState::RAW_OCCUPIED && source_index > 0U &&
        source_index <= storage_->sources.size()) {
      ++out.raw_occupied_count;
    } else if (state == EvidenceVoxelState::OBSERVED_FREE &&
               source_index > 0U &&
               source_index <= storage_->sources.size()) {
      const double age_s = evaluation_time_s -
          storage_->sources[source_index - 1U].observation_stamp_s;
      if (age_s >= -1.0e-6 &&
          age_s <= storage_->freshness_s + 1.0e-9) {
        ++out.observed_free_count;
      } else {
        ++out.unknown_count;
      }
    } else {
      ++out.unknown_count;
    }
  }
  out.unknown_fraction = out.voxel_count > 0U
      ? static_cast<double>(out.unknown_count) /
            static_cast<double>(out.voxel_count)
      : 1.0;
  return out;
}
