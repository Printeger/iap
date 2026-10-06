#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

enum class EvidenceVoxelState : std::uint8_t {
  UNKNOWN = 0,
  OBSERVED_FREE = 1,
  RAW_OCCUPIED = 2,
};

enum class LocalEvidenceReason : std::uint8_t {
  OK = 0,
  UNOBSERVED,
  OUT_OF_RANGE,
  STALE_OBSERVATION,
  EVALUATION_PRECEDES_OBSERVATION,
  COORDINATE_CONTRACT_MISMATCH,
  SENSOR_MODEL_MISMATCH,
  GENERATION_MISMATCH,
  INVALID_QUERY,
};

struct LocalEvidenceSource {
  std::int64_t frame_id = -1;
  double observation_stamp_s = std::numeric_limits<double>::quiet_NaN();
  std::string sensor_model_identity;
  std::string content_hash;
};

struct LocalEvidenceIdentity {
  std::uint64_t occupancy_generation = 0;
  std::uint64_t active_window_generation = 0;
  std::string coordinate_contract;
  std::string sensor_model_identity;
  std::uint32_t horizontal_samples = 0;
  std::uint32_t vertical_samples = 0;
  double horizontal_fov_rad = std::numeric_limits<double>::quiet_NaN();
  double vertical_min_rad = std::numeric_limits<double>::quiet_NaN();
  double vertical_max_rad = std::numeric_limits<double>::quiet_NaN();
  double min_range_m = std::numeric_limits<double>::quiet_NaN();
  double max_range_m = std::numeric_limits<double>::quiet_NaN();
  std::string source_set_hash;
  std::string content_hash;
};

struct LocalEvidenceQuery {
  EvidenceVoxelState state = EvidenceVoxelState::UNKNOWN;
  std::int64_t source_frame_id = -1;
  double observation_timestamp_s = std::numeric_limits<double>::quiet_NaN();
  double age_s = std::numeric_limits<double>::infinity();
  LocalEvidenceReason reason = LocalEvidenceReason::UNOBSERVED;
};

struct LocalEvidenceLosTrace {
  bool support_complete = false;
  std::size_t sample_count = 0;
  std::size_t observed_free_count = 0;
  std::size_t raw_occupied_count = 0;
  std::size_t unknown_count = 0;
  double first_unknown_distance_m = std::numeric_limits<double>::quiet_NaN();
  double first_occupied_distance_m = std::numeric_limits<double>::quiet_NaN();
  LocalEvidenceReason first_failure_reason = LocalEvidenceReason::OK;
};

struct LocalEvidenceCoverage {
  bool valid = false;
  std::size_t voxel_count = 0;
  std::size_t observed_free_count = 0;
  std::size_t raw_occupied_count = 0;
  std::size_t unknown_count = 0;
  double unknown_fraction = 1.0;
};

/**
 * Immutable, shared tri-state evidence for one occupancy transaction.
 *
 * States are packed at two bits per voxel. Source provenance is held as a
 * compact source-table index, so an LOS query never copies the lattice.
 * Collision inflation is intentionally absent: RAW_OCCUPIED means a sensor
 * return endpoint, never an inflated neighbour.
 */
class LocalEvidenceSnapshot {
 public:
  struct Geometry {
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    Eigen::Vector3i dimensions = Eigen::Vector3i::Zero();
    double resolution_m = 0.0;
  };

  struct ReadOnlyData {
    LocalEvidenceIdentity identity;
    Geometry geometry;
    std::vector<std::uint8_t> packed_states;
    std::vector<std::uint16_t> source_indices;
    std::vector<LocalEvidenceSource> sources;
    double freshness_s = 1.0;
  };
  const ReadOnlyData& readOnlyData() const { return *storage_; }
  static std::shared_ptr<const LocalEvidenceSnapshot> fromReadOnlyData(ReadOnlyData data);

  const LocalEvidenceIdentity& identity() const;
  const Geometry& geometry() const;
  const std::vector<LocalEvidenceSource>& sources() const;

  bool matches(std::uint64_t occupancy_generation,
               std::uint64_t active_window_generation,
               const std::string& coordinate_contract,
               const std::string& sensor_model_identity) const;

  LocalEvidenceQuery queryVoxel(
      const Eigen::Vector3d& point_map, double evaluation_time_s) const;
  LocalEvidenceQuery queryVoxel(
      const Eigen::Vector3d& point_map, double evaluation_time_s,
      std::uint64_t occupancy_generation,
      std::uint64_t active_window_generation,
      const std::string& coordinate_contract,
      const std::string& sensor_model_identity) const;

  LocalEvidenceLosTrace traceLos(
      const Eigen::Vector3d& origin_map,
      const Eigen::Vector3d& direction_map,
      double length_m, double step_m, double evaluation_time_s) const;

  // Whole-lattice coverage is diagnostic only. Route authorization must use
  // queryVoxel/traceLos over the actual swept and LOS scope.
  LocalEvidenceCoverage coverage(double evaluation_time_s) const;

 private:
  friend class RegisteredLidarWindow;
  explicit LocalEvidenceSnapshot(std::shared_ptr<const ReadOnlyData> storage);
  std::shared_ptr<const ReadOnlyData> storage_;
};
