#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <plan_env/local_evidence_snapshot.h>

#include <cstdint>
#include <array>
#include <initializer_list>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

enum class RegisteredVoxelState : std::uint8_t {
  UNKNOWN = 0,
  OBSERVED_FREE = 1,
  OCCUPIED = 2,
};

enum class RegisteredLidarBeamOutcome : std::uint8_t {
  INVALID = 0,
  HIT = 1,
  NO_RETURN = 2,
};

struct RegisteredLidarBeamData {
  Eigen::Vector3d direction_lidar = Eigen::Vector3d::Zero();
  RegisteredLidarBeamOutcome outcome = RegisteredLidarBeamOutcome::INVALID;
  double range_m = std::numeric_limits<double>::quiet_NaN();
};

struct RegisteredLidarFrameData {
  std::int64_t frame_id = -1;
  double stamp_s = 0.0;
  double scan_end_stamp_s = 0.0;
  std::uint64_t sensor_receipt_steady_ns = 0;
  Eigen::Isometry3d T_map_lidar = Eigen::Isometry3d::Identity();
  std::vector<Eigen::Vector3d> hits_lidar;
  std::string sensor_model_id;
  std::uint32_t horizontal_samples = 0;
  std::uint32_t vertical_samples = 0;
  double horizontal_fov_rad = std::numeric_limits<double>::quiet_NaN();
  double vertical_min_rad = std::numeric_limits<double>::quiet_NaN();
  double vertical_max_rad = std::numeric_limits<double>::quiet_NaN();
  double min_range_m = std::numeric_limits<double>::quiet_NaN();
  double max_range_m = std::numeric_limits<double>::quiet_NaN();
  bool beam_evidence_complete = false;
  std::string beam_content_hash;
  std::string beam_binding_reason;
  std::uint64_t beam_received_count = 0;
  std::uint64_t beam_invalid_count = 0;
  std::uint64_t beam_evicted_count = 0;
  double beam_history_oldest_stamp_s = std::numeric_limits<double>::quiet_NaN();
  double beam_history_newest_stamp_s = std::numeric_limits<double>::quiet_NaN();
  double beam_same_start_end_stamp_s = std::numeric_limits<double>::quiet_NaN();
  std::vector<RegisteredLidarBeamData> beams;
  std::string frame_contract_id;
  bool source_is_map_reference = false;
  bool source_health_valid = false;
  double source_health_stamp_s = 0.0;
  bool source_icp_degenerate = true;
  double source_icp_rmse = 0.0;
  double source_icp_condition = 0.0;
  double source_icp_gamma_lidar = 0.0;
  Eigen::Vector3d source_lidar_pl_enu_m = Eigen::Vector3d::Zero();
};

struct RegisteredLidarFrameMetadata {
  std::int64_t frame_id = -1;
  double stamp_s = 0.0;
  double scan_end_stamp_s = 0.0;
  std::uint64_t sensor_receipt_steady_ns = 0;
  Eigen::Isometry3d T_map_lidar = Eigen::Isometry3d::Identity();
  std::string frame_contract_id;
  bool source_is_map_reference = false;
  bool source_health_valid = false;
  double source_health_stamp_s = 0.0;
  bool source_icp_degenerate = true;
  double source_icp_rmse = 0.0;
  double source_icp_condition = 0.0;
  double source_icp_gamma_lidar = 0.0;
  Eigen::Vector3d source_lidar_pl_enu_m = Eigen::Vector3d::Zero();
  // Deterministic identity of the immutable pose and occupied contribution.
  std::string content_hash;
};

// Immutable obstacle provenance for one registered source frame.  Keeping the
// source grouping (rather than only the union of occupied voxels) lets the
// execution-assurance layer apply the error bound certified at that frame.
struct RegisteredLidarObstacleSource {
  RegisteredLidarFrameMetadata metadata;
  std::shared_ptr<const std::vector<Eigen::Vector3d>> occupied_voxel_centers;
};

struct ActiveLidarWindowDeltaData {
  std::string frame_contract_id;
  std::uint64_t base_generation = 0;
  std::uint64_t generation = 0;
  bool complete = false;
  std::vector<RegisteredLidarFrameData> added;
  std::vector<std::int64_t> removed_frame_ids;
  std::vector<std::pair<std::int64_t, Eigen::Isometry3d>> pose_updates;
};

struct RegisteredVoxelChange {
  Eigen::Vector3i index = Eigen::Vector3i::Constant(-1);
  RegisteredVoxelState state = RegisteredVoxelState::UNKNOWN;
};

struct RegisteredLidarWindowUpdate {
  // Which authoritative producer removed an observed contribution. Used
  // only for opt-in failure forensics, never as execution evidence.
  enum class Operation : uint8_t { NONE = 0, CURRENT_REPLACE = 1,
                                  ACTIVE_DELTA = 2, ACTIVE_REPLACE = 3 };
  Operation operation = Operation::NONE;
  bool accepted = false;
  bool recovery_required = false;
  std::string reason;
  std::uint64_t active_generation = 0;
  std::int64_t current_frame_id = -1;
  double stamp_s = 0.0;
  std::vector<RegisteredVoxelChange> changes;
};

/**
 * Accumulates the active GLIM keyframe window and one replaceable current
 * frame. The class owns only registered first-hit and successful-return ray
 * evidence; it performs no registration, SLAM, or risk inference.
 *
 * Mutations return only semantically changed voxels, so a GridMap consumer can
 * update its authoritative occupancy epoch without copying the full lattice.
 */
class RegisteredLidarWindow {
 public:
  struct Geometry {
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    Eigen::Vector3i dimensions = Eigen::Vector3i::Zero();
    double resolution_m = 0.0;
    std::string frame_contract_id;
  };

  explicit RegisteredLidarWindow(Geometry geometry);

  RegisteredLidarWindowUpdate applyCurrentFrame(
      const RegisteredLidarFrameData& frame);
  RegisteredLidarWindowUpdate applyActiveDelta(
      const ActiveLidarWindowDeltaData& delta);
  RegisteredLidarWindowUpdate replaceActiveWindow(
      std::uint64_t generation,
      const std::string& frame_contract_id,
      const std::vector<RegisteredLidarFrameData>& frames);

  RegisteredVoxelState stateAt(const Eigen::Vector3i& index) const;
  std::uint64_t activeGeneration() const { return active_generation_; }
  std::int64_t currentFrameId() const { return current_frame_id_; }
  const Geometry& geometry() const { return geometry_; }
  std::optional<RegisteredLidarFrameData> currentFrameSource() const;
  // current hit=1/free=2, active hit=4/free=8; retains the actual masks.
  std::vector<uint8_t> observationSourceFlags() const;
  // Diagnostic replay of every provided ray using the same traversal, with
  // endpoint deduplication disabled. Does not modify online evidence.
  std::vector<uint8_t> unthinnedObservationMask(
      const RegisteredLidarFrameData& frame) const;
  std::optional<RegisteredLidarFrameMetadata> currentFrameMetadata() const;
  std::shared_ptr<const std::vector<Eigen::Vector3d>>
  environmentOccupiedVoxelCenters() const;
  std::shared_ptr<const std::vector<Eigen::Vector3d>>
  currentOccupiedVoxelCenters() const;
  std::shared_ptr<const std::vector<RegisteredLidarObstacleSource>>
  activeObstacleSources() const;
  std::shared_ptr<const LocalEvidenceSnapshot> captureLocalEvidenceSnapshot(
      std::uint64_t occupancy_generation) const;

 private:
  using EnvironmentVoxelKey = std::array<int, 3>;
  struct EnvironmentVoxelKeyHash {
    std::size_t operator()(const EnvironmentVoxelKey& key) const {
      std::size_t seed = 1469598103934665603ULL;
      for (const int value : key) {
        seed ^= std::hash<int>{}(value);
        seed *= 1099511628211ULL;
      }
      return seed;
    }
  };

  struct FrameContribution {
    RegisteredLidarFrameData source;
    std::vector<int> hits;
    std::vector<int> observed_free;
    std::vector<EnvironmentVoxelKey> environment_hit_keys;
  };

  bool validGeometry() const;
  bool inBounds(const Eigen::Vector3i& index) const;
  int address(const Eigen::Vector3i& index) const;
  Eigen::Vector3i indexOf(const Eigen::Vector3d& point) const;
  Eigen::Vector3i indexFromAddress(int address) const;
  FrameContribution buildContribution(
      const RegisteredLidarFrameData& frame, bool deduplicate = true) const;
  RegisteredVoxelState stateAtAddress(int address) const;
  void removeActiveContribution(const FrameContribution& contribution);
  void addActiveContribution(const FrameContribution& contribution);
  void collectChanges(
      const std::unordered_map<int, RegisteredVoxelState>& before,
      RegisteredLidarWindowUpdate* update) const;
  bool addEnvironmentContribution(const FrameContribution& contribution);
  bool removeEnvironmentContribution(const FrameContribution& contribution);
  void publishEnvironmentOccupiedVoxelCenters();

  Geometry geometry_;
  std::uint64_t active_generation_ = 0;
  std::int64_t current_frame_id_ = -1;
  double current_stamp_s_ = 0.0;
  std::unordered_map<std::int64_t, FrameContribution> active_frames_;
  FrameContribution current_frame_;
  bool has_current_frame_ = false;
  // A frame contributes at most once to a voxel, but using a wide counter
  // keeps the accumulator correct even if a non-default active-window limit
  // is raised well beyond the production value of 15.
  std::vector<std::uint32_t> active_hit_count_;
  std::vector<std::uint32_t> active_free_count_;
  std::vector<std::uint8_t> current_hit_;
  std::vector<std::uint8_t> current_free_;
  std::unordered_map<EnvironmentVoxelKey, std::uint32_t,
                     EnvironmentVoxelKeyHash>
      environment_voxel_ref_count_;
  std::shared_ptr<const std::vector<Eigen::Vector3d>>
      environment_occupied_voxel_centers_ =
          std::make_shared<const std::vector<Eigen::Vector3d>>();
};
