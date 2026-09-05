#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <array>
#include <initializer_list>
#include <map>
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

struct RegisteredLidarFrameData {
  std::int64_t frame_id = -1;
  double stamp_s = 0.0;
  double scan_end_stamp_s = 0.0;
  std::uint64_t sensor_receipt_steady_ns = 0;
  Eigen::Isometry3d T_map_lidar = Eigen::Isometry3d::Identity();
  std::vector<Eigen::Vector3d> hits_lidar;
  std::string frame_contract_id;
};

struct RegisteredLidarFrameMetadata {
  std::int64_t frame_id = -1;
  double stamp_s = 0.0;
  double scan_end_stamp_s = 0.0;
  std::uint64_t sensor_receipt_steady_ns = 0;
  Eigen::Isometry3d T_map_lidar = Eigen::Isometry3d::Identity();
  std::string frame_contract_id;
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
  std::optional<RegisteredLidarFrameMetadata> currentFrameMetadata() const;
  std::shared_ptr<const std::vector<Eigen::Vector3d>>
  environmentOccupiedVoxelCenters() const;

 private:
  using EnvironmentVoxelKey = std::array<int, 3>;

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
      const RegisteredLidarFrameData& frame) const;
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
  std::map<EnvironmentVoxelKey, std::uint32_t> environment_voxel_ref_count_;
  std::shared_ptr<const std::vector<Eigen::Vector3d>>
      environment_occupied_voxel_centers_ =
          std::make_shared<const std::vector<Eigen::Vector3d>>();
};
