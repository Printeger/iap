#pragma once
// IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410:
// Shared target-handle abstraction for continuous-time LiDAR factors. This is
// the first step away from per-segment target runtime ownership.

#include <gtsam_points/config.hpp>
#include <gtsam_points/ann/ivox.hpp>

#include <cstddef>
#include <memory>

#ifdef GTSAM_POINTS_USE_CUDA
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#endif

namespace iap {

enum class SharedTargetHandleMode {
  Snapshot,
  GlobalReference,
};

#ifdef GTSAM_POINTS_USE_CUDA
struct SharedTargetGpuResources {
  std::shared_ptr<gtsam_points::PointCloudCPU> target_cpu_points;
  std::shared_ptr<gtsam_points::GaussianVoxelMapGPU> target_gpu;
  std::size_t point_count = 0;
  const void* identity = nullptr;
  std::size_t revision = 0;
};
#endif

class ISharedTargetHandle {
 public:
  virtual ~ISharedTargetHandle() = default;

  virtual SharedTargetHandleMode mode() const = 0;
  virtual std::size_t revision() const = 0;
  virtual const void* identity() const = 0;
  virtual const std::shared_ptr<const gtsam_points::iVox>& target_snapshot() const = 0;
  virtual const std::shared_ptr<const gtsam_points::NearestNeighborSearch>& target_tree() const = 0;
  virtual std::size_t contributing_frames() const = 0;
  virtual std::size_t point_count() const = 0;
  virtual std::size_t snapshot_frame_count() const = 0;
  virtual std::size_t snapshot_point_count() const = 0;
  virtual double snapshot_span_sec() const = 0;
  virtual bool snapshot_policy_accepted() const = 0;
  virtual double build_ms() const = 0;
#ifdef GTSAM_POINTS_USE_CUDA
  virtual const std::shared_ptr<const SharedTargetGpuResources>& gpu_resources() const = 0;
#endif
};

class SharedTargetHandle final : public ISharedTargetHandle {
 public:
  SharedTargetHandle(
    SharedTargetHandleMode mode,
    std::size_t revision,
    std::shared_ptr<const gtsam_points::iVox> target_snapshot,
    std::shared_ptr<const gtsam_points::NearestNeighborSearch> target_tree,
    std::size_t contributing_frames,
    std::size_t point_count,
    std::size_t snapshot_frame_count,
    std::size_t snapshot_point_count,
    double snapshot_span_sec,
    bool snapshot_policy_accepted,
    double build_ms
#ifdef GTSAM_POINTS_USE_CUDA
    ,
    std::shared_ptr<const SharedTargetGpuResources> gpu_resources = nullptr
#endif
    )
  : mode_(mode),
    revision_(revision),
    target_snapshot_(std::move(target_snapshot)),
    target_tree_(std::move(target_tree)),
    contributing_frames_(contributing_frames),
    point_count_(point_count),
    snapshot_frame_count_(snapshot_frame_count),
    snapshot_point_count_(snapshot_point_count),
    snapshot_span_sec_(snapshot_span_sec),
    snapshot_policy_accepted_(snapshot_policy_accepted),
    build_ms_(build_ms)
#ifdef GTSAM_POINTS_USE_CUDA
    ,
    gpu_resources_(std::move(gpu_resources))
#endif
  {}

  SharedTargetHandleMode mode() const override { return mode_; }
  std::size_t revision() const override { return revision_; }
  const void* identity() const override { return target_snapshot_.get(); }
  const std::shared_ptr<const gtsam_points::iVox>& target_snapshot() const override { return target_snapshot_; }
  const std::shared_ptr<const gtsam_points::NearestNeighborSearch>& target_tree() const override { return target_tree_; }
  std::size_t contributing_frames() const override { return contributing_frames_; }
  std::size_t point_count() const override { return point_count_; }
  std::size_t snapshot_frame_count() const override { return snapshot_frame_count_; }
  std::size_t snapshot_point_count() const override { return snapshot_point_count_; }
  double snapshot_span_sec() const override { return snapshot_span_sec_; }
  bool snapshot_policy_accepted() const override { return snapshot_policy_accepted_; }
  double build_ms() const override { return build_ms_; }
#ifdef GTSAM_POINTS_USE_CUDA
  const std::shared_ptr<const SharedTargetGpuResources>& gpu_resources() const override { return gpu_resources_; }
#endif

 private:
  SharedTargetHandleMode mode_ = SharedTargetHandleMode::Snapshot;
  std::size_t revision_ = 0;
  std::shared_ptr<const gtsam_points::iVox> target_snapshot_;
  std::shared_ptr<const gtsam_points::NearestNeighborSearch> target_tree_;
  std::size_t contributing_frames_ = 0;
  std::size_t point_count_ = 0;
  std::size_t snapshot_frame_count_ = 0;
  std::size_t snapshot_point_count_ = 0;
  double snapshot_span_sec_ = 0.0;
  bool snapshot_policy_accepted_ = false;
  double build_ms_ = 0.0;
#ifdef GTSAM_POINTS_USE_CUDA
  std::shared_ptr<const SharedTargetGpuResources> gpu_resources_;
#endif
};

}  // namespace iap
