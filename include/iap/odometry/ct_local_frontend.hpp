#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Local continuous-time frontend interface for the planned hybrid architecture.
// This boundary owns LiDAR/IMU local solve inputs and returns only compact
// backend handoff state, without exposing dense frontend factor internals.

#include <iap/odometry/ct_backend_summary.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/odometry/integrated_bspline_gicp_factor.hpp>
#include <iap/util/raw_points.hpp>

#include <Eigen/Core>
#include <gtsam_points/types/point_cloud.hpp>

#include <memory>
#include <vector>

namespace gtsam_points {
struct FlatContainer;
template <typename VoxelContents>
class IncrementalVoxelMap;
using iVox = IncrementalVoxelMap<FlatContainer>;
}  // namespace gtsam_points

namespace iap {

class CTLocalFrontend {
 public:
  enum class LidarBucketMode {
    TIME_EPS,
    FIXED_COUNT,
    SINGLE_BUCKET,
  };

  struct BucketConfig {
    LidarBucketMode mode{LidarBucketMode::TIME_EPS};
    double time_eps{1e-3};
    int max_buckets_per_scan{0};
    int fixed_buckets_per_scan{8};
  };

  // IAP-RQ-300 / IAP-RQ-410: Single IMU measurement for the local frontend solve window.
  struct IMUSample {
    double stamp = 0.0;
    Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_acc = Eigen::Vector3d::Zero();
  };

  struct SourceFrameInput {
    glim::RawPoints::ConstPtr raw_points;
    gtsam_points::PointCloud::ConstPtr source_cloud;
    double scan_start = 0.0;
    double scan_end = 0.0;
  };

  struct Input {
    glim::EstimationFrame::ConstPtr target_frame;
    std::vector<SourceFrameInput> source_frames;
    // IAP-RQ-300 / IAP-RQ-410: IMU measurements in the scan window for CT solve.
    std::vector<IMUSample> imu_samples;
    // IAP-RQ-300 / IAP-RQ-410: LiDAR registration target (null = skip LiDAR factors).
    std::shared_ptr<const gtsam_points::iVox> target_ivox;
    BucketConfig bucket_config;
    // Solver parameters.
    int lm_max_iterations{10};
    double accelerometer_precision{1.0};
    double gyroscope_precision{1.0};
    double max_correspondence_distance{1.0};
  };

  static std::vector<SplineBucketContext> create_lidar_buckets(
    const SplineStateLayout& layout,
    const SourceFrameInput& source_frame,
    const BucketConfig& bucket_config);

  CTLocalFrontendResult run(const Input& input) const;
};

}  // namespace iap
