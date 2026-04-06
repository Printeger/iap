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
class NearestNeighborSearch;
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
    double target_map_prep_ms{0.0};
    double target_snapshot_clone_ms{0.0};
    double target_voxel_lookup_prep_ms{0.0};
    double target_covariance_prep_ms{0.0};
    double source_to_target_transform_ms{0.0};
    BucketConfig bucket_config;
    // Solver parameters.
    int lm_max_iterations{10};
    double accelerometer_precision{1.0};
    double gyroscope_precision{1.0};
    double max_correspondence_distance{1.0};
    bool enable_lm_iteration_trace{false};
    bool enable_graph_problem_size{false};
  };

  struct LayerSegmentInput {
    std::size_t source_frame_index{0};
    SourceFrameInput source_frame;
    std::vector<IMUSample> imu_samples;
    std::shared_ptr<const gtsam_points::iVox> target_ivox;
    std::shared_ptr<const gtsam_points::NearestNeighborSearch> target_tree;
    std::array<std::size_t, kBSplineControlPointCount> control_indices{};
    std::size_t auxiliary_index{0};
  };

  struct LayerInput {
    BSplineUnifiedGraphContext graph_context;
    std::shared_ptr<const SplineStateLayout> imu_layout_override;
    std::shared_ptr<const SplineStateLayout> lidar_layout_override;
    std::vector<LayerSegmentInput> segments;
    BucketConfig bucket_config;
    int lm_max_iterations{10};
    double accelerometer_precision{1.0};
    double gyroscope_precision{1.0};
    double velocity_precision{1.0};
    double finite_difference_dt{0.01};
    double max_correspondence_distance{1.0};
    bool enable_lidar_factor_profiling{false};
    bool enable_graph_problem_size{false};
    IntegratedSplineGICPFactor::JacobianMode jacobian_mode{
      IntegratedSplineGICPFactor::JacobianMode::SEMI_ANALYTIC};
    double numeric_eps{1e-4};
    int correspondence_candidate_count{1};
    double correspondence_accept_ratio{0.0};
    double correspondence_min_score_gap{0.0};
    double outlier_mahalanobis_threshold{0.0};
    IntegratedSplineGICPFactor::RobustKernel robust_kernel{
      IntegratedSplineGICPFactor::RobustKernel::NONE};
    double robust_kernel_width{1.0};
    double robust_weight_floor{0.0};
  };

  static const char* bucket_mode_name(LidarBucketMode mode);

  static std::vector<SplineBucketContext> create_lidar_buckets(
    const SplineStateLayout& layout,
    const SourceFrameInput& source_frame,
    const BucketConfig& bucket_config);

  BSplineLocalLayerContribution assemble_local_layer(const LayerInput& input) const;
  CTLocalFrontendResult run(const Input& input) const;
  CTLocalFrontendShadowResult run_shadow_diagnostics(
    const LayerInput& input,
    const gtsam::Values& seed_values,
    double query_time) const;
};

}  // namespace iap
