#pragma once
#include <array>
#include <cstdint>
#include <vector>
#include <string>
#include <Eigen/Geometry>
namespace iap {
// POD written by the same CUDA transform that contributes to the reduction.
// Residual and covariance are in the target frame, in metres / square metres.
// The actual source and target indices retain repeated-match identities.
struct GpuMatchResidual {
  int valid=0, source_index=-1, target_index=-1, target_point_count=0;
  float residual_m[3]{};
  float covariance_m2_row_major[9]{};
  float source_covariance_m2_row_major[9]{};
  float target_covariance_m2_row_major[9]{};
  float source_mean_m[3]{};
  float target_mean_m[3]{};
  float mahalanobis_squared=0.f;
};
inline constexpr std::size_t kGpuMatchEvidenceLimit=128;
// All downloads share pinned storage; even small pageable destinations can
// introduce an implicit per-factor synchronization in cudaMemcpyAsync.
struct GpuMatchTransfer {
  GpuMatchResidual samples[kGpuMatchEvidenceLimit];
  float transform_column_major[16];
  int inliers;
  float cost;
};
struct GpuMatchOwner {
  std::int64_t source_frame_id=-1, target_frame_id=-1;
  double source_stamp=0., target_stamp=0., voxel_resolution_m=0.;
  int level_id=-1;
  bool target_is_fixed=false;
};
struct GpuMatchCapture {
  bool available=false;
  std::string failure_reason;
  std::uint64_t sequence=0;
  int original_source_count=0, original_inlier_count=0, sampling_stride=0;
  float original_cost=0.f;
  Eigen::Matrix4f linearization_transform=Eigen::Matrix4f::Identity();
  std::vector<GpuMatchResidual> samples;
};
}
