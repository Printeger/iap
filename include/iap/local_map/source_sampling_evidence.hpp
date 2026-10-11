#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <stdexcept>

namespace iap::local_map {

// Diagnostic observation measure, never collision or source-health authority.
// Both arrays keep original producer index order. Native means are the actual
// IMU-frame float32 matching inputs; matcher covariance is deliberately omitted.
struct SourceSamplingEvidence {
  std::string model="registered_pre_wire_returns_and_native_source_v1";
  std::string preprocessing_identity;
  Eigen::Isometry3d T_lidar_imu=Eigen::Isometry3d::Identity();
  std::vector<Eigen::Vector3d> returns_lidar;
  std::vector<Eigen::Vector3f> native_means_imu;
};

inline std::vector<Eigen::Vector3f> copyNativeMatchingMeans(
    const Eigen::Vector4d* points, const std::size_t count) {
  if(!points || count==0 || count>16384)
    throw std::invalid_argument("native source geometry missing or exceeds bound");
  std::vector<Eigen::Vector3f> copied;copied.reserve(count);
  for(std::size_t index=0;index<count;++index) {
    const Eigen::Vector3f mean=points[index].head<3>().cast<float>();
    if(!mean.allFinite())throw std::invalid_argument("nonfinite native source geometry");
    copied.push_back(mean);
  }
  return copied;
}

inline bool validSourceSamplingEvidence(const SourceSamplingEvidence& evidence) {
  if(evidence.model!="registered_pre_wire_returns_and_native_source_v1" ||
     evidence.preprocessing_identity.empty() || evidence.preprocessing_identity.size()>65536 ||
     evidence.returns_lidar.empty() || evidence.returns_lidar.size()>65536 ||
     evidence.native_means_imu.empty() || evidence.native_means_imu.size()>16384 ||
     !evidence.T_lidar_imu.matrix().allFinite() ||
     !evidence.T_lidar_imu.matrix().row(3).isApprox(Eigen::RowVector4d(0,0,0,1),1e-12) ||
     !(evidence.T_lidar_imu.linear().transpose()*evidence.T_lidar_imu.linear()).isApprox(Eigen::Matrix3d::Identity(),1e-9) ||
     std::abs(evidence.T_lidar_imu.linear().determinant()-1.)>1e-9)
    return false;
  for(const auto& point:evidence.returns_lidar) if(!point.allFinite())return false;
  for(const auto& point:evidence.native_means_imu) if(!point.allFinite())return false;
  return true;
}

}  // namespace iap::local_map
