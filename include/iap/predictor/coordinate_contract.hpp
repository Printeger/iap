#pragma once
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <string>

namespace iap {
// T_A_B maps B coordinates into A. Values come from one post-optimization
// estimate, never simulation truth. Horizontal/vertical PL remains map XY/Z.
struct AdvisoryCoordinateContract {
  bool valid = false;
  std::string failure_reason;
  std::int64_t frame_id = -1;
  double stamp = 0.;
  std::uint64_t epoch_source_identity = 0;
  Eigen::Vector3d enu_origin_ecef = Eigen::Vector3d::Zero();
  Eigen::Vector3d anchor_ecef = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_ecef_enu = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_ecef_world = Eigen::Matrix3d::Identity();
  Eigen::Matrix4d T_map_world = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_world_imu = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_lidar_imu = Eigen::Matrix4d::Identity();
  Eigen::Vector3d lever_arm_imu = Eigen::Vector3d::Zero();
  std::string map_frame = "map";
  std::string body_frame = "imu";
  std::string time_contract = "ros_utc_gpst2utc_scan_pose_v1";

  static bool rotation(const Eigen::Matrix3d& R) {
    return R.allFinite() && (R.transpose()*R-Eigen::Matrix3d::Identity()).norm()<1e-9 &&
           std::abs(R.determinant()-1.)<1e-9;
  }
  static bool rigid(const Eigen::Matrix4d& T) {
    return T.allFinite() && rotation(T.topLeftCorner<3,3>()) &&
           (T.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()<1e-12;
  }
  std::string rejection() const {
    if(!valid) return failure_reason.empty() ? "gnss_coordinate_unavailable" : "gnss_coordinate_unavailable:"+failure_reason;
    if(frame_id<0 || !std::isfinite(stamp) || !epoch_source_identity ||
       !enu_origin_ecef.allFinite() || !anchor_ecef.allFinite() || !lever_arm_imu.allFinite() ||
       !rotation(R_ecef_enu) || !rotation(R_ecef_world) || !rigid(T_map_world) ||
       !rigid(T_world_imu) || !rigid(T_lidar_imu) || map_frame.empty() || body_frame.empty() ||
       time_contract!="ros_utc_gpst2utc_scan_pose_v1") return "gnss_coordinate_invalid";
    return {};
  }
  std::uint64_t identity() const {
    std::uint64_t hash=1469598103934665603ULL;
    const auto append=[&](const auto& value) {
      const auto* bytes=reinterpret_cast<const unsigned char*>(&value);
      for(std::size_t i=0;i<sizeof(value);++i) {hash^=bytes[i];hash*=1099511628211ULL;}
    };
    append(valid);append(frame_id);append(stamp);append(epoch_source_identity);
    const auto matrix=[&](const auto& value) {for(int i=0;i<value.size();++i) append(value.data()[i]);};
    matrix(enu_origin_ecef);matrix(anchor_ecef);matrix(R_ecef_enu);matrix(R_ecef_world);
    matrix(T_map_world);matrix(T_world_imu);matrix(T_lidar_imu);matrix(lever_arm_imu);
    for(const auto* value:{&map_frame,&body_frame,&time_contract,&failure_reason}) {
      append(value->size());for(const auto ch:*value) append(ch);
    }
    return hash;
  }
  Eigen::Matrix3d R_map_enu() const {
    return T_map_world.topLeftCorner<3,3>()*R_ecef_world.transpose()*R_ecef_enu;
  }
  Eigen::Vector3d antenna_offset_map() const {
    return T_map_world.topLeftCorner<3,3>()*T_world_imu.topLeftCorner<3,3>()*lever_arm_imu;
  }
};
}
