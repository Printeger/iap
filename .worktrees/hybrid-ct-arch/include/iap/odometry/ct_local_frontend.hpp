#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Local continuous-time frontend interface for the planned hybrid architecture.
// This boundary owns LiDAR/IMU local solve inputs and returns only compact
// backend handoff state, without exposing dense frontend factor internals.

#include <iap/odometry/ct_backend_summary.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/util/raw_points.hpp>

#include <vector>

namespace iap {

class CTLocalFrontend {
 public:
  struct Input {
    glim::EstimationFrame::ConstPtr target_frame;
    std::vector<glim::RawPoints::ConstPtr> source_frames;
    std::size_t imu_sample_count{0};
  };

  CTLocalFrontendResult run(const Input& input) const;
};

}  // namespace iap
