#pragma once
// IAP-RQ-130: Trunk extension module for GLIM FGO
//
// Lifecycle:
//   1. Constructor registers on_new_frame + on_smoother_update callbacks.
//   2. on_new_frame_: runs TrunkDetector on new LiDAR frame, feeds result to
//      TrunkMap for data association, then publishes visualization markers.
//   3. on_smoother_update_: for each associated observation, inserts a
//      TrunkFactor(X(i), L(k)) into the factor graph.  New landmarks get
//      a Point2 initial value + prior.
//
// Visualization topics (RViz2):
//   ~/trunks  [visualization_msgs/msg/MarkerArray]
//     namespace "det" — current-frame detections  (yellow-green cylinders)
//     namespace "lm"  — confirmed landmark map    (bright green cylinders + ID labels)

#include <atomic>
#include <memory>
#include <mutex>
#include <unordered_set>
#include <vector>

#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <iap/trunk/trunk_detector.hpp>
#include <iap/trunk/trunk_map.hpp>
#include <iap/trunk/trunk_factor.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/util/extension_module_ros2.hpp>

namespace iap {

/**
 * @brief GLIM extension module that integrates trunk landmark observations
 *        into the factor graph (IAP-RQ-130/131/132).
 *
 * On each new LiDAR frame the module:
 *  1. Runs TrunkDetector to extract trunk observations from the point cloud.
 *  2. Feeds detections into TrunkMap for nearest-neighbour data association
 *     (landmark IDs assigned or spawned).
 *  3. In on_smoother_update: for each confirmed association, inserts a
 *     TrunkFactor connecting X(frame_id) and L(landmark_id).  New landmarks
 *     are initialised with the world-frame position + a loose prior.
 *
 * Landmark coordinate frame: world XY (same frame as Pose3 translation).
 */
class TrunkExtensionModule : public glim::ExtensionModuleROS2 {
 public:
  struct Params {
    // Noise on landmark prior [m] (loose — let the graph refine it)
    double sigma_landmark_prior = 1.0;
    // Minimum detections before inserting factors for a landmark
    int    min_confirm_count    = 2;
    // Visualization marker topic (published as ~/trunks)
    std::string marker_topic    = "trunks";
  };

  TrunkExtensionModule();
  explicit TrunkExtensionModule(const TrunkDetector::Params& det_params,
                                const TrunkMap::Params& map_params,
                                const Params& ext_params);
  TrunkExtensionModule(const TrunkDetector::Params& det_params,
                       const TrunkMap::Params& map_params)
      : TrunkExtensionModule(det_params, map_params, Params{}) {}
  ~TrunkExtensionModule() override = default;

  // ROS2 hook — creates the MarkerArray publisher
  std::vector<glim::GenericTopicSubscription::Ptr>
  create_subscriptions(rclcpp::Node& node) override;

 private:
  // ── Callback handlers ───────────────────────────────────────────────────
  void on_new_frame_(const glim::EstimationFrame::ConstPtr& frame);

  void on_smoother_update_(
      gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
      gtsam::NonlinearFactorGraph&                              new_factors,
      gtsam::Values&                                            new_values,
      std::map<std::uint64_t, double>&                          new_stamps);

  // ── Visualization ────────────────────────────────────────────────────────
  // Publishes current-frame detections (namespace "det") and confirmed
  // landmark cylinders + ID labels (namespace "lm") to ~/trunks.
  void publish_markers_(
      const std::vector<TrunkObservation>& world_trunks,  // center_xy in world frame
      double sensor_z,                                     // world-frame sensor height
      double stamp);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::string marker_frame_id_ = "map";

  // ── State ───────────────────────────────────────────────────────────────
  std::shared_ptr<spdlog::logger> logger_;

  TrunkDetector detector_;
  TrunkMap      map_;
  std::mutex    map_mutex_;  ///< guards map_ (accessed from odometry + smoother threads)
  Params        params_;

  // Latest detection result (written by on_new_frame_, read by on_smoother_update_)
  struct PendingDetection {
    long   frame_id  = -1;
    double stamp     = 0.0;
    std::vector<std::pair<int, TrunkObservation>> associations;  // (landmark_id, obs)
    Eigen::Isometry3d T_world_sensor = Eigen::Isometry3d::Identity();
  };
  mutable std::mutex pending_mutex_;
  std::vector<PendingDetection> pending_detections_;

  // Track which landmark IDs have been inserted into the graph
  std::unordered_set<int> inserted_landmarks_;

  // Frame tracking
  std::atomic<long>   last_frame_id_{-1};
  std::atomic<double> last_frame_stamp_{0.0};
  std::atomic<uint64_t> factor_count_{0};
};

}  // namespace iap
