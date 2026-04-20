// IAP-RQ-130: Trunk extension module implementation
//
// Data flow:
//   on_new_frame_()     → TrunkDetector::detect() → TrunkMap::update() → pending queue
//                       → publish_markers_()  (visualization_msgs/MarkerArray)
//   on_smoother_update_() → dequeue associations → insert TrunkFactor + L(k) priors

#include <iap/trunk/trunk_extension.hpp>

#include <spdlog/spdlog.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <iap/odometry/callbacks.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/preprocess/preprocessed_frame.hpp>
#include <iap/util/logging.hpp>
#include <iap/util/key_lifecycle_monitor.hpp>
#include <iap/util/extension_module.hpp>
#include <iap/util/shared_state.hpp>

namespace iap {

using Callbacks = glim::OdometryEstimationCallbacks;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

// ─────────────────────────────────────────────────────────────────────────────
TrunkExtensionModule::TrunkExtensionModule()
    : TrunkExtensionModule(TrunkDetector::Params{}, TrunkMap::Params{}) {}

TrunkExtensionModule::TrunkExtensionModule(
    const TrunkDetector::Params& det_params,
    const TrunkMap::Params& map_params,
    const Params& ext_params)
    : logger_(glim::create_module_logger("trunk_ext")),
      detector_(det_params),
      map_(map_params),
      params_(ext_params) {
  // Register on_new_frame: run detection + association
  Callbacks::on_new_frame.add([this](const glim::EstimationFrame::ConstPtr& f) {
    on_new_frame_(f);
  });

  // Register on_smoother_update: inject trunk factors
  Callbacks::on_smoother_update.add([this](
      gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
      gtsam::NonlinearFactorGraph&                              new_factors,
      gtsam::Values&                                            new_values,
      std::map<std::uint64_t, double>&                          new_stamps) {
    on_smoother_update_(smoother, new_factors, new_values, new_stamps);
  });

  glim::KeyLifecycleMonitor::instance().set_expected_owner('l', "trunk");

  logger_->info("TrunkExtensionModule created");
}

// ── on_new_frame: detect + associate ─────────────────────────────────────────
void TrunkExtensionModule::on_new_frame_(
    const glim::EstimationFrame::ConstPtr& frame) {
  if (!frame) return;

  // Use raw_frame (always CPU memory) for detection.
  // frame->frame may be a PointCloudGPU whose CPU points pointer is null —
  // accessing it directly causes SIGSEGV when running with GPU odometry.
  if (!frame->raw_frame || frame->raw_frame->points.empty()) return;

  last_frame_id_.store(frame->id);
  last_frame_stamp_.store(frame->stamp);

  // Wrap raw CPU points in a temporary PointCloudCPU for TrunkDetector.
  // PointCloudCPU(const std::vector<Eigen::Vector4d>&) copies into points_storage.
  const gtsam_points::PointCloudCPU cpu_cloud(frame->raw_frame->points);

  // Run trunk detection
  auto det_result = detector_.detect(cpu_cloud, frame->stamp);

  if (det_result.trunks.empty()) return;

  // World-frame sensor position (for data association in world XY)
  const Eigen::Isometry3d T_ws = frame->T_world_sensor();
  const Eigen::Vector2d sensor_xy = T_ws.translation().head<2>();

  // ── Transform trunk centres from sensor frame → world frame ──────────────
  // TrunkDetector produces centres in sensor frame; TrunkMap & TrunkFactor
  // need world-frame landmarks, so we apply the current pose estimate.
  const Eigen::Matrix2d R2 = T_ws.rotation().topLeftCorner<2, 2>();
  for (auto& trunk : det_result.trunks) {
    trunk.center_xy = R2 * trunk.center_xy + sensor_xy;
  }

  // Data association in world frame  (map_ is shared with smoother thread — lock)
  std::vector<std::pair<int, TrunkObservation>> associations;
  {
    std::lock_guard<std::mutex> lk(map_mutex_);
    associations = map_.update(det_result, sensor_xy);
  }

  // Share world-frame detection with integrity_extension for HAL computation
  IapSharedState::instance().set_trunk_detection(det_result);

  // Queue for smoother injection
  PendingDetection pd;
  pd.frame_id        = frame->id;
  pd.stamp           = frame->stamp;
  pd.associations    = std::move(associations);
  pd.T_world_sensor  = T_ws;

  {
    std::lock_guard<std::mutex> lk(pending_mutex_);
    pending_detections_.push_back(std::move(pd));
  }

  // ── Visualization: publish trunk markers ──────────────────────────────────
  publish_markers_(det_result.trunks, T_ws.translation().z(), frame->stamp);
}

// ── on_smoother_update: inject factors ───────────────────────────────────────
void TrunkExtensionModule::on_smoother_update_(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback& /*smoother*/,
    gtsam::NonlinearFactorGraph&                              new_factors,
    gtsam::Values&                                            new_values,
    std::map<std::uint64_t, double>&                          new_stamps) {

  // Drain pending detections
  std::vector<PendingDetection> detections;
  {
    std::lock_guard<std::mutex> lk(pending_mutex_);
    detections.swap(pending_detections_);
  }

  if (detections.empty()) return;

  int factors_added = 0;

  for (const auto& pd : detections) {
    const auto frame_id = static_cast<std::uint64_t>(pd.frame_id);

    for (const auto& [landmark_id, obs] : pd.associations) {
      if (landmark_id < 0) continue;  // newly spawned, not yet confirmed

      // Check if landmark is confirmed (enough sightings by TrunkMap)
      // Snapshot under lock to avoid racing with on_new_frame_ / map_.update()
      TrunkLandmark lm_copy;
      bool found = false;
      {
        std::lock_guard<std::mutex> lk(map_mutex_);
        for (const auto& l : map_.landmarks()) {
          if (l.id == landmark_id) { lm_copy = l; found = true; break; }
        }
      }
      if (!found || lm_copy.seen_count < params_.min_confirm_count) continue;
      const TrunkLandmark* lm = &lm_copy;

      const auto lm_key = L(static_cast<std::uint64_t>(landmark_id));

      // ── Insert landmark variable if first time ───────────────────────────
      if (inserted_landmarks_.find(landmark_id) == inserted_landmarks_.end()) {
        const gtsam::Point2 init_pos(lm->center_xy.x(), lm->center_xy.y());

        if (!new_values.exists(lm_key)) {
          new_values.insert(lm_key, init_pos);
          glim::KeyLifecycleMonitor::instance().record_write('l', "trunk");
        }

        // Loose prior on landmark position
        new_factors.addPrior<gtsam::Point2>(
            lm_key, init_pos,
            gtsam::noiseModel::Isotropic::Sigma(2, params_.sigma_landmark_prior));

        inserted_landmarks_.insert(landmark_id);
        logger_->debug("[trunk_ext] L({}) inserted: [{:.2f}, {:.2f}]",
                       landmark_id, init_pos.x(), init_pos.y());
      }

      // Keep landmark alive in the fixed-lag window
      new_stamps[lm_key] = pd.stamp;

      // ── Compute sensor-frame observation from world-frame centre ────────
      // obs.center_xy is in world frame (transformed in on_new_frame_);
      // TrunkFactor expects sensor-frame measurement.
      const Eigen::Matrix2d R2 = pd.T_world_sensor.rotation().topLeftCorner<2, 2>();
      const Eigen::Vector2d p_xy = pd.T_world_sensor.translation().head<2>();
      const Eigen::Vector2d meas_sensor = R2.transpose() * (obs.center_xy - p_xy);

      // Build noise model from confidence
      auto noise = TrunkFactor::make_noise(obs.confidence);

      // Add factor
      new_factors.emplace_shared<TrunkFactor>(
          X(frame_id), lm_key, meas_sensor, noise);
      ++factors_added;
    }
  }

  if (factors_added > 0) {
    const uint64_t n = factor_count_.fetch_add(factors_added);
    if (n == 0 || (n + factors_added) % 50 < static_cast<uint64_t>(factors_added)) {
      logger_->info("[trunk_ext] +{} trunk factors (total={})", factors_added, n + factors_added);
    } else {
      logger_->debug("[trunk_ext] +{} trunk factors (total={})", factors_added, n + factors_added);
    }
  }

  if (!detections.empty()) {
    glim::KeyLifecycleMonitor::instance().maybe_log(logger_, detections.back().stamp, 400, 5.0);
  }

  // Publish confirmed landmark count for integrity_extension
  IapSharedState::instance().set_n_confirmed_trunks(
      static_cast<int>(inserted_landmarks_.size()));
}

// ── create_subscriptions: create the MarkerArray publisher ───────────────────
std::vector<glim::GenericTopicSubscription::Ptr>
TrunkExtensionModule::create_subscriptions(rclcpp::Node& node) {
  using MA = visualization_msgs::msg::MarkerArray;
  const std::string topic = "~/" + params_.marker_topic;
  marker_pub_ = node.create_publisher<MA>(topic, rclcpp::QoS(1).transient_local());
  logger_->info("[trunk_ext] visualization publisher created → {}", topic);
  return {};  // no subscriptions needed
}

// ── publish_markers_: current detections + confirmed landmark map ─────────────
void TrunkExtensionModule::publish_markers_(
    const std::vector<TrunkObservation>& world_trunks,
    double sensor_z,
    double stamp) {
  if (!marker_pub_) return;

  using Marker = visualization_msgs::msg::Marker;
  using MA     = visualization_msgs::msg::MarkerArray;

  MA msg;

  // ── 0. Delete all old markers in both namespaces ─────────────────────────
  {
    Marker del;
    del.header.frame_id = marker_frame_id_;
    del.action          = Marker::DELETEALL;
    del.ns              = "det";
    msg.markers.push_back(del);
    del.ns = "lm";
    msg.markers.push_back(del);
    del.ns = "lm_label";
    msg.markers.push_back(del);
  }

  const rclcpp::Time ros_stamp(static_cast<int32_t>(stamp),
                                static_cast<uint32_t>((stamp - std::floor(stamp)) * 1e9));

  // ── 1. Current-frame detections (namespace "det", yellow-green cylinders) ─
  int det_id = 0;
  for (const auto& t : world_trunks) {
    const double height = t.z_max - t.z_min;
    const double center_z = sensor_z + (t.z_min + t.z_max) * 0.5;

    Marker m;
    m.header.frame_id    = marker_frame_id_;
    m.header.stamp       = ros_stamp;
    m.ns                 = "det";
    m.id                 = det_id++;
    m.type               = Marker::CYLINDER;
    m.action             = Marker::ADD;
    m.pose.position.x    = t.center_xy.x();
    m.pose.position.y    = t.center_xy.y();
    m.pose.position.z    = center_z;
    m.pose.orientation.w = 1.0;
    m.scale.x            = t.radius * 2.0;   // diameter
    m.scale.y            = t.radius * 2.0;
    m.scale.z            = std::max(height, 0.1);
    // Yellow-green, alpha scaled by confidence
    m.color.r = 0.6f;
    m.color.g = 1.0f;
    m.color.b = 0.1f;
    m.color.a = static_cast<float>(0.4 + 0.5 * t.confidence);
    m.lifetime = rclcpp::Duration(0, static_cast<uint32_t>(1.0 * 1e9));  // 1s ttl

    msg.markers.push_back(m);
  }

  // ── 2. Confirmed landmarks (namespace "lm", bright green cylinders + labels) ─
  // Snapshot under lock to avoid racing with on_new_frame_ / map_.update()
  std::vector<TrunkLandmark> lm_snapshot;
  {
    std::lock_guard<std::mutex> lk(map_mutex_);
    for (const auto* lm : map_.confirmed_landmarks()) {
      lm_snapshot.push_back(*lm);
    }
  }
  for (const auto& lm_ref : lm_snapshot) {
    const TrunkLandmark* lm = &lm_ref;
      // Cylinder
      Marker m;
      m.header.frame_id    = marker_frame_id_;
      m.header.stamp       = ros_stamp;
      m.ns                 = "lm";
      m.id                 = lm->id;
      m.type               = Marker::CYLINDER;
      m.action             = Marker::ADD;
      m.pose.position.x    = lm->center_xy.x();
      m.pose.position.y    = lm->center_xy.y();
      m.pose.position.z    = sensor_z + 1.0;   // fixed 0→2 m AGL (world frame)
      m.pose.orientation.w = 1.0;
      m.scale.x            = lm->radius * 2.0;
      m.scale.y            = lm->radius * 2.0;
      m.scale.z            = 2.0;
      // Bright green, more opaque when seen more often
      const float alpha = std::min(0.9f, 0.5f + 0.04f * static_cast<float>(lm->seen_count));
      m.color.r = 0.0f;
      m.color.g = 1.0f;
      m.color.b = 0.3f;
      m.color.a = alpha;
      m.lifetime = rclcpp::Duration(0, 0);  // persistent
      msg.markers.push_back(m);

      // ID label
      Marker label = m;
      label.ns     = "lm_label";
      label.type   = Marker::TEXT_VIEW_FACING;
      label.text   = "L" + std::to_string(lm->id);
      label.pose.position.z = sensor_z + 2.4;  // above cylinder
      label.scale.x = 0.0;
      label.scale.y = 0.0;
      label.scale.z = 0.3;  // text height [m]
      label.color.r = 1.0f;
      label.color.g = 1.0f;
      label.color.b = 1.0f;
      label.color.a = 0.9f;
      msg.markers.push_back(label);
  }

  marker_pub_->publish(msg);
}

}  // namespace iap

// ── Extension module entry point (required by GLIM dynamic loader) ────────────
extern "C" glim::ExtensionModule* create_extension_module() {
  return new iap::TrunkExtensionModule();
}
