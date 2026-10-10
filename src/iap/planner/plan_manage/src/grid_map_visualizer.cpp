#include <ego_planner/risk_display.h>
#include <iap/srv/get_grid_map_prediction_input.hpp>
#include <iap/util/run_log_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <sys/resource.h>
#include <unistd.h>
namespace ego_planner {
namespace {
uint32_t riskColor(double value, double minimum, double maximum) {
  const double t = std::clamp((value - minimum) / (maximum - minimum), 0.0, 1.0);
  // Keep the middle of the scale chromatic: linear blue-to-yellow RGB
  // interpolation passes through gray and hides the small PL differences.
  constexpr std::array<std::array<double, 3>, 4> stops{{
      {{25, 65, 190}}, {{35, 190, 225}},
      {{250, 220, 55}}, {{225, 50, 40}}}};
  const double scaled = t * (stops.size() - 1);
  const size_t left = std::min(static_cast<size_t>(scaled), stops.size() - 2);
  const double blend = scaled - left;
  auto channel = [&](size_t index) {
    return static_cast<uint32_t>(std::lround(
        stops[left][index] * (1.0 - blend) + stops[left + 1][index] * blend));
  };
  return (channel(0) << 16) | (channel(1) << 8) | channel(2);
}

std_msgs::msg::ColorRGBA markerColor(uint32_t packed, float alpha) {
  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<float>((packed >> 16) & 0xffu) / 255.0f;
  color.g = static_cast<float>((packed >> 8) & 0xffu) / 255.0f;
  color.b = static_cast<float>(packed & 0xffu) / 255.0f;
  color.a = alpha;
  return color;
}

visualization_msgs::msg::Marker markerBase(
    const std::string& frame, const rclcpp::Time& stamp,
    const std::string& ns, int id, int type, double lifetime_s) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color.a = 1.0;
  marker.lifetime = rclcpp::Duration::from_seconds(lifetime_s);
  return marker;
}

geometry_msgs::msg::Point markerPoint(const Eigen::Vector3d& point, double z_offset) {
  geometry_msgs::msg::Point out;
  out.x = point.x();
  out.y = point.y();
  out.z = point.z() + z_offset;
  return out;
}

sensor_msgs::msg::PointCloud2 makeCloud(
    const std::vector<SlicePoint>& samples, const std::string& frame,
    const rclcpp::Time& stamp, const std::string& metric,
    double hpl_min, double hpl_max, double vpl_min, double vpl_max) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame;
  cloud.header.stamp = stamp;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(7,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32,
      "hpl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "vpl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "status", 1, sensor_msgs::msg::PointField::UINT8);
  modifier.resize(samples.size());
  cloud.is_dense = false; // Invalid PL fields are NaN, while XYZ is finite.
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> rgb(cloud, "rgb"), hpl(cloud, "hpl"), vpl(cloud, "vpl");
  sensor_msgs::PointCloud2Iterator<uint8_t> status(cloud, "status");
  for (const auto& sample : samples) {
    *x = static_cast<float>(sample.center.x());
    *y = static_cast<float>(sample.center.y());
    // Project only the display geometry. PL and visibility still use the
    // original slice voxel at flight height (or the configured fixed slice).
    *z = 0.0f;
    *hpl = static_cast<float>(sample.risk.hpl);
    *vpl = static_cast<float>(sample.risk.vpl);
    *status = static_cast<uint8_t>(sample.risk.status);
    const uint32_t packed = sample.risk.status == GridRiskStatus::VALID
        ? riskColor(metric == "hpl" ? sample.risk.hpl : sample.risk.vpl,
                    metric == "hpl" ? hpl_min : vpl_min,
                    metric == "hpl" ? hpl_max : vpl_max)
        : 0x9900ccu;
    std::memcpy(&*rgb, &packed, sizeof(packed));
    ++x; ++y; ++z; ++rgb; ++hpl; ++vpl; ++status;
  }
  return cloud;
}

void appendSurfaceTriangle(visualization_msgs::msg::Marker& marker,
                           const SlicePoint& a, const SlicePoint& b,
                           const SlicePoint& c, const std::string& metric,
                           double minimum, double maximum) {
  for (const SlicePoint* sample : {&a, &b, &c}) {
    auto point = markerPoint(sample->center, 0.0);
    point.z = 0.0;
    marker.points.push_back(point);
    const double value = metric == "hpl" ? sample->risk.hpl : sample->risk.vpl;
    marker.colors.push_back(markerColor(riskColor(value, minimum, maximum), 0.42f));
  }
}

void appendBarVertex(visualization_msgs::msg::Marker& marker,
                     double x, double y, double z, uint32_t rgb) {
  geometry_msgs::msg::Point point;
  point.x = x; point.y = y; point.z = z;
  marker.points.push_back(point);
  marker.colors.push_back(markerColor(rgb, 0.95f));
}

visualization_msgs::msg::MarkerArray makeLegend(
    const std::string& frame, const rclcpp::Time& stamp,
    const Eigen::Vector3d& corner, const std::string& metric,
    double minimum, double maximum, double retention_s) {
  visualization_msgs::msg::MarkerArray array;
  auto bar = markerBase(frame, stamp, "risk_legend", 0,
                        visualization_msgs::msg::Marker::TRIANGLE_LIST, 1.5);
  bar.scale.x = bar.scale.y = bar.scale.z = 1.0;
  for (int i = 0; i < 12; ++i) {
    const double x0 = corner.x() + 0.25 * i;
    const double x1 = x0 + 0.25;
    const double y0 = corner.y(), y1 = y0 + 0.20, z = corner.z();
    const uint32_t left = riskColor(minimum + (maximum - minimum) * i / 12.0,
                                    minimum, maximum);
    const uint32_t right = riskColor(minimum + (maximum - minimum) * (i + 1) / 12.0,
                                     minimum, maximum);
    appendBarVertex(bar, x0, y0, z, left);
    appendBarVertex(bar, x1, y0, z, right);
    appendBarVertex(bar, x1, y1, z, right);
    appendBarVertex(bar, x0, y0, z, left);
    appendBarVertex(bar, x1, y1, z, right);
    appendBarVertex(bar, x0, y1, z, left);
  }
  array.markers.push_back(std::move(bar));
  for (int i = 0; i < 3; ++i) {
    auto label = markerBase(frame, stamp, "risk_legend", i + 1,
                            visualization_msgs::msg::Marker::TEXT_VIEW_FACING, 1.5);
    label.pose.position.x = corner.x() + 1.5 * i;
    label.pose.position.y = corner.y() - 0.26;
    label.pose.position.z = corner.z() + 0.05;
    label.scale.z = 0.24;
    label.color = markerColor(0xffffffu, 1.0f);
    std::ostringstream text;
    text << std::fixed << std::setprecision(2)
         << minimum + (maximum - minimum) * i / 2.0 << " m";
    label.text = text.str();
    array.markers.push_back(std::move(label));
  }
  auto title = markerBase(frame, stamp, "risk_legend", 4,
                          visualization_msgs::msg::Marker::TEXT_VIEW_FACING, 1.5);
  title.pose.position.x = corner.x() + 1.5;
  title.pose.position.y = corner.y() - 0.55;
  title.pose.position.z = corner.z() + 0.05;
  title.scale.z = 0.18;
  title.color = markerColor(0xffffffu, 1.0f);
  std::ostringstream text;
  text << (metric == "hpl" ? "HPL" : "VPL")
       << ": slice PL projected to z=0, dots predicted, surface interpolated, persists "
       << std::fixed << std::setprecision(0) << retention_s << " s";
  title.text = text.str();
  array.markers.push_back(std::move(title));
  return array;
}


visualization_msgs::msg::MarkerArray drawSurfaces(const RiskDisplayFrame& data,
    const rclcpp::Time& stamp, const std::string& metric, double minimum, double maximum, bool paint) {
  const double remaining=data.expires_at_s-stamp.seconds();
  auto surface=markerBase(data.frame_id,stamp,"risk_surface",data.id,
      visualization_msgs::msg::Marker::TRIANGLE_LIST,std::max(.001,remaining));
  surface.scale.x=surface.scale.y=surface.scale.z=1;
  if (paint) for (int x=0;x<9;++x) for(int y=0;y<9;++y) if (data.surfaces[x*9+y]) {
    const auto& a=data.samples[data.lattice[x*10+y]];
    const auto& b=data.samples[data.lattice[(x+1)*10+y]];
    const auto& c=data.samples[data.lattice[x*10+y+1]];
    const auto& d=data.samples[data.lattice[(x+1)*10+y+1]];
    appendSurfaceTriangle(surface,a,b,d,metric,minimum,maximum);
    appendSurfaceTriangle(surface,a,d,c,metric,minimum,maximum);
  }
  if (surface.points.empty()) surface.action=visualization_msgs::msg::Marker::DELETE;
  auto age=markerBase(data.frame_id,stamp,"risk_history_age",data.id,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING,std::max(.001,remaining));
  age.pose.position=markerPoint(data.vehicle,0.0); age.pose.position.z=.3; age.scale.z=.16;
  age.color=markerColor(0xffffff,1);
  std::ostringstream label;
  label<<"historical PL ref="<<std::fixed<<std::setprecision(3)<<data.reference_time_s
       <<" age="<<std::setprecision(1)<<stamp.seconds()-data.reference_time_s<<" s";
  age.text=label.str();
  visualization_msgs::msg::MarkerArray out; if (paint) out.markers.push_back(std::move(surface));
  out.markers.push_back(std::move(age)); return out;
}
}

class GridMapVisualizer : public rclcpp::Node {
 public:
  GridMapVisualizer() : Node("grid_map_visualizer") {
    metric_=declare_parameter<std::string>("risk_viz/metric","hpl");
    z_mode_=declare_parameter<std::string>("risk_viz/z_mode","follow");
    fixed_z_=declare_parameter("risk_viz/fixed_z_m",1.5);
    hmin_=declare_parameter("risk_viz/hpl_min_m",.25); hmax_=declare_parameter("risk_viz/hpl_max_m",.65);
    vmin_=declare_parameter("risk_viz/vpl_min_m",.20); vmax_=declare_parameter("risk_viz/vpl_max_m",.55);
    retention_=declare_parameter("risk_viz/surface_lifetime_s",60.0);
    anchor_step_=declare_parameter("risk_viz/surface_snapshot_step_m",4.0);
    if ((metric_!="hpl"&&metric_!="vpl") || (z_mode_!="follow"&&z_mode_!="fixed") ||
        !std::isfinite(fixed_z_) || !std::isfinite(hmin_) || !std::isfinite(hmax_) || hmax_<=hmin_ ||
        !std::isfinite(vmin_) || !std::isfinite(vmax_) || vmax_<=vmin_ ||
        !std::isfinite(retention_) || retention_<=0 || retention_>60 ||
        !std::isfinite(anchor_step_) || anchor_step_<=0) throw std::invalid_argument("invalid risk_viz display parameters");
    cloud_=create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/risk_slice",1);
    status_=create_publisher<visualization_msgs::msg::Marker>("grid_map/risk_status",1);
    surface_=create_publisher<visualization_msgs::msg::MarkerArray>("grid_map/risk_surface",1);
    legend_=create_publisher<visualization_msgs::msg::MarkerArray>("grid_map/risk_legend",1);
    path_=create_publisher<nav_msgs::msg::Path>("grid_map/glio_path",1);
    input_=create_client<iap::srv::GetGridMapPredictionInput>("grid_map/prediction_input");
    parameter_callback_=add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& values){
      rcl_interfaces::msg::SetParametersResult result; result.successful=true;
      for (const auto& value:values) if(value.get_name()=="risk_viz/metric") {
        if (value.get_type()!=rclcpp::ParameterType::PARAMETER_STRING ||
            (value.as_string()!="hpl"&&value.as_string()!="vpl")) {
          result.successful=false; result.reason="risk_viz/metric must be hpl or vpl"; return result;
        }
      }
      for (const auto& value:values) if(value.get_name()=="risk_viz/metric") { metric_=value.as_string(); repaint_=true; }
      return result;
    });
    odom_=create_subscription<nav_msgs::msg::Odometry>("odom_world",50,[this](nav_msgs::msg::Odometry::ConstSharedPtr msg){
      if (geometry_frame_.empty() || msg->header.frame_id!=geometry_frame_) return;
      const auto& p=msg->pose.pose.position; const Eigen::Vector3d point(p.x,p.y,p.z);
      if (!point.allFinite()) return;
      if (!glio_path_.poses.empty()) {
        const auto& previous=glio_path_.poses.back().pose.position;
        if ((point-Eigen::Vector3d(previous.x,previous.y,previous.z)).norm()<.05) return;
      }
      glio_path_.header=msg->header; geometry_msgs::msg::PoseStamped pose; pose.header=msg->header; pose.pose=msg->pose.pose;
      glio_path_.poses.push_back(pose);
      if(glio_path_.poses.size()>500) glio_path_.poses.erase(glio_path_.poses.begin());
      path_->publish(glio_path_);
    });
    if (const auto log=glim::RunLogManager::get_if_initialized()) {
      const auto name="grid_map_visualizer_"+std::to_string(getpid());
      metrics_.open(log->profiling_path(name+".csv"));
      metrics_<<"reference_time,generation,preparation_s,queries,query_s,budget_s,overrun_s,preparation_overrun_s,scalar_overrun_s,incomplete,pending_high_water,replaced_inputs,cpu_s,peak_rss_kib\n";
      std::ofstream manifest(log->metadata_path("manifests/"+name+".json"));
      manifest<<"{\"schema\":\"iap_grid_map_visualizer_v1\",\"module\":\"grid_map_visualizer\",\"artifacts\":[\"profiling/"<<name<<".csv\"],\"cpu_limit\":null,\"cpu_affinity\":null}\n";
    }
    clear_service_=create_service<std_srvs::srv::Trigger>("grid_map/clear_risk_history",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        ++cancellation_;
        { std::lock_guard<std::mutex> lock(mutex_); pending_.reset(); }
        clearHistory(); response->success=true; response->message="display history cleared";
      });
    worker_=std::thread([this](){ work(); });
    request_timer_=create_wall_timer(std::chrono::seconds(1),[this](){ request(); });
    display_timer_=create_wall_timer(std::chrono::milliseconds(200),[this](){ display(); });
  }
  ~GridMapVisualizer() override {
    { std::lock_guard<std::mutex> lock(mutex_); stopping_=true; pending_.reset(); ++cancellation_; }
    wake_.notify_one(); if(worker_.joinable()) worker_.join();
  }
 private:
  using Clock=std::chrono::steady_clock;
  struct Pending { std::vector<uint8_t> payload; uint64_t cancellation=0; };
  void request() {
    if (request_in_flight_ && Clock::now()-request_started_>std::chrono::seconds(3)) {
      input_->remove_pending_request(request_id_); request_in_flight_=false;
      latest_prediction_valid_=false; reason_="read-only export request timed out";
    }
    if (request_in_flight_ || !input_->service_is_ready()) return;
    request_in_flight_=true; request_started_=Clock::now();
    const auto token=++request_token_;
    auto request=std::make_shared<iap::srv::GetGridMapPredictionInput::Request>();
    // Display the latest retained planning identity. Freezing a fresh map for
    // every display tick takes the sensor/update mutex on the execution path.
    // Missing retained input is reported by the existing service; a display
    // must not fall back to a new live-map freeze.
    request->planning_input=true;
    const auto handle=input_->async_send_request(request,
      [this,token](rclcpp::Client<iap::srv::GetGridMapPredictionInput>::SharedFuture future){
        if (token!=request_token_) return;
        request_in_flight_=false;
        try {
          auto value=future.get();
          if(!value->available) { reason_=value->reason; latest_prediction_valid_=false; return; }
          const bool geometry_changed=!geometry_id_.empty() && geometry_id_!=value->geometry_id;
          geometry_frame_=value->frame_id; geometry_id_=value->geometry_id; latest_generation_=value->generation;
          if (geometry_changed) { ++cancellation_; clearHistory(); }
          { std::lock_guard<std::mutex> lock(mutex_);
            if (pending_) ++replaced_inputs_;
            pending_=Pending{std::move(value->payload),cancellation_.load()}; }
          wake_.notify_one();
        } catch(const std::exception& e) { reason_=e.what(); latest_prediction_valid_=false; }
      });
    request_id_=handle.request_id;
  }
  void work() {
    std::deque<double> preparations,points;
    uint64_t last_identity=0, last_cancellation=0;
    while(true) {
      Pending task;
      { std::unique_lock<std::mutex> lock(mutex_); wake_.wait(lock,[&](){return stopping_||pending_.has_value();});
        if(stopping_) return;
        task=std::move(*pending_); pending_.reset(); }
      const auto started=Clock::now();
      bool calibrating=points.empty();
      // One startup scalar probe establishes point timing on this same input.
      // Its result is retained in the ordinary sample table, never repeated.
      RiskDisplayFrame frame; frame.budget_s=calibrating ? .2 : visualizationBudget(preparations,points);
      try {
        auto input=decodePredictionInput(task.payload);
        if(task.cancellation!=cancellation_) continue;
        if(task.cancellation!=last_cancellation) { last_identity=0; last_cancellation=task.cancellation; }
        const auto identity=predictionInputIdentity(input);
        if(identity==last_identity) continue;
        last_identity=identity;
        frame.vehicle=input.integrity.p_wb; frame.generation=input.occupancy->generation;
        frame.frame_id=input.occupancy->frame_id; frame.geometry_id=input.occupancy->geometry_id;
        frame.reference_time_s=input.reference_time_s; frame.expires_at_s=frame.reference_time_s+retention_;
        // Thread count only changes scheduling; the exported mathematical
        // model and all safety/advisory thresholds are preserved.
        input.params.gnss.geometry_params.hypothesis_threads=1;
        auto calls=std::make_shared<std::atomic<uint64_t>>(0);
        auto predictor=makeRiskPrediction(input,calls);
        frame.valid_until_s=predictor.valid_until_s;
        const auto prepared=Clock::now(); frame.preparation_s=std::chrono::duration<double>(prepared-started).count();
        preparations.push_back(frame.preparation_s); if(preparations.size()>32) preparations.pop_front();
        frame.preparation_overrun_s=std::max(0.0,frame.preparation_s-frame.budget_s);
        frame.samples.reserve(100);
        const auto expired=[&](){return std::chrono::duration<double>(Clock::now()-started).count()>=frame.budget_s;};
        if (!predictor.predict || !frame.vehicle.allFinite() || !std::isfinite(predictor.valid_until_s) ||
            input.reference_time_s>predictor.valid_until_s) {
          frame.budget_s=visualizationBudget(preparations,points);
          frame.reason="prediction inputs unavailable or expired";
        }
        else {
          double z=z_mode_=="follow" ? frame.vehicle.z() : fixed_z_;
          const auto z_cell=GridMap::queryFrozenOccupancy(*input.occupancy,Eigen::Vector3d(frame.vehicle.x(),frame.vehicle.y(),z));
          if(!z_cell.available) frame.reason="slice outside GridMap";
          else {
            z=input.occupancy->lattice_origin.z()+(z_cell.voxel_index.z()+.5)*input.occupancy->resolution_m;
            std::unordered_map<int,GridRiskVoxel> queried;
            for(int x=0;x<10;++x) for(int y=0;y<10;++y) {
              if(task.cancellation!=cancellation_) break;
              if(expired()) {frame.incomplete=true; break;}
              const Eigen::Vector3d p(frame.vehicle.x()+x-4.5,frame.vehicle.y()+y-4.5,z);
              const auto physical=GridMap::queryFrozenOccupancy(*input.occupancy,p);
              if(!physical.available || !physical.observed || physical.raw_occupied || physical.inflated_occupied) continue;
              const auto center=input.occupancy->lattice_origin+(physical.voxel_index.cast<double>().array()+.5).matrix()*input.occupancy->resolution_m;
              const int address=(physical.voxel_index.x()*input.occupancy->voxel_dimensions.y()+physical.voxel_index.y())*input.occupancy->voxel_dimensions.z()+physical.voxel_index.z();
              auto result=queried.find(address);
              if(result==queried.end()) {
                const auto point_started=Clock::now();
                const auto risk=predictor.predict(center);
                const double point_s=std::chrono::duration<double>(Clock::now()-point_started).count();
                points.push_back(point_s); if(points.size()>256) points.pop_front(); frame.query_s+=point_s;
                if(calibrating) { frame.budget_s=visualizationBudget(preparations,points); calibrating=false; }
                frame.scalar_overrun_s=std::max(frame.scalar_overrun_s,
                    std::max(0.0,std::chrono::duration<double>(Clock::now()-started).count()-frame.budget_s));
                result=queried.emplace(address,risk).first;
              }
              frame.lattice[x*10+y]=frame.samples.size(); frame.samples.push_back({center,result->second});
            }
            for(int x=0;x<9;++x) for(int y=0;y<9;++y) {
              const std::array<int,4> ids{frame.lattice[x*10+y],frame.lattice[(x+1)*10+y],frame.lattice[x*10+y+1],frame.lattice[(x+1)*10+y+1]};
              bool valid=std::all_of(ids.begin(),ids.end(),[&](int i){return i>=0&&frame.samples[i].risk.status==GridRiskStatus::VALID;});
              frame.surfaces[x*9+y]=valid&&observedSurfaceRectangle(*input.occupancy,frame.samples[ids[0]].center,frame.samples[ids[3]].center);
            }
          }
        }
        frame.predictor_calls=calls->load();
        const double elapsed=std::chrono::duration<double>(Clock::now()-started).count();
        frame.preparation_overrun_s=std::max(0.0,frame.preparation_s-frame.budget_s);
        frame.overrun_s=std::max(0.0,elapsed-frame.budget_s);
        frame.incomplete|=elapsed>=frame.budget_s;
        if(task.cancellation!=cancellation_) continue;
        struct rusage usage{}; getrusage(RUSAGE_SELF,&usage);
        { std::lock_guard<std::mutex> lock(mutex_);
          if(task.cancellation!=cancellation_) continue;
          if(metrics_) { metrics_<<std::setprecision(12)<<frame.reference_time_s<<','<<frame.generation<<','<<frame.preparation_s<<','<<frame.predictor_calls<<','<<frame.query_s<<','<<frame.budget_s<<','<<frame.overrun_s<<','<<frame.preparation_overrun_s<<','<<frame.scalar_overrun_s<<','<<frame.incomplete<<",1,"<<replaced_inputs_<<','<<usage.ru_utime.tv_sec+usage.ru_utime.tv_usec*1e-6+usage.ru_stime.tv_sec+usage.ru_stime.tv_usec*1e-6<<','<<usage.ru_maxrss<<'\n'; metrics_.flush(); }
          completed_=std::move(frame); }
      } catch(const std::exception& e) {
        frame.reason=e.what(); std::lock_guard<std::mutex> lock(mutex_);
        if(task.cancellation==cancellation_) completed_=std::move(frame);
      }
    }
  }
  void clearHistory() {
    if (request_in_flight_) {
      input_->remove_pending_request(request_id_); request_in_flight_=false; ++request_token_;
    }
    {std::lock_guard<std::mutex> lock(mutex_); completed_.reset();}
    history_.clear(); glio_path_.poses.clear(); latest_prediction_valid_=false;
    const auto stamp=now();
    cloud_->publish(makeCloud({},geometry_frame_,stamp,metric_,hmin_,hmax_,vmin_,vmax_));
    glio_path_.header.frame_id=geometry_frame_; glio_path_.header.stamp=stamp; path_->publish(glio_path_);
    visualization_msgs::msg::MarkerArray clear; visualization_msgs::msg::Marker marker;
    marker.action=visualization_msgs::msg::Marker::DELETEALL; clear.markers.push_back(marker); surface_->publish(clear);
  }
  void display() {
    const auto stamp=now();
    if(last_display_time_>=0 && stamp.seconds()<last_display_time_) {
      ++cancellation_; {std::lock_guard<std::mutex> lock(mutex_); pending_.reset(); completed_.reset();}
      clearHistory();
    }
    last_display_time_=stamp.seconds();
    std::optional<RiskDisplayFrame> frame;
    {std::lock_guard<std::mutex> lock(mutex_); frame=std::move(completed_); completed_.reset();}
    int updated_id=-1;
    if(frame) {
      latest_prediction_valid_=std::any_of(frame->samples.begin(),frame->samples.end(),
          [](const SlicePoint& p){return p.risk.status==GridRiskStatus::VALID;});
      reason_=frame->reason;
      cloud_->publish(makeCloud(frame->samples,frame->frame_id,
          rclcpp::Time(static_cast<int64_t>(frame->reference_time_s*1e9),stamp.get_clock_type()),metric_,hmin_,hmax_,vmin_,vmax_));
      if(std::any_of(frame->samples.begin(),frame->samples.end(),[](const SlicePoint& p){return p.risk.status==GridRiskStatus::VALID;}) && frame->vehicle.allFinite()) {
        if(history_.empty() || (frame->vehicle-history_.back().vehicle).norm()>=anchor_step_) {
          frame->id=next_id_++; history_.push_back(std::move(*frame));
        } else if (std::count(frame->surfaces.begin(),frame->surfaces.end(),true) <
                   std::count(history_.back().surfaces.begin(),history_.back().surfaces.end(),true)) {
          frame->id=next_id_++; history_.push_back(std::move(*frame));
        } else { frame->id=history_.back().id; history_.back()=std::move(*frame); }
        updated_id=history_.back().id;
      }
    }
    while(history_.size()>61) history_.pop_front();
    while(!history_.empty() && history_.front().expires_at_s<=stamp.seconds()) history_.pop_front();
    // Each retained input has an immutable expiry. Recolouring sets only the
    // remaining lifetime; it cannot renew an old map or run prediction.
    const double minimum=metric_=="hpl"?hmin_:vmin_, maximum=metric_=="hpl"?hmax_:vmax_;
    if(history_.empty()) surface_->publish(visualization_msgs::msg::MarkerArray{});
    if(updated_id>=0 || repaint_ || stamp.seconds()-last_age_refresh_>=1.0) {
      for(const auto& data:history_) surface_->publish(drawSurfaces(data,stamp,metric_,minimum,maximum,repaint_||data.id==updated_id));
      if(repaint_&&!history_.empty()) cloud_->publish(makeCloud(history_.back().samples,history_.back().frame_id,
          rclcpp::Time(static_cast<int64_t>(history_.back().reference_time_s*1e9),stamp.get_clock_type()),metric_,hmin_,hmax_,vmin_,vmax_));
      repaint_=false; last_age_refresh_=stamp.seconds();
    }
    Eigen::Vector3d corner=Eigen::Vector3d::Zero();
    if(!history_.empty()) corner=history_.back().vehicle+Eigen::Vector3d(-4.5,-4.5,0);
    corner.z()=0.0;
    legend_->publish(makeLegend(geometry_frame_.empty()?"map":geometry_frame_,stamp,corner,metric_,minimum,maximum,retention_));
    auto label=markerBase(geometry_frame_.empty()?"map":geometry_frame_,stamp,"risk_status",0,visualization_msgs::msg::Marker::TEXT_VIEW_FACING,1.5);
    label.pose.position=markerPoint(corner,.5); label.scale.z=.22; label.color=markerColor(0xffffff,1);
    std::ostringstream text; text<<"Advisory spatial PL "<<metric_<<" ";
    if(history_.empty()) text<<"waiting: "<<reason_;
    else { const auto& latest=history_.back(); text<<(latest_prediction_valid_&&latest.current(stamp.seconds(),latest_generation_)?"current":"historical")
        <<" ref="<<std::fixed<<std::setprecision(3)<<latest.reference_time_s<<" age="<<std::setprecision(1)<<stamp.seconds()-latest.reference_time_s
        <<" s queries="<<latest.predictor_calls<<" incomplete="<<latest.incomplete; }
    label.text=text.str(); status_->publish(label);
  }
  std::string metric_,z_mode_,geometry_id_,geometry_frame_,reason_="waiting for read-only GridMap export";
  double fixed_z_,hmin_,hmax_,vmin_,vmax_,retention_,anchor_step_,last_display_time_=-1;
  uint64_t latest_generation_=0;
  std::atomic<uint64_t> cancellation_{0};
  std::mutex mutex_; std::condition_variable wake_;
  std::optional<Pending> pending_; std::optional<RiskDisplayFrame> completed_;
  bool stopping_=false,request_in_flight_=false,repaint_=false,latest_prediction_valid_=false;
  double last_age_refresh_=-1;
  uint64_t replaced_inputs_=0,request_token_=0; int64_t request_id_=0; Clock::time_point request_started_;
  std::thread worker_; std::ofstream metrics_;
  std::deque<RiskDisplayFrame> history_; int next_id_=1; nav_msgs::msg::Path glio_path_;
  rclcpp::Client<iap::srv::GetGridMapPredictionInput>::SharedPtr input_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr status_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr surface_,legend_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::TimerBase::SharedPtr request_timer_,display_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  if(std::getenv("IAP_RUN_DIR")) glim::RunLogManager::initialize("grid_map_visualizer");
  rclcpp::spin(std::make_shared<ego_planner::GridMapVisualizer>());
  rclcpp::shutdown(); return 0;
}
