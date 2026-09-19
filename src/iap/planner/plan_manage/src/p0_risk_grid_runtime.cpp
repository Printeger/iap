#include <ego_planner/p0_risk_grid_runtime.h>
#include <plan_env/grid_map.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <exception>
#include <future>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <iap/planner/predictor_risk_conversion.hpp>
#include <iap/predictor/predictor_module.hpp>
#include <Eigen/Eigenvalues>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace ego_planner {

class P0RiskGridWorkerPool {
 public:
  explicit P0RiskGridWorkerPool(const int worker_count) {
    const int count = std::max(1, worker_count);
    workers_.reserve(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index) {
      workers_.emplace_back([this]() { workerLoop(); });
    }
  }

  ~P0RiskGridWorkerPool() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    cv_.notify_all();
    for (auto& worker : workers_) {
      if (worker.joinable()) worker.join();
    }
  }

  std::future<void> submit(std::function<void()> task) {
    std::packaged_task<void()> packaged(std::move(task));
    auto future = packaged.get_future();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) throw std::runtime_error("risk_grid_worker_pool_stopped");
      tasks_.push_back(std::move(packaged));
    }
    cv_.notify_one();
    return future;
  }

 private:
  void workerLoop() {
    while (true) {
      std::packaged_task<void()> task;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return stopping_ || !tasks_.empty(); });
        if (stopping_ && tasks_.empty()) return;
        task = std::move(tasks_.front());
        tasks_.pop_front();
      }
      task();
    }
  }

  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<std::packaged_task<void()>> tasks_;
  std::vector<std::thread> workers_;
  bool stopping_ = false;
};

namespace {

constexpr double kLightSpeed = 2.99792458e8;

template <typename T>
bool sameSharedOwner(const std::shared_ptr<const T>& lhs,
                     const std::shared_ptr<const T>& rhs) {
  return !lhs.owner_before(rhs) && !rhs.owner_before(lhs);
}

bool p0LocalSlamHealthValid(const iap::CurrentIntegrityState& current) {
  return !current.icp_degenerate && std::isfinite(current.icp_rmse) &&
      current.icp_rmse >= 0.0 && std::isfinite(current.icp_condition) &&
      current.icp_condition >= 0.0 &&
      std::isfinite(current.icp_gamma_lidar) &&
      current.icp_gamma_lidar >= 1.0;
}

enum class P0SemanticFailure {
  NONE = 0,
  OCCUPANCY_SNAPSHOT_UNAVAILABLE,
  OCCUPANCY_LOS_ADAPTER_INVALID,
  OCCUPANCY_FRAME_MISMATCH,
  OCCUPANCY_STALE,
  OCCUPANCY_GENERATION_CHANGED,
  PRIOR_GENERATION_CHANGED,
  INVALID_COVARIANCE_GROWTH_PARAMETER,
  MISSING_COVARIANCE_GROWTH_PRIOR,
  STALE_COVARIANCE_GROWTH_PRIOR,
  INVALID_COVARIANCE_GROWTH_PRIOR,
};

const char* semanticFailureReason(const P0SemanticFailure failure) {
  switch (failure) {
    case P0SemanticFailure::NONE:
      return "none";
    case P0SemanticFailure::OCCUPANCY_SNAPSHOT_UNAVAILABLE:
      return "occupancy_snapshot_unavailable";
    case P0SemanticFailure::OCCUPANCY_LOS_ADAPTER_INVALID:
      return "occupancy_los_adapter_invalid";
    case P0SemanticFailure::OCCUPANCY_FRAME_MISMATCH:
      return "occupancy_frame_mismatch";
    case P0SemanticFailure::OCCUPANCY_STALE:
      return "occupancy_stale";
    case P0SemanticFailure::OCCUPANCY_GENERATION_CHANGED:
      return "occupancy_generation_changed";
    case P0SemanticFailure::PRIOR_GENERATION_CHANGED:
      return "prior_generation_changed";
    case P0SemanticFailure::INVALID_COVARIANCE_GROWTH_PARAMETER:
      return "invalid_covariance_growth_parameter";
    case P0SemanticFailure::MISSING_COVARIANCE_GROWTH_PRIOR:
      return "missing_covariance_growth_prior";
    case P0SemanticFailure::STALE_COVARIANCE_GROWTH_PRIOR:
      return "stale_covariance_growth_prior";
    case P0SemanticFailure::INVALID_COVARIANCE_GROWTH_PRIOR:
      return "invalid_covariance_growth_prior";
  }
  return "invalid_semantic_failure";
}

bool finiteSpd(const Eigen::Matrix3d& matrix) {
  if (!matrix.allFinite()) {
    return false;
  }
  const double scale = std::max(1.0, matrix.cwiseAbs().maxCoeff());
  if ((matrix - matrix.transpose()).cwiseAbs().maxCoeff() >
      1.0e-12 * scale) {
    return false;
  }
  const Eigen::Matrix3d symmetric = 0.5 * (matrix + matrix.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(
      symmetric, Eigen::EigenvaluesOnly);
  return eigen.info() == Eigen::Success &&
         eigen.eigenvalues().allFinite() &&
         eigen.eigenvalues().minCoeff() > 0.0;
}

double stampToSec(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) +
         1.0e-9 * static_cast<double>(stamp.nanosec);
}

bool finite(double value) {
  return std::isfinite(value);
}

void advanceNonzeroGeneration(uint64_t* generation) {
  if (!generation) return;
  ++*generation;
  if (*generation == 0u) ++*generation;
}

double steadyNowSeconds() {
  static const auto epoch = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - epoch)
      .count();
}

std::string jsonNumber(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream oss;
  oss << std::setprecision(17) << value;
  return oss.str();
}

std::string jsonString(const std::string& value) {
  std::ostringstream oss;
  oss << '"';
  for (const char c : value) {
    switch (c) {
      case '\\':
        oss << "\\\\";
        break;
      case '"':
        oss << "\\\"";
        break;
      case '\n':
        oss << "\\n";
        break;
      case '\r':
        oss << "\\r";
        break;
      case '\t':
        oss << "\\t";
        break;
      default:
        oss << c;
        break;
    }
  }
  oss << '"';
  return oss.str();
}

std::string jsonBool(bool value) {
  return value ? "true" : "false";
}

bool hasPointField(const sensor_msgs::msg::PointCloud2& msg,
                   const std::string& name) {
  return std::any_of(msg.fields.begin(), msg.fields.end(),
                     [&](const auto& field) {
                       return field.name == name;
                     });
}

P0ExecutionRiskSnapshot::LocalObstacleSourceCertification
makeLocalObstacleSourceCertification(
    const RegisteredLidarObstacleSource& source) {
  P0ExecutionRiskSnapshot::LocalObstacleSourceCertification certification;
  certification.frame_id = source.metadata.frame_id;
  certification.source_stamp_s = source.metadata.stamp_s;
  certification.occupied_centers = source.occupied_voxel_centers;
  constexpr double kExactSourceStampToleranceS = 1.0e-6;
  const bool embedded_health_valid =
      source.metadata.source_health_valid &&
      source.metadata.frame_id >= 0 &&
      !source.metadata.content_hash.empty() &&
      std::isfinite(source.metadata.source_health_stamp_s) &&
      std::abs(source.metadata.source_health_stamp_s -
               source.metadata.stamp_s) <= kExactSourceStampToleranceS &&
      !source.metadata.source_icp_degenerate &&
      std::isfinite(source.metadata.source_icp_rmse) &&
      std::isfinite(source.metadata.source_icp_condition) &&
      std::isfinite(source.metadata.source_icp_gamma_lidar) &&
      source.metadata.source_icp_rmse >= 0.0 &&
      source.metadata.source_icp_condition >= 0.0 &&
      source.metadata.source_icp_gamma_lidar >= 1.0;
  if (embedded_health_valid) {
    certification.certified = true;
    certification.icp_degenerate = source.metadata.source_icp_degenerate;
    certification.icp_rmse_m = source.metadata.source_icp_rmse;
    certification.icp_condition = source.metadata.source_icp_condition;
    certification.icp_gamma = source.metadata.source_icp_gamma_lidar;
  }
  std::ostringstream identity;
  identity << "frame=" << certification.frame_id << ";stamp="
           << std::setprecision(17) << certification.source_stamp_s
           << ";content=" << source.metadata.content_hash
           << ";embedded_health=" << embedded_health_valid
           << ";certified=" << certification.certified
           << ";icp_rmse=" << certification.icp_rmse_m
           << ";icp_condition=" << certification.icp_condition
           << ";icp_gamma=" << certification.icp_gamma;
  certification.identity = identity.str();
  return certification;
}

struct PredictorPositionCacheKey {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  bool operator==(const PredictorPositionCacheKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct PredictorPositionCacheHash {
  std::size_t operator()(const PredictorPositionCacheKey& key) const {
    std::size_t seed = std::hash<double>{}(key.x);
    seed ^= std::hash<double>{}(key.y) + 0x9e3779b9u + (seed << 6) +
            (seed >> 2);
    seed ^= std::hash<double>{}(key.z) + 0x9e3779b9u + (seed << 6) +
            (seed >> 2);
    return seed;
  }
};

PredictorPositionCacheKey makePredictorPositionCacheKey(
    const Eigen::Vector3d& position) {
  return PredictorPositionCacheKey{position.x(), position.y(), position.z()};
}

iap::PredictorSourceMode parsePredictorSourceMode(
    const std::string& value) {
  if (value == "fusion") {
    return iap::PredictorSourceMode::Fusion;
  }
  if (value == "gnss_only") {
    return iap::PredictorSourceMode::GnssOnly;
  }
  if (value == "lidar_only") {
    return iap::PredictorSourceMode::LidarOnly;
  }
  throw std::invalid_argument(
      "invalid p0.predictor.source_mode '" + value +
      "'; expected one of: fusion, gnss_only, lidar_only");
}

iap::PredictorGnssEpochPolicy parsePredictorGnssEpochPolicy(
    const std::string& value) {
  if (value == "auto") {
    return iap::PredictorGnssEpochPolicy::Auto;
  }
  if (value == "required") {
    return iap::PredictorGnssEpochPolicy::Required;
  }
  if (value == "optional") {
    return iap::PredictorGnssEpochPolicy::Optional;
  }
  if (value == "disabled") {
    return iap::PredictorGnssEpochPolicy::Disabled;
  }
  throw std::invalid_argument(
      "invalid p0.predictor.gnss_epoch_policy '" + value +
      "'; expected one of: auto, required, optional, disabled");
}

class PredictorModuleRiskProvider final : public iap::RiskPredictionProvider {
 public:
  PredictorModuleRiskProvider(
      iap::RollingSpatialAdvisoryWindow* rolling_window,
      iap::RollingSpatialWindowGeometry geometry,
      std::shared_ptr<const iap::LocalOccupancyGrid> occupancy_owner,
      std::shared_ptr<const std::vector<Eigen::Vector3d>> lidar_map_points,
      std::shared_ptr<const std::vector<iap::LidarFimPrimitive>>
          lidar_fim_primitives,
      iap::RollingSpatialRetentionPolicy retention_policy,
      iap::RollingSpatialSourceProvenance source_provenance,
      iap::PredictorModule module,
      iap::IntegritySnapshot snapshot,
      iap::GlobalNavigationTaskMode task_mode,
      double evaluation_time_s, int worker_count, double hal_m, double val_m,
      std::chrono::steady_clock::time_point deadline,
      P0RiskGridWorkerPool* worker_pool,
      std::function<bool(std::chrono::steady_clock::time_point)>
          execution_priority_gate)
      : occupancy_owner_(std::move(occupancy_owner)),
        rolling_window_(rolling_window), snapshot_(std::move(snapshot)),
        evaluation_time_s_(evaluation_time_s),
        worker_count_(std::max(1, worker_count)),
        hal_m_(hal_m), val_m_(val_m), deadline_(deadline),
        worker_pool_(worker_pool),
        execution_priority_gate_(std::move(execution_priority_gate)) {
    iap::RollingSpatialRefreshInput input;
    input.geometry = std::move(geometry);
    input.module = std::move(module);
    input.snapshot = snapshot_;
    input.task_mode = task_mode;
    input.policy = retention_policy;
    input.provenance = source_provenance;
    input.occupancy_owner = occupancy_owner_;
    input.lidar_map_points_owner = std::move(lidar_map_points);
    input.lidar_fim_primitives_owner = std::move(lidar_fim_primitives);
    ready_ = rolling_window_ && rolling_window_->beginRefresh(
        std::move(input), &begin_failure_reason_);
    if (rolling_window_ && !ready_) {
      begin_diagnostics_ = rolling_window_->diagnostics();
    }
  }

  ~PredictorModuleRiskProvider() override {
    if (rolling_window_ && ready_) {
      rolling_window_->abortRefresh();
    }
  }

  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    if (results == nullptr || !ready_) {
      return false;
    }
    results->assign(queries.size(), iap::RiskPredictionResult{});
    if (queries.empty()) {
      return true;
    }
    std::unordered_map<PredictorPositionCacheKey, std::size_t,
                       PredictorPositionCacheHash> group_by_position;
    std::vector<std::vector<std::size_t>> groups;
    group_by_position.reserve(queries.size());
    for (std::size_t index = 0; index < queries.size(); ++index) {
      const auto key = makePredictorPositionCacheKey(queries[index].position_w);
      const auto inserted = group_by_position.emplace(key, groups.size());
      if (inserted.second) groups.emplace_back();
      groups[inserted.first->second].push_back(index);
    }
    return batchQueryGrouped(queries, groups, results);
  }

  bool batchQueryPositionHorizons(
      const std::vector<Eigen::Vector3d>& positions_w,
      const std::vector<double>& horizons_s,
      const double evaluation_time_s,
      std::vector<iap::RiskPredictionResult>* results) override {
    if (results == nullptr || !ready_) return false;
    std::vector<iap::RiskPredictionQuery> queries;
    queries.reserve(positions_w.size() * horizons_s.size());
    for (const double horizon_s : horizons_s) {
      for (const auto& position_w : positions_w) {
        queries.push_back({position_w, evaluation_time_s + horizon_s,
                           horizon_s});
      }
    }
    results->assign(queries.size(), iap::RiskPredictionResult{});
    if (queries.empty()) return true;
    std::vector<std::vector<std::size_t>> groups(positions_w.size());
    for (std::size_t position = 0; position < positions_w.size(); ++position) {
      groups[position].reserve(horizons_s.size());
      for (std::size_t horizon = 0; horizon < horizons_s.size(); ++horizon) {
        groups[position].push_back(horizon * positions_w.size() + position);
      }
    }
    return batchQueryGrouped(queries, groups, results);
  }

 private:
  bool batchQueryGrouped(
      const std::vector<iap::RiskPredictionQuery>& queries,
      const std::vector<std::vector<std::size_t>>& groups,
      std::vector<iap::RiskPredictionResult>* results) {
    const int worker_count = std::min<int>(worker_count_, groups.size());
    struct WorkerOutcome {
      iap::PredictorBatchDiagnostics diagnostics;
      bool growth_valid = true;
    };
    std::vector<WorkerOutcome> outcomes(static_cast<std::size_t>(worker_count));
    std::vector<std::future<void>> workers;
    workers.reserve(static_cast<std::size_t>(worker_count));
    std::atomic<std::size_t> next_group{0u};
    for (int worker_id = 0; worker_id < worker_count; ++worker_id) {
      auto work = [this, &queries, &groups, results, &next_group,
                   &outcomes, worker_id]() {
            WorkerOutcome outcome;
            while (true) {
              if (cancelled_.load(std::memory_order_relaxed) ||
                  std::chrono::steady_clock::now() >= deadline_) {
                cancelled_.store(true, std::memory_order_relaxed);
                outcome.growth_valid = false;
                outcomes[static_cast<std::size_t>(worker_id)] = outcome;
                return;
              }
              if (execution_priority_gate_ &&
                  !execution_priority_gate_(deadline_)) {
                cancelled_.store(true, std::memory_order_relaxed);
                outcome.growth_valid = false;
                outcomes[static_cast<std::size_t>(worker_id)] = outcome;
                return;
              }
              // Dynamic dispatch is essential to cooperative priority. With
              // fixed striding, a worker that yielded for each 10 Hz snapshot
              // retained an entire sixth of the batch and became the tail.
              // The active workers now consume its unclaimed positions while
              // it waits, without changing any query or output identity.
              constexpr std::size_t kDispatchChunk = 32u;
              const std::size_t group_begin = next_group.fetch_add(
                  kDispatchChunk, std::memory_order_relaxed);
              if (group_begin >= groups.size()) {
                break;
              }
              const std::size_t group_end = std::min(
                  groups.size(), group_begin + kDispatchChunk);
              for (std::size_t group_index = group_begin;
                   group_index < group_end; ++group_index) {
                std::vector<iap::PredictorQueryInput> inputs;
                inputs.reserve(groups[group_index].size());
                for (const std::size_t index : groups[group_index]) {
                  const auto& query = queries[index];
                  inputs.emplace_back(query.position_w, snapshot_,
                      query.query_time_s, query.horizon_s, "map",
                      evaluation_time_s_);
                }
                iap::PredictorBatchDiagnostics diagnostics;
                const auto predictions = rolling_window_->queryPositionHorizons(
                    inputs, &diagnostics);
                if (predictions.size() != inputs.size()) {
                  outcome.growth_valid = false;
                  outcomes[static_cast<std::size_t>(worker_id)] = outcome;
                  return;
                }
                for (std::size_t local = 0; local < predictions.size(); ++local) {
                  if (inputs[local].horizon_s > 0.0 &&
                      predictions[local].covariance_growth_status !=
                          iap::CovarianceGrowthStatus::APPLIED) {
                    outcome.growth_valid = false;
                  }
                  (*results)[groups[group_index][local]] =
                      iap::makeRiskPredictionResult(
                          predictions[local], hal_m_, val_m_);
                }
                outcome.diagnostics.query_count += diagnostics.query_count;
                outcome.diagnostics.unique_positions +=
                    diagnostics.unique_positions;
                outcome.diagnostics.lidar_evaluations +=
                    diagnostics.lidar_evaluations;
                outcome.diagnostics.lidar_cache_hits +=
                    diagnostics.lidar_cache_hits;
                outcome.diagnostics.spatial_advisory_recompute_count +=
                    diagnostics.spatial_advisory_recompute_count;
                outcome.diagnostics.spatial_advisory_reuse_count +=
                    diagnostics.spatial_advisory_reuse_count;
                outcome.diagnostics.gnss_advisory_invocations +=
                    diagnostics.gnss_advisory_invocations;
                outcome.diagnostics.lidar_advisory_invocations +=
                    diagnostics.lidar_advisory_invocations;
                outcome.diagnostics.fusion_advisory_invocations +=
                    diagnostics.fusion_advisory_invocations;
              }
            }
            outcomes[static_cast<std::size_t>(worker_id)] = outcome;
          };
      workers.push_back(worker_pool_
          ? worker_pool_->submit(std::move(work))
          : std::async(std::launch::async, std::move(work)));
    }
    last_diagnostics_ = {};
    bool growth_valid = true;
    for (std::size_t index = 0; index < workers.size(); ++index) {
      workers[index].get();
      const auto& outcome = outcomes[index];
      const auto& diagnostics = outcome.diagnostics;
      growth_valid = growth_valid && outcome.growth_valid;
      last_diagnostics_.query_count += diagnostics.query_count;
      last_diagnostics_.unique_positions += diagnostics.unique_positions;
      last_diagnostics_.lidar_evaluations += diagnostics.lidar_evaluations;
      last_diagnostics_.lidar_cache_hits += diagnostics.lidar_cache_hits;
      last_diagnostics_.spatial_advisory_recompute_count +=
          diagnostics.spatial_advisory_recompute_count;
      last_diagnostics_.spatial_advisory_reuse_count +=
          diagnostics.spatial_advisory_reuse_count;
      last_diagnostics_.gnss_advisory_invocations +=
          diagnostics.gnss_advisory_invocations;
      last_diagnostics_.lidar_advisory_invocations +=
          diagnostics.lidar_advisory_invocations;
      last_diagnostics_.fusion_advisory_invocations +=
          diagnostics.fusion_advisory_invocations;
    }
    return growth_valid && !cancelled_.load(std::memory_order_relaxed);
  }

 public:

  const iap::PredictorBatchDiagnostics& diagnostics() const {
    return last_diagnostics_;
  }

  iap::RollingSpatialRefreshDiagnostics rollingDiagnostics() const {
    return ready_ && rolling_window_ ? rolling_window_->diagnostics()
                                    : begin_diagnostics_;
  }

  bool ready() const { return ready_; }
  const std::string& beginFailureReason() const {
    return begin_failure_reason_;
  }

  void finish(const bool success) {
    if (!rolling_window_ || !ready_) return;
    if (success) {
      rolling_window_->commitRefresh();
    } else {
      rolling_window_->abortRefresh();
    }
    ready_ = false;
  }

 private:
  std::shared_ptr<const iap::LocalOccupancyGrid> occupancy_owner_;
  iap::RollingSpatialAdvisoryWindow* rolling_window_ = nullptr;
  iap::IntegritySnapshot snapshot_;
  double evaluation_time_s_ = std::numeric_limits<double>::quiet_NaN();
  int worker_count_ = 1;
  double hal_m_ = 10.0;
  double val_m_ = 20.0;
  bool ready_ = false;
  std::string begin_failure_reason_ = "provider_refresh_failed";
  iap::RollingSpatialRefreshDiagnostics begin_diagnostics_;
  iap::PredictorBatchDiagnostics last_diagnostics_;
  std::chrono::steady_clock::time_point deadline_;
  P0RiskGridWorkerPool* worker_pool_ = nullptr;
  std::function<bool(std::chrono::steady_clock::time_point)>
      execution_priority_gate_;
  std::atomic<bool> cancelled_{false};
};

class TimedRiskProvider final : public iap::RiskPredictionProvider {
 public:
  TimedRiskProvider(iap::RiskPredictionProvider* provider,
                    std::chrono::steady_clock::time_point deadline)
      : provider_(provider), deadline_(deadline) {}

  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    const auto start = std::chrono::steady_clock::now();
    const bool success = provider_ && provider_->batchQuery(queries, results);
    duration_ms_ += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    return success && std::chrono::steady_clock::now() < deadline_;
  }

  bool batchQueryPositionHorizons(
      const std::vector<Eigen::Vector3d>& positions_w,
      const std::vector<double>& horizons_s,
      const double evaluation_time_s,
      std::vector<iap::RiskPredictionResult>* results) override {
    const auto start = std::chrono::steady_clock::now();
    const bool success = provider_ && provider_->batchQueryPositionHorizons(
        positions_w, horizons_s, evaluation_time_s, results);
    duration_ms_ += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    return success && std::chrono::steady_clock::now() < deadline_;
  }

  double durationMs() const { return duration_ms_; }

 private:
  iap::RiskPredictionProvider* provider_ = nullptr;
  double duration_ms_ = 0.0;
  std::chrono::steady_clock::time_point deadline_;
};

}  // namespace

P0RiskGridRuntime::Config P0RiskGridRuntime::declareAndReadConfig(
    const rclcpp::Node::SharedPtr& node) {
  Config config;
  config.enable_risk_grid =
      node->declare_parameter<bool>("p0.enable_risk_grid", false);
  config.grid.resolution_m =
      node->declare_parameter<double>("p0.resolution_m", 0.75);
  config.grid.size_x_m =
      node->declare_parameter<double>("p0.size_x_m", 30.0);
  config.grid.size_y_m =
      node->declare_parameter<double>("p0.size_y_m", 30.0);
  config.grid.size_z_m =
      node->declare_parameter<double>("p0.size_z_m", 6.0);
  const Eigen::Vector3d fixed_origin(
      node->declare_parameter<double>(
          "p0.origin_x_m", std::numeric_limits<double>::quiet_NaN()),
      node->declare_parameter<double>(
          "p0.origin_y_m", std::numeric_limits<double>::quiet_NaN()),
      node->declare_parameter<double>(
          "p0.origin_z_m", std::numeric_limits<double>::quiet_NaN()));
  if (fixed_origin.allFinite()) {
    config.grid.use_fixed_origin = true;
    config.grid.fixed_origin_w = fixed_origin;
  }
  config.grid.horizons_s = node->declare_parameter<std::vector<double>>(
      "p0.horizons_s", std::vector<double>{0.0, 0.5, 1.0, 1.5, 2.0});
  config.grid.refresh_period_s =
      node->declare_parameter<double>("p0.refresh_period_s", 0.5);
  config.execution_snapshot_period_s = node->declare_parameter<double>(
      "p0.execution_snapshot_period_s", 0.05);
  config.execution_batch_budget_ms = node->declare_parameter<double>(
      "p0.execution_batch_budget_ms", 150.0);
  config.risk_grid_build_budget_ms = node->declare_parameter<double>(
      "p0.risk_grid_build_budget_ms", 500.0);
  if (!std::isfinite(config.execution_snapshot_period_s) ||
      config.execution_snapshot_period_s <= 0.0 ||
      !std::isfinite(config.execution_batch_budget_ms) ||
      config.execution_batch_budget_ms <= 0.0 ||
      !std::isfinite(config.risk_grid_build_budget_ms) ||
      config.risk_grid_build_budget_ms <= 0.0) {
    throw std::invalid_argument("invalid P0 execution/grid scheduling budget");
  }
  config.refresh_start_delay_s =
      node->declare_parameter<double>("p0.refresh_start_delay_s", 0.0);
  config.grid.stale_timeout_s =
      node->declare_parameter<double>("p0.stale_timeout_s", 1.0);
  config.grid.skip_occupied_voxels =
      node->declare_parameter<bool>("p0.skip_occupied_voxels",
                                    config.grid.skip_occupied_voxels);
  const std::string provider_cost_source = node->declare_parameter<std::string>(
      "p0.provider_cost_source", "legacy_safety_pl");
  if (provider_cost_source == "legacy_safety_pl") {
    config.grid.provider_cost_source =
        iap::RiskProviderCostSource::LEGACY_SAFETY_PL;
  } else if (provider_cost_source == "pre_conservative_fim_ratio") {
    config.grid.provider_cost_source =
        iap::RiskProviderCostSource::PRE_CONSERVATIVE_FIM_RATIO;
  } else {
    throw std::invalid_argument("unknown p0.provider_cost_source: " +
                                provider_cost_source);
  }
  config.grid.require_safety_ratio_below_one_for_cost =
      node->declare_parameter<bool>(
          "p0.require_safety_ratio_below_one_for_cost", false);
  config.grid.alert_limit_policy_id = node->declare_parameter<std::string>(
      "p0.alert_limit_policy_id", "legacy_unspecified");
  config.predictor_hal_m = node->declare_parameter<double>(
      "p0.alert_limit_h_m", 10.0);
  config.predictor_val_m = node->declare_parameter<double>(
      "p0.alert_limit_v_m", 20.0);
  config.grid.alert_limit_h_m = config.predictor_hal_m;
  config.grid.alert_limit_v_m = config.predictor_val_m;
  if (!std::isfinite(config.predictor_hal_m) ||
      !std::isfinite(config.predictor_val_m) ||
      config.predictor_hal_m <= 0.0 || config.predictor_val_m <= 0.0 ||
      config.grid.alert_limit_policy_id.empty()) {
    throw std::invalid_argument("invalid P0 alert-limit policy");
  }
  if (config.grid.alert_limit_policy_id == "fixed_hal10_val20_v1" &&
      (std::abs(config.predictor_hal_m - 10.0) > 1.0e-12 ||
       std::abs(config.predictor_val_m - 20.0) > 1.0e-12)) {
    throw std::invalid_argument(
        "fixed_hal10_val20_v1 requires HAL=10 m and VAL=20 m");
  }
  config.debug_metrics_enable =
      node->declare_parameter<bool>("p0.debug_metrics_enable", false);
  config.online_mapping_mode = node->declare_parameter<bool>(
      "p0.online_mapping_mode", false);
  config.grid.require_observed_support = config.online_mapping_mode;
  config.fit_grid_to_map_cloud = node->declare_parameter<bool>(
      "p0.fit_grid_to_map_cloud", false);
  config.odom_topic = node->declare_parameter<std::string>(
      "p0.odom_topic", "/drone_0_visual_slam/odom");
  config.integrity_topic =
      node->declare_parameter<std::string>("p0.integrity_topic",
                                           "/iap/integrity");
  config.range_meas_topic =
      node->declare_parameter<std::string>("p0.range_meas_topic",
                                           "/ublox_driver/range_meas");
  config.ephem_topic =
      node->declare_parameter<std::string>("p0.ephem_topic",
                                           "/ublox_driver/ephem");
  config.glo_ephem_topic =
      node->declare_parameter<std::string>("p0.glo_ephem_topic",
                                           "/ublox_driver/glo_ephem");
  config.receiver_lla_topic =
      node->declare_parameter<std::string>("p0.receiver_lla_topic",
                                           "/ublox_driver/receiver_lla");
  config.iono_topic =
      node->declare_parameter<std::string>("p0.iono_topic",
                                           "/ublox_driver/iono_params");
  config.map_topic = node->declare_parameter<std::string>(
      "p0.map_topic", "/map_generator/global_cloud");
  if (config.online_mapping_mode &&
      (config.fit_grid_to_map_cloud || !config.map_topic.empty())) {
    throw std::invalid_argument(
        "online P0 requires fit_grid_to_map_cloud=false and empty map_topic");
  }
  // Raw health is an acceptance artifact and must not inherit a node
  // namespace.  Keep declaring the old parameter for launch compatibility,
  // but deliberately publish the authoritative absolute topic.
  node->declare_parameter<std::string>("p0.health_topic",
                                       "/planning/risk_grid_health");
  config.health_topic = "/planning/risk_grid_health";
  config.gnss_epoch_max_age_s =
      node->declare_parameter<double>("p0.gnss_epoch_max_age_s", 2.0);
  config.gnss_pr_noise_base_m = node->declare_parameter<double>(
      "p0.gnss_pr_noise_base_m", 5.0);
  config.gnss_dop_noise_base_mps = node->declare_parameter<double>(
      "p0.gnss_dop_noise_base_mps", 0.5);
  config.predictor_gnss_admission_epochs = std::max<int>(1,
      static_cast<int>(node->declare_parameter<int>(
          "p0.predictor.gnss.admission_epochs", 3)));
  if (!std::isfinite(config.gnss_pr_noise_base_m) ||
      config.gnss_pr_noise_base_m <= 0.0 ||
      !std::isfinite(config.gnss_dop_noise_base_mps) ||
      config.gnss_dop_noise_base_mps <= 0.0) {
    throw std::invalid_argument("invalid P0 GNSS source noise parameters");
  }
  config.predictor_source_mode = parsePredictorSourceMode(
      node->declare_parameter<std::string>("p0.predictor.source_mode",
                                           "fusion"));
  config.predictor_gnss_epoch_policy = parsePredictorGnssEpochPolicy(
      node->declare_parameter<std::string>(
          "p0.predictor.gnss_epoch_policy", "auto"));
  std::string task_mode = "mission_best_effort";
  if (node->has_parameter("p4.assurance.task_mode")) {
    node->get_parameter("p4.assurance.task_mode", task_mode);
  } else {
    task_mode = node->declare_parameter<std::string>(
        "p4.assurance.task_mode", task_mode);
  }
  if (!iap::parseGlobalNavigationTaskMode(task_mode, &config.task_mode)) {
    throw std::invalid_argument(
        "p4.assurance.task_mode must be strict_global or "
        "mission_best_effort");
  }
  config.predictor_gnss_measured_epoch_support_radius_m =
      node->declare_parameter<double>(
          "p0.predictor.gnss_measured_epoch_support_radius_m", 0.0);
  if (!std::isfinite(
          config.predictor_gnss_measured_epoch_support_radius_m) ||
      config.predictor_gnss_measured_epoch_support_radius_m < 0.0) {
    throw std::invalid_argument(
        "invalid P0 measured GNSS epoch support radius");
  }
  config.predictor_gnss_measured_epoch_integrity_max_delta_s =
      node->declare_parameter<double>(
          "p0.predictor.gnss_measured_epoch_integrity_max_delta_s", 0.25);
  if (!std::isfinite(
          config.predictor_gnss_measured_epoch_integrity_max_delta_s) ||
      config.predictor_gnss_measured_epoch_integrity_max_delta_s < 0.0) {
    throw std::invalid_argument(
        "invalid P0 measured GNSS/integrity epoch alignment tolerance");
  }
  config.predictor_gnss_clearance_transition_m =
      node->declare_parameter<double>(
          "p0.predictor.gnss.clearance_transition_m", 0.0);
  if (!std::isfinite(config.predictor_gnss_clearance_transition_m) ||
      config.predictor_gnss_clearance_transition_m < 0.0 ||
      config.predictor_gnss_clearance_transition_m >
          iap::LocalOccupancyGrid::kMaxClearanceTransitionM) {
    throw std::invalid_argument(
        "invalid P0 GNSS clearance transition width");
  }
  config.predictor_use_current_integrity_prior =
      node->declare_parameter<bool>(
          "p0.predictor.use_current_integrity_prior", true);
  config.predictor_conservative_max_with_gnss =
      node->declare_parameter<bool>(
          "p0.predictor.conservative_max_with_gnss", false);
  config.predictor_lidar_legacy_observability =
      node->declare_parameter<bool>(
          "p0.predictor.lidar_legacy_observability", true);
  config.predictor_lidar_fim_radius_m =
      node->declare_parameter<double>(
          "p0.predictor.lidar_fim_radius_m",
          config.predictor_lidar_fim_radius_m);
  config.predictor_sigma_grow_m_sqrt_s =
      node->declare_parameter<double>(
          "p0.predictor.sigma_grow_m_sqrt_s",
          config.predictor_sigma_grow_m_sqrt_s);
  config.predictor_requested_worker_count = static_cast<int>(std::max<int64_t>(1,
      node->declare_parameter<int>("p0.predictor.worker_count", 1)));
  config.predictor_effective_worker_count = config.predictor_requested_worker_count;
  config.p0_6_fixture.enabled =
      node->declare_parameter<bool>("p0_6.fixture.enabled", false);
  config.p0_6_fixture.name =
      node->declare_parameter<std::string>("p0_6.fixture.name", "");
  config.p0_6_fixture.x_min_m =
      node->declare_parameter<double>("p0_6.fixture.x_min",
                                      config.p0_6_fixture.x_min_m);
  config.p0_6_fixture.x_max_m =
      node->declare_parameter<double>("p0_6.fixture.x_max",
                                      config.p0_6_fixture.x_max_m);
  config.p0_6_fixture.y_min_m =
      node->declare_parameter<double>("p0_6.fixture.y_min",
                                      config.p0_6_fixture.y_min_m);
  config.p0_6_fixture.y_max_m =
      node->declare_parameter<double>("p0_6.fixture.y_max",
                                      config.p0_6_fixture.y_max_m);
  config.p0_6_fixture.z_min_m =
      node->declare_parameter<double>("p0_6.fixture.z_min",
                                      config.p0_6_fixture.z_min_m);
  config.p0_6_fixture.z_max_m =
      node->declare_parameter<double>("p0_6.fixture.z_max",
                                      config.p0_6_fixture.z_max_m);
  config.p0_6_fixture.raw_hpl_m =
      node->declare_parameter<double>("p0_6.fixture.raw_hpl_m",
                                      config.p0_6_fixture.raw_hpl_m);
  config.p0_6_fixture.raw_vpl_m =
      node->declare_parameter<double>("p0_6.fixture.raw_vpl_m",
                                      config.p0_6_fixture.raw_vpl_m);
  config.p0_6_fixture.raw_c_pi =
      node->declare_parameter<double>("p0_6.fixture.raw_c_pi",
                                      config.p0_6_fixture.raw_c_pi);
  config.p0_6_fixture.low_raw_cost_threshold =
      node->declare_parameter<double>(
          "p0_6.fixture.low_raw_cost_threshold",
          config.p0_6_fixture.low_raw_cost_threshold);
  if (config.p0_6_fixture.x_min_m > config.p0_6_fixture.x_max_m) {
    std::swap(config.p0_6_fixture.x_min_m,
              config.p0_6_fixture.x_max_m);
  }
  if (config.p0_6_fixture.y_min_m > config.p0_6_fixture.y_max_m) {
    std::swap(config.p0_6_fixture.y_min_m,
              config.p0_6_fixture.y_max_m);
  }
  if (config.p0_6_fixture.z_min_m > config.p0_6_fixture.z_max_m) {
    std::swap(config.p0_6_fixture.z_min_m,
              config.p0_6_fixture.z_max_m);
  }
  auto& p5_3_fixture = config.grid.p5_3_fixture;
  p5_3_fixture.enabled =
      node->declare_parameter<bool>("p5_3.fixture.enabled",
                                    p5_3_fixture.enabled);
  p5_3_fixture.name =
      node->declare_parameter<std::string>("p5_3.fixture.name",
                                           p5_3_fixture.name);
  p5_3_fixture.x_min_m =
      node->declare_parameter<double>("p5_3.fixture.x_min",
                                      p5_3_fixture.x_min_m);
  p5_3_fixture.x_max_m =
      node->declare_parameter<double>("p5_3.fixture.x_max",
                                      p5_3_fixture.x_max_m);
  p5_3_fixture.y_min_m =
      node->declare_parameter<double>("p5_3.fixture.y_min",
                                      p5_3_fixture.y_min_m);
  p5_3_fixture.y_max_m =
      node->declare_parameter<double>("p5_3.fixture.y_max",
                                      p5_3_fixture.y_max_m);
  p5_3_fixture.z_min_m =
      node->declare_parameter<double>("p5_3.fixture.z_min",
                                      p5_3_fixture.z_min_m);
  p5_3_fixture.z_max_m =
      node->declare_parameter<double>("p5_3.fixture.z_max",
                                      p5_3_fixture.z_max_m);
  p5_3_fixture.tau_min_s =
      node->declare_parameter<double>("p5_3.fixture.tau_min",
                                      p5_3_fixture.tau_min_s);
  p5_3_fixture.tau_max_s =
      node->declare_parameter<double>("p5_3.fixture.tau_max",
                                      p5_3_fixture.tau_max_s);
  p5_3_fixture.hpl_pred_m =
      node->declare_parameter<double>("p5_3.fixture.hpl_pred_m",
                                      p5_3_fixture.hpl_pred_m);
  p5_3_fixture.vpl_pred_m =
      node->declare_parameter<double>("p5_3.fixture.vpl_pred_m",
                                      p5_3_fixture.vpl_pred_m);
  if (p5_3_fixture.x_min_m > p5_3_fixture.x_max_m) {
    std::swap(p5_3_fixture.x_min_m, p5_3_fixture.x_max_m);
  }
  if (p5_3_fixture.y_min_m > p5_3_fixture.y_max_m) {
    std::swap(p5_3_fixture.y_min_m, p5_3_fixture.y_max_m);
  }
  if (p5_3_fixture.z_min_m > p5_3_fixture.z_max_m) {
    std::swap(p5_3_fixture.z_min_m, p5_3_fixture.z_max_m);
  }
  if (p5_3_fixture.tau_min_s > p5_3_fixture.tau_max_s) {
    std::swap(p5_3_fixture.tau_min_s, p5_3_fixture.tau_max_s);
  }
  auto& p5_4_fixture = config.grid.p5_4_fixture;
  p5_4_fixture.enabled =
      node->declare_parameter<bool>("p5_4.fixture.enabled",
                                    p5_4_fixture.enabled);
  p5_4_fixture.name =
      node->declare_parameter<std::string>("p5_4.fixture.name",
                                           p5_4_fixture.name);
  p5_4_fixture.x_min_m =
      node->declare_parameter<double>("p5_4.fixture.x_min",
                                      p5_4_fixture.x_min_m);
  p5_4_fixture.x_max_m =
      node->declare_parameter<double>("p5_4.fixture.x_max",
                                      p5_4_fixture.x_max_m);
  p5_4_fixture.y_min_m =
      node->declare_parameter<double>("p5_4.fixture.y_min",
                                      p5_4_fixture.y_min_m);
  p5_4_fixture.y_max_m =
      node->declare_parameter<double>("p5_4.fixture.y_max",
                                      p5_4_fixture.y_max_m);
  p5_4_fixture.z_min_m =
      node->declare_parameter<double>("p5_4.fixture.z_min",
                                      p5_4_fixture.z_min_m);
  p5_4_fixture.z_max_m =
      node->declare_parameter<double>("p5_4.fixture.z_max",
                                      p5_4_fixture.z_max_m);
  p5_4_fixture.tau_min_s =
      node->declare_parameter<double>("p5_4.fixture.tau_min",
                                      p5_4_fixture.tau_min_s);
  p5_4_fixture.tau_max_s =
      node->declare_parameter<double>("p5_4.fixture.tau_max",
                                      p5_4_fixture.tau_max_s);
  p5_4_fixture.hpl_pred_m =
      node->declare_parameter<double>("p5_4.fixture.hpl_pred_m",
                                      p5_4_fixture.hpl_pred_m);
  p5_4_fixture.vpl_pred_m =
      node->declare_parameter<double>("p5_4.fixture.vpl_pred_m",
                                      p5_4_fixture.vpl_pred_m);
  if (p5_4_fixture.x_min_m > p5_4_fixture.x_max_m) {
    std::swap(p5_4_fixture.x_min_m, p5_4_fixture.x_max_m);
  }
  if (p5_4_fixture.y_min_m > p5_4_fixture.y_max_m) {
    std::swap(p5_4_fixture.y_min_m, p5_4_fixture.y_max_m);
  }
  if (p5_4_fixture.z_min_m > p5_4_fixture.z_max_m) {
    std::swap(p5_4_fixture.z_min_m, p5_4_fixture.z_max_m);
  }
  if (p5_4_fixture.tau_min_s > p5_4_fixture.tau_max_s) {
    std::swap(p5_4_fixture.tau_min_s, p5_4_fixture.tau_max_s);
  }
  auto& p5_6_fixture = config.grid.p5_6_fixture;
  p5_6_fixture.enabled =
      node->declare_parameter<bool>("p5_6.fixture.enabled",
                                    p5_6_fixture.enabled);
  p5_6_fixture.name =
      node->declare_parameter<std::string>("p5_6.fixture.name",
                                           p5_6_fixture.name);
  p5_6_fixture.x_min_m =
      node->declare_parameter<double>("p5_6.fixture.x_min",
                                      p5_6_fixture.x_min_m);
  p5_6_fixture.x_max_m =
      node->declare_parameter<double>("p5_6.fixture.x_max",
                                      p5_6_fixture.x_max_m);
  p5_6_fixture.y_min_m =
      node->declare_parameter<double>("p5_6.fixture.y_min",
                                      p5_6_fixture.y_min_m);
  p5_6_fixture.y_max_m =
      node->declare_parameter<double>("p5_6.fixture.y_max",
                                      p5_6_fixture.y_max_m);
  p5_6_fixture.z_min_m =
      node->declare_parameter<double>("p5_6.fixture.z_min",
                                      p5_6_fixture.z_min_m);
  p5_6_fixture.z_max_m =
      node->declare_parameter<double>("p5_6.fixture.z_max",
                                      p5_6_fixture.z_max_m);
  p5_6_fixture.tau_min_s =
      node->declare_parameter<double>("p5_6.fixture.tau_min",
                                      p5_6_fixture.tau_min_s);
  p5_6_fixture.tau_max_s =
      node->declare_parameter<double>("p5_6.fixture.tau_max",
                                      p5_6_fixture.tau_max_s);
  if (p5_6_fixture.x_min_m > p5_6_fixture.x_max_m) {
    std::swap(p5_6_fixture.x_min_m, p5_6_fixture.x_max_m);
  }
  if (p5_6_fixture.y_min_m > p5_6_fixture.y_max_m) {
    std::swap(p5_6_fixture.y_min_m, p5_6_fixture.y_max_m);
  }
  if (p5_6_fixture.z_min_m > p5_6_fixture.z_max_m) {
    std::swap(p5_6_fixture.z_min_m, p5_6_fixture.z_max_m);
  }
  if (p5_6_fixture.tau_min_s > p5_6_fixture.tau_max_s) {
    std::swap(p5_6_fixture.tau_min_s, p5_6_fixture.tau_max_s);
  }
  auto& p5_7_fixture = config.grid.p5_7_fixture;
  p5_7_fixture.enabled =
      node->declare_parameter<bool>("p5_7.fixture.enabled",
                                    p5_7_fixture.enabled);
  p5_7_fixture.effective_enabled =
      node->declare_parameter<bool>("p5_7.fixture.effective_enabled",
                                    p5_7_fixture.effective_enabled);
  p5_7_fixture.name =
      node->declare_parameter<std::string>("p5_7.fixture.name",
                                           p5_7_fixture.name);
  p5_7_fixture.x_min_m =
      node->declare_parameter<double>("p5_7.fixture.x_min",
                                      p5_7_fixture.x_min_m);
  p5_7_fixture.x_max_m =
      node->declare_parameter<double>("p5_7.fixture.x_max",
                                      p5_7_fixture.x_max_m);
  p5_7_fixture.y_min_m =
      node->declare_parameter<double>("p5_7.fixture.y_min",
                                      p5_7_fixture.y_min_m);
  p5_7_fixture.y_max_m =
      node->declare_parameter<double>("p5_7.fixture.y_max",
                                      p5_7_fixture.y_max_m);
  p5_7_fixture.z_min_m =
      node->declare_parameter<double>("p5_7.fixture.z_min",
                                      p5_7_fixture.z_min_m);
  p5_7_fixture.z_max_m =
      node->declare_parameter<double>("p5_7.fixture.z_max",
                                      p5_7_fixture.z_max_m);
  p5_7_fixture.tau_min_s =
      node->declare_parameter<double>("p5_7.fixture.tau_min",
                                      p5_7_fixture.tau_min_s);
  p5_7_fixture.tau_max_s =
      node->declare_parameter<double>("p5_7.fixture.tau_max",
                                      p5_7_fixture.tau_max_s);
  p5_7_fixture.hpl_pred_m =
      node->declare_parameter<double>("p5_7.fixture.hpl_pred_m",
                                      p5_7_fixture.hpl_pred_m);
  p5_7_fixture.vpl_pred_m =
      node->declare_parameter<double>("p5_7.fixture.vpl_pred_m",
                                      p5_7_fixture.vpl_pred_m);
  if (p5_7_fixture.x_min_m > p5_7_fixture.x_max_m) {
    std::swap(p5_7_fixture.x_min_m, p5_7_fixture.x_max_m);
  }
  if (p5_7_fixture.y_min_m > p5_7_fixture.y_max_m) {
    std::swap(p5_7_fixture.y_min_m, p5_7_fixture.y_max_m);
  }
  if (p5_7_fixture.z_min_m > p5_7_fixture.z_max_m) {
    std::swap(p5_7_fixture.z_min_m, p5_7_fixture.z_max_m);
  }
  if (p5_7_fixture.tau_min_s > p5_7_fixture.tau_max_s) {
    std::swap(p5_7_fixture.tau_min_s, p5_7_fixture.tau_max_s);
  }
  return config;
}

std::unique_ptr<P0RiskGridRuntime> P0RiskGridRuntime::createIfEnabled(
    const rclcpp::Node::SharedPtr& node) {
  Config config = declareAndReadConfig(node);
  if (!config.enable_risk_grid) {
    return nullptr;
  }
  return std::make_unique<P0RiskGridRuntime>(node, std::move(config));
}

P0RiskGridRuntime::P0RiskGridRuntime(
    rclcpp::Node::SharedPtr node,
    Config config,
    std::unique_ptr<iap::RiskPredictionProvider> provider)
    : node_(std::move(node)),
      config_(std::move(config)),
      risk_grid_(config_.grid),
      provider_(std::move(provider)),
      risk_grid_worker_pool_(std::make_unique<P0RiskGridWorkerPool>(
          config_.predictor_effective_worker_count)),
      gnss_satellite_admission_(config_.predictor_gnss_admission_epochs) {
  createRosInterfaces();
  if (config_.enable_risk_grid && !provider_) {
    execution_snapshot_worker_ =
        std::thread([this]() { executionSnapshotWorkerLoop(); });
  }
}

P0RiskGridRuntime::~P0RiskGridRuntime() {
  shutdown();
}

void P0RiskGridRuntime::shutdown() {
  {
    std::lock_guard<std::mutex> lock(execution_snapshot_worker_mutex_);
    execution_snapshot_worker_stop_ = true;
    pending_execution_snapshot_request_.reset();
  }
  execution_snapshot_worker_cv_.notify_all();
  if (execution_snapshot_worker_.joinable()) {
    execution_snapshot_worker_.join();
  }
}

const char* p0ExecutionSnapshotAttemptStatusName(
    const P0ExecutionSnapshotAttemptStatus status) {
  switch (status) {
    case P0ExecutionSnapshotAttemptStatus::PUBLISHED:
      return "PUBLISHED";
    case P0ExecutionSnapshotAttemptStatus::DEDUPLICATED:
      return "DEDUPLICATED";
    case P0ExecutionSnapshotAttemptStatus::CAPTURE_UNAVAILABLE:
      return "CAPTURE_UNAVAILABLE";
    case P0ExecutionSnapshotAttemptStatus::CAPTURE_ADAPTER_INVALID:
      return "CAPTURE_ADAPTER_INVALID";
    case P0ExecutionSnapshotAttemptStatus::OCCUPANCY_INVALID:
      return "OCCUPANCY_INVALID";
    case P0ExecutionSnapshotAttemptStatus::OCCUPANCY_STALE:
      return "OCCUPANCY_STALE";
    case P0ExecutionSnapshotAttemptStatus::INTEGRITY_UNAVAILABLE:
      return "INTEGRITY_UNAVAILABLE";
    case P0ExecutionSnapshotAttemptStatus::INTEGRITY_UNSAFE:
      return "INTEGRITY_UNSAFE";
    case P0ExecutionSnapshotAttemptStatus::MAP_POINTS_MISSING:
      return "MAP_POINTS_MISSING";
    case P0ExecutionSnapshotAttemptStatus::FINAL_FRESHNESS_FAILED:
      return "FINAL_FRESHNESS_FAILED";
    case P0ExecutionSnapshotAttemptStatus::SUPERSEDED:
      return "SUPERSEDED";
    case P0ExecutionSnapshotAttemptStatus::WORKER_EXCEPTION:
      return "WORKER_EXCEPTION";
    case P0ExecutionSnapshotAttemptStatus::WORKER_STOPPED:
      return "WORKER_STOPPED";
  }
  return "UNKNOWN";
}

iap::RiskGridHealth P0RiskGridRuntime::health() const {
  const double now_s = liveNowSeconds();
  return addLidarPredictorInputHealth(
      std::isfinite(now_s) ? risk_grid_.health(now_s)
                           : risk_grid_.health());
}

std::shared_ptr<const P0PlanningSnapshot>
P0RiskGridRuntime::acquirePlanningSnapshot() const {
  std::lock_guard<std::mutex> lock(planning_snapshot_mutex_);
  return planning_snapshot_;
}

std::shared_ptr<const P0ExecutionRiskSnapshot>
P0RiskGridRuntime::acquireExecutionRiskSnapshot() const {
  std::lock_guard<std::mutex> lock(planning_snapshot_mutex_);
  return execution_snapshot_;
}

std::shared_ptr<const P0ExecutionRiskSnapshot>
P0RiskGridRuntime::selectExecutionRiskSnapshotForEvaluation(
    const std::shared_ptr<const P0ExecutionRiskSnapshot>& latest,
    const std::shared_ptr<const P0ExecutionRiskSnapshot>& grid_bound,
    const double evaluation_time_s) {
  if (latest && latest->localFreshAt(evaluation_time_s)) {
    return latest;
  }
  if (grid_bound && grid_bound != latest &&
      grid_bound->localFreshAt(evaluation_time_s)) {
    return grid_bound;
  }
  return nullptr;
}

std::shared_ptr<const P0ExecutionRiskSnapshot>
P0RiskGridRuntime::selectExecutionRiskSnapshotHistoryForEvaluation(
    const std::deque<std::shared_ptr<const P0ExecutionRiskSnapshot>>&
        completed,
    const std::shared_ptr<const P0ExecutionRiskSnapshot>& grid_bound,
    const double evaluation_time_s) {
  for (auto it = completed.rbegin(); it != completed.rend(); ++it) {
    if (*it && (*it)->localFreshAt(evaluation_time_s)) {
      return *it;
    }
  }
  if (grid_bound && grid_bound->localFreshAt(evaluation_time_s)) {
    return grid_bound;
  }
  return nullptr;
}

std::shared_ptr<const P0ExecutionRiskSnapshot>
P0RiskGridRuntime::acquireExecutionRiskSnapshotForEvaluation(
    const double evaluation_time_s) const {
  std::lock_guard<std::mutex> lock(planning_snapshot_mutex_);
  return selectExecutionRiskSnapshotHistoryForEvaluation(
      execution_snapshot_history_,
      planning_snapshot_ ? planning_snapshot_->execution : nullptr,
      evaluation_time_s);
}

std::shared_ptr<const iap::RiskGridSnapshot>
P0RiskGridRuntime::acquireSnapshot() const {
  const auto planning = acquirePlanningSnapshot();
  return planning ? planning->risk : nullptr;
}

bool P0RiskGridRuntime::gnssEpochFreshAt(
    const double epoch_stamp_s, const double evaluation_time_s) const {
  if (!std::isfinite(epoch_stamp_s) || !std::isfinite(evaluation_time_s)) {
    return false;
  }
  const double age_s = evaluation_time_s - epoch_stamp_s;
  return age_s >= -1.0e-6 &&
      (config_.gnss_epoch_max_age_s < 0.0 ||
       age_s <= config_.gnss_epoch_max_age_s);
}

bool P0RiskGridRuntime::executionSnapshotFreshAt(
    const std::shared_ptr<const P0ExecutionRiskSnapshot>& snapshot,
    const double evaluation_time_s) const {
  return snapshot && snapshot->freshAt(
      evaluation_time_s, config_.gnss_epoch_max_age_s);
}

bool P0RiskGridRuntime::executionSnapshotLocalFreshAt(
    const std::shared_ptr<const P0ExecutionRiskSnapshot>& snapshot,
    const double evaluation_time_s) const {
  return snapshot && snapshot->localFreshAt(evaluation_time_s);
}

bool P0RiskGridRuntime::executionSnapshotGlobalFreshAt(
    const std::shared_ptr<const P0ExecutionRiskSnapshot>& snapshot,
    const double evaluation_time_s) const {
  return snapshot && snapshot->globalFreshAt(
      evaluation_time_s, config_.gnss_epoch_max_age_s);
}

bool P0RiskGridRuntime::currentIntegrityForExecution(
    const double evaluation_time_s,
    iap::CurrentIntegrityState* current) const {
  if (!current || !std::isfinite(evaluation_time_s)) {
    return false;
  }
  std::lock_guard<std::mutex> lock(health_state_mutex_);
  if (!current_integrity_seen_) {
    return false;
  }

  // Input and planner callbacks are intentionally concurrent.  A callback for
  // t+dt may therefore be visible while the planner is still checking the
  // trajectory at t.  Use the newest causal sample, rather than rejecting the
  // whole monitor as "from the future".  Keep the latest-field fallback for
  // deterministic fixtures that seed state without invoking ROS callbacks.
  const iap::CurrentIntegrityState* selected = nullptr;
  bool selected_valid = false;
  double selected_stamp_s = -std::numeric_limits<double>::infinity();
  for (const auto& [generation, sample] : current_integrity_history_) {
    (void)generation;
    if (std::isfinite(sample.stamp) &&
        sample.stamp <= evaluation_time_s + 1.0e-6 &&
        sample.stamp >= selected_stamp_s) {
      selected = &sample;
      selected_valid = sample.valid;
      selected_stamp_s = sample.stamp;
    }
  }
  if (!selected && std::isfinite(latest_current_.stamp) &&
      latest_current_.stamp <= evaluation_time_s + 1.0e-6) {
    selected = &latest_current_;
    selected_valid = latest_current_valid_ && latest_current_.valid;
  }
  if (!selected || !selected_valid || !std::isfinite(selected->stamp)) {
    return false;
  }
  const double age_s = evaluation_time_s - selected->stamp;
  if (age_s < -1.0e-6 ||
      (config_.grid.stale_timeout_s >= 0.0 &&
       age_s > config_.grid.stale_timeout_s)) {
    return false;
  }
  *current = *selected;
  return true;
}

bool P0RiskGridRuntime::currentLocalHealthForExecution(
    const double evaluation_time_s,
    iap::CurrentIntegrityState* current) const {
  if (!current || !std::isfinite(evaluation_time_s)) {
    return false;
  }
  std::lock_guard<std::mutex> lock(health_state_mutex_);
  if (!current_integrity_seen_) {
    return false;
  }

  const iap::CurrentIntegrityState* selected = nullptr;
  double selected_stamp_s = -std::numeric_limits<double>::infinity();
  for (const auto& [generation, sample] : current_integrity_history_) {
    (void)generation;
    if (std::isfinite(sample.stamp) &&
        sample.stamp <= evaluation_time_s + 1.0e-6 &&
        sample.stamp >= selected_stamp_s) {
      selected = &sample;
      selected_stamp_s = sample.stamp;
    }
  }
  if (!selected && std::isfinite(latest_current_.stamp) &&
      latest_current_.stamp <= evaluation_time_s + 1.0e-6) {
    selected = &latest_current_;
  }
  if (!selected || !std::isfinite(selected->stamp) ||
      !p0LocalSlamHealthValid(*selected)) {
    return false;
  }
  const double age_s = evaluation_time_s - selected->stamp;
  if (age_s < -1.0e-6 ||
      (config_.grid.stale_timeout_s >= 0.0 &&
       age_s > config_.grid.stale_timeout_s)) {
    return false;
  }
  *current = *selected;
  return true;
}

bool P0RiskGridRuntime::refreshOnceForTest() {
  synchronous_test_refresh_.store(true, std::memory_order_release);
  refreshTimerCallback();
  synchronous_test_refresh_.store(false, std::memory_order_release);
  std::lock_guard<std::mutex> lock(health_state_mutex_);
  return last_refresh_succeeded_;
}

void P0RiskGridRuntime::setOccupancyPredicate(
    iap::RiskGridMap::OccupancyPredicate predicate) {
  std::lock_guard<std::mutex> lock(source_factory_mutex_);
  occupancy_predicate_ = std::move(predicate);
}

void P0RiskGridRuntime::setOccupancyDiagnosticQuery(
    iap::RiskGridMap::OccupancyDiagnosticQuery query) {
  std::lock_guard<std::mutex> lock(source_factory_mutex_);
  occupancy_diagnostic_query_ = std::move(query);
}

void P0RiskGridRuntime::setOccupancyDiagnosticQueryFactory(
    std::function<iap::RiskGridMap::OccupancyDiagnosticQuery()> factory) {
  std::lock_guard<std::mutex> lock(source_factory_mutex_);
  occupancy_diagnostic_query_factory_ = std::move(factory);
}

void P0RiskGridRuntime::setOccupancyEpochFactory(
    std::function<P0OccupancyEpochCapture()> factory) {
  std::lock_guard<std::mutex> lock(source_factory_mutex_);
  occupancy_epoch_factory_ = std::move(factory);
}

void P0RiskGridRuntime::setExecutionOccupancyEpochFactory(
    std::function<P0OccupancyEpochCapture()> factory) {
  std::lock_guard<std::mutex> lock(source_factory_mutex_);
  execution_occupancy_epoch_factory_ = std::move(factory);
}

void P0RiskGridRuntime::setOccupancyGenerationProvider(
    std::function<uint64_t()> provider) {
  std::lock_guard<std::mutex> lock(source_factory_mutex_);
  occupancy_generation_provider_ = std::move(provider);
}

void P0RiskGridRuntime::notifyOccupancyCommitted(
    const uint64_t generation, const double source_stamp_s) {
  if (generation == 0u || provider_) {
    return;
  }
  ExecutionSnapshotRequest request;
  request.generation = generation;
  request.source_stamp_s = source_stamp_s;
  request.request_ros_stamp_s = diagnosticRosNowSeconds();
  request.request_steady_s = steadyNowSeconds();
  {
    std::lock_guard<std::mutex> lock(execution_snapshot_worker_mutex_);
    if (execution_snapshot_worker_stop_ ||
        generation <= last_requested_occupancy_generation_) {
      return;
    }
    last_requested_occupancy_generation_ = generation;
    if (pending_execution_snapshot_request_) {
      ++execution_snapshot_pending_overwrite_count_;
    }
    request.overwritten_count = execution_snapshot_pending_overwrite_count_;
    pending_execution_snapshot_request_ = request;
  }
  execution_snapshot_worker_cv_.notify_all();
}

P0ExecutionSnapshotAttemptEvidence
P0RiskGridRuntime::lastExecutionSnapshotAttempt() const {
  std::lock_guard<std::mutex> lock(execution_snapshot_worker_mutex_);
  return last_execution_snapshot_attempt_;
}

bool P0RiskGridRuntime::yieldRiskGridToExecutionSnapshot(
    const std::chrono::steady_clock::time_point deadline) {
  std::unique_lock<std::mutex> lock(execution_snapshot_worker_mutex_);
  if (!pending_execution_snapshot_request_ &&
      !execution_snapshot_worker_in_flight_) {
    return std::chrono::steady_clock::now() < deadline;
  }
  // Keep two RiskGrid workers running when the configured pool has parallel
  // capacity. One worker missed the live 500 ms budget by 6--12 ms; two still
  // leave most of the pool to the execution builder while preventing every
  // 10 Hz snapshot from becoming a full-pool stop. A single-worker grid yields
  // completely, as execution authorization has priority.
  const std::size_t worker_count = static_cast<std::size_t>(
      std::max(1, config_.predictor_effective_worker_count));
  const std::size_t retained_grid_workers = worker_count > 2u ? 2u : 1u;
  const std::size_t waiter_limit = worker_count > retained_grid_workers
      ? worker_count - retained_grid_workers : 1u;
  if (risk_grid_yield_waiter_count_ >= waiter_limit) {
    return std::chrono::steady_clock::now() < deadline;
  }
  const bool records_episode = risk_grid_yield_waiter_count_ == 0u;
  ++risk_grid_yield_waiter_count_;
  const auto start = std::chrono::steady_clock::now();
  if (records_episode) {
    ++risk_grid_yield_count_;
  }
  // A timed wait made the slowest worker absorb every 10 Hz snapshot build;
  // even 25 us at each of tens of thousands of batch boundaries added about
  // 55 ms to the live BDS p95. Relinquish the current scheduler timeslice
  // instead. Four of six grid workers take this path while two retain forward
  // progress, so the execution thread becomes runnable without manufacturing
  // a fixed delay at every boundary.
  lock.unlock();
  std::this_thread::yield();
  lock.lock();
  if (records_episode) {
    risk_grid_yield_duration_ms_ +=
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
  }
  if (risk_grid_yield_waiter_count_ > 0u) {
    --risk_grid_yield_waiter_count_;
  }
  execution_snapshot_worker_cv_.notify_all();
  return !execution_snapshot_worker_stop_ &&
      std::chrono::steady_clock::now() < deadline;
}

bool P0RiskGridRuntime::p0_6_fixture_occupied(
    const Eigen::Vector3d& pos) const {
  const auto& fixture = config_.p0_6_fixture;
  if (!fixture.enabled || fixture.name != "occupied_overlap_box_v1" ||
      !pos.allFinite()) {
    return false;
  }
  return pos.x() >= fixture.x_min_m && pos.x() <= fixture.x_max_m &&
         pos.y() >= fixture.y_min_m && pos.y() <= fixture.y_max_m &&
         pos.z() >= fixture.z_min_m && pos.z() <= fixture.z_max_m;
}

iap::RiskGridMap::OccupancyPredicate
P0RiskGridRuntime::combinedOccupancyPredicate() const {
  iap::RiskGridMap::OccupancyPredicate occupancy_predicate;
  {
    std::lock_guard<std::mutex> lock(source_factory_mutex_);
    occupancy_predicate = occupancy_predicate_;
  }
  const bool fixture_enabled =
      config_.p0_6_fixture.enabled &&
      config_.p0_6_fixture.name == "occupied_overlap_box_v1";
  if (!fixture_enabled) {
    return occupancy_predicate;
  }
  return [this, occupancy_predicate = std::move(occupancy_predicate)](
             const Eigen::Vector3d& pos) {
    if (occupancy_predicate && occupancy_predicate(pos)) {
      return true;
    }
    return p0_6_fixture_occupied(pos);
  };
}

iap::RiskGridMap::OccupancyDiagnosticQuery
P0RiskGridRuntime::combinedOccupancyDiagnosticQuery(
    iap::RiskGridMap::OccupancyDiagnosticQuery base_query) const {
  auto query = std::move(base_query);
  bool factory_configured = false;
  if (!query) {
    std::function<iap::RiskGridMap::OccupancyDiagnosticQuery()> factory;
    {
      std::lock_guard<std::mutex> lock(source_factory_mutex_);
      factory = occupancy_diagnostic_query_factory_;
      query = occupancy_diagnostic_query_;
    }
    factory_configured = static_cast<bool>(factory);
    if (factory) {
      query = factory();
    }
  }
  if (!query && factory_configured) {
    query = [](const Eigen::Vector3d&) {
      iap::RiskOccupancyDiagnostic diagnostic;
      diagnostic.source = "occupancy_snapshot_unavailable";
      return diagnostic;
    };
  }
  if (!query) {
    return {};
  }
  const bool fixture_enabled =
      config_.p0_6_fixture.enabled &&
      config_.p0_6_fixture.name == "occupied_overlap_box_v1";
  if (!fixture_enabled) {
    return query;
  }
  return [this, query = std::move(query)](const Eigen::Vector3d& pos) {
    auto diagnostic = query(pos);
    if (p0_6_fixture_occupied(pos)) {
      diagnostic.available = true;
      diagnostic.raw_occupied = true;
      diagnostic.inflated_occupied = true;
      diagnostic.source = "p0_6_fixture";
    }
    return diagnostic;
  };
}

void P0RiskGridRuntime::createRosInterfaces() {
  if (!node_ || !config_.enable_risk_grid) {
    return;
  }
  input_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  predictor_input_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  map_input_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  execution_snapshot_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  refresh_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  health_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = input_callback_group_;
  rclcpp::SubscriptionOptions predictor_subscription_options;
  predictor_subscription_options.callback_group =
      predictor_input_callback_group_;
  rclcpp::SubscriptionOptions map_subscription_options;
  map_subscription_options.callback_group = map_input_callback_group_;
  const rclcpp::QoS qos(50);
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      config_.odom_topic, qos,
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        odomCallback(msg);
      },
      subscription_options);
  integrity_sub_ = node_->create_subscription<iap::msg::IntegrityReport>(
      config_.integrity_topic, qos,
      [this](const iap::msg::IntegrityReport::ConstSharedPtr msg) {
        integrityCallback(msg);
      },
      subscription_options);
  range_sub_ = node_->create_subscription<gnss_comm::msg::GnssMeasMsg>(
      config_.range_meas_topic, qos,
      [this](const gnss_comm::msg::GnssMeasMsg::ConstSharedPtr msg) {
        rangeCallback(msg);
      },
      predictor_subscription_options);
  ephem_sub_ = node_->create_subscription<gnss_comm::msg::GnssEphemMsg>(
      config_.ephem_topic, qos,
          [this](const gnss_comm::msg::GnssEphemMsg::ConstSharedPtr msg) {
            ephemCallback(msg);
          },
          predictor_subscription_options);
  glo_ephem_sub_ =
      node_->create_subscription<gnss_comm::msg::GnssGloEphemMsg>(
          config_.glo_ephem_topic, qos,
          [this](const gnss_comm::msg::GnssGloEphemMsg::ConstSharedPtr msg) {
            gloEphemCallback(msg);
          },
          predictor_subscription_options);
  receiver_lla_sub_ =
      node_->create_subscription<sensor_msgs::msg::NavSatFix>(
          config_.receiver_lla_topic, qos,
          [this](const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
            receiverLlaCallback(msg);
          },
          predictor_subscription_options);
  iono_sub_ =
      node_->create_subscription<gnss_comm::msg::GnssIonosphereParameter>(
          config_.iono_topic, qos,
          [this](const gnss_comm::msg::GnssIonosphereParameter::ConstSharedPtr msg) {
            ionoCallback(msg);
          },
          predictor_subscription_options);
  if (!config_.online_mapping_mode) {
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.map_topic, rclcpp::QoS(2).transient_local().reliable(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          cloudCallback(msg);
        },
        map_subscription_options);
  }

  // Do not couple the machine-readable health contract to RViz/debug flags.
  health_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/planning/risk_grid_health", 10);
  execution_snapshot_attempt_pub_ =
      node_->create_publisher<std_msgs::msg::String>(
          "/planning/execution_snapshot_attempt", 20);
  safety_viz_ = std::make_shared<SafetyRvizPublisher>(
      node_, SafetyRvizPublisher::declareAndReadConfig(node_));
  const double period_s =
      std::max(0.001, config_.grid.refresh_period_s);
  const double refresh_start_delay_s =
      std::max(0.0, config_.refresh_start_delay_s);
  auto start_periodic_refresh = [this, period_s]() {
    refresh_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(period_s),
        [this]() { refreshTimerCallback(); }, refresh_callback_group_);
    refreshTimerCallback();
  };
  {
    std::lock_guard<std::mutex> lock(health_state_mutex_);
    next_refresh_scheduled_steady_s_ =
        steadyNowSeconds() + period_s + refresh_start_delay_s;
  }
  if (refresh_start_delay_s > 0.0) {
    refresh_start_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(period_s + refresh_start_delay_s),
        [this, start_periodic_refresh]() {
          refresh_start_timer_->cancel();
          start_periodic_refresh();
        },
        refresh_callback_group_);
  } else {
    refresh_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(period_s),
        [this]() { refreshTimerCallback(); }, refresh_callback_group_);
  }
  execution_snapshot_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(config_.execution_snapshot_period_s),
      [this]() { executionSnapshotTimerCallback(); },
      execution_snapshot_callback_group_);
  // Health must remain observable while a full grid refresh is evaluating a
  // large predictor batch.  It intentionally publishes the latest snapshot
  // state rather than waiting for that batch to finish.
  health_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(period_s),
      [this]() { healthTimerCallback(); },
      health_callback_group_);
}

void P0RiskGridRuntime::executionSnapshotTimerCallback() {
  // Test providers deliberately own their deterministic snapshot lifecycle in
  // refreshOnceForTest().  This timer is only a lost-notification watchdog;
  // normal production publication is driven by occupancy commit generations.
  std::function<P0OccupancyEpochCapture()> occupancy_factory;
  std::function<uint64_t()> generation_provider;
  {
    std::lock_guard<std::mutex> lock(source_factory_mutex_);
    occupancy_factory = occupancy_epoch_factory_;
    generation_provider = occupancy_generation_provider_;
  }
  if (!config_.enable_risk_grid || provider_ || !occupancy_factory) {
    return;
  }
  const uint64_t generation = generation_provider
      ? generation_provider() : 0u;
  if (generation != 0u) {
    notifyOccupancyCommitted(
        generation, std::numeric_limits<double>::quiet_NaN());
  }
}

void P0RiskGridRuntime::executionSnapshotWorkerLoop() {
  while (true) {
    ExecutionSnapshotRequest request;
    {
      std::unique_lock<std::mutex> lock(execution_snapshot_worker_mutex_);
      execution_snapshot_worker_cv_.wait(lock, [this]() {
        return execution_snapshot_worker_stop_ ||
            pending_execution_snapshot_request_.has_value();
      });
      if (execution_snapshot_worker_stop_) {
        return;
      }
      request = *pending_execution_snapshot_request_;
      pending_execution_snapshot_request_.reset();
      execution_snapshot_worker_in_flight_ = true;
    }
    try {
      buildAndPublishExecutionSnapshot(request);
    } catch (const std::exception& error) {
      P0ExecutionSnapshotAttemptEvidence evidence;
      evidence.attempt_id = next_execution_snapshot_attempt_id_.fetch_add(
          1, std::memory_order_relaxed);
      evidence.requested_occupancy_generation = request.generation;
      evidence.requested_occupancy_stamp_s = request.source_stamp_s;
      evidence.request_ros_stamp_s = request.request_ros_stamp_s;
      evidence.request_steady_s = request.request_steady_s;
      evidence.start_ros_stamp_s = diagnosticRosNowSeconds();
      evidence.start_steady_s = steadyNowSeconds();
      evidence.status = P0ExecutionSnapshotAttemptStatus::WORKER_EXCEPTION;
      evidence.reason = std::string("execution_snapshot_worker_exception:") +
          error.what();
      recordExecutionSnapshotAttempt(std::move(evidence));
    } catch (...) {
      P0ExecutionSnapshotAttemptEvidence evidence;
      evidence.attempt_id = next_execution_snapshot_attempt_id_.fetch_add(
          1, std::memory_order_relaxed);
      evidence.requested_occupancy_generation = request.generation;
      evidence.requested_occupancy_stamp_s = request.source_stamp_s;
      evidence.request_ros_stamp_s = request.request_ros_stamp_s;
      evidence.request_steady_s = request.request_steady_s;
      evidence.start_ros_stamp_s = diagnosticRosNowSeconds();
      evidence.start_steady_s = steadyNowSeconds();
      evidence.status = P0ExecutionSnapshotAttemptStatus::WORKER_EXCEPTION;
      evidence.reason = "execution_snapshot_worker_unknown_exception";
      recordExecutionSnapshotAttempt(std::move(evidence));
    }
    {
      std::lock_guard<std::mutex> lock(execution_snapshot_worker_mutex_);
      execution_snapshot_worker_in_flight_ = false;
    }
    execution_snapshot_worker_cv_.notify_all();
  }
}

void P0RiskGridRuntime::recordExecutionSnapshotAttempt(
    P0ExecutionSnapshotAttemptEvidence evidence) {
  evidence.finish_ros_stamp_s = diagnosticRosNowSeconds();
  evidence.finish_steady_s = steadyNowSeconds();
  evidence.queue_delay_ms =
      1000.0 * (evidence.start_steady_s - evidence.request_steady_s);
  evidence.build_duration_ms =
      1000.0 * (evidence.finish_steady_s - evidence.start_steady_s);
  {
    std::lock_guard<std::mutex> lock(execution_snapshot_worker_mutex_);
    evidence.pending_overwrite_count =
        execution_snapshot_pending_overwrite_count_;
    evidence.risk_grid_yield_count = risk_grid_yield_count_;
    evidence.risk_grid_yield_duration_ms = risk_grid_yield_duration_ms_;
    last_execution_snapshot_attempt_ = evidence;
    ++execution_snapshot_attempt_count_;
    if (evidence.status == P0ExecutionSnapshotAttemptStatus::PUBLISHED) {
      ++execution_snapshot_publish_count_;
    } else if (evidence.status !=
                   P0ExecutionSnapshotAttemptStatus::DEDUPLICATED) {
      ++execution_snapshot_failure_count_;
      last_execution_snapshot_failure_ = evidence;
    }
  }
  execution_snapshot_worker_cv_.notify_all();

  if (execution_snapshot_attempt_pub_) {
    std_msgs::msg::String msg;
    std::ostringstream json;
    json << std::setprecision(17)
         << "{\"schema\":\"p0_execution_snapshot_attempt_v1\""
         << ",\"attempt_id\":" << evidence.attempt_id
         << ",\"status\":\""
         << p0ExecutionSnapshotAttemptStatusName(evidence.status) << "\""
         << ",\"reason\":" << jsonString(evidence.reason)
         << ",\"requested_occupancy_generation\":"
         << evidence.requested_occupancy_generation
         << ",\"requested_occupancy_stamp_s\":"
         << jsonNumber(evidence.requested_occupancy_stamp_s)
         << ",\"captured_occupancy_generation\":"
         << evidence.captured_occupancy_generation
         << ",\"captured_occupancy_stamp_s\":"
         << jsonNumber(evidence.captured_occupancy_stamp_s)
         << ",\"support_stamp_s\":" << jsonNumber(evidence.support_stamp_s)
         << ",\"support_generation\":" << evidence.support_generation
         << ",\"lidar_stamp_s\":" << jsonNumber(evidence.lidar_stamp_s)
         << ",\"lidar_generation\":" << evidence.lidar_generation
         << ",\"lidar_receive_steady_s\":"
         << jsonNumber(evidence.lidar_receive_steady_s)
         << ",\"gnss_epoch_stamp_s\":"
         << jsonNumber(evidence.gnss_epoch_stamp_s)
         << ",\"gnss_epoch_generation\":"
         << evidence.gnss_epoch_generation
         << ",\"gnss_epoch_identity\":" << evidence.gnss_epoch_identity
         << ",\"integrity_stamp_s\":"
         << jsonNumber(evidence.integrity_stamp_s)
         << ",\"integrity_generation\":"
         << evidence.integrity_generation
         << ",\"execution_snapshot_id\":"
         << evidence.published_execution_snapshot_id
         << ",\"request_ros_stamp_s\":"
         << jsonNumber(evidence.request_ros_stamp_s)
         << ",\"start_ros_stamp_s\":"
         << jsonNumber(evidence.start_ros_stamp_s)
         << ",\"occupancy_capture_finish_ros_stamp_s\":"
         << jsonNumber(evidence.occupancy_capture_finish_ros_stamp_s)
         << ",\"support_ready_ros_stamp_s\":"
         << jsonNumber(evidence.occupancy_capture_finish_ros_stamp_s)
         << ",\"predictor_ready_ros_stamp_s\":"
         << jsonNumber(evidence.predictor_ready_ros_stamp_s)
         << ",\"snapshot_publish_ros_stamp_s\":"
         << jsonNumber(evidence.snapshot_publish_ros_stamp_s)
         << ",\"finish_ros_stamp_s\":"
         << jsonNumber(evidence.finish_ros_stamp_s)
         << ",\"request_steady_s\":"
         << jsonNumber(evidence.request_steady_s)
         << ",\"start_steady_s\":"
         << jsonNumber(evidence.start_steady_s)
         << ",\"occupancy_capture_finish_steady_s\":"
         << jsonNumber(evidence.occupancy_capture_finish_steady_s)
         << ",\"support_ready_steady_s\":"
         << jsonNumber(evidence.occupancy_capture_finish_steady_s)
         << ",\"predictor_ready_steady_s\":"
         << jsonNumber(evidence.predictor_ready_steady_s)
         << ",\"finish_steady_s\":"
         << jsonNumber(evidence.finish_steady_s)
         << ",\"queue_delay_ms\":" << jsonNumber(evidence.queue_delay_ms)
         << ",\"build_duration_ms\":"
         << jsonNumber(evidence.build_duration_ms)
         << ",\"publish_age_s\":" << jsonNumber(evidence.publish_age_s)
         << ",\"pending_overwrite_count_at_request\":"
         << evidence.pending_overwrite_count_at_request
         << ",\"pending_overwrite_count\":"
         << evidence.pending_overwrite_count
         << ",\"risk_grid_yield_count\":"
         << evidence.risk_grid_yield_count
         << ",\"risk_grid_yield_duration_ms\":"
         << evidence.risk_grid_yield_duration_ms
         << ",\"frame_contract_id\":"
         << jsonString(evidence.frame_contract_id)
         << ",\"geometry_id\":" << jsonString(evidence.geometry_id)
         << ",\"support_identity\":"
         << jsonString(evidence.support_identity) << "}";
    msg.data = json.str();
    execution_snapshot_attempt_pub_->publish(msg);
  }
}

void P0RiskGridRuntime::buildAndPublishExecutionSnapshot(
    const ExecutionSnapshotRequest& request) {
  P0ExecutionSnapshotAttemptEvidence evidence;
  evidence.attempt_id = next_execution_snapshot_attempt_id_.fetch_add(
      1, std::memory_order_relaxed);
  evidence.requested_occupancy_generation = request.generation;
  evidence.requested_occupancy_stamp_s = request.source_stamp_s;
  evidence.pending_overwrite_count_at_request = request.overwritten_count;
  evidence.start_ros_stamp_s = diagnosticRosNowSeconds();
  evidence.start_steady_s = steadyNowSeconds();
  evidence.request_ros_stamp_s = std::isfinite(request.request_ros_stamp_s)
      ? request.request_ros_stamp_s : evidence.start_ros_stamp_s;
  evidence.request_steady_s = std::isfinite(request.request_steady_s)
      ? request.request_steady_s : evidence.start_steady_s;
  const auto finish = [this, &evidence](
                          const P0ExecutionSnapshotAttemptStatus status,
                          const char* reason) {
    evidence.status = status;
    evidence.reason = reason;
    recordExecutionSnapshotAttempt(evidence);
  };
  const auto build_start = std::chrono::steady_clock::now();
  double evaluation_time_s = liveNowSeconds();
  std::function<P0OccupancyEpochCapture()> execution_factory;
  {
    std::lock_guard<std::mutex> lock(source_factory_mutex_);
    execution_factory = execution_occupancy_epoch_factory_
        ? execution_occupancy_epoch_factory_ : occupancy_epoch_factory_;
  }
  if (!execution_factory) {
    finish(P0ExecutionSnapshotAttemptStatus::CAPTURE_UNAVAILABLE,
           "occupancy_capture_factory_missing");
    return;
  }
  P0OccupancyEpochCapture capture = execution_factory();
  evidence.occupancy_capture_finish_ros_stamp_s =
      diagnosticRosNowSeconds();
  evidence.occupancy_capture_finish_steady_s = steadyNowSeconds();
  if (capture.status != P0OccupancyEpochCaptureStatus::VALID ||
      !capture.epoch) {
    finish(capture.status == P0OccupancyEpochCaptureStatus::ADAPTER_INVALID
               ? P0ExecutionSnapshotAttemptStatus::CAPTURE_ADAPTER_INVALID
               : P0ExecutionSnapshotAttemptStatus::CAPTURE_UNAVAILABLE,
           capture.status == P0OccupancyEpochCaptureStatus::ADAPTER_INVALID
               ? "occupancy_capture_adapter_invalid"
               : "occupancy_capture_unavailable");
    return;
  }
  auto occupancy = std::make_shared<P0OccupancyEpoch>(
      std::move(*capture.epoch));
  evidence.captured_occupancy_generation = occupancy->generation;
  evidence.captured_occupancy_stamp_s = occupancy->cloud_stamp_s;
  evidence.lidar_generation = occupancy->generation;
  evidence.lidar_stamp_s = occupancy->cloud_stamp_s;
  evidence.frame_contract_id = occupancy->frame_contract_id;
  evidence.geometry_id = occupancy->geometry.geometry_id;
  evidence.support_stamp_s = occupancy->cloud_stamp_s;
  evidence.support_generation = occupancy->generation;
  evidence.support_identity = "strict_observation";
  if (occupancy->trusted_local_map_support) {
    evidence.support_stamp_s = occupancy->trusted_local_map_support->stamp_s;
    evidence.support_generation = occupancy->generation;
    evidence.lidar_receive_steady_s = occupancy->trusted_local_map_support->
        sensor_receipt_steady_s;
    evidence.support_identity =
        occupancy->trusted_local_map_support->identity();
  }
  if (config_.online_mapping_mode &&
      std::isfinite(occupancy->cloud_stamp_s)) {
    evaluation_time_s = std::max(
        evaluation_time_s, occupancy->cloud_stamp_s);
  }
  if (!occupancy->diagnostic_query || !occupancy->los_owner ||
      !occupancy->source_owner || !occupancy->live_source_owner ||
      !occupancy->live_generation || occupancy->generation == 0u ||
      occupancy->frame_id != "map" ||
      !geometryMatchesRiskOverlay(occupancy->geometry, config_.grid, 5)) {
    finish(P0ExecutionSnapshotAttemptStatus::OCCUPANCY_INVALID,
           "occupancy_epoch_incomplete_or_geometry_mismatch");
    return;
  }
  if (request.generation != 0u && occupancy->generation < request.generation) {
    finish(P0ExecutionSnapshotAttemptStatus::SUPERSEDED,
           "captured_generation_precedes_requested_generation");
    return;
  }
  bool superseded_after_capture = false;
  {
    std::lock_guard<std::mutex> lock(execution_snapshot_worker_mutex_);
    superseded_after_capture =
        occupancy->generation < last_requested_occupancy_generation_;
  }
  if (superseded_after_capture) {
    finish(P0ExecutionSnapshotAttemptStatus::SUPERSEDED,
           "captured_generation_superseded_while_in_flight");
    return;
  }
  const double occupancy_age_s =
      evaluation_time_s - occupancy->cloud_stamp_s;
  if (!std::isfinite(occupancy_age_s) || occupancy_age_s < -1.0e-6 ||
      (config_.grid.stale_timeout_s >= 0.0 &&
       occupancy_age_s > config_.grid.stale_timeout_s)) {
    finish(P0ExecutionSnapshotAttemptStatus::OCCUPANCY_STALE,
           "occupancy_source_age_exceeded");
    return;
  }

  iap::IntegritySnapshot integrity;
  PredictorSourceCapture sources;
  InputReadiness readiness;
  const bool snapshot_built = buildSnapshot(
      evaluation_time_s, &integrity, &sources, &readiness);
  const bool current_authority_available =
      config_.task_mode == iap::GlobalNavigationTaskMode::STRICT_GLOBAL
          ? integrity.current.valid
          : p0LocalSlamHealthValid(integrity.current);
  if (!snapshot_built || !current_authority_available ||
      !std::isfinite(integrity.current.stamp) ||
      evaluation_time_s < integrity.current.stamp ||
      evaluation_time_s - integrity.current.stamp >
          config_.grid.stale_timeout_s) {
    finish(P0ExecutionSnapshotAttemptStatus::INTEGRITY_UNAVAILABLE,
           "integrity_snapshot_unavailable_or_stale");
    return;
  }
  evidence.integrity_stamp_s = integrity.current.stamp;
  evidence.integrity_generation = sources.current_generation;
  evidence.gnss_epoch_stamp_s = sources.gnss_epoch_stamp;
  evidence.gnss_epoch_generation = sources.gnss_epoch_generation;
  evidence.gnss_epoch_identity = integrity.current.gnss_epoch_identity;
  // All captured sources use the same ROS/simulator clock, but callbacks can
  // be delivered out of order by a few tens of milliseconds.  Evaluate the
  // immutable tuple at its newest source stamp so a GNSS epoch that arrived
  // just ahead of odometry/occupancy is not mislabeled as "future" data.
  // This does not extend any freshness timeout: older tuple members are still
  // checked against the unchanged one-second age bound below and in freshAt.
  const auto advance_evaluation_time = [&evaluation_time_s](
      const double source_stamp_s) {
    if (std::isfinite(source_stamp_s)) {
      evaluation_time_s = std::max(evaluation_time_s, source_stamp_s);
    }
  };
  advance_evaluation_time(occupancy->cloud_stamp_s);
  advance_evaluation_time(integrity.current.stamp);
  advance_evaluation_time(sources.current_stamp);
  advance_evaluation_time(sources.gnss_epoch_stamp);
  // Global GNSS alert-limit status is carried by the immutable snapshot but
  // is not a prerequisite for publishing local occupancy/support authority.
  // STRICT_GLOBAL enforces it at trajectory assurance; best-effort missions
  // may still move when local motion and certified braking remain safe.

  iap::PredictorParams predictor_params;
  predictor_params.freshness.enabled = true;
  predictor_params.freshness.max_odom_age_s =
      config_.grid.stale_timeout_s;
  predictor_params.freshness.max_integrity_age_s =
      config_.grid.stale_timeout_s;
  predictor_params.freshness.max_gnss_age_s = config_.gnss_epoch_max_age_s;
  predictor_params.freshness.max_snapshot_age_s =
      config_.grid.stale_timeout_s;
  predictor_params.source_mode = config_.predictor_source_mode;
  predictor_params.gnss.visibility_params.clearance_transition_m =
      config_.predictor_gnss_clearance_transition_m;
  predictor_params.gnss_epoch_policy = config_.predictor_gnss_epoch_policy;
  predictor_params.gnss.measured_epoch_support_radius_m =
      config_.predictor_gnss_measured_epoch_support_radius_m;
  predictor_params.gnss.measured_epoch_integrity_max_delta_s =
      config_.predictor_gnss_measured_epoch_integrity_max_delta_s;
  predictor_params.fusion.conservative_max_with_gnss =
      config_.predictor_conservative_max_with_gnss;
  predictor_params.lidar.enable_legacy_observability =
      config_.predictor_lidar_legacy_observability;
  predictor_params.covariance_growth.sigma_grow_m_sqrt_s =
      config_.predictor_sigma_grow_m_sqrt_s;
  if (std::isfinite(config_.predictor_lidar_fim_radius_m) &&
      config_.predictor_lidar_fim_radius_m > 0.0) {
    predictor_params.lidar.fim_params.fim_radius_m =
        config_.predictor_lidar_fim_radius_m;
    predictor_params.lidar.fim_params.search_radius_m =
        config_.predictor_lidar_fim_radius_m;
  }

  const auto trusted_support = occupancy->trusted_local_map_support;
  iap::RiskGridSourceIdentity source_identity;
  source_identity.occupancy_generation = occupancy->generation;
  source_identity.occupancy_stamp_s = occupancy->cloud_stamp_s;
  source_identity.prior_generation = sources.current_generation;
  source_identity.prior_stamp_s = sources.current_stamp;
  source_identity.gnss_generation = sources.gnss_epoch_generation;
  source_identity.gnss_stamp_s = sources.gnss_epoch_stamp;
  source_identity.gnss_epoch_identity =
      integrity.current.gnss_epoch_identity;
  source_identity.lidar_generation = occupancy->generation;
  source_identity.lidar_stamp_s = occupancy->cloud_stamp_s;
  source_identity.local_map_support_identity = trusted_support
      ? trusted_support->identity() : "strict_observation";
  std::ostringstream algorithm_identity;
  algorithm_identity << std::setprecision(17)
      << "forward_risk_v4;gnss_support_model=continuous_unknown_fraction_v1;"
         "clearance_transition_m="
      << config_.predictor_gnss_clearance_transition_m
      << ";admission_epochs=" << config_.predictor_gnss_admission_epochs
      << ";geometry_solver=sherman_morrison_v1"
      << ";execution_satellite_set=braking_window_core_v2"
      << ";task_mode="
      << iap::globalNavigationTaskModeName(config_.task_mode);
  source_identity.predictor_algorithm_identity = algorithm_identity.str();
  source_identity.alert_limit_policy_id =
      config_.grid.alert_limit_policy_id;
  const std::string source_hash =
      iap::canonicalRiskGridSourceIdentityHash(source_identity);
  bool duplicate_tuple = false;
  {
    std::lock_guard<std::mutex> lock(planning_snapshot_mutex_);
    if (execution_snapshot_ &&
        iap::canonicalRiskGridSourceIdentityHash(
            execution_snapshot_->source_identity) == source_hash &&
        execution_snapshot_->frame_contract_id ==
            occupancy->frame_contract_id &&
        execution_snapshot_->geometry_id == occupancy->geometry.geometry_id &&
        iap::canonicalRiskGridConfigHash(execution_snapshot_->risk_policy) ==
            iap::canonicalRiskGridConfigHash(config_.grid)) {
      duplicate_tuple = true;
      evidence.published_execution_snapshot_id =
          execution_snapshot_->execution_snapshot_id;
    }
  }
  if (duplicate_tuple) {
    evidence.publish_age_s = evaluation_time_s - occupancy->cloud_stamp_s;
    finish(P0ExecutionSnapshotAttemptStatus::DEDUPLICATED,
           "input_tuple_already_published");
    return;
  }

  const auto map_points = occupancy->environment_occupied_voxel_centers;
  if (!map_points) {
    finish(P0ExecutionSnapshotAttemptStatus::MAP_POINTS_MISSING,
           "environment_obstacle_centers_missing");
    return;
  }
  iap::LidarFimPrimitiveGenerationDiagnostics fim_diagnostics;
  std::shared_ptr<const std::vector<iap::LidarFimPrimitive>> fim_primitives;
  bool reused_fim_primitives = false;
  {
    std::lock_guard<std::mutex> lock(lidar_predictor_input_mutex_);
    if (latest_lidar_generation_ == occupancy->generation &&
        latest_lidar_stamp_ == occupancy->cloud_stamp_s &&
        sameSharedOwner(latest_lidar_map_points_, map_points) &&
        latest_lidar_fim_primitives_) {
      fim_primitives = latest_lidar_fim_primitives_;
      fim_diagnostics = latest_lidar_fim_diagnostics_;
      reused_fim_primitives = true;
    }
  }
  if (!reused_fim_primitives) {
    fim_primitives = iap::make_lidar_fim_primitives(
        *map_points, nullptr, iap::LidarFimPrimitiveGenerationParams{},
        &fim_diagnostics);
  }
  {
    std::lock_guard<std::mutex> lock(lidar_predictor_input_mutex_);
    if (occupancy->generation >= latest_lidar_generation_) {
      latest_lidar_map_points_ = map_points;
      latest_lidar_fim_primitives_ = fim_primitives;
      latest_lidar_fim_diagnostics_ = fim_diagnostics;
      latest_lidar_map_point_count_ = map_points->size();
      latest_lidar_fim_primitive_count_ =
          fim_primitives ? fim_primitives->size() : 0u;
      latest_lidar_fim_valid_normal_count_ = static_cast<std::size_t>(
          std::max(0, fim_diagnostics.lidar_pca_valid_normals));
      latest_lidar_fim_fallback_reason_ = fim_diagnostics.fallback_reason;
      latest_lidar_generation_ = occupancy->generation;
      latest_lidar_stamp_ = occupancy->cloud_stamp_s;
    }
  }
  auto module = std::make_shared<iap::PredictorModule>(predictor_params);
  module->set_local_occupancy(occupancy->los_owner.get());
  const auto observed_support_query = occupancy->diagnostic_query;
  if (trusted_support) {
    module->set_support_query(
        [occupancy](const Eigen::Vector3d& position,
                    const double evaluation_time,
                    const double query_time) {
          return queryP0LocalMapSupport(
              *occupancy, position, evaluation_time, query_time);
        });
  } else {
    module->set_observation_predicate(
        [observed_support_query](const Eigen::Vector3d& position) {
          const auto diagnostic = observed_support_query(position);
          return diagnostic.available && diagnostic.observed &&
                 diagnostic.state != iap::RiskOccupancyState::UNKNOWN;
        });
  }
  module->set_lidar_map_points(map_points);
  module->set_lidar_fim_primitives(fim_primitives);
  evidence.predictor_ready_ros_stamp_s = diagnosticRosNowSeconds();
  evidence.predictor_ready_steady_s = steadyNowSeconds();

  auto execution = std::make_shared<P0ExecutionRiskSnapshot>();
  execution->execution_snapshot_id =
      next_execution_snapshot_id_.fetch_add(1, std::memory_order_relaxed);
  if (execution->execution_snapshot_id == 0u) {
    execution->execution_snapshot_id =
        next_execution_snapshot_id_.fetch_add(1, std::memory_order_relaxed);
  }
  execution->evaluation_time_s = evaluation_time_s;
  execution->occupancy = occupancy;
  execution->integrity_anchor = integrity;
  execution->source_identity = source_identity;
  execution->risk_policy = config_.grid;
  execution->gnss_max_age_s = config_.gnss_epoch_max_age_s;
  execution->risk_policy.geometry_id = occupancy->geometry.geometry_id;
  execution->lidar_generation = occupancy->generation;
  execution->lidar_stamp_s = occupancy->cloud_stamp_s;
  execution->frame_contract_id = occupancy->frame_contract_id;
  execution->geometry_id = occupancy->geometry.geometry_id;
  execution->predictor_algorithm_identity =
      source_identity.predictor_algorithm_identity;
  if (occupancy->frozen_grid_map_epoch &&
      occupancy->frozen_grid_map_epoch->active_window_obstacle_sources) {
    for (const auto& source :
         *occupancy->frozen_grid_map_epoch->active_window_obstacle_sources) {
      execution->local_obstacle_source_certifications.push_back(
          makeLocalObstacleSourceCertification(source));
    }
  }
  const double hal = config_.predictor_hal_m;
  const double val = config_.predictor_val_m;
  const double batch_budget_ms = config_.execution_batch_budget_ms;
  execution->forward_risk_batch =
      [module, occupancy, integrity, hal, val, batch_budget_ms](
          const iap::ForwardRiskBatchRequest& input) {
        (void)occupancy;
        iap::ForwardRiskBatchRequest request = input;
        request.snapshot = integrity;
        request.hal = hal;
        request.val = val;
        request.compute_budget_ms =
            std::isfinite(request.compute_budget_ms) &&
                request.compute_budget_ms > 0.0
            ? std::min(request.compute_budget_ms, batch_budget_ms)
            : batch_budget_ms;
        return module->queryForwardRiskBatch(request);
      };
  execution->diagnostic_forward_risk_batch =
      [module, occupancy, hal, val, batch_budget_ms](
          const iap::ForwardRiskBatchRequest& input) {
        (void)occupancy;
        iap::ForwardRiskBatchRequest request = input;
        request.hal = hal;
        request.val = val;
        request.compute_budget_ms =
            std::isfinite(request.compute_budget_ms) &&
                request.compute_budget_ms > 0.0
            ? std::min(request.compute_budget_ms, batch_budget_ms)
            : batch_budget_ms;
        return module->queryForwardRiskBatch(request);
      };
  execution->publish_time_s = std::max(
      liveNowSeconds(), execution->evaluation_time_s);
  evidence.snapshot_publish_ros_stamp_s = execution->publish_time_s;
  // Publishing local execution authority is driven by the registered map.
  // A missing/stale GNSS epoch is carried as global-navigation evidence and
  // is interpreted by the task policy; it must not suppress a locally fresh
  // snapshot needed to certify braking and best-effort motion.
  if (!execution->localFreshAt(execution->publish_time_s)) {
    finish(P0ExecutionSnapshotAttemptStatus::FINAL_FRESHNESS_FAILED,
           "snapshot_became_stale_before_publish");
    return;
  }
  bool superseded_before_publish = false;
  {
    // notifyOccupancyCommitted() uses the same mutex. Comparing and publishing
    // under this short critical section gives latest-wins a linearization
    // point: an older in-flight build can never become authority after a newer
    // generation has been requested.
    std::lock_guard<std::mutex> worker_lock(
        execution_snapshot_worker_mutex_);
    superseded_before_publish =
        occupancy->generation < last_requested_occupancy_generation_;
    if (!superseded_before_publish) {
      std::lock_guard<std::mutex> planning_lock(planning_snapshot_mutex_);
      execution_snapshot_ = execution;
      execution_snapshot_history_.push_back(execution);
      while (execution_snapshot_history_.size() > 4U) {
        execution_snapshot_history_.pop_front();
      }
    }
  }
  if (superseded_before_publish) {
    finish(P0ExecutionSnapshotAttemptStatus::SUPERSEDED,
           "snapshot_superseded_before_atomic_publish");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(health_state_mutex_);
    last_execution_snapshot_id_ = execution->execution_snapshot_id;
    last_execution_snapshot_evaluation_stamp_s_ =
        execution->evaluation_time_s;
    last_execution_snapshot_publish_stamp_s_ = execution->publish_time_s;
    last_execution_snapshot_publish_latency_ms_ =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - build_start).count();
  }
  evidence.published_execution_snapshot_id = execution->execution_snapshot_id;
  evidence.publish_age_s =
      execution->publish_time_s - occupancy->cloud_stamp_s;
  finish(P0ExecutionSnapshotAttemptStatus::PUBLISHED,
         "execution_snapshot_published");
}

void P0RiskGridRuntime::healthTimerCallback() {
  if (!config_.enable_risk_grid) {
    return;
  }
  const auto callback_start = std::chrono::steady_clock::now();
  const double callback_steady_s = steadyNowSeconds();
  const double now_s = liveNowSeconds();
  {
    std::lock_guard<std::mutex> lock(health_state_mutex_);
    last_health_callback_stamp_s_ = now_s;
    last_health_callback_steady_s_ = callback_steady_s;
    ++health_callback_count_;
    // A wall timer has no exposed scheduled-fire stamp.  The callback group
    // is reentrant, so this remains a conservative observable queue delay.
    last_health_callback_queue_delay_ms_ = 0.0;
  }
  publishHealth(std::isfinite(now_s) ? risk_grid_.health(now_s)
                                     : risk_grid_.health(), now_s);
  const double duration_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - callback_start).count();
  {
    std::lock_guard<std::mutex> lock(health_state_mutex_);
    last_health_callback_duration_ms_ = duration_ms;
  }
}

bool P0RiskGridRuntime::geometryMatchesRiskOverlay(
    const iap::PlanningLatticeGeometry& geometry,
    const iap::RiskGridMapParams& risk_grid,
    const int required_ego_voxels_per_axis) {
  if (!geometry.valid() || required_ego_voxels_per_axis <= 0 ||
      !risk_grid.use_fixed_origin) {
    return false;
  }
  const Eigen::Vector3d configured_extent(
      risk_grid.size_x_m, risk_grid.size_y_m, risk_grid.size_z_m);
  const double ratio = risk_grid.resolution_m / geometry.resolution_m;
  return std::isfinite(ratio) &&
      std::abs(ratio - static_cast<double>(required_ego_voxels_per_axis)) <=
          1.0e-9 &&
      geometry.frame_id == risk_grid.frame_id &&
      geometry.origin_w.isApprox(risk_grid.fixed_origin_w, 1.0e-9) &&
      geometry.extent_m.isApprox(configured_extent, 1.0e-9);
}

void P0RiskGridRuntime::refreshTimerCallback() {
  if (!config_.enable_risk_grid) {
    return;
  }
  const auto refresh_start = std::chrono::steady_clock::now();
  // The public budget ends at the atomic RiskGrid commit.  Keep a small
  // return-path allowance so the measured build duration cannot cross the
  // limit immediately after a just-in-budget commit.  Planning-snapshot
  // binding and health publication happen after this independently measured
  // build and do not delay availability of the committed grid.
  const double risk_grid_commit_reserve_ms = std::min(
      10.0, std::max(0.0, 0.02 * config_.risk_grid_build_budget_ms));
  const double risk_grid_compute_budget_ms = std::max(
      0.0, config_.risk_grid_build_budget_ms -
               risk_grid_commit_reserve_ms);
  const auto risk_grid_deadline = refresh_start +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double, std::milli>(
              risk_grid_compute_budget_ms));
  const double refresh_start_steady_s = steadyNowSeconds();
  double now_s = liveNowSeconds();
  std::optional<P0OccupancyEpoch> occupancy_epoch;
  P0OccupancyEpochCaptureStatus occupancy_capture_status =
      P0OccupancyEpochCaptureStatus::VALID;
  std::function<P0OccupancyEpochCapture()> occupancy_factory;
  {
    std::lock_guard<std::mutex> lock(source_factory_mutex_);
    occupancy_factory = occupancy_epoch_factory_;
  }
  const bool occupancy_capture_attempted =
      static_cast<bool>(occupancy_factory);
  if (occupancy_capture_attempted) {
    // Execution authority owns the high-priority freeze. Dense RiskGrid work
    // waits for that single-slot worker and reuses its immutable occupancy
    // object whenever it represents the current committed generation.
    if (!provider_) {
      (void)yieldRiskGridToExecutionSnapshot(risk_grid_deadline);
    }
    const auto execution = !provider_ ? acquireExecutionRiskSnapshot()
                                      : nullptr;
    if (execution && execution->occupancy &&
        executionSnapshotFreshAt(execution, liveNowSeconds())) {
      // Build against this coherent, fresh immutable generation even if a
      // newer occupancy transaction committed meanwhile. A new generation is
      // a request for the next latest-wins snapshot, not grounds to discard
      // the already frozen tuple and repeat the expensive dense-map capture.
      occupancy_epoch = *execution->occupancy;
    } else {
      P0OccupancyEpochCapture capture = occupancy_factory();
      occupancy_capture_status = capture.status;
      occupancy_epoch = std::move(capture.epoch);
    }
  }
  if (config_.online_mapping_mode && occupancy_epoch.has_value()) {
    const auto& geometry = occupancy_epoch->geometry;
    const bool geometry_matches = geometryMatchesRiskOverlay(
        geometry, config_.grid, 5);
    if (!geometry_matches) {
      occupancy_capture_status =
          P0OccupancyEpochCaptureStatus::ADAPTER_INVALID;
      occupancy_epoch.reset();
    } else {
      if (!online_geometry_bound_) {
        config_.grid.geometry_id = geometry.geometry_id;
        std::string configure_reason;
        if (!risk_grid_.configure(config_.grid, &configure_reason)) {
          occupancy_capture_status =
              P0OccupancyEpochCaptureStatus::ADAPTER_INVALID;
          occupancy_epoch.reset();
        } else {
          online_geometry_bound_ = true;
        }
      }
      if (occupancy_epoch.has_value()) {
        // Reuse the exact immutable hit centers captured by the occupancy
        // adapter. This work is performed once with the frozen epoch, never on
        // the latency-sensitive planner callback and never from world truth.
        const auto points =
            occupancy_epoch->environment_occupied_voxel_centers;
        if (!points) {
          occupancy_capture_status =
              P0OccupancyEpochCaptureStatus::ADAPTER_INVALID;
          occupancy_epoch.reset();
        } else {
          iap::LidarFimPrimitiveGenerationDiagnostics diagnostics;
          std::shared_ptr<const std::vector<iap::LidarFimPrimitive>>
              primitives;
          {
            std::lock_guard<std::mutex> lock(lidar_predictor_input_mutex_);
            if (latest_lidar_generation_ == occupancy_epoch->generation &&
                latest_lidar_stamp_ == occupancy_epoch->cloud_stamp_s &&
                latest_lidar_fim_primitives_) {
              primitives = latest_lidar_fim_primitives_;
              diagnostics = latest_lidar_fim_diagnostics_;
            }
          }
          if (!primitives) {
            primitives = iap::make_lidar_fim_primitives(
                *points, nullptr, iap::LidarFimPrimitiveGenerationParams{},
                &diagnostics);
          }
          {
            std::lock_guard<std::mutex> lock(lidar_predictor_input_mutex_);
            if (occupancy_epoch->generation >= latest_lidar_generation_) {
              latest_lidar_map_points_ = points;
              latest_lidar_fim_primitives_ = primitives;
              latest_lidar_fim_diagnostics_ = diagnostics;
              latest_lidar_map_point_count_ = points->size();
              latest_lidar_fim_primitive_count_ =
                  primitives ? primitives->size() : 0u;
              latest_lidar_fim_valid_normal_count_ =
                  static_cast<std::size_t>(
                      std::max(0, diagnostics.lidar_pca_valid_normals));
              latest_lidar_fim_fallback_reason_ = diagnostics.fallback_reason;
              latest_lidar_generation_ = occupancy_epoch->generation;
              latest_lidar_stamp_ = occupancy_epoch->cloud_stamp_s;
            }
          }
          {
            std::lock_guard<std::mutex> lock(health_state_mutex_);
            map_seen_ = true;
            latest_map_stamp_ = occupancy_epoch->cloud_stamp_s;
          }
        }
      }
    }
  }
  // The registered GLIM frame carries its optimized pose and can reach this
  // executor just before the matching odometry/integrity DDS samples. Use the
  // newest captured source stamp as the transaction clock; the normal
  // per-source freshness gates below still reject an odometry or integrity
  // anchor that is genuinely too old. This removes transport-order flicker
  // without accepting a future-dated legacy occupancy source.
  if (config_.online_mapping_mode && occupancy_epoch.has_value() &&
      std::isfinite(occupancy_epoch->cloud_stamp_s)) {
    now_s = std::max(now_s, occupancy_epoch->cloud_stamp_s);
  }
  const std::string pre_build_failure = snapshotFailureReason(now_s);
  const InputReadiness refresh_input_readiness = inputReadiness(now_s);
  const iap::RiskGridHealth active_health_at_start =
      std::isfinite(now_s) ? risk_grid_.health(now_s) : risk_grid_.health();
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    last_refresh_stamp_s_ = now_s;
    last_refresh_scheduled_steady_s_ = next_refresh_scheduled_steady_s_;
    last_refresh_queue_delay_ms_ = std::isfinite(next_refresh_scheduled_steady_s_)
        ? std::max(0.0, 1000.0 * (refresh_start_steady_s -
                                  next_refresh_scheduled_steady_s_))
        : std::numeric_limits<double>::quiet_NaN();
    const double period_s = std::max(0.001, config_.grid.refresh_period_s);
    if (!std::isfinite(next_refresh_scheduled_steady_s_))
      next_refresh_scheduled_steady_s_ = refresh_start_steady_s + period_s;
    while (next_refresh_scheduled_steady_s_ <= refresh_start_steady_s)
      next_refresh_scheduled_steady_s_ += period_s;
    last_refresh_start_stamp_s_ = now_s;
    last_refresh_start_steady_s_ = refresh_start_steady_s;
    last_snapshot_available_ = false;
    last_refresh_succeeded_ = false;
    last_snapshot_failure_reason_ = pre_build_failure;
    last_refresh_query_count_ = 0;
    last_predictor_unique_positions_ = 0;
    last_predictor_lidar_evaluations_ = 0;
    last_predictor_lidar_cache_hits_ = 0;
    last_predictor_spatial_advisory_recompute_count_ = 0;
    last_predictor_spatial_advisory_reuse_count_ = 0;
    last_predictor_gnss_advisory_invocation_count_ = 0;
    last_predictor_lidar_advisory_invocation_count_ = 0;
    last_predictor_horizon_fusion_count_ = 0;
    last_rolling_spatial_diagnostics_ = {};
    last_provider_batch_duration_ms_ =
        std::numeric_limits<double>::quiet_NaN();
    last_refresh_elapsed_ms_ = std::numeric_limits<double>::quiet_NaN();
    last_generation_interval_ms_ = std::numeric_limits<double>::quiet_NaN();
    last_refresh_end_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
    last_refresh_end_steady_s_ = std::numeric_limits<double>::quiet_NaN();
    refresh_evidence_ = {};
    refresh_evidence_.refresh_attempt_id = next_refresh_attempt_id_++;
    refresh_evidence_.state = RefreshEvidenceState::IN_PROGRESS;
    refresh_evidence_.previous_successful_generation_id =
        last_successful_generation_id_;
    refresh_evidence_.publication = healthPublicationStateSnapshot();
    refresh_evidence_.health = active_health_at_start;
    refresh_evidence_.health.provider_query_count = 0;
    refresh_evidence_.health.occupied_skip_count = 0;
    refresh_evidence_.snapshot_failure_reason = "none";
    refresh_input_readiness_ = refresh_input_readiness;
  }
  if (!std::isfinite(now_s) || now_s <= 0.0) {
    risk_grid_.markRefreshFailure(now_s, "message_stamp_unavailable");
    const double refresh_end_stamp_s = liveNowSeconds();
    {
      std::lock_guard<std::mutex> health_lock(health_state_mutex_);
      last_refresh_elapsed_ms_ = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - refresh_start).count();
      last_refresh_end_stamp_s_ = refresh_end_stamp_s;
      last_refresh_end_steady_s_ = steadyNowSeconds();
    }
    completeRefreshEvidence(risk_grid_.health(), now_s, false);
    publishHealth(risk_grid_.health(), now_s);
    return;
  }
  const auto fail_semantic_refresh = [&](const P0SemanticFailure failure) {
    const std::string failure_reason = semanticFailureReason(failure);
    risk_grid_.markRefreshFailure(now_s, failure_reason);
    const double refresh_end_stamp_s = liveNowSeconds();
    {
      std::lock_guard<std::mutex> health_lock(health_state_mutex_);
      last_snapshot_failure_reason_ = failure_reason;
      last_refresh_succeeded_ = false;
      last_refresh_elapsed_ms_ = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - refresh_start).count();
      last_refresh_end_stamp_s_ = refresh_end_stamp_s;
      last_refresh_end_steady_s_ = steadyNowSeconds();
    }
    const iap::RiskGridHealth failed_health = risk_grid_.health(now_s);
    completeRefreshEvidence(failed_health, now_s, false);
    publishHealth(failed_health, now_s);
  };
  const bool production_predictor = provider_ == nullptr;
  if (production_predictor && config_.predictor_use_current_integrity_prior) {
    const InputReadiness readiness = inputReadiness(now_s);
    if (readiness.current_integrity_seen &&
        readiness.current_integrity_valid &&
        !readiness.current_integrity_fresh &&
        std::isfinite(readiness.current_integrity_stamp_s)) {
      fail_semantic_refresh(P0SemanticFailure::STALE_COVARIANCE_GROWTH_PRIOR);
      return;
    }
  }
  iap::IntegritySnapshot snapshot;
  PredictorSourceCapture captured_predictor_sources;
  InputReadiness captured_input_readiness;
  const bool snapshot_built = buildSnapshot(
      now_s, &snapshot, &captured_predictor_sources,
      &captured_input_readiness);
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    refresh_input_readiness_ = captured_input_readiness;
  }
  if (!snapshot_built) {
    const bool invalid_current_provenance =
        production_predictor &&
        (captured_predictor_sources.current_generation == 0u ||
         !std::isfinite(captured_predictor_sources.current_stamp));
    const bool invalid_gnss_provenance =
        production_predictor &&
        config_.predictor_source_mode != iap::PredictorSourceMode::LidarOnly &&
        config_.predictor_gnss_epoch_policy !=
            iap::PredictorGnssEpochPolicy::Disabled &&
        captured_predictor_sources.gnss_epoch_generation != 0u &&
        !std::isfinite(captured_predictor_sources.gnss_epoch_stamp);
    const std::string failure_reason =
        invalid_current_provenance
            ? "invalid_current_provenance"
            : invalid_gnss_provenance ? "invalid_gnss_epoch_identity"
                                      : "snapshot_unavailable";
    risk_grid_.markRefreshFailure(now_s, failure_reason);
    const double refresh_end_stamp_s = liveNowSeconds();
    {
      std::lock_guard<std::mutex> health_lock(health_state_mutex_);
      last_refresh_elapsed_ms_ = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - refresh_start).count();
      last_refresh_end_stamp_s_ = refresh_end_stamp_s;
      last_refresh_end_steady_s_ = steadyNowSeconds();
      if (invalid_current_provenance || invalid_gnss_provenance) {
        last_snapshot_failure_reason_ = failure_reason;
        last_rolling_spatial_diagnostics_.invalid_source_provenance_count =
            1u;
        last_rolling_spatial_diagnostics_.invalidation_reason =
            iap::RollingSpatialInvalidationReason::SourceProvenanceInvalid;
      } else if (pre_build_failure == "none") {
        last_snapshot_failure_reason_ = "snapshot_builder_invalid";
      }
    }
    const iap::RiskGridHealth failed_health = risk_grid_.health(now_s);
    completeRefreshEvidence(failed_health, now_s, false);
    publishHealth(failed_health, now_s);
    return;
  }
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    last_snapshot_available_ = true;
    last_snapshot_failure_reason_ = "none";
  }

  if (occupancy_capture_attempted) {
    if (occupancy_capture_status ==
        P0OccupancyEpochCaptureStatus::SNAPSHOT_UNAVAILABLE) {
      fail_semantic_refresh(
          P0SemanticFailure::OCCUPANCY_SNAPSHOT_UNAVAILABLE);
      return;
    }
    if (occupancy_capture_status ==
            P0OccupancyEpochCaptureStatus::ADAPTER_INVALID ||
        !occupancy_epoch.has_value()) {
      fail_semantic_refresh(P0SemanticFailure::OCCUPANCY_LOS_ADAPTER_INVALID);
      return;
    }
    if (!occupancy_epoch->diagnostic_query ||
        !occupancy_epoch->los_owner || !occupancy_epoch->source_owner ||
        !occupancy_epoch->live_source_owner ||
        !occupancy_epoch->live_generation ||
        occupancy_epoch->generation == 0u) {
      fail_semantic_refresh(P0SemanticFailure::OCCUPANCY_LOS_ADAPTER_INVALID);
      return;
    }
    if (occupancy_epoch->frame_id != config_.grid.frame_id ||
        occupancy_epoch->frame_id != "map") {
      fail_semantic_refresh(P0SemanticFailure::OCCUPANCY_FRAME_MISMATCH);
      return;
    }
    const double occupancy_age_s = now_s - occupancy_epoch->cloud_stamp_s;
    if (!std::isfinite(occupancy_epoch->cloud_stamp_s) ||
        !std::isfinite(occupancy_age_s) || occupancy_age_s < 0.0 ||
        (config_.grid.stale_timeout_s >= 0.0 &&
         occupancy_age_s > config_.grid.stale_timeout_s)) {
      RCLCPP_WARN(
          node_->get_logger(),
          "[p0] rejecting occupancy epoch generation=%lu stamp=%.9f "
          "transaction_now=%.9f age=%.6f stale_timeout=%.6f",
          static_cast<unsigned long>(occupancy_epoch->generation),
          occupancy_epoch->cloud_stamp_s, now_s, occupancy_age_s,
          config_.grid.stale_timeout_s);
      fail_semantic_refresh(P0SemanticFailure::OCCUPANCY_STALE);
      return;
    }
  }

  if (production_predictor && !occupancy_epoch.has_value()) {
    fail_semantic_refresh(P0SemanticFailure::OCCUPANCY_SNAPSHOT_UNAVAILABLE);
    return;
  }
  if (production_predictor &&
      (!std::isfinite(config_.predictor_sigma_grow_m_sqrt_s) ||
       config_.predictor_sigma_grow_m_sqrt_s < 0.0)) {
    fail_semantic_refresh(
        P0SemanticFailure::INVALID_COVARIANCE_GROWTH_PARAMETER);
    return;
  }
  if (production_predictor && !snapshot.has_lambda_base) {
    fail_semantic_refresh(
        config_.predictor_use_current_integrity_prior
            ? P0SemanticFailure::INVALID_COVARIANCE_GROWTH_PRIOR
            : P0SemanticFailure::MISSING_COVARIANCE_GROWTH_PRIOR);
    return;
  }
  if (production_predictor && !finiteSpd(snapshot.lambda_base_pos)) {
    fail_semantic_refresh(P0SemanticFailure::INVALID_COVARIANCE_GROWTH_PRIOR);
    return;
  }

  std::unique_ptr<iap::RiskPredictionProvider> local_provider;
  iap::RiskPredictionProvider* provider = provider_.get();
  PredictorModuleRiskProvider* predictor_provider = nullptr;
  bool validate_gnss_spatial_source = false;
  bool validate_lidar_spatial_source = false;
  bool validate_lidar_legacy_source = false;
  bool active_gnss_occupancy_content = false;
  uint64_t candidate_occupancy_content_identity = 0;
  std::shared_ptr<const std::vector<Eigen::Vector3d>>
      captured_lidar_map_points;
  std::shared_ptr<const std::vector<iap::LidarFimPrimitive>>
      captured_lidar_fim_primitives;
  std::shared_ptr<iap::PredictorModule> forward_risk_module;
  std::shared_ptr<const iap::LocalOccupancyGrid>
      forward_risk_occupancy_owner;
  double forward_gnss_support_ray_length_m =
      std::numeric_limits<double>::quiet_NaN();
  bool forward_gnss_hard_occlusion = false;
  uint64_t captured_lidar_generation = 0;
  double captured_lidar_stamp =
      std::numeric_limits<double>::quiet_NaN();
  if (provider == nullptr) {
    iap::PredictorParams predictor_params;
    predictor_params.freshness.enabled = true;
    predictor_params.freshness.max_odom_age_s =
        config_.grid.stale_timeout_s;
    predictor_params.freshness.max_integrity_age_s =
        config_.grid.stale_timeout_s;
    predictor_params.freshness.max_gnss_age_s =
        config_.gnss_epoch_max_age_s;
    predictor_params.freshness.max_snapshot_age_s =
        config_.grid.stale_timeout_s;
    predictor_params.source_mode = config_.predictor_source_mode;
    predictor_params.gnss.visibility_params.clearance_transition_m =
        config_.predictor_gnss_clearance_transition_m;
    forward_gnss_hard_occlusion =
        predictor_params.gnss.visibility_params.hard_occlusion;
    forward_gnss_support_ray_length_m = forward_gnss_hard_occlusion
        ? predictor_params.gnss.visibility_params.occ_range
        : predictor_params.gnss.visibility_params.occ_L;
    predictor_params.gnss_epoch_policy =
        config_.predictor_gnss_epoch_policy;
    predictor_params.gnss.measured_epoch_support_radius_m =
        config_.predictor_gnss_measured_epoch_support_radius_m;
    predictor_params.gnss.measured_epoch_integrity_max_delta_s =
        config_.predictor_gnss_measured_epoch_integrity_max_delta_s;
    predictor_params.fusion.conservative_max_with_gnss =
        config_.predictor_conservative_max_with_gnss;
    predictor_params.lidar.enable_legacy_observability =
        config_.predictor_lidar_legacy_observability;
    predictor_params.covariance_growth.sigma_grow_m_sqrt_s =
        config_.predictor_sigma_grow_m_sqrt_s;
    const iap::PredictorSpatialSourceUsage source_projection =
        iap::predictorSpatialSourceUsage(predictor_params);
    validate_gnss_spatial_source =
        source_projection.gnss && snapshot.has_epoch;
    validate_lidar_spatial_source = source_projection.lidar;
    validate_lidar_legacy_source = source_projection.legacy_lidar;
    active_gnss_occupancy_content =
        source_projection.gnss && snapshot.has_epoch;
    if (active_gnss_occupancy_content) {
      if (!occupancy_epoch->raw_identity) {
        fail_semantic_refresh(
            P0SemanticFailure::OCCUPANCY_LOS_ADAPTER_INVALID);
        return;
      }
      const bool has_committed_base =
          rolling_occupancy_owner_ && rolling_raw_occupancy_identity_ &&
          rolling_occupancy_source_owner_ &&
          rolling_occupancy_geometry_.valid() &&
          !rolling_occupancy_frame_contract_id_.empty() &&
          rolling_occupancy_generation_ != 0u &&
          std::isfinite(rolling_occupancy_stamp_) &&
          rolling_occupancy_content_identity_ != 0u;
      const bool partial_committed_base =
          static_cast<bool>(rolling_occupancy_owner_) ||
          static_cast<bool>(rolling_raw_occupancy_identity_) ||
          static_cast<bool>(rolling_occupancy_source_owner_) ||
          rolling_occupancy_geometry_.valid() ||
          !rolling_occupancy_frame_contract_id_.empty() ||
          rolling_occupancy_generation_ != 0u ||
          std::isfinite(rolling_occupancy_stamp_) ||
          rolling_occupancy_content_identity_ != 0u;
      if (!has_committed_base && partial_committed_base) {
        fail_semantic_refresh(
            P0SemanticFailure::OCCUPANCY_LOS_ADAPTER_INVALID);
        return;
      }

      const std::string support_identity =
          occupancy_epoch->trusted_local_map_support
              ? occupancy_epoch->trusted_local_map_support->identity()
              : std::string("strict_observation");
      bool retain_committed_content = false;
      if (has_committed_base) {
        P0OccupancyEpoch committed_base;
        committed_base.los_owner = rolling_occupancy_owner_;
        committed_base.raw_identity = rolling_raw_occupancy_identity_;
        committed_base.source_owner = rolling_occupancy_source_owner_;
        committed_base.geometry = rolling_occupancy_geometry_;
        committed_base.frame_contract_id =
            rolling_occupancy_frame_contract_id_;
        committed_base.generation = rolling_occupancy_generation_;
        committed_base.cloud_stamp_s = rolling_occupancy_stamp_;
        committed_base.frame_id = rolling_raw_occupancy_identity_->frameId();
        if (sameSharedOwner(rolling_occupancy_source_owner_,
                            occupancy_epoch->source_owner)) {
          if (occupancy_epoch->generation == rolling_occupancy_generation_) {
            if (!P0OccupancyEpochAdapter::sameVersion(
                    committed_base, *occupancy_epoch)) {
              fail_semantic_refresh(
                  P0SemanticFailure::OCCUPANCY_LOS_ADAPTER_INVALID);
              return;
            }
            retain_committed_content =
                support_identity == rolling_support_identity_;
          } else if (occupancy_epoch->generation <
                     rolling_occupancy_generation_) {
            fail_semantic_refresh(
                P0SemanticFailure::OCCUPANCY_LOS_ADAPTER_INVALID);
            return;
          } else {
            const auto delta = P0OccupancyEpochAdapter::completeDelta(
                committed_base, *occupancy_epoch);
            retain_committed_content = delta && delta->empty() &&
                support_identity == rolling_support_identity_ &&
                !occupancy_epoch->trusted_local_map_support;
          }
        }
      }

      if (retain_committed_content) {
        occupancy_epoch->los_owner = rolling_occupancy_owner_;
        candidate_occupancy_content_identity =
            rolling_occupancy_content_identity_;
      } else {
        if (rolling_occupancy_content_identity_ ==
            std::numeric_limits<uint64_t>::max()) {
          fail_semantic_refresh(
              P0SemanticFailure::OCCUPANCY_LOS_ADAPTER_INVALID);
          return;
        }
        candidate_occupancy_content_identity =
            rolling_occupancy_content_identity_ + 1u;
      }
    }
    if (std::isfinite(config_.predictor_lidar_fim_radius_m) &&
        config_.predictor_lidar_fim_radius_m > 0.0) {
      predictor_params.lidar.fim_params.fim_radius_m =
          config_.predictor_lidar_fim_radius_m;
      predictor_params.lidar.fim_params.search_radius_m =
          config_.predictor_lidar_fim_radius_m;
    }
    std::shared_ptr<const std::vector<Eigen::Vector3d>> lidar_map_points;
    std::shared_ptr<const std::vector<iap::LidarFimPrimitive>>
        lidar_fim_primitives;
    {
      std::lock_guard<std::mutex> lock(lidar_predictor_input_mutex_);
      lidar_map_points = latest_lidar_map_points_;
      lidar_fim_primitives = latest_lidar_fim_primitives_;
      captured_lidar_generation = latest_lidar_generation_;
      captured_lidar_stamp = latest_lidar_stamp_;
    }
    captured_lidar_map_points = lidar_map_points;
    captured_lidar_fim_primitives = lidar_fim_primitives;
    iap::PredictorModule module(predictor_params);
    if (config_.online_mapping_mode) {
      const auto trusted_support = occupancy_epoch->trusted_local_map_support;
      const auto observed_support_query = occupancy_epoch->diagnostic_query;
      if (trusted_support) {
        module.set_support_query(
            [occupancy_epoch](const Eigen::Vector3d& position,
                              const double evaluation_time_s,
                              const double query_time_s) {
              return queryP0LocalMapSupport(
                  *occupancy_epoch, position, evaluation_time_s,
                  query_time_s);
            });
      } else {
        module.set_observation_predicate(
          [observed_support_query](const Eigen::Vector3d& position) {
            const auto diagnostic = observed_support_query(position);
            return diagnostic.available && diagnostic.observed &&
                   diagnostic.state != iap::RiskOccupancyState::UNKNOWN;
          });
      }
    }
    module.set_lidar_map_points(lidar_map_points);
    module.set_lidar_fim_primitives(lidar_fim_primitives);
    forward_risk_module =
        std::make_shared<iap::PredictorModule>(predictor_params);
    forward_risk_occupancy_owner = occupancy_epoch->los_owner;
    forward_risk_module->set_local_occupancy(
        forward_risk_occupancy_owner.get());
    if (config_.online_mapping_mode) {
      const auto trusted_support = occupancy_epoch->trusted_local_map_support;
      const auto observed_support_query = occupancy_epoch->diagnostic_query;
      if (trusted_support) {
        forward_risk_module->set_support_query(
            [occupancy_epoch](const Eigen::Vector3d& position,
                              const double evaluation_time_s,
                              const double query_time_s) {
              return queryP0LocalMapSupport(
                  *occupancy_epoch, position, evaluation_time_s,
                  query_time_s);
            });
      } else {
        forward_risk_module->set_observation_predicate(
          [observed_support_query](const Eigen::Vector3d& position) {
            const auto diagnostic = observed_support_query(position);
            return diagnostic.available && diagnostic.observed &&
                   diagnostic.state != iap::RiskOccupancyState::UNKNOWN;
          });
      }
    }
    forward_risk_module->set_lidar_map_points(lidar_map_points);
    forward_risk_module->set_lidar_fim_primitives(lidar_fim_primitives);
    iap::RollingSpatialWindowGeometry rolling_geometry;
    rolling_geometry.frame_id = config_.grid.frame_id;
    rolling_geometry.lattice_anchor_w = config_.grid.lattice_anchor_w;
    rolling_geometry.resolution_m = config_.grid.resolution_m;
    rolling_geometry.shape = risk_grid_.voxelNum();
    iap::RollingSpatialRetentionPolicy retention_policy;
    retention_policy.gnss_spatial_ttl_s =
        config_.predictor_gnss_spatial_ttl_s;
    retention_policy.legacy_current_spatial_ttl_s =
        config_.predictor_legacy_current_spatial_ttl_s;
    retention_policy.full_refresh_watchdog_s =
        config_.predictor_full_refresh_watchdog_s;
    iap::RollingSpatialSourceProvenance source_provenance;
    source_provenance.gnss_epoch_generation =
        captured_predictor_sources.gnss_epoch_generation;
    source_provenance.gnss_epoch_stamp =
        captured_predictor_sources.gnss_epoch_stamp;
    source_provenance.occupancy_generation = occupancy_epoch->generation;
    source_provenance.occupancy_stamp = occupancy_epoch->cloud_stamp_s;
    source_provenance.occupancy_content_identity =
        candidate_occupancy_content_identity;
    source_provenance.lidar_generation = captured_lidar_generation;
    source_provenance.lidar_stamp = captured_lidar_stamp;
    source_provenance.current_generation =
        captured_predictor_sources.current_generation;
    source_provenance.current_stamp =
        captured_predictor_sources.current_stamp;
    source_provenance.refresh_reference_time_s = now_s;
    auto owned_predictor_provider = std::make_unique<PredictorModuleRiskProvider>(
        &rolling_spatial_window_, std::move(rolling_geometry),
        occupancy_epoch->los_owner,
        std::move(lidar_map_points), std::move(lidar_fim_primitives),
        retention_policy, source_provenance,
        std::move(module), snapshot,
        config_.task_mode,
        now_s,
        config_.predictor_effective_worker_count,
        config_.predictor_hal_m, config_.predictor_val_m,
        risk_grid_deadline,
        risk_grid_worker_pool_.get(),
        [this](const std::chrono::steady_clock::time_point deadline) {
          return yieldRiskGridToExecutionSnapshot(deadline);
        });
    predictor_provider = owned_predictor_provider.get();
    local_provider = std::move(owned_predictor_provider);
    provider = local_provider.get();
  }

  if (predictor_provider && !predictor_provider->ready()) {
    const std::string failure_reason = predictor_provider->beginFailureReason();
    risk_grid_.markRefreshFailure(now_s, failure_reason);
    const double refresh_end_stamp_s = liveNowSeconds();
    {
      std::lock_guard<std::mutex> health_lock(health_state_mutex_);
      last_snapshot_failure_reason_ = failure_reason;
      last_rolling_spatial_diagnostics_ =
          predictor_provider->rollingDiagnostics();
      last_refresh_elapsed_ms_ = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - refresh_start).count();
      last_refresh_end_stamp_s_ = refresh_end_stamp_s;
      last_refresh_end_steady_s_ = steadyNowSeconds();
    }
    const iap::RiskGridHealth failed_health = risk_grid_.health(now_s);
    completeRefreshEvidence(failed_health, now_s, false);
    publishHealth(failed_health, now_s);
    return;
  }

  std::string reason;
  const Eigen::Vector3i voxel_num = risk_grid_.voxelNum();
  const auto layer_voxel_count =
      static_cast<std::size_t>(std::max(0, voxel_num.x())) *
      static_cast<std::size_t>(std::max(0, voxel_num.y())) *
      static_cast<std::size_t>(std::max(0, voxel_num.z()));
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    last_refresh_query_count_ =
        layer_voxel_count * config_.grid.horizons_s.size();
  }
  const auto combined_occupancy_diagnostic_query =
      combinedOccupancyDiagnosticQuery(
      occupancy_epoch ? occupancy_epoch->diagnostic_query
                      : iap::RiskGridMap::OccupancyDiagnosticQuery{});
  iap::RiskGridMap::OccupancyDiagnosticQuery occupancy_diagnostic_query =
      combined_occupancy_diagnostic_query;
  if (occupancy_epoch && occupancy_epoch->trusted_local_map_support &&
      combined_occupancy_diagnostic_query) {
    const auto trusted_support =
        occupancy_epoch->trusted_local_map_support;
    occupancy_diagnostic_query =
        [combined_occupancy_diagnostic_query, trusted_support, now_s](
            const Eigen::Vector3d & position) {
          auto diagnostic = combined_occupancy_diagnostic_query(position);
          diagnostic.model_support = trusted_support->query(position, now_s);
          return diagnostic;
        };
  }
  const auto occupancy_predicate = combinedOccupancyPredicate();
  iap::RiskGridMap::SourceValidator source_validator;
  if (occupancy_epoch) {
    const auto captured_occupancy_source_owner =
        occupancy_epoch->source_owner;
    const auto live_occupancy_source_owner =
        occupancy_epoch->live_source_owner;
    const auto live_occupancy_generation = occupancy_epoch->live_generation;
    const uint64_t captured_occupancy_generation =
        occupancy_epoch->generation;
    const uint64_t captured_prior_generation =
        snapshot.prior_source_generation;
    source_validator =
        [this, captured_occupancy_source_owner,
         live_occupancy_source_owner, live_occupancy_generation,
         captured_occupancy_generation,
         captured_prior_generation, validate_gnss_spatial_source,
         validate_lidar_spatial_source, validate_lidar_legacy_source,
         captured_predictor_sources, captured_lidar_generation,
         captured_lidar_stamp, captured_lidar_map_points,
         captured_lidar_fim_primitives, risk_grid_deadline]() {
          if (std::chrono::steady_clock::now() >= risk_grid_deadline) {
            return iap::RiskGridSourceValidation::COMPUTE_BUDGET_EXCEEDED;
          }
          if (!captured_occupancy_source_owner ||
              !live_occupancy_source_owner || !live_occupancy_generation ||
              captured_occupancy_generation == 0u) {
            return iap::RiskGridSourceValidation::
                OCCUPANCY_GENERATION_CHANGED;
          }
          const uint64_t live_occupancy_version =
              live_occupancy_generation();
          if (live_occupancy_version == 0u ||
              live_occupancy_version < captured_occupancy_generation ||
              (live_occupancy_version == captured_occupancy_generation &&
               !sameSharedOwner(captured_occupancy_source_owner,
                                live_occupancy_source_owner()))) {
            return iap::RiskGridSourceValidation::
                OCCUPANCY_GENERATION_CHANGED;
          }
          uint64_t live_prior_generation = 0;
          uint64_t live_gnss_epoch_generation = 0;
          bool live_gnss_epoch_available = false;
          double live_current_stamp =
              std::numeric_limits<double>::quiet_NaN();
          double live_gnss_epoch_stamp =
              std::numeric_limits<double>::quiet_NaN();
          {
            std::lock_guard<std::mutex> lock(health_state_mutex_);
            live_prior_generation = latest_current_generation_;
            live_gnss_epoch_generation = latest_gnss_epoch_generation_;
            live_gnss_epoch_available = latest_epoch_.has_value();
            live_current_stamp = latest_current_.stamp;
            live_gnss_epoch_stamp = latest_epoch_
                ? latest_epoch_->stamp
                : std::numeric_limits<double>::quiet_NaN();
          }
          if (captured_prior_generation == 0u ||
              captured_predictor_sources.current_generation == 0u ||
              captured_prior_generation !=
                  captured_predictor_sources.current_generation ||
              !std::isfinite(captured_predictor_sources.current_stamp) ||
              live_prior_generation < captured_prior_generation ||
              (live_prior_generation == captured_prior_generation &&
               live_current_stamp !=
                   captured_predictor_sources.current_stamp)) {
            return iap::RiskGridSourceValidation::PRIOR_GENERATION_CHANGED;
          }
          if (validate_gnss_spatial_source) {
            if (captured_predictor_sources.gnss_epoch_generation == 0u ||
                !std::isfinite(
                    captured_predictor_sources.gnss_epoch_stamp) ||
                live_gnss_epoch_generation <
                    captured_predictor_sources.gnss_epoch_generation ||
                (live_gnss_epoch_generation >
                     captured_predictor_sources.gnss_epoch_generation &&
                 (!live_gnss_epoch_available ||
                  !std::isfinite(live_gnss_epoch_stamp))) ||
                (live_gnss_epoch_generation ==
                     captured_predictor_sources.gnss_epoch_generation &&
                 live_gnss_epoch_stamp !=
                     captured_predictor_sources.gnss_epoch_stamp)) {
              return iap::RiskGridSourceValidation::
                  PREDICTOR_SPATIAL_SOURCE_CHANGED;
            }
          }
          if (validate_lidar_spatial_source) {
            std::lock_guard<std::mutex> lock(lidar_predictor_input_mutex_);
            if (captured_lidar_generation == 0u ||
                !captured_lidar_fim_primitives ||
                latest_lidar_generation_ < captured_lidar_generation ||
                (latest_lidar_generation_ == captured_lidar_generation &&
                 (!sameSharedOwner(latest_lidar_fim_primitives_,
                                   captured_lidar_fim_primitives) ||
                  latest_lidar_stamp_ != captured_lidar_stamp))) {
              return iap::RiskGridSourceValidation::
                  PREDICTOR_SPATIAL_SOURCE_CHANGED;
            }
            if (validate_lidar_legacy_source &&
                (!captured_lidar_map_points ||
                 (latest_lidar_generation_ == captured_lidar_generation &&
                  !sameSharedOwner(latest_lidar_map_points_,
                                   captured_lidar_map_points)))) {
              return iap::RiskGridSourceValidation::
                  PREDICTOR_SPATIAL_SOURCE_CHANGED;
            }
          }
          return iap::RiskGridSourceValidation::VALID;
        };
  }
  TimedRiskProvider timed_provider(provider, risk_grid_deadline);
  iap::RiskGridSourceIdentity source_identity;
  source_identity.occupancy_generation = occupancy_epoch
      ? occupancy_epoch->generation : 0u;
  source_identity.occupancy_stamp_s = occupancy_epoch
      ? occupancy_epoch->cloud_stamp_s
      : std::numeric_limits<double>::quiet_NaN();
  source_identity.prior_generation =
      captured_predictor_sources.current_generation;
  source_identity.prior_stamp_s = captured_predictor_sources.current_stamp;
  source_identity.gnss_generation =
      captured_predictor_sources.gnss_epoch_generation;
  source_identity.gnss_stamp_s =
      captured_predictor_sources.gnss_epoch_stamp;
  source_identity.gnss_epoch_identity =
      snapshot.current.gnss_epoch_identity;
  source_identity.lidar_generation = captured_lidar_generation;
  source_identity.lidar_stamp_s = captured_lidar_stamp;
  source_identity.local_map_support_identity = occupancy_epoch &&
      occupancy_epoch->trusted_local_map_support
      ? occupancy_epoch->trusted_local_map_support->identity() : "strict";
  {
    std::ostringstream predictor_identity;
    predictor_identity << std::setprecision(17)
        << "forward_risk_v4;gnss_support_model=continuous_unknown_fraction_v1;"
           "clearance_transition_m="
        << config_.predictor_gnss_clearance_transition_m
        << ";admission_epochs=" << config_.predictor_gnss_admission_epochs
        << ";geometry_solver=sherman_morrison_v1"
        << ";execution_satellite_set=braking_window_core_v2"
        << ";task_mode="
        << iap::globalNavigationTaskModeName(config_.task_mode);
    source_identity.predictor_algorithm_identity = predictor_identity.str();
  }
  source_identity.alert_limit_policy_id =
      config_.grid.alert_limit_policy_id;
  std::shared_ptr<const P0ExecutionRiskSnapshot> published_execution;
  if (forward_risk_module && forward_risk_occupancy_owner &&
      occupancy_epoch) {
    auto execution = std::make_shared<P0ExecutionRiskSnapshot>();
    execution->execution_snapshot_id = next_execution_snapshot_id_++;
    if (execution->execution_snapshot_id == 0u) {
      execution->execution_snapshot_id = next_execution_snapshot_id_++;
    }
    execution->evaluation_time_s = now_s;
    execution->publish_time_s = liveNowSeconds();
    execution->occupancy =
        std::make_shared<P0OccupancyEpoch>(*occupancy_epoch);
    execution->integrity_anchor = snapshot;
    execution->source_identity = source_identity;
    execution->risk_policy = config_.grid;
    execution->gnss_max_age_s = config_.gnss_epoch_max_age_s;
    execution->lidar_generation = captured_lidar_generation;
    execution->lidar_stamp_s = captured_lidar_stamp;
    execution->geometry_id = config_.grid.geometry_id;
    execution->predictor_algorithm_identity =
        source_identity.predictor_algorithm_identity;
    // The collision validator retains the producer-owned frozen epoch and
    // checks its frame contract at commit time. The direct risk snapshot
    // binds the same planner-map geometry without dereferencing that opaque
    // plan_env type here.
    execution->frame_contract_id = occupancy_epoch->frame_contract_id;
    if (occupancy_epoch->frozen_grid_map_epoch &&
        occupancy_epoch->frozen_grid_map_epoch->active_window_obstacle_sources) {
      for (const auto& source :
           *occupancy_epoch->frozen_grid_map_epoch->
                active_window_obstacle_sources) {
        execution->local_obstacle_source_certifications.push_back(
            makeLocalObstacleSourceCertification(source));
      }
    }
    execution->forward_risk_batch =
        [forward_risk_module, forward_risk_occupancy_owner, snapshot,
         hal = config_.predictor_hal_m,
         val = config_.predictor_val_m,
         budget_ms = config_.execution_batch_budget_ms](
            const iap::ForwardRiskBatchRequest& input) {
          (void)forward_risk_occupancy_owner;
          iap::ForwardRiskBatchRequest request = input;
          request.snapshot = snapshot;
          request.hal = hal;
          request.val = val;
          request.compute_budget_ms =
              std::isfinite(request.compute_budget_ms) &&
                  request.compute_budget_ms > 0.0
              ? std::min(request.compute_budget_ms, budget_ms)
              : budget_ms;
          return forward_risk_module->queryForwardRiskBatch(request);
        };
    execution->diagnostic_forward_risk_batch =
        [forward_risk_module, forward_risk_occupancy_owner,
         hal = config_.predictor_hal_m,
         val = config_.predictor_val_m,
         budget_ms = config_.execution_batch_budget_ms](
            const iap::ForwardRiskBatchRequest& input) {
          (void)forward_risk_occupancy_owner;
          iap::ForwardRiskBatchRequest request = input;
          request.hal = hal;
          request.val = val;
          request.compute_budget_ms =
              std::isfinite(request.compute_budget_ms) &&
                  request.compute_budget_ms > 0.0
              ? std::min(request.compute_budget_ms, budget_ms)
              : budget_ms;
          return forward_risk_module->queryForwardRiskBatch(request);
        };
    published_execution = execution;
    // A live production runtime has a separate 50 ms execution channel.  The
    // grid-bound object remains attached to this planning generation for
    // search reproducibility, but must never overwrite the newer execution
    // authority merely because a dense grid happened to start.
    if (!execution_snapshot_timer_ || provider_ ||
        synchronous_test_refresh_.load(std::memory_order_acquire)) {
      {
        std::lock_guard<std::mutex> lock(planning_snapshot_mutex_);
        if (!execution_snapshot_ ||
            execution_snapshot_->execution_snapshot_id <
                execution->execution_snapshot_id) {
          execution_snapshot_ = execution;
          execution_snapshot_history_.push_back(execution);
          while (execution_snapshot_history_.size() > 4U) {
            execution_snapshot_history_.pop_front();
          }
        }
      }
      {
        std::lock_guard<std::mutex> lock(health_state_mutex_);
        last_execution_snapshot_id_ = execution->execution_snapshot_id;
        last_execution_snapshot_evaluation_stamp_s_ =
            execution->evaluation_time_s;
        last_execution_snapshot_publish_stamp_s_ = execution->publish_time_s;
        last_execution_snapshot_publish_latency_ms_ =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - refresh_start).count();
      }
    }
  }
  bool refresh_succeeded = false;
  if (occupancy_diagnostic_query) {
    refresh_succeeded = risk_grid_.refreshFromProvider(
        snapshot.p_wb, now_s, timed_provider,
        occupancy_diagnostic_query, source_validator, source_identity,
        &reason);
  } else {
    refresh_succeeded = risk_grid_.refreshFromProvider(
        snapshot.p_wb, now_s, timed_provider, occupancy_predicate, &reason);
  }
  const double risk_grid_build_elapsed_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - refresh_start).count();
  const auto viz_snapshot = risk_grid_.acquireSnapshot();
  if (refresh_succeeded && viz_snapshot) {
    const auto& identity = viz_snapshot->sourceIdentity();
    const bool occupancy_identity_matches = !occupancy_epoch ||
        (identity.occupancy_generation == occupancy_epoch->generation &&
         identity.occupancy_stamp_s == occupancy_epoch->cloud_stamp_s);
    if (occupancy_identity_matches) {
      auto planning = std::make_shared<P0PlanningSnapshot>();
      planning->risk = viz_snapshot;
      planning->execution = published_execution;
      planning->integrity_anchor = snapshot;
      planning->gnss_hard_occlusion = forward_gnss_hard_occlusion;
      planning->gnss_support_ray_length_m =
          forward_gnss_support_ray_length_m;
      if (occupancy_epoch) {
        planning->occupancy =
            std::make_shared<P0OccupancyEpoch>(*occupancy_epoch);
      }
      if (published_execution) {
        planning->forward_risk_batch =
            published_execution->forward_risk_batch;
        planning->diagnostic_forward_risk_batch =
            published_execution->diagnostic_forward_risk_batch;
      } else if (forward_risk_module && forward_risk_occupancy_owner) {
        planning->forward_risk_batch =
            [forward_risk_module, forward_risk_occupancy_owner, snapshot,
             hal = config_.predictor_hal_m,
             val = config_.predictor_val_m](
                const iap::ForwardRiskBatchRequest& input) {
              (void)forward_risk_occupancy_owner;
              iap::ForwardRiskBatchRequest request = input;
              request.snapshot = snapshot;
              request.hal = hal;
              request.val = val;
              return forward_risk_module->queryForwardRiskBatch(request);
            };
        planning->diagnostic_forward_risk_batch =
            [forward_risk_module, forward_risk_occupancy_owner,
             hal = config_.predictor_hal_m,
             val = config_.predictor_val_m](
                const iap::ForwardRiskBatchRequest& input) {
              (void)forward_risk_occupancy_owner;
              iap::ForwardRiskBatchRequest request = input;
              request.hal = hal;
              request.val = val;
              return forward_risk_module->queryForwardRiskBatch(request);
            };
      }
      std::lock_guard<std::mutex> lock(planning_snapshot_mutex_);
      planning_snapshot_ = std::move(planning);
    } else {
      refresh_succeeded = false;
      reason = "risk_occupancy_commit_identity_mismatch";
      risk_grid_.markRefreshFailure(now_s, reason);
    }
  }
  const iap::RiskGridHealth health = risk_grid_.health(now_s);
  const double refresh_end_stamp_s = liveNowSeconds();
  iap::RollingSpatialRefreshDiagnostics rolling_diagnostics;
  if (predictor_provider) {
    rolling_diagnostics = predictor_provider->rollingDiagnostics();
    predictor_provider->finish(refresh_succeeded);
    if (!refresh_succeeded) {
      rolling_diagnostics = {};
    }
    if (refresh_succeeded && occupancy_epoch &&
        active_gnss_occupancy_content) {
      rolling_occupancy_owner_ = occupancy_epoch->los_owner;
      rolling_raw_occupancy_identity_ = occupancy_epoch->raw_identity;
      rolling_occupancy_source_owner_ = occupancy_epoch->source_owner;
      rolling_occupancy_geometry_ = occupancy_epoch->geometry;
      rolling_occupancy_frame_contract_id_ =
          occupancy_epoch->frame_contract_id;
      rolling_occupancy_generation_ = occupancy_epoch->generation;
      rolling_occupancy_stamp_ = occupancy_epoch->cloud_stamp_s;
      rolling_occupancy_content_identity_ =
          candidate_occupancy_content_identity;
      rolling_support_identity_ = occupancy_epoch->trusted_local_map_support
          ? occupancy_epoch->trusted_local_map_support->identity()
          : std::string("strict_observation");
    }
  }
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    last_grid_stamp_s_ = viz_snapshot ? viz_snapshot->stamp_s()
                                      : std::numeric_limits<double>::quiet_NaN();
    last_refresh_elapsed_ms_ = risk_grid_build_elapsed_ms;
    last_provider_batch_duration_ms_ = timed_provider.durationMs();
    if (predictor_provider)
    {
      const auto& diagnostics = predictor_provider->diagnostics();
      last_predictor_unique_positions_ = diagnostics.unique_positions;
      last_predictor_lidar_evaluations_ = diagnostics.lidar_evaluations;
      last_predictor_lidar_cache_hits_ = diagnostics.lidar_cache_hits;
      last_predictor_spatial_advisory_recompute_count_ =
          diagnostics.spatial_advisory_recompute_count;
      last_predictor_spatial_advisory_reuse_count_ =
          diagnostics.spatial_advisory_reuse_count;
      last_predictor_gnss_advisory_invocation_count_ =
          diagnostics.gnss_advisory_invocations;
      last_predictor_lidar_advisory_invocation_count_ =
          diagnostics.lidar_advisory_invocations;
      last_predictor_horizon_fusion_count_ =
          diagnostics.fusion_advisory_invocations;
      last_rolling_spatial_diagnostics_ = rolling_diagnostics;
    }
    last_refresh_end_stamp_s_ = refresh_end_stamp_s;
    last_refresh_succeeded_ = refresh_succeeded;
    last_refresh_end_steady_s_ = steadyNowSeconds();
    if (refresh_succeeded) {
      last_generation_interval_ms_ =
          std::isfinite(last_generation_end_steady_s_)
              ? 1000.0 * (last_refresh_end_steady_s_ -
                          last_generation_end_steady_s_)
              : std::numeric_limits<double>::quiet_NaN();
      last_generation_end_steady_s_ = last_refresh_end_steady_s_;
    } else {
      last_generation_interval_ms_ =
          std::numeric_limits<double>::quiet_NaN();
    }
  }
  completeRefreshEvidence(health, now_s, refresh_succeeded);
  publishHealth(health, now_s);
  if (safety_viz_) {
    safety_viz_->publishPredictedPLCloud(viz_snapshot, snapshot.p_wb.z(),
                                         now_s);
    safety_viz_->publishRiskValidityCloud(viz_snapshot, snapshot.p_wb.z(),
                                          now_s);
  }
}

void P0RiskGridRuntime::publishHealth(const iap::RiskGridHealth& health,
                                      const double now_s) {
  // Never serialize or invoke ROS publishers while the input/refresh state
  // mutex is held. A slow subscriber or RViz transport must not block input
  // callbacks or turn health into a self-fulfilling stale signal.
  const iap::RiskGridHealth enriched_health = addLidarPredictorInputHealth(health);
  const auto mutex_wait_start = std::chrono::steady_clock::now();
  HealthPublicationState state;
  RefreshEvidenceRecord evidence;
  std::string snapshot_failure_reason = "none";
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    const auto mutex_acquired = std::chrono::steady_clock::now();
    last_publish_stamp_s_ = now_s;
    last_publish_steady_s_ = steadyNowSeconds();
    const double process_cpu_ms = 1000.0 * static_cast<double>(std::clock()) /
        static_cast<double>(CLOCKS_PER_SEC);
    last_process_cpu_delta_ms_ = std::isfinite(last_process_cpu_ms_)
        ? process_cpu_ms - last_process_cpu_ms_
        : std::numeric_limits<double>::quiet_NaN();
    last_process_cpu_ms_ = process_cpu_ms;
    const HealthPublicationState live_state = healthPublicationStateSnapshot();
    evidence = refresh_evidence_;
    state = evidence.publication;
    state.health_callback_stamp_s = live_state.health_callback_stamp_s;
    state.publish_stamp_s = live_state.publish_stamp_s;
    state.health_callback_steady_s = live_state.health_callback_steady_s;
    state.publish_steady_s = live_state.publish_steady_s;
    state.input_callback_count = live_state.input_callback_count;
    state.health_callback_count = live_state.health_callback_count;
    state.process_cpu_delta_ms = live_state.process_cpu_delta_ms;
    state.health_callback_duration_ms = live_state.health_callback_duration_ms;
    state.health_callback_queue_delay_ms = live_state.health_callback_queue_delay_ms;
    state.input_callback_age_s =
        std::isfinite(last_input_callback_steady_s_)
            ? std::max(0.0, last_publish_steady_s_ -
                                last_input_callback_steady_s_)
            : std::numeric_limits<double>::quiet_NaN();
    last_health_state_mutex_wait_ms_ =
        std::chrono::duration<double, std::milli>(mutex_acquired - mutex_wait_start).count();
    last_health_state_mutex_hold_ms_ =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - mutex_acquired).count();
    state.health_state_mutex_wait_ms = last_health_state_mutex_wait_ms_;
    state.health_state_mutex_hold_ms = last_health_state_mutex_hold_ms_;
    snapshot_failure_reason = evidence.snapshot_failure_reason;
  }
  if (safety_viz_) {
    safety_viz_->publishRiskGridHealth(enriched_health, now_s);
  }
  if (!node_ || !health_pub_) {
    return;
  }
  const iap::RiskGridHealth& out_health = evidence.health;
  const InputReadiness& readiness = evidence.readiness;
  // Refresh evidence is intentionally frozen for reproducibility, but a
  // long-running/failed RiskGrid build must not make current source health
  // look stale. Latency attribution uses a fresh view of the input chain;
  // the captured readiness remains serialized with the refresh transaction.
  const InputReadiness live_readiness = inputReadiness(now_s);
  const auto& health_snapshot = evidence.snapshot;
  const iap::RiskGridMapParams* grid_params = health_snapshot
      ? &health_snapshot->params() : nullptr;
  const Eigen::Vector3d grid_origin = health_snapshot
      ? health_snapshot->origin()
      : Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
  const Eigen::Vector3i grid_dimensions = health_snapshot
      ? health_snapshot->voxelNum() : Eigen::Vector3i::Zero();
  const iap::RiskGridSourceIdentity source_identity = health_snapshot
      ? health_snapshot->sourceIdentity() : iap::RiskGridSourceIdentity{};
  const std::string snapshot_config_hash = health_snapshot
      ? iap::canonicalRiskGridConfigHash(health_snapshot->params()) : "";
  const auto planning_snapshot = evidence.planning_snapshot;
  const bool anchor_matches_health = planning_snapshot && health_snapshot &&
      planning_snapshot->risk &&
      planning_snapshot->risk->generation_id() ==
          health_snapshot->generation_id();
  const auto& integrity_anchor = anchor_matches_health
      ? planning_snapshot->integrity_anchor.current
      : iap::CurrentIntegrityState{};
  const auto active_execution = acquireExecutionRiskSnapshot();
  const auto execution_attempt = lastExecutionSnapshotAttempt();
  P0ExecutionSnapshotAttemptEvidence execution_failure;
  uint64_t execution_attempt_count = 0;
  uint64_t execution_publish_count = 0;
  uint64_t execution_failure_count = 0;
  uint64_t execution_pending_overwrite_count = 0;
  uint64_t risk_grid_yield_count = 0;
  double risk_grid_yield_duration_ms = 0.0;
  bool execution_snapshot_pending_or_in_flight = false;
  uint64_t last_requested_occupancy_generation = 0;
  {
    std::lock_guard<std::mutex> lock(execution_snapshot_worker_mutex_);
    execution_attempt_count = execution_snapshot_attempt_count_;
    execution_publish_count = execution_snapshot_publish_count_;
    execution_failure_count = execution_snapshot_failure_count_;
    execution_failure = last_execution_snapshot_failure_;
    execution_pending_overwrite_count =
        execution_snapshot_pending_overwrite_count_;
    risk_grid_yield_count = risk_grid_yield_count_;
    risk_grid_yield_duration_ms = risk_grid_yield_duration_ms_;
    execution_snapshot_pending_or_in_flight =
        pending_execution_snapshot_request_.has_value() ||
        execution_snapshot_worker_in_flight_;
    last_requested_occupancy_generation =
        last_requested_occupancy_generation_;
  }
  const bool source_data_gap =
      !live_readiness.odom_fresh ||
      !live_readiness.current_integrity_fresh ||
      !live_readiness.gnss_epoch_fresh || !live_readiness.map_fresh;
  const bool occupancy_build_lag =
      snapshot_failure_reason.find("occupancy") != std::string::npos ||
      execution_attempt.status ==
          P0ExecutionSnapshotAttemptStatus::CAPTURE_UNAVAILABLE ||
      execution_attempt.status ==
          P0ExecutionSnapshotAttemptStatus::CAPTURE_ADAPTER_INVALID ||
      execution_attempt.status ==
          P0ExecutionSnapshotAttemptStatus::OCCUPANCY_INVALID ||
      execution_attempt.status ==
          P0ExecutionSnapshotAttemptStatus::MAP_POINTS_MISSING;
  // Queue lag is reserved for a healthy source tuple whose requested
  // generation has not reached an atomic execution publication. Build
  // failures remain separately attributable above.
  const bool snapshot_queue_lag =
      !source_data_gap && !occupancy_build_lag &&
      (execution_snapshot_pending_or_in_flight || !active_execution ||
       (last_requested_occupancy_generation != 0u && active_execution &&
        active_execution->source_identity.occupancy_generation <
            last_requested_occupancy_generation) ||
       execution_attempt.status ==
           P0ExecutionSnapshotAttemptStatus::FINAL_FRESHNESS_FAILED ||
       execution_attempt.status ==
           P0ExecutionSnapshotAttemptStatus::WORKER_EXCEPTION);
  const bool risk_grid_build_lag =
      snapshot_failure_reason == "risk_grid_build_budget_exceeded" ||
      (std::isfinite(state.refresh_elapsed_ms) &&
       state.refresh_elapsed_ms > config_.risk_grid_build_budget_ms);
  const char* latency_primary_cause = source_data_gap
      ? "SOURCE_DATA_GAP"
      : occupancy_build_lag
          ? "OCCUPANCY_BUILD_LAG"
          : snapshot_queue_lag
              ? "SNAPSHOT_QUEUE_LAG"
              : risk_grid_build_lag ? "RISK_GRID_BUILD_LAG" : "NONE";
  std::vector<std::string> latency_contributors;
  if (source_data_gap) latency_contributors.emplace_back("SOURCE_DATA_GAP");
  if (occupancy_build_lag)
    latency_contributors.emplace_back("OCCUPANCY_BUILD_LAG");
  if (snapshot_queue_lag)
    latency_contributors.emplace_back("SNAPSHOT_QUEUE_LAG");
  if (risk_grid_build_lag)
    latency_contributors.emplace_back("RISK_GRID_BUILD_LAG");
  std::ostringstream latency_contributors_json;
  latency_contributors_json << '[';
  for (std::size_t index = 0; index < latency_contributors.size(); ++index) {
    if (index > 0) latency_contributors_json << ',';
    latency_contributors_json << jsonString(latency_contributors[index]);
  }
  latency_contributors_json << ']';
  std::ostringstream oss;
  oss << "{"
      << "\"refresh_attempt_id\":" << evidence.refresh_attempt_id << ","
      << "\"refresh_evidence_state\":"
      << jsonString(refreshEvidenceStateName(evidence.state)) << ","
      << "\"result_generation_id\":"
      << evidence.result_generation_id << ","
      << "\"previous_successful_generation_id\":"
      << evidence.previous_successful_generation_id << ","
      << "\"ready\":" << (out_health.ready ? "true" : "false") << ","
      << "\"stale\":" << (out_health.stale ? "true" : "false") << ","
      << "\"age_s\":" << jsonNumber(out_health.age_s) << ","
      << "\"valid_ratio\":" << jsonNumber(out_health.valid_ratio) << ","
      << "\"unknown_ratio\":" << jsonNumber(out_health.unknown_ratio) << ","
      << "\"generation_id\":" << out_health.generation_id << ","
      << "\"snapshot_config_hash\":"
      << jsonString(snapshot_config_hash) << ","
      << "\"source_identity_hash\":"
      << jsonString(iap::canonicalRiskGridSourceIdentityHash(
             source_identity)) << ","
      << "\"snapshot_identity_available\":"
      << (health_snapshot ? "true" : "false") << ","
      << "\"geometry_id\":"
      << jsonString(grid_params ? grid_params->geometry_id : "") << ","
      << "\"frame_id\":"
      << jsonString(grid_params ? grid_params->frame_id : "") << ","
      << "\"grid_origin_m\":[" << jsonNumber(grid_origin.x()) << ","
      << jsonNumber(grid_origin.y()) << "," << jsonNumber(grid_origin.z())
      << "],"
      << "\"grid_extent_m\":["
      << jsonNumber(grid_params ? grid_params->size_x_m
                                : std::numeric_limits<double>::quiet_NaN())
      << ","
      << jsonNumber(grid_params ? grid_params->size_y_m
                                : std::numeric_limits<double>::quiet_NaN())
      << ","
      << jsonNumber(grid_params ? grid_params->size_z_m
                                : std::numeric_limits<double>::quiet_NaN())
      << "],"
      << "\"grid_dimensions\":[" << grid_dimensions.x() << ","
      << grid_dimensions.y() << "," << grid_dimensions.z() << "],"
      << "\"grid_resolution_m\":"
      << jsonNumber(grid_params ? grid_params->resolution_m
                                : std::numeric_limits<double>::quiet_NaN())
      << ","
      << "\"alert_limit_policy_id\":"
      << jsonString(grid_params ? grid_params->alert_limit_policy_id : "")
      << ","
      << "\"alert_limit_h_m\":"
      << jsonNumber(grid_params ? grid_params->alert_limit_h_m
                                : std::numeric_limits<double>::quiet_NaN())
      << ",\"alert_limit_v_m\":"
      << jsonNumber(grid_params ? grid_params->alert_limit_v_m
                                : std::numeric_limits<double>::quiet_NaN())
      << ","
      << "\"gnss_anchor_available\":"
      << (anchor_matches_health && integrity_anchor.valid &&
                  integrity_anchor.gnss_valid
              ? "true" : "false") << ","
      << "\"gnss_anchor_hpl_m\":"
      << jsonNumber(anchor_matches_health ? integrity_anchor.hpl
                                          : std::numeric_limits<double>::quiet_NaN())
      << ",\"gnss_anchor_vpl_m\":"
      << jsonNumber(anchor_matches_health ? integrity_anchor.vpl
                                          : std::numeric_limits<double>::quiet_NaN())
      << ",\"gnss_anchor_stamp_s\":"
      << jsonNumber(anchor_matches_health ? integrity_anchor.stamp
                                          : std::numeric_limits<double>::quiet_NaN())
      << ",\"gnss_anchor_epoch_delta_s\":"
      << jsonNumber(anchor_matches_health
            ? std::abs(integrity_anchor.stamp -
                       planning_snapshot->integrity_anchor.gnss_epoch.stamp)
            : std::numeric_limits<double>::quiet_NaN())
      << ",\"gnss_support_ray_length_m\":"
      << jsonNumber(anchor_matches_health
            ? planning_snapshot->gnss_support_ray_length_m
            : std::numeric_limits<double>::quiet_NaN())
      << ",\"gnss_hard_occlusion\":"
      << (anchor_matches_health && planning_snapshot->gnss_hard_occlusion
              ? "true" : "false") << ","
      << "\"source_occupancy_generation\":"
      << source_identity.occupancy_generation << ","
      << "\"source_occupancy_stamp_s\":"
      << jsonNumber(source_identity.occupancy_stamp_s) << ","
      << "\"source_prior_generation\":"
      << source_identity.prior_generation << ","
      << "\"source_prior_stamp_s\":"
      << jsonNumber(source_identity.prior_stamp_s) << ","
      << "\"source_gnss_generation\":"
      << source_identity.gnss_generation << ","
      << "\"source_gnss_stamp_s\":"
      << jsonNumber(source_identity.gnss_stamp_s) << ","
      << "\"source_lidar_generation\":"
      << source_identity.lidar_generation << ","
      << "\"source_lidar_stamp_s\":"
      << jsonNumber(source_identity.lidar_stamp_s) << ","
      << "\"source_local_map_support_identity\":"
      << jsonString(source_identity.local_map_support_identity) << ","
      << "\"provider_query_count\":" << out_health.provider_query_count << ","
      << "\"occupied_skip_count\":" << out_health.occupied_skip_count << ","
      << "\"provider_stale_count\":" << out_health.provider_stale_count << ","
      << "\"provider_invalid_count\":" << out_health.provider_invalid_count << ","
      << "\"predictor_gnss_used_count\":"
      << out_health.predictor_gnss_used_count << ","
      << "\"predictor_lidar_used_count\":"
      << out_health.predictor_lidar_used_count << ","
      << "\"predictor_prior_used_count\":"
      << out_health.predictor_prior_used_count << ","
      << "\"predictor_stale_current_prior_count\":"
      << out_health.predictor_stale_current_prior_count << ","
      << "\"predictor_regularized_count\":"
      << out_health.predictor_regularized_count << ","
      << "\"predictor_conservative_max_count\":"
      << out_health.predictor_conservative_max_count << ","
      << "\"predictor_lidar_map_point_count\":"
      << out_health.predictor_lidar_map_point_count << ","
      << "\"predictor_lidar_fim_primitive_count\":"
      << out_health.predictor_lidar_fim_primitive_count << ","
      << "\"predictor_lidar_fim_valid_normal_count\":"
      << out_health.predictor_lidar_fim_valid_normal_count << ","
      << "\"predictor_lidar_fim_fallback_reason\":"
      << jsonString(out_health.predictor_lidar_fim_fallback_reason) << ","
      << "\"occupancy_support_scan_ms\":"
      << jsonNumber(out_health.occupancy_support_scan_ms) << ","
      << "\"query_layout_ms\":"
      << jsonNumber(out_health.query_layout_ms) << ","
      << "\"provider_batch_ms\":"
      << jsonNumber(out_health.provider_batch_ms) << ","
      << "\"voxel_materialization_ms\":"
      << jsonNumber(out_health.voxel_materialization_ms) << ","
      << "\"risk_grid_commit_ms\":"
      << jsonNumber(out_health.commit_ms) << ","
      << "\"risk_grid_map_build_total_ms\":"
      << jsonNumber(out_health.build_total_ms) << ","
      << "\"dominant_unknown_reason\":"
      << jsonString(out_health.dominant_unknown_reason) << ","
      << "\"dominant_unknown_count\":"
      << out_health.dominant_unknown_count << ","
      << "\"refresh_stamp_s\":" << jsonNumber(state.refresh_stamp_s) << ","
      << "\"refresh_callback_start_stamp_s\":" << jsonNumber(state.refresh_start_stamp_s) << ","
      << "\"refresh_callback_end_stamp_s\":" << jsonNumber(state.refresh_end_stamp_s) << ","
      << "\"health_callback_stamp_s\":" << jsonNumber(state.health_callback_stamp_s) << ","
      << "\"publish_stamp_s\":" << jsonNumber(state.publish_stamp_s) << ","
      << "\"refresh_callback_start_steady_s\":" << jsonNumber(state.refresh_start_steady_s) << ","
      << "\"refresh_scheduled_steady_s\":" << jsonNumber(state.refresh_scheduled_steady_s) << ","
      << "\"refresh_callback_end_steady_s\":" << jsonNumber(state.refresh_end_steady_s) << ","
      << "\"health_callback_steady_s\":" << jsonNumber(state.health_callback_steady_s) << ","
      << "\"publish_steady_s\":" << jsonNumber(state.publish_steady_s) << ","
      << "\"last_grid_stamp_s\":" << jsonNumber(state.last_grid_stamp_s) << ","
      << "\"refresh_elapsed_ms\":" << jsonNumber(state.refresh_elapsed_ms) << ","
      << "\"refresh_duration_ms\":" << jsonNumber(state.refresh_elapsed_ms) << ","
      << "\"refresh_queue_delay_ms\":" << jsonNumber(state.refresh_queue_delay_ms) << ","
      << "\"provider_batch_duration_ms\":" << jsonNumber(state.provider_batch_duration_ms) << ","
      << "\"generation_interval_ms\":" << jsonNumber(state.generation_interval_ms) << ","
      << "\"execution_snapshot_id\":" << state.execution_snapshot_id << ","
      << "\"execution_snapshot_evaluation_stamp_s\":"
      << jsonNumber(state.execution_snapshot_evaluation_stamp_s) << ","
      << "\"execution_snapshot_publish_stamp_s\":"
      << jsonNumber(state.execution_snapshot_publish_stamp_s) << ","
      << "\"execution_snapshot_publish_latency_ms\":"
      << jsonNumber(state.execution_snapshot_publish_latency_ms) << ","
      << "\"execution_snapshot_attempt_count\":"
      << execution_attempt_count << ","
      << "\"execution_snapshot_publish_count\":"
      << execution_publish_count << ","
      << "\"execution_snapshot_failure_count\":"
      << execution_failure_count << ","
      << "\"execution_snapshot_pending_overwrite_count\":"
      << execution_pending_overwrite_count << ","
      << "\"execution_snapshot_last_attempt_status\":"
      << jsonString(p0ExecutionSnapshotAttemptStatusName(
             execution_attempt.status)) << ","
      << "\"execution_snapshot_last_failure\":"
      << jsonString(execution_failure_count > 0u
             ? execution_failure.reason : "") << ","
      << "\"execution_snapshot_last_failure_status\":"
      << jsonString(execution_failure_count > 0u
             ? p0ExecutionSnapshotAttemptStatusName(execution_failure.status)
             : "") << ","
      << "\"execution_snapshot_last_failure_attempt_id\":"
      << (execution_failure_count > 0u ? execution_failure.attempt_id : 0u)
      << ","
      << "\"execution_snapshot_last_queue_delay_ms\":"
      << jsonNumber(execution_attempt.queue_delay_ms) << ","
      << "\"execution_snapshot_last_build_duration_ms\":"
      << jsonNumber(execution_attempt.build_duration_ms) << ","
      << "\"execution_snapshot_last_publish_age_s\":"
      << jsonNumber(execution_attempt.publish_age_s) << ","
      << "\"risk_grid_execution_priority_yield_count\":"
      << risk_grid_yield_count << ","
      << "\"risk_grid_execution_priority_yield_duration_ms\":"
      << jsonNumber(risk_grid_yield_duration_ms) << ","
      << "\"input_callback_age_s\":" << jsonNumber(state.input_callback_age_s) << ","
      << "\"input_callback_count\":" << state.input_callback_count << ","
      << "\"health_callback_count\":" << state.health_callback_count << ","
      << "\"process_cpu_delta_ms\":" << jsonNumber(state.process_cpu_delta_ms) << ","
      << "\"health_callback_duration_ms\":" << jsonNumber(state.health_callback_duration_ms) << ","
      << "\"health_callback_queue_delay_ms\":" << jsonNumber(state.health_callback_queue_delay_ms) << ","
      << "\"health_state_mutex_wait_ms\":" << jsonNumber(state.health_state_mutex_wait_ms) << ","
      << "\"health_state_mutex_hold_ms\":" << jsonNumber(state.health_state_mutex_hold_ms) << ","
      << "\"snapshot_available\":"
      << (state.snapshot_available ? "true" : "false") << ","
      << "\"snapshot_failure_reason\":"
      << jsonString(snapshot_failure_reason) << ","
      << "\"odom_seen\":" << jsonBool(readiness.odom_seen) << ","
      << "\"odom_valid\":" << jsonBool(readiness.odom_valid) << ","
      << "\"odom_fresh\":" << jsonBool(readiness.odom_fresh) << ","
      << "\"odom_stamp_s\":" << jsonNumber(readiness.odom_stamp_s) << ","
      << "\"current_integrity_seen\":"
      << jsonBool(readiness.current_integrity_seen) << ","
      << "\"current_integrity_valid\":"
      << jsonBool(readiness.current_integrity_valid) << ","
      << "\"current_integrity_fresh\":"
      << jsonBool(readiness.current_integrity_fresh) << ","
      << "\"current_integrity_stamp_s\":"
      << jsonNumber(readiness.current_integrity_stamp_s) << ","
      << "\"gnss_epoch_seen\":" << jsonBool(readiness.gnss_epoch_seen) << ","
      << "\"gnss_epoch_valid\":" << jsonBool(readiness.gnss_epoch_valid) << ","
      << "\"gnss_epoch_fresh\":" << jsonBool(readiness.gnss_epoch_fresh) << ","
      << "\"gnss_epoch_stamp_s\":"
      << jsonNumber(readiness.gnss_epoch_stamp_s) << ","
      << "\"gnss_epoch_satellite_count\":"
      << readiness.gnss_epoch_satellite_count << ","
      << "\"origin_seen\":" << jsonBool(readiness.origin_seen) << ","
      << "\"origin_valid\":" << jsonBool(readiness.origin_valid) << ","
      << "\"origin_fresh\":" << jsonBool(readiness.origin_fresh) << ","
      << "\"origin_stamp_s\":" << jsonNumber(readiness.origin_stamp_s) << ","
      << "\"map_seen\":" << jsonBool(readiness.map_seen) << ","
      << "\"map_valid\":" << jsonBool(readiness.map_valid) << ","
      << "\"map_fresh\":" << jsonBool(readiness.map_fresh) << ","
      << "\"map_stamp_s\":" << jsonNumber(readiness.map_stamp_s) << ","
      << "\"map_point_count\":" << readiness.map_point_count << ","
      << "\"refresh_query_count\":" << state.refresh_query_count << ","
      << "\"predictor_unique_positions\":" << state.predictor_unique_positions << ","
      << "\"predictor_lidar_evaluations\":" << state.predictor_lidar_evaluations << ","
      << "\"predictor_lidar_cache_hits\":" << state.predictor_lidar_cache_hits << ","
      << "\"predictor_spatial_advisory_recompute_count\":"
      << state.predictor_spatial_advisory_recompute_count << ","
      << "\"predictor_spatial_advisory_reuse_count\":"
      << state.predictor_spatial_advisory_reuse_count << ","
      << "\"predictor_gnss_advisory_invocation_count\":"
      << state.predictor_gnss_advisory_invocation_count << ","
      << "\"predictor_lidar_advisory_invocation_count\":"
      << state.predictor_lidar_advisory_invocation_count << ","
      << "\"predictor_horizon_fusion_count\":"
      << state.predictor_horizon_fusion_count << ","
      << "\"predictor_spatial_retained_position_count\":"
      << state.predictor_spatial_retained_position_count << ","
      << "\"predictor_spatial_entered_position_count\":"
      << state.predictor_spatial_entered_position_count << ","
      << "\"predictor_spatial_evicted_position_count\":"
      << state.predictor_spatial_evicted_position_count << ","
      << "\"predictor_spatial_full_invalidation_count\":"
      << state.predictor_spatial_full_invalidation_count << ","
      << "\"predictor_spatial_exact_retained_position_count\":"
      << state.predictor_spatial_exact_retained_position_count << ","
      << "\"predictor_spatial_ttl_retained_position_count\":"
      << state.predictor_spatial_ttl_retained_position_count << ","
      << "\"predictor_spatial_gnss_ttl_expired_position_count\":"
      << state.predictor_spatial_gnss_ttl_expired_position_count << ","
      << "\"predictor_spatial_legacy_current_ttl_expired_position_count\":"
      << state.predictor_spatial_legacy_current_ttl_expired_position_count
      << ","
      << "\"predictor_spatial_watchdog_forced_full_rebuild_count\":"
      << state.predictor_spatial_watchdog_forced_full_rebuild_count << ","
      << "\"predictor_spatial_invalid_source_provenance_count\":"
      << state.predictor_spatial_invalid_source_provenance_count << ","
      << "\"predictor_spatial_invalidation_reason\":"
      << jsonString(state.predictor_spatial_invalidation_reason) << ","
      << "\"predictor_requested_worker_count\":" << config_.predictor_requested_worker_count << ","
      << "\"predictor_effective_worker_count\":" << config_.predictor_effective_worker_count << ","
      << "\"risk_grid_build_budget_ms\":"
      << jsonNumber(config_.risk_grid_build_budget_ms) << ","
      << "\"execution_snapshot_period_s\":"
      << jsonNumber(config_.execution_snapshot_period_s) << ","
      << "\"execution_batch_budget_ms\":"
      << jsonNumber(config_.execution_batch_budget_ms) << ","
      << "\"execution_frame_contract_id\":"
      << jsonString(active_execution
                        ? active_execution->frame_contract_id : "") << ","
      << "\"latency_contributors\":"
      << latency_contributors_json.str() << ","
      << "\"latency_primary_cause\":"
      << jsonString(latency_primary_cause) << ","
      << "\"reason\":" << jsonString(out_health.reason)
      << "}";
  std_msgs::msg::String msg;
  msg.data = oss.str();
  health_pub_->publish(msg);
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                       "[p0] risk grid ready=%d stale=%d age=%.3f "
                       "valid=%.3f unknown=%.3f gen=%lu "
                       "provider_queries=%lu occupied_skip=%lu "
                       "provider_stale=%lu provider_invalid=%lu "
                       "gnss_used=%lu lidar_used=%lu prior_used=%lu "
                       "stale_current_prior=%lu "
                       "regularized=%lu conservative_max=%lu "
                       "lidar_points=%lu lidar_fim_primitives=%lu "
                       "lidar_fim_valid_normals=%lu lidar_fim_fallback=%s "
                       "dominant_unknown=%s:%lu "
                       "refresh_stamp=%.6f grid_stamp=%.6f elapsed_ms=%.3f "
                       "queries=%zu reason=%s",
                       out_health.ready, out_health.stale, out_health.age_s,
                       out_health.valid_ratio, out_health.unknown_ratio,
                       static_cast<unsigned long>(out_health.generation_id),
                       static_cast<unsigned long>(
                           out_health.provider_query_count),
                       static_cast<unsigned long>(
                           out_health.occupied_skip_count),
                       static_cast<unsigned long>(
                           out_health.provider_stale_count),
                       static_cast<unsigned long>(
                           out_health.provider_invalid_count),
                       static_cast<unsigned long>(
                           out_health.predictor_gnss_used_count),
                       static_cast<unsigned long>(
                           out_health.predictor_lidar_used_count),
                       static_cast<unsigned long>(
                           out_health.predictor_prior_used_count),
                       static_cast<unsigned long>(
                           out_health.predictor_stale_current_prior_count),
                       static_cast<unsigned long>(
                           out_health.predictor_regularized_count),
                       static_cast<unsigned long>(
                           out_health.predictor_conservative_max_count),
                       static_cast<unsigned long>(
                           out_health.predictor_lidar_map_point_count),
                       static_cast<unsigned long>(
                           out_health.predictor_lidar_fim_primitive_count),
                       static_cast<unsigned long>(
                           out_health.predictor_lidar_fim_valid_normal_count),
                       out_health.predictor_lidar_fim_fallback_reason.c_str(),
                       out_health.dominant_unknown_reason.c_str(),
                       static_cast<unsigned long>(
                           out_health.dominant_unknown_count),
                       state.refresh_stamp_s, state.last_grid_stamp_s,
                       state.refresh_elapsed_ms, state.refresh_query_count,
                       out_health.reason.c_str());
}

P0RiskGridRuntime::HealthPublicationState
P0RiskGridRuntime::healthPublicationStateSnapshot() const {
  // Caller owns health_state_mutex_. Keeping this as a copy-only helper makes
  // it hard to regress into publishing under the mutex.
  HealthPublicationState state;
  state.refresh_stamp_s = last_refresh_stamp_s_;
  state.refresh_scheduled_steady_s = last_refresh_scheduled_steady_s_;
  state.refresh_start_stamp_s = last_refresh_start_stamp_s_;
  state.refresh_end_stamp_s = last_refresh_end_stamp_s_;
  state.health_callback_stamp_s = last_health_callback_stamp_s_;
  state.publish_stamp_s = last_publish_stamp_s_;
  state.refresh_start_steady_s = last_refresh_start_steady_s_;
  state.refresh_end_steady_s = last_refresh_end_steady_s_;
  state.health_callback_steady_s = last_health_callback_steady_s_;
  state.publish_steady_s = last_publish_steady_s_;
  state.last_grid_stamp_s = last_grid_stamp_s_;
  state.refresh_elapsed_ms = last_refresh_elapsed_ms_;
  state.refresh_queue_delay_ms = last_refresh_queue_delay_ms_;
  state.provider_batch_duration_ms = last_provider_batch_duration_ms_;
  state.generation_interval_ms = last_generation_interval_ms_;
  state.execution_snapshot_id = last_execution_snapshot_id_;
  state.execution_snapshot_evaluation_stamp_s =
      last_execution_snapshot_evaluation_stamp_s_;
  state.execution_snapshot_publish_stamp_s =
      last_execution_snapshot_publish_stamp_s_;
  state.execution_snapshot_publish_latency_ms =
      last_execution_snapshot_publish_latency_ms_;
  state.process_cpu_delta_ms = last_process_cpu_delta_ms_;
  state.health_callback_duration_ms = last_health_callback_duration_ms_;
  state.health_callback_queue_delay_ms = last_health_callback_queue_delay_ms_;
  state.health_state_mutex_wait_ms = last_health_state_mutex_wait_ms_;
  state.health_state_mutex_hold_ms = last_health_state_mutex_hold_ms_;
  state.refresh_query_count = last_refresh_query_count_;
  state.predictor_unique_positions = last_predictor_unique_positions_;
  state.predictor_lidar_evaluations = last_predictor_lidar_evaluations_;
  state.predictor_lidar_cache_hits = last_predictor_lidar_cache_hits_;
  state.predictor_spatial_advisory_recompute_count =
      last_predictor_spatial_advisory_recompute_count_;
  state.predictor_spatial_advisory_reuse_count =
      last_predictor_spatial_advisory_reuse_count_;
  state.predictor_gnss_advisory_invocation_count =
      last_predictor_gnss_advisory_invocation_count_;
  state.predictor_lidar_advisory_invocation_count =
      last_predictor_lidar_advisory_invocation_count_;
  state.predictor_horizon_fusion_count =
      last_predictor_horizon_fusion_count_;
  state.predictor_spatial_retained_position_count =
      last_rolling_spatial_diagnostics_.retained_position_count;
  state.predictor_spatial_entered_position_count =
      last_rolling_spatial_diagnostics_.entered_position_count;
  state.predictor_spatial_evicted_position_count =
      last_rolling_spatial_diagnostics_.evicted_position_count;
  state.predictor_spatial_full_invalidation_count =
      last_rolling_spatial_diagnostics_.full_invalidation_count;
  state.predictor_spatial_exact_retained_position_count =
      last_rolling_spatial_diagnostics_.exact_retained_position_count;
  state.predictor_spatial_ttl_retained_position_count =
      last_rolling_spatial_diagnostics_.ttl_retained_position_count;
  state.predictor_spatial_gnss_ttl_expired_position_count =
      last_rolling_spatial_diagnostics_.gnss_ttl_expired_position_count;
  state.predictor_spatial_legacy_current_ttl_expired_position_count =
      last_rolling_spatial_diagnostics_
          .legacy_current_ttl_expired_position_count;
  state.predictor_spatial_watchdog_forced_full_rebuild_count =
      last_rolling_spatial_diagnostics_
          .watchdog_forced_full_rebuild_count;
  state.predictor_spatial_invalid_source_provenance_count =
      last_rolling_spatial_diagnostics_
          .invalid_source_provenance_count;
  state.predictor_spatial_invalidation_reason =
      iap::rollingSpatialInvalidationReasonName(
          last_rolling_spatial_diagnostics_.invalidation_reason);
  state.input_callback_count = input_callback_count_;
  state.health_callback_count = health_callback_count_;
  state.snapshot_available = last_snapshot_available_;
  return state;
}

P0RiskGridRuntime::RefreshEvidenceRecord
P0RiskGridRuntime::refreshEvidenceRecordSnapshot() const {
  std::lock_guard<std::mutex> lock(health_state_mutex_);
  return refresh_evidence_;
}

const char* P0RiskGridRuntime::refreshEvidenceStateName(
    const RefreshEvidenceState state) {
  switch (state) {
    case RefreshEvidenceState::PRE_REFRESH:
      return "PRE_REFRESH";
    case RefreshEvidenceState::IN_PROGRESS:
      return "IN_PROGRESS";
    case RefreshEvidenceState::COMPLETED_SUCCESS:
      return "COMPLETED_SUCCESS";
    case RefreshEvidenceState::COMPLETED_FAILURE:
      return "COMPLETED_FAILURE";
  }
  return "UNKNOWN";
}

void P0RiskGridRuntime::completeRefreshEvidence(
    const iap::RiskGridHealth& health, const double now_s,
    const bool succeeded) {
  (void)now_s;
  const auto completed_snapshot = risk_grid_.acquireSnapshot();
  const auto completed_planning_snapshot = acquirePlanningSnapshot();
  std::scoped_lock lock(health_state_mutex_, lidar_predictor_input_mutex_);
  const iap::RiskGridHealth completed_health =
      addLidarPredictorInputHealthLocked(health);
  refresh_evidence_.state = succeeded
      ? RefreshEvidenceState::COMPLETED_SUCCESS
      : RefreshEvidenceState::COMPLETED_FAILURE;
  refresh_evidence_.result_generation_id =
      succeeded ? completed_health.generation_id : 0u;
  refresh_evidence_.previous_successful_generation_id =
      last_successful_generation_id_;
  refresh_evidence_.publication = healthPublicationStateSnapshot();
  refresh_evidence_.readiness = refresh_input_readiness_;
  refresh_evidence_.health = completed_health;
  refresh_evidence_.snapshot.reset();
  refresh_evidence_.planning_snapshot.reset();
  if (completed_snapshot &&
      completed_snapshot->generation_id() == completed_health.generation_id) {
    refresh_evidence_.snapshot = completed_snapshot;
    if (completed_planning_snapshot && completed_planning_snapshot->risk &&
        completed_planning_snapshot->risk->generation_id() ==
            completed_snapshot->generation_id()) {
      refresh_evidence_.planning_snapshot = completed_planning_snapshot;
    }
  }
  refresh_evidence_.snapshot_failure_reason = last_snapshot_failure_reason_;
  if (succeeded) {
    last_successful_generation_id_ = completed_health.generation_id;
  }
}

P0RiskGridRuntime::InputReadiness
P0RiskGridRuntime::inputReadiness(const double now_s) const {
  std::scoped_lock lock(health_state_mutex_, lidar_predictor_input_mutex_);
  return inputReadinessLocked(now_s);
}

P0RiskGridRuntime::InputReadiness
P0RiskGridRuntime::inputReadinessLocked(const double now_s) const {
  InputReadiness readiness;
  const double odom_stamp = latest_odom_stamp_;
  const double current_stamp = latest_current_.stamp;
  const double map_stamp = latest_map_stamp_;
  const double origin_stamp = latest_origin_stamp_;
  const double gnss_epoch_stamp = latest_epoch_
      ? latest_epoch_->stamp : latest_gnss_epoch_stamp_;
  const uint64_t epoch_satellite_count = latest_epoch_
      ? static_cast<uint64_t>(latest_epoch_->sats.size())
      : latest_gnss_epoch_satellite_count_;
  const std::size_t map_point_count = latest_lidar_map_point_count_;

  const double stale_timeout = config_.grid.stale_timeout_s;
  const double gnss_timeout = config_.gnss_epoch_max_age_s;
  const bool finite_now = std::isfinite(now_s);

  readiness.odom_seen = odom_seen_;
  readiness.odom_valid = odom_seen_ && latest_odom_pose_valid_;
  readiness.odom_stamp_s = odom_stamp;
  readiness.odom_fresh =
      readiness.odom_valid && finite_now && std::isfinite(odom_stamp) &&
      odom_stamp > 0.0 && (now_s - odom_stamp) <= stale_timeout;

  readiness.current_integrity_seen = current_integrity_seen_;
  readiness.current_integrity_valid =
      current_integrity_seen_ && latest_current_valid_;
  readiness.current_integrity_stamp_s = current_stamp;
  readiness.current_integrity_fresh =
      readiness.current_integrity_valid && finite_now &&
      std::isfinite(current_stamp) && current_stamp > 0.0 &&
      (now_s - current_stamp) <= stale_timeout;

  readiness.gnss_epoch_seen = gnss_epoch_seen_;
  readiness.gnss_epoch_valid = gnss_epoch_seen_ && latest_epoch_.has_value() &&
                               std::isfinite(gnss_epoch_stamp) &&
                               epoch_satellite_count > 0;
  readiness.gnss_epoch_stamp_s = gnss_epoch_stamp;
  readiness.gnss_epoch_satellite_count = epoch_satellite_count;
  readiness.gnss_epoch_fresh =
      readiness.gnss_epoch_valid && finite_now &&
      std::isfinite(gnss_epoch_stamp) &&
      (gnss_timeout < 0.0 || (now_s - gnss_epoch_stamp) <= gnss_timeout);

  readiness.origin_seen = origin_seen_;
  readiness.origin_valid = origin_seen_ && origin_set_;
  readiness.origin_stamp_s = origin_stamp;
  readiness.origin_fresh =
      readiness.origin_valid && finite_now && std::isfinite(origin_stamp) &&
      origin_stamp > 0.0 && (now_s - origin_stamp) <= stale_timeout;

  readiness.map_seen = map_seen_;
  readiness.map_valid =
      map_seen_ && std::isfinite(map_stamp) && map_point_count > 0;
  readiness.map_stamp_s = map_stamp;
  readiness.map_point_count = static_cast<uint64_t>(map_point_count);
  readiness.map_fresh =
      readiness.map_valid && finite_now && std::isfinite(map_stamp) &&
      map_stamp > 0.0 && (now_s - map_stamp) <= stale_timeout;
  return readiness;
}

std::string P0RiskGridRuntime::snapshotFailureReason(
    const double now_s) const {
  if (!std::isfinite(now_s) || now_s <= 0.0) {
    return "message_stamp_unavailable";
  }
  const InputReadiness readiness = inputReadiness(now_s);
  if (!readiness.odom_seen) {
    return "odom_missing";
  }
  if (!readiness.odom_valid) {
    return "odom_invalid";
  }
  if (!readiness.current_integrity_seen) {
    return "current_integrity_missing";
  }
  if (!readiness.current_integrity_valid) {
    return "current_integrity_invalid";
  }
  return "none";
}

void P0RiskGridRuntime::recordInputCallback() {
  std::lock_guard<std::mutex> health_lock(health_state_mutex_);
  last_input_callback_steady_s_ = steadyNowSeconds();
  ++input_callback_count_;
}

void P0RiskGridRuntime::odomCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  if (!msg) {
    return;
  }
  recordInputCallback();
  std::lock_guard<std::mutex> health_lock(health_state_mutex_);
  odom_seen_ = true;
  latest_odom_stamp_ = stampToSec(msg->header.stamp);
  latest_odom_p_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                   msg->pose.pose.position.y,
                                   msg->pose.pose.position.z);
  latest_odom_q_ = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                      msg->pose.pose.orientation.x,
                                      msg->pose.pose.orientation.y,
                                      msg->pose.pose.orientation.z);
  latest_odom_pose_valid_ =
      latest_odom_p_.allFinite() && std::isfinite(latest_odom_q_.w()) &&
      std::isfinite(latest_odom_q_.x()) &&
      std::isfinite(latest_odom_q_.y()) &&
      std::isfinite(latest_odom_q_.z());
}

void P0RiskGridRuntime::integrityCallback(
    const iap::msg::IntegrityReport::ConstSharedPtr msg) {
  if (!msg) {
    return;
  }
  recordInputCallback();
  std::lock_guard<std::mutex> health_lock(health_state_mutex_);
  advanceNonzeroGeneration(&latest_current_generation_);
  current_integrity_seen_ = true;
  latest_current_ = currentFromMsg(*msg);
  latest_current_valid_ = latest_current_.valid;
  current_integrity_history_.emplace_back(
      latest_current_generation_, latest_current_);
  constexpr std::size_t kMaxCurrentIntegrityHistory = 512u;
  while (current_integrity_history_.size() >
         kMaxCurrentIntegrityHistory) {
    current_integrity_history_.pop_front();
  }
}

void P0RiskGridRuntime::rangeCallback(
    const gnss_comm::msg::GnssMeasMsg::ConstSharedPtr msg) {
  if (!msg) {
    return;
  }
  recordInputCallback();
  const std::uint64_t source_identity =
      iap::gnss_measurement_source_identity(*msg);

  bool origin_valid = false;
  Eigen::Vector3d origin_ecef = Eigen::Vector3d::Zero();
  std::vector<double> iono_params;
  std::unordered_map<uint32_t, gnss_comm::EphemPtr> ephem_cache;
  std::unordered_map<uint32_t, gnss_comm::GloEphemPtr> glo_ephem_cache;
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    origin_valid = origin_set_;
    origin_ecef = origin_ecef_;
    iono_params = iono_params_;
    ephem_cache = ephem_cache_;
    glo_ephem_cache = glo_ephem_cache_;
  }
  const auto obs_list = origin_valid
      ? gnss_comm::msg2meas(msg)
      : decltype(gnss_comm::msg2meas(msg)){};

  iap::GnssEpoch epoch;
  if (!obs_list.empty() && obs_list.front()) {
    const auto utc_t = gnss_comm::gpst2utc(obs_list.front()->time);
    epoch.stamp = static_cast<double>(utc_t.time) + utc_t.sec;
    epoch.gps_sec = static_cast<double>(obs_list.front()->time.time) +
                    obs_list.front()->time.sec;
    epoch.iono_params = iono_params;

    for (const auto& obs : obs_list) {
      if (!obs) {
        continue;
      }
      int l1_idx = -1;
      const double freq = gnss_comm::L1_freq(obs, &l1_idx);
      if (l1_idx < 0 || freq < 0.0 ||
          static_cast<int>(obs->psr.size()) <= l1_idx) {
        continue;
      }
      const double pr = obs->psr[l1_idx];
      if (pr <= 0.0 || !std::isfinite(pr)) {
        continue;
      }

      const uint32_t sat_id = obs->sat;
      const uint32_t sys = gnss_comm::satsys(sat_id, nullptr);
      Eigen::Vector3d sat_ecef_pos = Eigen::Vector3d::Zero();
      Eigen::Vector3d sat_ecef_vel = Eigen::Vector3d::Zero();
      double svdt = 0.0;
      double svddt = 0.0;
      double tgd = 0.0;
      const auto t_tx = gnss_comm::time_add(obs->time, -pr / kLightSpeed);

      if (sys == SYS_GLO) {
        const auto it = glo_ephem_cache.find(sat_id);
        if (it == glo_ephem_cache.end()) {
          continue;
        }
        sat_ecef_pos = gnss_comm::geph2pos(t_tx, it->second, &svdt);
        sat_ecef_vel = gnss_comm::geph2vel(t_tx, it->second, &svddt);
      } else {
        const auto it = ephem_cache.find(sat_id);
        if (it == ephem_cache.end()) {
          continue;
        }
        sat_ecef_pos = gnss_comm::eph2pos(t_tx, it->second, &svdt);
        sat_ecef_vel = gnss_comm::eph2vel(t_tx, it->second, &svddt);
        tgd = it->second->tgd[0];
      }
      if (!sat_ecef_pos.allFinite() || !sat_ecef_vel.allFinite()) {
        continue;
      }

      double azel[2] = {0.0, M_PI / 2.0};
      gnss_comm::sat_azel(origin_ecef, sat_ecef_pos, azel);
      if (azel[1] < 10.0 * M_PI / 180.0) {
        continue;
      }

      double dop_meas = 0.0;
      double dop_sigma = config_.gnss_dop_noise_base_mps;
      if (static_cast<int>(obs->dopp.size()) > l1_idx && freq > 0.0) {
        const double doppler_hz = obs->dopp[l1_idx];
        if (std::isfinite(doppler_hz)) {
          dop_meas = -doppler_hz * (kLightSpeed / freq);
        }
      }
      if (static_cast<int>(obs->dopp_std.size()) > l1_idx && freq > 0.0) {
        const double converted_sigma =
            obs->dopp_std[l1_idx] * (kLightSpeed / freq);
        if (converted_sigma > 0.01) {
          dop_sigma = converted_sigma;
        }
      }

      iap::SatObs sat;
      sat.sat_id = static_cast<int>(sat_id);
      sat.constellation = (sys == SYS_GLO) ? 'R'
                          : (sys == SYS_GAL) ? 'E'
                          : (sys == SYS_BDS) ? 'C'
                                             : 'G';
      sat.pr_meas = pr + svdt * kLightSpeed;
      sat.dop_meas = dop_meas + svddt * kLightSpeed;
      sat.pr_sigma =
          static_cast<int>(obs->psr_std.size()) > l1_idx &&
                  obs->psr_std[l1_idx] > 0.05
              ? obs->psr_std[l1_idx]
              : config_.gnss_pr_noise_base_m;
      sat.dop_sigma = dop_sigma;
      sat.sat_pos = sat_ecef_pos;
      sat.sat_vel = sat_ecef_vel;
      sat.elevation = azel[1];
      sat.azimuth = azel[0];
      sat.tgd = tgd;
      sat.svddt = svddt;
      epoch.sats.push_back(sat);
    }
  }

  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    gnss_satellite_admission_.apply(&epoch);
    advanceNonzeroGeneration(&latest_gnss_epoch_generation_);
    gnss_epoch_seen_ = true;
    latest_gnss_epoch_satellite_count_ =
        static_cast<uint64_t>(epoch.sats.size());
    if (!epoch.sats.empty()) {
      epoch.source_identity = source_identity;
      latest_gnss_epoch_stamp_ = epoch.stamp;
      gnss_epoch_history_.emplace_back(latest_gnss_epoch_generation_,
                                       std::move(epoch));
      pruneGnssEpochHistoryLocked(latest_gnss_epoch_stamp_);
      latest_epoch_ = gnss_epoch_history_.back().second;
    } else {
      latest_gnss_epoch_stamp_ =
          std::numeric_limits<double>::quiet_NaN();
      latest_epoch_.reset();
    }
  }
}

void P0RiskGridRuntime::pruneGnssEpochHistoryLocked(
    const double newest_stamp_s) {
  constexpr std::size_t kGnssEpochHistoryHardCapacity = 4096;
  const double retention_window_s = std::max(
      {0.25, config_.gnss_epoch_max_age_s,
       config_.predictor_gnss_measured_epoch_integrity_max_delta_s}) + 0.25;
  if (std::isfinite(newest_stamp_s) && std::isfinite(retention_window_s)) {
    while (!gnss_epoch_history_.empty() &&
           std::isfinite(gnss_epoch_history_.front().second.stamp) &&
           newest_stamp_s - gnss_epoch_history_.front().second.stamp >
               retention_window_s) {
      gnss_epoch_history_.pop_front();
    }
  }
  while (gnss_epoch_history_.size() > kGnssEpochHistoryHardCapacity) {
    gnss_epoch_history_.pop_front();
  }
}

void P0RiskGridRuntime::ephemCallback(
    const gnss_comm::msg::GnssEphemMsg::ConstSharedPtr msg) {
  if (msg) {
    recordInputCallback();
  }
  auto ephem = gnss_comm::msg2ephem(msg);
  if (ephem) {
    std::lock_guard<std::mutex> lock(health_state_mutex_);
    ephem_cache_[ephem->sat] = ephem;
  }
}

void P0RiskGridRuntime::gloEphemCallback(
    const gnss_comm::msg::GnssGloEphemMsg::ConstSharedPtr msg) {
  if (msg) {
    recordInputCallback();
  }
  auto ephem = gnss_comm::msg2glo_ephem(msg);
  if (ephem) {
    std::lock_guard<std::mutex> lock(health_state_mutex_);
    glo_ephem_cache_[ephem->sat] = ephem;
  }
}

void P0RiskGridRuntime::receiverLlaCallback(
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
  if (!msg) {
    return;
  }
  recordInputCallback();
  const double message_stamp = stampToSec(msg->header.stamp);
  std::lock_guard<std::mutex> lock(health_state_mutex_);
  origin_seen_ = true;
  latest_origin_stamp_ = message_stamp;
  if (origin_set_) {
    return;
  }
  if (std::isfinite(msg->latitude) && std::isfinite(msg->longitude) &&
      std::isfinite(msg->altitude)) {
    origin_ecef_ =
        gnss_comm::geo2ecef(Eigen::Vector3d(msg->latitude, msg->longitude,
                                            msg->altitude));
    origin_set_ = true;
  }
}

void P0RiskGridRuntime::ionoCallback(
    const gnss_comm::msg::GnssIonosphereParameter::ConstSharedPtr msg) {
  if (msg) {
    recordInputCallback();
  }
  if (msg && msg->type == 0 && msg->parameters.size() >= 8) {
    std::lock_guard<std::mutex> lock(health_state_mutex_);
    iono_params_.assign(msg->parameters.begin(), msg->parameters.begin() + 8);
  }
}

void P0RiskGridRuntime::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  if (msg) {
    recordInputCallback();
  }
  auto clear_lidar_inputs =
      [this](const std::string& reason, const double source_stamp) {
    iap::LidarFimPrimitiveGenerationDiagnostics diagnostics;
    diagnostics.valid = false;
    diagnostics.fallback_reason = reason;
    std::lock_guard<std::mutex> lock(lidar_predictor_input_mutex_);
    latest_lidar_map_points_.reset();
    latest_lidar_fim_primitives_.reset();
    latest_lidar_fim_diagnostics_ = diagnostics;
    latest_lidar_map_point_count_ = 0;
    latest_lidar_fim_primitive_count_ = 0;
    latest_lidar_fim_valid_normal_count_ = 0;
    latest_lidar_fim_fallback_reason_ = reason;
    advanceNonzeroGeneration(&latest_lidar_generation_);
    latest_lidar_stamp_ = source_stamp;
  };

  if (!msg) {
    clear_lidar_inputs("null_lidar_pointcloud",
                       std::numeric_limits<double>::quiet_NaN());
    return;
  }

  const double source_stamp = stampToSec(msg->header.stamp);

  if (!config_.fit_grid_to_map_cloud) {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    map_seen_ = true;
    latest_map_stamp_ = source_stamp;
  }

  auto points = std::make_shared<std::vector<Eigen::Vector3d>>();
  auto normals = std::make_shared<std::vector<Eigen::Vector3d>>();
  const bool cloud_has_normals =
      hasPointField(*msg, "normal_x") &&
      hasPointField(*msg, "normal_y") &&
      hasPointField(*msg, "normal_z");
  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    if (cloud_has_normals) {
      sensor_msgs::PointCloud2ConstIterator<float> iter_nx(*msg, "normal_x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_ny(*msg, "normal_y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_nz(*msg, "normal_z");
      for (; iter_x != iter_x.end();
           ++iter_x, ++iter_y, ++iter_z, ++iter_nx, ++iter_ny, ++iter_nz) {
        const Eigen::Vector3d point(*iter_x, *iter_y, *iter_z);
        if (!point.allFinite()) {
          continue;
        }
        const Eigen::Vector3d normal(*iter_nx, *iter_ny, *iter_nz);
        points->push_back(point);
        normals->push_back(normal.allFinite() ? normal
                                              : Eigen::Vector3d::Zero());
      }
    } else {
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const Eigen::Vector3d point(*iter_x, *iter_y, *iter_z);
        if (point.allFinite()) {
          points->push_back(point);
        }
      }
    }
  } catch (const std::exception& e) {
    clear_lidar_inputs(std::string("invalid_lidar_pointcloud:") + e.what(),
                       source_stamp);
    return;
  }

  if (points->empty()) {
    clear_lidar_inputs("empty_lidar_pointcloud", source_stamp);
    return;
  }

  std::string geometry_reason;
  if (!initializeGridDimensionsFromMapCloud(
          *points, msg->header.frame_id, &geometry_reason)) {
    clear_lidar_inputs("invalid_scene_geometry:" + geometry_reason,
                       source_stamp);
    return;
  }

  const std::shared_ptr<std::vector<Eigen::Vector3d>> predictor_normals =
      cloud_has_normals && normals->size() == points->size() ? normals
                                                             : nullptr;
  iap::LidarFimPrimitiveGenerationDiagnostics diagnostics;
  auto primitives =
      iap::make_lidar_fim_primitives(*points, predictor_normals.get(),
                                     iap::LidarFimPrimitiveGenerationParams{},
                                     &diagnostics);
  const std::size_t primitive_count =
      primitives ? primitives->size() : static_cast<std::size_t>(0);
  const std::size_t valid_normal_count =
      std::max(0, diagnostics.lidar_pca_valid_normals);
  const std::string fallback_reason =
      diagnostics.fallback_reason.empty() ? std::string()
                                          : diagnostics.fallback_reason;
  {
    std::lock_guard<std::mutex> lock(lidar_predictor_input_mutex_);
    latest_lidar_map_points_ = points;
    latest_lidar_fim_primitives_ = primitives;
    latest_lidar_fim_diagnostics_ = diagnostics;
    latest_lidar_map_point_count_ = points->size();
    latest_lidar_fim_primitive_count_ = primitive_count;
    latest_lidar_fim_valid_normal_count_ = valid_normal_count;
    latest_lidar_fim_fallback_reason_ = fallback_reason;
    advanceNonzeroGeneration(&latest_lidar_generation_);
    latest_lidar_stamp_ = source_stamp;
  }
  if (config_.fit_grid_to_map_cloud) {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    map_seen_ = true;
    latest_map_stamp_ = source_stamp;
  }
}

bool P0RiskGridRuntime::initializeGridDimensionsFromMapCloud(
    const std::vector<Eigen::Vector3d>& points,
    const std::string& frame_id,
    std::string* reason) {
  if (!config_.fit_grid_to_map_cloud) {
    if (reason) {
      *reason = "disabled";
    }
    return true;
  }
  if (points.empty() || frame_id != config_.grid.frame_id) {
    if (reason) {
      *reason = points.empty() ? "empty_map_cloud" : "map_frame_mismatch";
    }
    return false;
  }
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    if (scene_grid_dimensions_ready_) {
      if (reason) {
        *reason = "already_fitted";
      }
      return true;
    }
  }

  Eigen::Vector3d min_w = points.front();
  Eigen::Vector3d max_w = points.front();
  for (const auto& point : points) {
    if (!point.allFinite()) {
      continue;
    }
    min_w = min_w.cwiseMin(point);
    max_w = max_w.cwiseMax(point);
  }
  const Eigen::Vector3d span = max_w - min_w;
  if (!min_w.allFinite() || !max_w.allFinite() ||
      (span.array() < 0.0).any()) {
    if (reason) {
      *reason = "non_finite_map_bounds";
    }
    return false;
  }

  iap::RiskGridMapParams fitted = config_.grid;
  fitted.size_x_m = std::max(fitted.resolution_m, span.x());
  fitted.size_y_m = std::max(fitted.resolution_m, span.y());
  fitted.size_z_m = std::max(fitted.resolution_m, span.z());
  std::string configure_reason;
  if (!risk_grid_.configure(fitted, &configure_reason)) {
    if (reason) {
      *reason = "risk_grid_configure_failed:" + configure_reason;
    }
    return false;
  }

  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    scene_grid_dimensions_ready_ = true;
  }
  RCLCPP_INFO(
      node_->get_logger(),
      "[p0] fitted RiskGridMap to scene cloud frame=%s min=(%.3f,%.3f,%.3f) "
      "max=(%.3f,%.3f,%.3f) size=(%.3f,%.3f,%.3f) resolution=%.3f",
      frame_id.c_str(), min_w.x(), min_w.y(), min_w.z(), max_w.x(),
      max_w.y(), max_w.z(), fitted.size_x_m, fitted.size_y_m,
      fitted.size_z_m, fitted.resolution_m);
  if (reason) {
    *reason = "fitted";
  }
  return true;
}

iap::CurrentIntegrityState P0RiskGridRuntime::currentFromMsg(
    const iap::msg::IntegrityReport& msg) const {
  iap::CurrentIntegrityState current;
  current.stamp = stampToSec(msg.header.stamp);
  current.estimation_frame_id = msg.estimation_frame_id;
  current.gnss_valid = msg.gnss_valid;
  current.gnss_hpl = msg.gnss_hpl;
  current.gnss_vpl = msg.gnss_vpl;
  current.gnss_epoch_stamp = msg.gnss_epoch_stamp;
  current.gnss_epoch_identity = msg.gnss_epoch_identity;
  current.lidar_valid = msg.lidar_valid;
  current.lidar_hpl = msg.lidar_hpl;
  current.lidar_vpl = msg.lidar_vpl;
  current.lidar_pl_e = msg.lidar_pl_e;
  current.lidar_pl_n = msg.lidar_pl_n;
  current.lidar_pl_u = msg.lidar_pl_u;
  current.icp_degenerate = msg.icp_degenerate;
  current.icp_rmse = msg.icp_rmse;
  current.icp_condition = msg.icp_condition;
  current.icp_gamma_lidar = msg.icp_gamma_lidar;
  current.integrity_state = msg.integrity_state;
  current.hpl = msg.hpl;
  current.vpl = msg.vpl;
  current.pl_e = msg.pl_e;
  current.pl_n = msg.pl_n;
  current.pl_u = msg.pl_u;
  current.pl = iap::current_pl_scalar(msg.hpl, msg.vpl);
  current.hal = msg.hal;
  current.val = msg.val;
  current.im = msg.im;
  current.pl_ff = msg.pl_ff;
  current.pl_ff_v = msg.pl_ff_v;
  current.k_ff_used = msg.k_ff_used;
  current.k_fa_used = msg.k_fa_used;
  current.n_sv_used = msg.n_sv_used;
  current.n_constellations = msg.n_constellations;
  current.pdop = msg.pdop;
  current.sigma_h = msg.sigma_h;
  current.n_hypotheses = msg.n_hypotheses;
  current.n_detected = msg.n_detected;
  current.excluded_prns.assign(msg.excluded_prns.begin(),
                               msg.excluded_prns.end());
  current.excluded_trunk_ids.assign(msg.excluded_trunk_ids.begin(),
                                    msg.excluded_trunk_ids.end());
  current.n_trunks_observed = msg.n_trunks_observed;
  current.tdop = msg.tdop;
  current.valid = finite(current.hpl) && finite(current.vpl) &&
                  finite(current.hal) && finite(current.val) &&
                  finite(current.im);
  return current;
}

const iap::GnssEpoch* P0RiskGridRuntime::activeGnssEpoch(
    const double query_stamp) const {
  if (!latest_epoch_) {
    return nullptr;
  }
  const double age_s = query_stamp - latest_epoch_->stamp;
  if (!std::isfinite(age_s)) {
    return nullptr;
  }
  if (config_.gnss_epoch_max_age_s >= 0.0 &&
      age_s > config_.gnss_epoch_max_age_s) {
    return nullptr;
  }
  return &*latest_epoch_;
}

Eigen::Matrix3d P0RiskGridRuntime::currentPriorInformation(
    const iap::CurrentIntegrityState& current) const {
  Eigen::Matrix3d lambda = Eigen::Matrix3d::Zero();
  if (!current.valid) {
    return lambda;
  }
  constexpr double k_h = 5.0;
  constexpr double k_v = 5.0;
  const double sigma_h =
      std::isfinite(current.hpl) && current.hpl > 0.0 ? current.hpl / k_h
                                                       : 0.0;
  const double sigma_v =
      std::isfinite(current.vpl) && current.vpl > 0.0 ? current.vpl / k_v
                                                       : 0.0;
  if (sigma_h > 0.0 && sigma_v > 0.0) {
    lambda(0, 0) = 1.0 / (sigma_h * sigma_h);
    lambda(1, 1) = 1.0 / (sigma_h * sigma_h);
    lambda(2, 2) = 1.0 / (sigma_v * sigma_v);
  }
  return lambda;
}

bool P0RiskGridRuntime::buildSnapshot(
    const double now_s,
    iap::IntegritySnapshot* snapshot,
    PredictorSourceCapture* source_capture,
    InputReadiness* readiness_capture) const {
  if (snapshot == nullptr) {
    return false;
  }
  // Copy a coherent input state before the expensive predictor work starts.
  // Input callbacks may now continue concurrently with refresh.
  double odom_stamp = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d odom_position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond odom_orientation = Eigen::Quaterniond::Identity();
  bool odom_valid = false;
  iap::CurrentIntegrityState current;
  bool current_valid = false;
  bool current_seen = false;
  uint64_t prior_source_generation = 0;
  Eigen::Matrix3d lambda_prior = Eigen::Matrix3d::Zero();
  std::optional<iap::GnssEpoch> epoch;
  uint64_t captured_gnss_epoch_generation = 0;
  {
    std::scoped_lock lock(health_state_mutex_, lidar_predictor_input_mutex_);
    odom_stamp = latest_odom_stamp_;
    odom_position = latest_odom_p_;
    odom_orientation = latest_odom_q_;
    odom_valid = latest_odom_pose_valid_;
    current = latest_current_;
    current_valid = latest_current_valid_;
    current_seen = current_integrity_seen_;
    prior_source_generation = latest_current_generation_;
    if (config_.predictor_use_current_integrity_prior) {
      lambda_prior = currentPriorInformation(current);
    }
    epoch = latest_epoch_;
    captured_gnss_epoch_generation = latest_gnss_epoch_generation_;
    if (current.gnss_epoch_identity != 0) {
      for (auto it = gnss_epoch_history_.rbegin();
           it != gnss_epoch_history_.rend(); ++it) {
        const double epoch_delta_s =
            std::abs(current.gnss_epoch_stamp - it->second.stamp);
        if (std::isfinite(epoch_delta_s) &&
            epoch_delta_s <=
                config_.predictor_gnss_measured_epoch_integrity_max_delta_s &&
            iap::gnss_epoch_identity(it->second, current.excluded_prns) ==
                current.gnss_epoch_identity) {
          captured_gnss_epoch_generation = it->first;
          epoch = it->second;
          break;
        }
      }
    }
    if (readiness_capture) {
      *readiness_capture = inputReadinessLocked(now_s);
    }
  }
  if (source_capture) {
    source_capture->current_generation = prior_source_generation;
    source_capture->current_stamp = current.stamp;
    source_capture->gnss_epoch_generation =
        captured_gnss_epoch_generation;
    source_capture->gnss_epoch_stamp =
        epoch ? epoch->stamp : std::numeric_limits<double>::quiet_NaN();
  }
  // A globally invalid/over-AL current solution does not erase explicit
  // per-satellite Integrity exclusions.  Apply them whenever the exact epoch
  // identity is aligned; MISSION_BEST_EFFORT changes motion policy, not which
  // measurements the integrity monitor has excluded.
  const bool integrity_epoch_aligned = epoch &&
      std::isfinite(current.gnss_epoch_stamp) &&
      std::isfinite(epoch->stamp) &&
      std::abs(current.gnss_epoch_stamp - epoch->stamp) <=
          config_.predictor_gnss_measured_epoch_integrity_max_delta_s &&
      current.gnss_epoch_identity != 0 &&
      current.gnss_epoch_identity ==
          iap::gnss_epoch_identity(*epoch, current.excluded_prns);
  if (integrity_epoch_aligned && !current.excluded_prns.empty()) {
    const std::unordered_set<int> excluded(
        current.excluded_prns.begin(), current.excluded_prns.end());
    for (auto& sat : epoch->sats) {
      sat.excluded = sat.excluded || excluded.count(sat.sat_id) > 0;
    }
  }
  const bool best_effort_local_health =
      config_.task_mode ==
          iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT &&
      current_seen && p0LocalSlamHealthValid(current);
  if (!odom_valid || (!current_valid && !best_effort_local_health)) {
    return false;
  }
  double transaction_now_s = now_s;
  if (std::isfinite(odom_stamp)) {
    transaction_now_s = std::max(transaction_now_s, odom_stamp);
  }
  if (std::isfinite(current.stamp)) {
    transaction_now_s = std::max(transaction_now_s, current.stamp);
  }
  const auto age_valid = [](double stamp, double now_s, double max_age_s) {
    if (!std::isfinite(stamp) || !std::isfinite(now_s) || stamp <= 0.0) {
      return false;
    }
    const double age_s = now_s - stamp;
    if (!std::isfinite(age_s) || age_s < 0.0) {
      return false;
    }
    return max_age_s < 0.0 || age_s <= max_age_s;
  };
  if (!age_valid(odom_stamp, transaction_now_s,
                 config_.grid.stale_timeout_s)) {
    return false;
  }
  if (!age_valid(current.stamp, transaction_now_s,
                 config_.grid.stale_timeout_s)) {
    return false;
  }
  if (epoch && !std::isfinite(epoch->stamp)) {
    return false;
  }
  iap::IntegritySnapshotBuilderInput input;
  input.stamp = transaction_now_s;
  input.has_pose = odom_valid;
  input.pose_stamp = odom_stamp;
  input.p_wb = odom_position;
  input.q_wb = odom_orientation;
  input.current = current;
  if (epoch) {
    const double age_s = transaction_now_s - epoch->stamp;
    if (std::isfinite(age_s) &&
        (config_.gnss_epoch_max_age_s < 0.0 || age_s <= config_.gnss_epoch_max_age_s)) {
      input.gnss_epoch = &*epoch;
    }
  }
  if (config_.predictor_use_current_integrity_prior &&
      lambda_prior.trace() > 0.0 && lambda_prior.allFinite()) {
    input.lambda_base_pos = &lambda_prior;
  }
  *snapshot = snapshot_builder_.build_from_latest(input);
  if (snapshot->has_lambda_base) {
    snapshot->prior_source_generation = prior_source_generation;
  }
  return snapshot->valid ||
      (best_effort_local_health && snapshot->has_pose);
}

iap::RiskGridHealth P0RiskGridRuntime::addLidarPredictorInputHealth(
    iap::RiskGridHealth health) const {
  std::lock_guard<std::mutex> lock(lidar_predictor_input_mutex_);
  return addLidarPredictorInputHealthLocked(std::move(health));
}

iap::RiskGridHealth P0RiskGridRuntime::addLidarPredictorInputHealthLocked(
    iap::RiskGridHealth health) const {
  health.predictor_lidar_map_point_count =
      static_cast<uint64_t>(latest_lidar_map_point_count_);
  health.predictor_lidar_fim_primitive_count =
      static_cast<uint64_t>(latest_lidar_fim_primitive_count_);
  health.predictor_lidar_fim_valid_normal_count =
      static_cast<uint64_t>(latest_lidar_fim_valid_normal_count_);
  health.predictor_lidar_fim_fallback_reason =
      latest_lidar_fim_fallback_reason_;
  return health;
}

double P0RiskGridRuntime::currentMessageStamp() const {
  std::lock_guard<std::mutex> health_lock(health_state_mutex_);
  if (std::isfinite(latest_odom_stamp_) && latest_odom_stamp_ > 0.0) {
    return latest_odom_stamp_;
  }
  if (std::isfinite(latest_current_.stamp) && latest_current_.stamp > 0.0) {
    return latest_current_.stamp;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

double P0RiskGridRuntime::diagnosticRosNowSeconds() const {
  if (!node_) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return node_->now().seconds();
}

double P0RiskGridRuntime::liveNowSeconds() const {
  const double message_stamp_s = currentMessageStamp();
  const bool use_sim_time =
      node_ && node_->has_parameter("use_sim_time") &&
      node_->get_parameter("use_sim_time").as_bool();
  if (use_sim_time) {
    const double now_s = node_->now().seconds();
    if (std::isfinite(now_s) && now_s > 0.0) {
      // ROS delivery can expose a sensor/odom message a few milliseconds
      // before the matching /clock sample. Keep one message time domain and
      // never classify that valid newest input as being from the future.
      return std::isfinite(message_stamp_s)
          ? std::max(now_s, message_stamp_s)
          : now_s;
    }
  }
  return message_stamp_s;
}

double P0RiskGridRuntime::currentRefreshStamp() const {
  return currentMessageStamp();
}

}  // namespace ego_planner
