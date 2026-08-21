// Repository-local, non-ROS profiling seam for the production-shaped P0
// Predictor provider workload. This executable is diagnostic only: it does
// not alter the runtime provider, formal worker count, caching, or results.

#include <iap/planner/predictor_risk_conversion.hpp>
#include <iap/predictor/predictor_module.hpp>

#include <nlohmann/json.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
using Json = nlohmann::json;

constexpr int kGridX = 40;
constexpr int kGridY = 40;
constexpr int kGridZ = 8;
constexpr double kResolutionM = 0.75;
constexpr std::array<double, 6> kHorizons{{0.0, 0.5, 1.0, 1.5, 2.0, 2.5}};
constexpr std::size_t kLogicalPositionCount =
    static_cast<std::size_t>(kGridX * kGridY * kGridZ);
constexpr std::size_t kLogicalQueryCount =
    kLogicalPositionCount * kHorizons.size();
constexpr std::uint64_t kFnvOffset = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;
constexpr double kPi = 3.14159265358979323846;

struct Options {
  std::filesystem::path output;
  int warmup_iterations = 2;
  int measured_iterations = 7;
};

double elapsed_ms(const Clock::time_point begin) {
  return std::chrono::duration<double, std::milli>(Clock::now() - begin)
      .count();
}

void hash_bytes(std::uint64_t* hash, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t index = 0; index < size; ++index) {
    *hash ^= bytes[index];
    *hash *= kFnvPrime;
  }
}

template <typename T>
void hash_scalar(std::uint64_t* hash, const T& value) {
  hash_bytes(hash, &value, sizeof(value));
}

void hash_bool(std::uint64_t* hash, const bool value) {
  const std::uint8_t normalized = value ? 1U : 0U;
  hash_scalar(hash, normalized);
}

void hash_double(std::uint64_t* hash, const double value) {
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "double must be 64-bit");
  std::memcpy(&bits, &value, sizeof(bits));
  hash_scalar(hash, bits);
}

void hash_string(std::uint64_t* hash, const std::string& value) {
  hash_scalar(hash, value.size());
  hash_bytes(hash, value.data(), value.size());
}

template <typename Derived>
void hash_eigen(std::uint64_t* hash, const Eigen::MatrixBase<Derived>& value) {
  for (Eigen::Index row = 0; row < value.rows(); ++row) {
    for (Eigen::Index column = 0; column < value.cols(); ++column) {
      hash_double(hash, value(row, column));
    }
  }
}

void hash_int_vector(std::uint64_t* hash, const std::vector<int>& values) {
  hash_scalar(hash, values.size());
  for (const int value : values) hash_scalar(hash, value);
}

void hash_gnss(std::uint64_t* hash, const iap::GnssAdvisoryResult& value) {
  hash_bool(hash, value.available);
  hash_bool(hash, value.valid);
  hash_bool(hash, value.fallback);
  hash_string(hash, value.fallback_reason);
  hash_scalar(hash, static_cast<int>(value.information_state));
  for (const double item :
       {value.hpl, value.vpl, value.pl_scalar, value.pl_e, value.pl_n,
        value.pl_u, value.pl_ff_h, value.pl_ff_v, value.sigma_h,
        value.sigma_v, value.pdop, value.hdop, value.vdop,
        value.effective_sigma_mean, value.effective_sigma_max}) {
    hash_double(hash, item);
  }
  for (const int item : {value.n_visible, value.n_used, value.n_hypotheses,
                         value.n_excluded}) {
    hash_scalar(hash, item);
  }
  hash_int_vector(hash, value.visible_sat_ids);
  hash_int_vector(hash, value.used_sat_ids);
  hash_int_vector(hash, value.excluded_sat_ids);
  hash_eigen(hash, value.lambda_gnss);
  hash_bool(hash, value.fim_valid);
  hash_bool(hash, value.fim_regularized);
  for (const double item :
       {value.lambda_trace, value.lambda_min_eig, value.lambda_max_eig,
        value.lambda_condition}) {
    hash_double(hash, item);
  }
  hash_string(hash, value.fim_fallback_reason);
}

void hash_lidar(std::uint64_t* hash, const iap::LidarAdvisoryResult& value) {
  hash_bool(hash, value.available);
  hash_bool(hash, value.valid);
  hash_bool(hash, value.fallback);
  hash_string(hash, value.fallback_reason);
  hash_scalar(hash, static_cast<int>(value.information_state));
  hash_eigen(hash, value.lambda_lidar);
  hash_eigen(hash, value.legacy_delta_lambda);
  hash_bool(hash, value.fim_valid);
  hash_bool(hash, value.legacy_valid);
  hash_bool(hash, value.fim_regularized);
  for (const double item :
       {value.lidar_alpha, value.tdop_proxy, value.condition, value.bias_h,
        value.bias_v, value.lambda_trace, value.lambda_min_eig,
        value.lambda_max_eig, value.lambda_condition}) {
    hash_double(hash, item);
  }
  hash_scalar(hash, value.n_primitives);
  hash_scalar(hash, value.n_valid_normals);
}

void hash_fusion(std::uint64_t* hash,
                 const iap::FusionAdvisoryResult& value) {
  hash_bool(hash, value.available);
  hash_bool(hash, value.valid);
  hash_bool(hash, value.fallback);
  hash_string(hash, value.fallback_reason);
  hash_scalar(hash, static_cast<int>(value.information_state));
  for (const double item : {value.hpl, value.vpl, value.pl_scalar,
                            value.sigma_h, value.sigma_v}) {
    hash_double(hash, item);
  }
  hash_eigen(hash, value.lambda_prior);
  hash_eigen(hash, value.lambda_gnss);
  hash_eigen(hash, value.lambda_lidar);
  hash_eigen(hash, value.lambda_pred);
  hash_eigen(hash, value.sigma_pos);
  for (const bool item :
       {value.prior_valid, value.gnss_used, value.lidar_used,
        value.epsilon_applied, value.degeneracy_regularized,
        value.conservative_max_applied}) {
    hash_bool(hash, item);
  }
  hash_scalar(hash, static_cast<int>(value.fusion_mode));
  for (const double item :
       {value.lambda_prior_trace, value.lambda_gnss_trace,
        value.lambda_lidar_trace, value.lambda_pred_trace,
        value.lambda_pred_min_eig, value.lambda_pred_max_eig,
        value.lambda_pred_condition}) {
    hash_double(hash, item);
  }
}

std::uint64_t scientific_hash(const iap::PredictorQueryResult& value) {
  std::uint64_t hash = kFnvOffset;
  hash_bool(&hash, value.available);
  hash_bool(&hash, value.valid);
  hash_bool(&hash, value.fallback);
  hash_string(&hash, value.fallback_reason);
  hash_string(&hash, value.query_source);
  hash_scalar(&hash, value.source_flags);
  hash_gnss(&hash, value.gnss);
  hash_lidar(&hash, value.lidar);
  hash_fusion(&hash, value.fused);
  return hash;
}

std::uint64_t production_result_hash(const iap::RiskPredictionResult& value) {
  std::uint64_t hash = kFnvOffset;
  hash_bool(&hash, value.available);
  hash_bool(&hash, value.valid);
  hash_bool(&hash, value.stale);
  hash_double(&hash, value.hpl_pred);
  hash_double(&hash, value.vpl_pred);
  hash_scalar(&hash, value.source_flags);
  hash_string(&hash, value.reason);
  return hash;
}

std::string hex_hash(const std::uint64_t value) {
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << value;
  return output.str();
}

iap::GnssEpoch make_epoch() {
  iap::GnssEpoch epoch;
  epoch.stamp = 100.0;
  epoch.gps_sec = 2100000.0;
  for (int index = 0; index < 31; ++index) {
    iap::SatObs satellite;
    satellite.sat_id = 100 + index;
    satellite.constellation = 'G';
    satellite.azimuth = 2.0 * kPi * static_cast<double>(index) / 31.0;
    satellite.elevation = 0.25 + 0.06 * static_cast<double>(index % 10);
    satellite.pr_sigma = 3.0 + 0.25 * static_cast<double>(index % 4);
    epoch.sats.push_back(satellite);
  }
  return epoch;
}

iap::IntegritySnapshot make_snapshot() {
  iap::IntegritySnapshot snapshot;
  snapshot.stamp = 100.0;
  snapshot.valid = true;
  snapshot.has_pose = true;
  snapshot.pose_stamp = 100.0;
  snapshot.p_wb = Eigen::Vector3d::Zero();
  snapshot.q_wb = Eigen::Quaterniond::Identity();
  snapshot.current.stamp = 100.0;
  snapshot.current.valid = true;
  snapshot.current.hpl = 4.0;
  snapshot.current.vpl = 5.0;
  snapshot.current.pl = 5.0;
  snapshot.current.hal = 30.0;
  snapshot.current.val = 20.0;
  snapshot.current.im = 15.0;
  snapshot.current.n_sv_used = 31;
  snapshot.current.pdop = 2.0;
  snapshot.current.n_hypotheses = 31;
  snapshot.current.tdop = 2.0;
  snapshot.current.n_trunks_observed = 4;
  snapshot.has_epoch = true;
  snapshot.gnss_epoch = make_epoch();
  snapshot.has_lambda_base = true;
  snapshot.lambda_base_pos = 0.25 * Eigen::Matrix3d::Identity();
  return snapshot;
}

std::shared_ptr<const std::vector<iap::LidarFimPrimitive>>
make_lidar_primitives() {
  auto primitives = std::make_shared<std::vector<iap::LidarFimPrimitive>>();
  primitives->reserve(704);
  for (int x = 0; x < 16; ++x) {
    for (int y = 0; y < 11; ++y) {
      for (int z = 0; z < 4; ++z) {
        iap::LidarFimPrimitive primitive;
        primitive.center_w = Eigen::Vector3d(-14.0 + 1.8 * x,
                                             -14.0 + 2.7 * y,
                                             -2.0 + 1.3 * z);
        const int axis = (x + y + z) % 3;
        primitive.normal_w = axis == 0 ? Eigen::Vector3d::UnitX()
                             : axis == 1 ? Eigen::Vector3d::UnitY()
                                         : Eigen::Vector3d::UnitZ();
        primitives->push_back(primitive);
      }
    }
  }
  return primitives;
}

std::shared_ptr<const std::vector<Eigen::Vector3d>> make_lidar_map_points() {
  auto points = std::make_shared<std::vector<Eigen::Vector3d>>();
  points->reserve(23309);
  for (std::size_t index = 0; index < 23309; ++index) {
    const int x = static_cast<int>(index % 47);
    const int y = static_cast<int>((index / 47) % 47);
    const int z = static_cast<int>((index / (47 * 47)) % 11);
    points->emplace_back(-14.5 + 0.63 * x, -14.5 + 0.63 * y,
                         -2.5 + 0.55 * z);
  }
  return points;
}

iap::PredictorModule make_module(
    const iap::LocalOccupancyGrid* occupancy,
    std::shared_ptr<const std::vector<iap::LidarFimPrimitive>> primitives,
    std::shared_ptr<const std::vector<Eigen::Vector3d>> map_points) {
  iap::PredictorParams params;
  params.freshness.enabled = true;
  params.freshness.max_odom_age_s = 1.0;
  params.freshness.max_integrity_age_s = 1.0;
  params.freshness.max_gnss_age_s = 2.0;
  params.freshness.max_snapshot_age_s = 1.0;
  params.source_mode = iap::PredictorSourceMode::Fusion;
  params.gnss_epoch_policy = iap::PredictorGnssEpochPolicy::Auto;
  params.fusion.conservative_max_with_gnss = false;
  params.lidar.enable_legacy_observability = true;
  params.lidar.fim_params.fim_radius_m = 12.0;
  params.lidar.fim_params.search_radius_m = 12.0;
  iap::PredictorModule module(params);
  if (occupancy != nullptr) {
    module.set_local_occupancy(occupancy);
  }
  module.set_lidar_fim_primitives(std::move(primitives));
  module.set_lidar_map_points(std::move(map_points));
  return module;
}

std::vector<std::string> scientific_field_whitelist() {
  return {
      "PredictorQueryResult.available",
      "PredictorQueryResult.valid",
      "PredictorQueryResult.fallback",
      "PredictorQueryResult.fallback_reason",
      "PredictorQueryResult.query_source",
      "PredictorQueryResult.source_flags",
      "GnssAdvisoryResult.available",
      "GnssAdvisoryResult.valid",
      "GnssAdvisoryResult.fallback",
      "GnssAdvisoryResult.fallback_reason",
      "GnssAdvisoryResult.information_state",
      "GnssAdvisoryResult.hpl",
      "GnssAdvisoryResult.vpl",
      "GnssAdvisoryResult.pl_scalar",
      "GnssAdvisoryResult.pl_e",
      "GnssAdvisoryResult.pl_n",
      "GnssAdvisoryResult.pl_u",
      "GnssAdvisoryResult.pl_ff_h",
      "GnssAdvisoryResult.pl_ff_v",
      "GnssAdvisoryResult.sigma_h",
      "GnssAdvisoryResult.sigma_v",
      "GnssAdvisoryResult.pdop",
      "GnssAdvisoryResult.hdop",
      "GnssAdvisoryResult.vdop",
      "GnssAdvisoryResult.effective_sigma_mean",
      "GnssAdvisoryResult.effective_sigma_max",
      "GnssAdvisoryResult.n_visible",
      "GnssAdvisoryResult.n_used",
      "GnssAdvisoryResult.n_hypotheses",
      "GnssAdvisoryResult.n_excluded",
      "GnssAdvisoryResult.visible_sat_ids",
      "GnssAdvisoryResult.used_sat_ids",
      "GnssAdvisoryResult.excluded_sat_ids",
      "GnssAdvisoryResult.lambda_gnss",
      "GnssAdvisoryResult.fim_valid",
      "GnssAdvisoryResult.fim_regularized",
      "GnssAdvisoryResult.lambda_trace",
      "GnssAdvisoryResult.lambda_min_eig",
      "GnssAdvisoryResult.lambda_max_eig",
      "GnssAdvisoryResult.lambda_condition",
      "GnssAdvisoryResult.fim_fallback_reason",
      "LidarAdvisoryResult.available",
      "LidarAdvisoryResult.valid",
      "LidarAdvisoryResult.fallback",
      "LidarAdvisoryResult.fallback_reason",
      "LidarAdvisoryResult.information_state",
      "LidarAdvisoryResult.lambda_lidar",
      "LidarAdvisoryResult.legacy_delta_lambda",
      "LidarAdvisoryResult.fim_valid",
      "LidarAdvisoryResult.legacy_valid",
      "LidarAdvisoryResult.fim_regularized",
      "LidarAdvisoryResult.lidar_alpha",
      "LidarAdvisoryResult.tdop_proxy",
      "LidarAdvisoryResult.condition",
      "LidarAdvisoryResult.n_primitives",
      "LidarAdvisoryResult.n_valid_normals",
      "LidarAdvisoryResult.bias_h",
      "LidarAdvisoryResult.bias_v",
      "LidarAdvisoryResult.lambda_trace",
      "LidarAdvisoryResult.lambda_min_eig",
      "LidarAdvisoryResult.lambda_max_eig",
      "LidarAdvisoryResult.lambda_condition",
      "FusionAdvisoryResult.available",
      "FusionAdvisoryResult.valid",
      "FusionAdvisoryResult.fallback",
      "FusionAdvisoryResult.fallback_reason",
      "FusionAdvisoryResult.information_state",
      "FusionAdvisoryResult.hpl",
      "FusionAdvisoryResult.vpl",
      "FusionAdvisoryResult.pl_scalar",
      "FusionAdvisoryResult.sigma_h",
      "FusionAdvisoryResult.sigma_v",
      "FusionAdvisoryResult.lambda_prior",
      "FusionAdvisoryResult.lambda_gnss",
      "FusionAdvisoryResult.lambda_lidar",
      "FusionAdvisoryResult.lambda_pred",
      "FusionAdvisoryResult.sigma_pos",
      "FusionAdvisoryResult.prior_valid",
      "FusionAdvisoryResult.gnss_used",
      "FusionAdvisoryResult.lidar_used",
      "FusionAdvisoryResult.epsilon_applied",
      "FusionAdvisoryResult.degeneracy_regularized",
      "FusionAdvisoryResult.conservative_max_applied",
      "FusionAdvisoryResult.fusion_mode",
      "FusionAdvisoryResult.lambda_prior_trace",
      "FusionAdvisoryResult.lambda_gnss_trace",
      "FusionAdvisoryResult.lambda_lidar_trace",
      "FusionAdvisoryResult.lambda_pred_trace",
      "FusionAdvisoryResult.lambda_pred_min_eig",
      "FusionAdvisoryResult.lambda_pred_max_eig",
      "FusionAdvisoryResult.lambda_pred_condition",
  };
}

struct LogicalQuery {
  Eigen::Vector3d position;
  double query_time_s = 0.0;
  double horizon_s = 0.0;
};

std::vector<LogicalQuery> make_queries() {
  std::vector<LogicalQuery> queries;
  queries.reserve(kLogicalQueryCount);
  const Eigen::Vector3d origin(-0.5 * kGridX * kResolutionM,
                               -0.5 * kGridY * kResolutionM,
                               -0.5 * kGridZ * kResolutionM);
  for (const double horizon_s : kHorizons) {
    for (int x = 0; x < kGridX; ++x) {
      for (int y = 0; y < kGridY; ++y) {
        for (int z = 0; z < kGridZ; ++z) {
          const Eigen::Vector3d id(x, y, z);
          queries.push_back({(id + Eigen::Vector3d::Constant(0.5)) *
                                  kResolutionM + origin,
                             100.0 + horizon_s, horizon_s});
        }
      }
    }
  }
  return queries;
}

struct PositionKey {
  double x;
  double y;
  double z;
  bool operator==(const PositionKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct PositionHash {
  std::size_t operator()(const PositionKey& key) const {
    std::size_t seed = std::hash<double>{}(key.x);
    seed ^= std::hash<double>{}(key.y) + 0x9e3779b9u + (seed << 6) +
            (seed >> 2);
    seed ^= std::hash<double>{}(key.z) + 0x9e3779b9u + (seed << 6) +
            (seed >> 2);
    return seed;
  }
};

struct WorkerMetrics {
  double module_setup_ms = 0.0;
  double input_construction_ms = 0.0;
  double predictor_batch_ms = 0.0;
  double result_materialization_ms = 0.0;
  std::size_t dispatched_query_count = 0;
  std::size_t production_conversion_count = 0;
  iap::PredictorBatchDiagnostics diagnostics;
};

void add_diagnostics(iap::PredictorBatchDiagnostics* target,
                     const iap::PredictorBatchDiagnostics& source) {
  target->collect_component_timing =
      target->collect_component_timing || source.collect_component_timing;
  target->query_count += source.query_count;
  target->unique_positions += source.unique_positions;
  target->lidar_evaluations += source.lidar_evaluations;
  target->lidar_cache_hits += source.lidar_cache_hits;
  target->gnss_advisory_invocations += source.gnss_advisory_invocations;
  target->lidar_advisory_invocations += source.lidar_advisory_invocations;
  target->fusion_advisory_invocations += source.fusion_advisory_invocations;
  target->gnss_advisory_duration_ns += source.gnss_advisory_duration_ns;
  target->lidar_advisory_duration_ns += source.lidar_advisory_duration_ns;
  target->fusion_advisory_duration_ns += source.fusion_advisory_duration_ns;
}

struct IterationMetrics {
  bool finite = true;
  bool collect_component_timing = false;
  double grouping_index_ms = 0.0;
  double worker_wall_ms = 0.0;
  double total_provider_ms = 0.0;
  double cumulative_module_setup_ms = 0.0;
  double cumulative_input_construction_ms = 0.0;
  double cumulative_predictor_batch_ms = 0.0;
  double cumulative_result_materialization_ms = 0.0;
  double scientific_validation_replay_ms = 0.0;
  iap::PredictorBatchDiagnostics diagnostics;
  std::size_t dispatched_query_count = 0;
  std::size_t production_conversion_count = 0;
  std::size_t horizon_scientific_mismatch_count = 0;
  std::size_t valid_count = 0;
  std::size_t available_count = 0;
  std::size_t fallback_count = 0;
  std::size_t gnss_valid_count = 0;
  std::size_t lidar_valid_count = 0;
  std::size_t fusion_valid_count = 0;
  std::map<std::uint32_t, std::size_t> source_flags_histogram;
  std::string checksum;
  std::string production_result_checksum;
};

std::string count_signature(const IterationMetrics& item) {
  std::ostringstream output;
  output << item.valid_count << ':' << item.available_count << ':'
         << item.fallback_count << ':' << item.gnss_valid_count << ':'
         << item.lidar_valid_count << ':' << item.fusion_valid_count;
  for (const auto& [flags, count] : item.source_flags_histogram) {
    output << ':' << flags << '=' << count;
  }
  return output.str();
}

IterationMetrics run_iteration(const iap::PredictorModule& base_module,
                               const iap::IntegritySnapshot& snapshot,
                               const std::vector<LogicalQuery>& queries,
                               const int requested_workers,
                               const bool collect_component_timing) {
  IterationMetrics metrics;
  metrics.collect_component_timing = collect_component_timing;
  const auto provider_begin = Clock::now();

  const auto grouping_begin = Clock::now();
  std::unordered_map<PositionKey, std::size_t, PositionHash> group_by_position;
  std::vector<std::vector<std::size_t>> groups;
  group_by_position.reserve(queries.size());
  for (std::size_t index = 0; index < queries.size(); ++index) {
    const auto& position = queries[index].position;
    const PositionKey key{position.x(), position.y(), position.z()};
    const auto inserted = group_by_position.emplace(key, groups.size());
    if (inserted.second) groups.emplace_back();
    groups[inserted.first->second].push_back(index);
  }
  metrics.grouping_index_ms = elapsed_ms(grouping_begin);

  std::vector<iap::RiskPredictionResult> production_results(queries.size());
  const int worker_count = std::min<int>(requested_workers, groups.size());
  const auto worker_begin = Clock::now();
  std::vector<std::future<WorkerMetrics>> workers;
  workers.reserve(static_cast<std::size_t>(worker_count));
  for (int worker_id = 0; worker_id < worker_count; ++worker_id) {
    workers.push_back(std::async(
        std::launch::async,
        [&, worker_id, worker_count]() {
          WorkerMetrics worker_metrics;
          const auto setup_begin = Clock::now();
          iap::PredictorModule worker_module = base_module;
          worker_metrics.module_setup_ms = elapsed_ms(setup_begin);
          for (std::size_t group_index = static_cast<std::size_t>(worker_id);
               group_index < groups.size();
               group_index += static_cast<std::size_t>(worker_count)) {
            const auto input_begin = Clock::now();
            std::vector<iap::PredictorQueryInput> inputs;
            inputs.reserve(groups[group_index].size());
            for (const std::size_t index : groups[group_index]) {
              const auto& query = queries[index];
              inputs.emplace_back(query.position, snapshot, query.query_time_s,
                                  query.horizon_s, "map", snapshot.stamp);
            }
            worker_metrics.input_construction_ms += elapsed_ms(input_begin);

            iap::PredictorBatchDiagnostics diagnostics;
            diagnostics.collect_component_timing = collect_component_timing;
            const auto batch_begin = Clock::now();
            auto predictions = worker_module.queryBatch(inputs, &diagnostics);
            worker_metrics.predictor_batch_ms += elapsed_ms(batch_begin);
            worker_metrics.dispatched_query_count += inputs.size();
            add_diagnostics(&worker_metrics.diagnostics, diagnostics);

            const auto materialization_begin = Clock::now();
            for (std::size_t local = 0; local < predictions.size(); ++local) {
              production_results[groups[group_index][local]] =
                  iap::makeRiskPredictionResult(predictions[local]);
              ++worker_metrics.production_conversion_count;
            }
            worker_metrics.result_materialization_ms +=
                elapsed_ms(materialization_begin);
          }
          return worker_metrics;
        }));
  }
  for (auto& worker : workers) {
    const WorkerMetrics result = worker.get();
    metrics.cumulative_module_setup_ms += result.module_setup_ms;
    metrics.cumulative_input_construction_ms += result.input_construction_ms;
    metrics.cumulative_predictor_batch_ms += result.predictor_batch_ms;
    metrics.cumulative_result_materialization_ms +=
        result.result_materialization_ms;
    metrics.dispatched_query_count += result.dispatched_query_count;
    metrics.production_conversion_count += result.production_conversion_count;
    add_diagnostics(&metrics.diagnostics, result.diagnostics);
  }
  metrics.worker_wall_ms = elapsed_ms(worker_begin);
  metrics.total_provider_ms = elapsed_ms(provider_begin);

  // Full Predictor science is replayed only after the production-shaped outer
  // wall timer stops. The replay is real execution over identical inputs and
  // timing mode, but it cannot inflate the provider p50/p95 budget evidence.
  const auto validation_begin = Clock::now();
  std::vector<iap::PredictorQueryResult> results(queries.size());
  std::vector<std::future<void>> validation_workers;
  validation_workers.reserve(static_cast<std::size_t>(worker_count));
  for (int worker_id = 0; worker_id < worker_count; ++worker_id) {
    validation_workers.push_back(std::async(
        std::launch::async,
        [&, worker_id, worker_count]() {
          iap::PredictorModule worker_module = base_module;
          for (std::size_t group_index = static_cast<std::size_t>(worker_id);
               group_index < groups.size();
               group_index += static_cast<std::size_t>(worker_count)) {
            std::vector<iap::PredictorQueryInput> inputs;
            inputs.reserve(groups[group_index].size());
            for (const std::size_t index : groups[group_index]) {
              const auto& query = queries[index];
              inputs.emplace_back(query.position, snapshot, query.query_time_s,
                                  query.horizon_s, "map", snapshot.stamp);
            }
            iap::PredictorBatchDiagnostics replay_diagnostics;
            replay_diagnostics.collect_component_timing =
                collect_component_timing;
            auto predictions =
                worker_module.queryBatch(inputs, &replay_diagnostics);
            for (std::size_t local = 0; local < predictions.size(); ++local) {
              results[groups[group_index][local]] =
                  std::move(predictions[local]);
            }
          }
        }));
  }
  for (auto& worker : validation_workers) worker.get();
  metrics.scientific_validation_replay_ms = elapsed_ms(validation_begin);

  std::uint64_t aggregate_hash = kFnvOffset;
  std::uint64_t aggregate_production_hash = kFnvOffset;
  std::vector<std::uint64_t> per_result_hash;
  per_result_hash.reserve(results.size());
  for (const auto& result : results) {
    const std::uint64_t result_hash = scientific_hash(result);
    per_result_hash.push_back(result_hash);
    hash_scalar(&aggregate_hash, result_hash);
    metrics.valid_count += result.valid ? 1U : 0U;
    metrics.available_count += result.available ? 1U : 0U;
    metrics.fallback_count += result.fallback ? 1U : 0U;
    metrics.gnss_valid_count += result.gnss.valid ? 1U : 0U;
    metrics.lidar_valid_count += result.lidar.valid ? 1U : 0U;
    metrics.fusion_valid_count += result.fused.valid ? 1U : 0U;
    ++metrics.source_flags_histogram[result.source_flags];
  }
  metrics.checksum = hex_hash(aggregate_hash);
  for (const auto& result : production_results) {
    const std::uint64_t result_hash = production_result_hash(result);
    hash_scalar(&aggregate_production_hash, result_hash);
  }
  metrics.production_result_checksum = hex_hash(aggregate_production_hash);
  for (const auto& group : groups) {
    if (group.size() != kHorizons.size()) {
      ++metrics.horizon_scientific_mismatch_count;
      continue;
    }
    const std::uint64_t expected = per_result_hash[group.front()];
    for (const std::size_t index : group) {
      if (per_result_hash[index] != expected) {
        ++metrics.horizon_scientific_mismatch_count;
        break;
      }
    }
  }

  for (const double value :
       {metrics.grouping_index_ms, metrics.worker_wall_ms,
        metrics.total_provider_ms, metrics.cumulative_module_setup_ms,
        metrics.cumulative_input_construction_ms,
        metrics.cumulative_predictor_batch_ms,
        metrics.cumulative_result_materialization_ms,
        metrics.scientific_validation_replay_ms,
        static_cast<double>(metrics.diagnostics.gnss_advisory_duration_ns),
        static_cast<double>(metrics.diagnostics.lidar_advisory_duration_ns),
        static_cast<double>(metrics.diagnostics.fusion_advisory_duration_ns)}) {
    if (!std::isfinite(value) || value < 0.0) metrics.finite = false;
  }
  return metrics;
}

double percentile(std::vector<double> values, const double fraction) {
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(values.begin(), values.end());
  const double index = fraction * static_cast<double>(values.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(index));
  const auto upper = static_cast<std::size_t>(std::ceil(index));
  const double weight = index - static_cast<double>(lower);
  return values[lower] * (1.0 - weight) + values[upper] * weight;
}

Json summary_pair(const std::vector<double>& values) {
  return {{"p50_ms", percentile(values, 0.50)},
          {"p95_ms", percentile(values, 0.95)}};
}

Json summarize_phase(const std::string& phase,
                     const bool collect_component_timing,
                     const std::vector<IterationMetrics>& iterations) {
  std::vector<double> total;
  std::vector<double> grouping;
  std::vector<double> worker_wall;
  std::vector<double> module_setup;
  std::vector<double> input_construction;
  std::vector<double> predictor_batch;
  std::vector<double> result_materialization;
  std::vector<double> scientific_validation_replay;
  std::vector<double> gnss;
  std::vector<double> lidar;
  std::vector<double> fusion;
  Json raw = Json::array();
  for (std::size_t index = 0; index < iterations.size(); ++index) {
    const auto& item = iterations[index];
    total.push_back(item.total_provider_ms);
    grouping.push_back(item.grouping_index_ms);
    worker_wall.push_back(item.worker_wall_ms);
    module_setup.push_back(item.cumulative_module_setup_ms);
    input_construction.push_back(item.cumulative_input_construction_ms);
    predictor_batch.push_back(item.cumulative_predictor_batch_ms);
    result_materialization.push_back(item.cumulative_result_materialization_ms);
    scientific_validation_replay.push_back(
        item.scientific_validation_replay_ms);
    gnss.push_back(1.0e-6 * item.diagnostics.gnss_advisory_duration_ns);
    lidar.push_back(1.0e-6 * item.diagnostics.lidar_advisory_duration_ns);
    fusion.push_back(1.0e-6 * item.diagnostics.fusion_advisory_duration_ns);
    Json histogram = Json::object();
    for (const auto& [flags, count] : item.source_flags_histogram) {
      histogram[std::to_string(flags)] = count;
    }
    raw.push_back({
        {"iteration", index},
        {"finite", item.finite},
        {"collect_component_timing", item.collect_component_timing},
        {"scientific_checksum_fnv1a64", item.checksum},
        {"production_result_checksum_fnv1a64",
         item.production_result_checksum},
        {"horizon_scientific_mismatch_count",
         item.horizon_scientific_mismatch_count},
        {"logical_query_count", kLogicalQueryCount},
        {"dispatched_predictor_query_count", item.dispatched_query_count},
        {"production_result_conversion_count",
         item.production_conversion_count},
        {"valid_count", item.valid_count},
        {"available_count", item.available_count},
        {"fallback_count", item.fallback_count},
        {"gnss_valid_count", item.gnss_valid_count},
        {"lidar_valid_count", item.lidar_valid_count},
        {"fusion_valid_count", item.fusion_valid_count},
        {"source_flags_histogram", histogram},
        {"counts",
         {{"unique_positions", item.diagnostics.unique_positions},
          {"gnss_advisory_invocations",
           item.diagnostics.gnss_advisory_invocations},
          {"lidar_advisory_invocations",
           item.diagnostics.lidar_advisory_invocations},
          {"lidar_evaluations", item.diagnostics.lidar_evaluations},
          {"lidar_cache_hits", item.diagnostics.lidar_cache_hits},
          {"fusion_advisory_invocations",
           item.diagnostics.fusion_advisory_invocations}}},
        {"timings_ms",
         {{"query_grouping_index", item.grouping_index_ms},
          {"worker_wall", item.worker_wall_ms},
          {"total_predictor_provider", item.total_provider_ms},
          {"cumulative_worker_module_setup",
           item.cumulative_module_setup_ms},
          {"cumulative_input_construction",
           item.cumulative_input_construction_ms},
          {"cumulative_predictor_batch",
           item.cumulative_predictor_batch_ms},
          {"cumulative_result_materialization",
           item.cumulative_result_materialization_ms},
          {"scientific_validation_replay_outside_provider_timer",
           item.scientific_validation_replay_ms},
          {"cumulative_gnss_advisory", gnss.back()},
          {"cumulative_lidar_advisory", lidar.back()},
          {"cumulative_fusion_advisory", fusion.back()}}}});
  }
  return {
      {"phase", phase},
      {"collect_component_timing", collect_component_timing},
      {"iterations", raw},
      {"summary_ms",
       {{"query_grouping_index", summary_pair(grouping)},
        {"worker_wall", summary_pair(worker_wall)},
        {"total_predictor_provider", summary_pair(total)},
        {"cumulative_worker_module_setup", summary_pair(module_setup)},
        {"cumulative_input_construction", summary_pair(input_construction)},
        {"cumulative_predictor_batch", summary_pair(predictor_batch)},
        {"cumulative_result_materialization",
         summary_pair(result_materialization)},
        {"scientific_validation_replay_outside_provider_timer",
         summary_pair(scientific_validation_replay)},
        {"cumulative_gnss_advisory", summary_pair(gnss)},
        {"cumulative_lidar_advisory", summary_pair(lidar)},
        {"cumulative_fusion_advisory", summary_pair(fusion)}}}};
}

Options parse_options(const int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--output" && index + 1 < argc) {
      options.output = argv[++index];
    } else if (argument == "--warmup" && index + 1 < argc) {
      options.warmup_iterations = std::stoi(argv[++index]);
    } else if (argument == "--iterations" && index + 1 < argc) {
      options.measured_iterations = std::stoi(argv[++index]);
    } else {
      throw std::invalid_argument("unknown or incomplete argument: " +
                                  argument);
    }
  }
  if (options.output.empty()) {
    throw std::invalid_argument("--output is required");
  }
  if (options.warmup_iterations < 1 || options.measured_iterations < 5) {
    throw std::invalid_argument("require warmup >= 1 and iterations >= 5");
  }
  return options;
}

std::filesystem::path repository_local_output(
    const std::filesystem::path& requested) {
  const auto root = std::filesystem::weakly_canonical(IAP_SOURCE_ROOT);
  const auto absolute = std::filesystem::absolute(requested).lexically_normal();
  const std::string root_prefix = root.string() + "/";
  if (absolute.string().rfind(root_prefix, 0) != 0) {
    throw std::invalid_argument("output must be inside repository root");
  }
  return absolute;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    const auto output_path = repository_local_output(options.output);
    const auto snapshot = make_snapshot();
    const auto lidar_primitives = make_lidar_primitives();
    const auto lidar_map_points = make_lidar_map_points();
    iap::LocalOccupancyGrid::Params occupancy_params;
    occupancy_params.voxel_size = kResolutionM;
    iap::LocalOccupancyGrid occupancy(occupancy_params);
    std::vector<Eigen::Vector3d> occupancy_points;
    occupancy_points.reserve(lidar_primitives->size());
    for (const auto& primitive : *lidar_primitives) {
      occupancy_points.push_back(primitive.center_w);
    }
    occupancy.insert_points(occupancy_points);
    const auto queries = make_queries();
    if (queries.size() != kLogicalQueryCount) {
      throw std::runtime_error("logical query shape mismatch");
    }

    struct ModeDefinition {
      const char* name;
      const char* production_label;
      const iap::LocalOccupancyGrid* occupancy;
    };
    const std::array<ModeDefinition, 2> mode_definitions{{
        {"frozen_runtime", "CURRENT_PRODUCTION", nullptr},
        {"map_los_candidate", "NOT_CURRENT_PRODUCTION", &occupancy},
    }};

    Json modes = Json::array();
    bool all_finite = true;
    bool all_checksums_stable = true;
    bool all_production_checksums_stable = true;
    bool all_counts_stable = true;
    bool all_query_shapes_exact = true;
    bool all_phase_contracts_exact = true;
    bool frozen_horizon_invariant = false;

    for (const auto& mode_definition : mode_definitions) {
      const auto module = make_module(mode_definition.occupancy,
                                      lidar_primitives, lidar_map_points);
      Json workers_json = Json::array();
      std::string reference_checksum;
      std::string reference_production_checksum;
      std::string reference_count_signature;
      double worker_one_counter_p50 =
          std::numeric_limits<double>::quiet_NaN();
      bool mode_finite = true;
      bool mode_checksums_stable = true;
      bool mode_production_checksums_stable = true;
      bool mode_counts_stable = true;
      bool mode_query_shape_exact = true;
      bool mode_phase_contracts_exact = true;
      bool mode_horizon_invariant = true;
      bool worker_one_component_timing_perturbs = false;

      for (const int worker_count : {1, 2, 4}) {
        for (int warmup = 0; warmup < options.warmup_iterations; ++warmup) {
          const auto ignored = run_iteration(module, snapshot, queries,
                                             worker_count, false);
          mode_finite = mode_finite && ignored.finite;
        }
        std::vector<IterationMetrics> counter_only;
        for (int iteration = 0; iteration < options.measured_iterations;
             ++iteration) {
          counter_only.push_back(run_iteration(module, snapshot, queries,
                                               worker_count, false));
        }

        for (int warmup = 0; warmup < options.warmup_iterations; ++warmup) {
          const auto ignored = run_iteration(module, snapshot, queries,
                                             worker_count, true);
          mode_finite = mode_finite && ignored.finite;
        }
        std::vector<IterationMetrics> component_timed;
        for (int iteration = 0; iteration < options.measured_iterations;
             ++iteration) {
          component_timed.push_back(run_iteration(module, snapshot, queries,
                                                  worker_count, true));
        }

        const auto validate_iteration = [&](const IterationMetrics& item,
                                            const bool expected_timing) {
          if (reference_checksum.empty()) reference_checksum = item.checksum;
          if (reference_production_checksum.empty()) {
            reference_production_checksum = item.production_result_checksum;
          }
          if (reference_count_signature.empty()) {
            reference_count_signature = count_signature(item);
          }
          mode_finite = mode_finite && item.finite;
          mode_checksums_stable = mode_checksums_stable &&
                                  item.checksum == reference_checksum;
          mode_production_checksums_stable =
              mode_production_checksums_stable &&
              item.production_result_checksum ==
                  reference_production_checksum;
          mode_counts_stable = mode_counts_stable &&
                               count_signature(item) ==
                                   reference_count_signature;
          mode_query_shape_exact = mode_query_shape_exact &&
              item.dispatched_query_count == kLogicalQueryCount &&
              item.production_conversion_count == kLogicalQueryCount &&
              item.diagnostics.query_count == kLogicalQueryCount &&
              item.diagnostics.gnss_advisory_invocations ==
                  kLogicalQueryCount &&
              item.diagnostics.fusion_advisory_invocations ==
                  kLogicalQueryCount &&
              item.diagnostics.lidar_advisory_invocations ==
                  kLogicalPositionCount &&
              item.diagnostics.lidar_evaluations ==
                  kLogicalPositionCount &&
              item.diagnostics.lidar_cache_hits ==
                  kLogicalQueryCount - kLogicalPositionCount;
          const bool component_durations_present =
              item.diagnostics.gnss_advisory_duration_ns > 0 &&
              item.diagnostics.lidar_advisory_duration_ns > 0 &&
              item.diagnostics.fusion_advisory_duration_ns > 0;
          const bool component_durations_absent =
              item.diagnostics.gnss_advisory_duration_ns == 0 &&
              item.diagnostics.lidar_advisory_duration_ns == 0 &&
              item.diagnostics.fusion_advisory_duration_ns == 0;
          mode_phase_contracts_exact = mode_phase_contracts_exact &&
              item.collect_component_timing == expected_timing &&
              item.diagnostics.collect_component_timing == expected_timing &&
              (expected_timing ? component_durations_present
                               : component_durations_absent);
          mode_horizon_invariant = mode_horizon_invariant &&
              item.horizon_scientific_mismatch_count == 0;
        };
        for (const auto& item : counter_only) {
          validate_iteration(item, false);
        }
        for (const auto& item : component_timed) {
          validate_iteration(item, true);
        }

        Json counter_summary = summarize_phase(
            "counter_only_budget", false, counter_only);
        Json component_summary = summarize_phase(
            "component_timed_cost_ranking", true, component_timed);
        const double counter_p50 = counter_summary["summary_ms"]
            ["total_predictor_provider"]["p50_ms"];
        const double counter_p95 = counter_summary["summary_ms"]
            ["total_predictor_provider"]["p95_ms"];
        const double component_p50 = component_summary["summary_ms"]
            ["total_predictor_provider"]["p50_ms"];
        const double component_p95 = component_summary["summary_ms"]
            ["total_predictor_provider"]["p95_ms"];
        if (worker_count == 1) {
          worker_one_counter_p50 = counter_p50;
          worker_one_component_timing_perturbs =
              std::abs(100.0 * (component_p50 - counter_p50) /
                       counter_p50) > 5.0;
        }
        counter_summary["budget_timing_authority"] = true;
        counter_summary["speedup_vs_worker_1_p50"] =
            worker_one_counter_p50 / counter_p50;
        counter_summary["p95_within_400_ms_diagnostic_budget"] =
            counter_p95 <= 400.0;
        component_summary["budget_timing_authority"] = false;
        component_summary["component_cost_percentages_of_outer_p50"] = {
            {"gnss_advisory", 100.0 * static_cast<double>(
                 component_summary["summary_ms"]["cumulative_gnss_advisory"]
                                  ["p50_ms"]) / component_p50},
            {"lidar_advisory", 100.0 * static_cast<double>(
                 component_summary["summary_ms"]["cumulative_lidar_advisory"]
                                  ["p50_ms"]) / component_p50},
            {"fusion_advisory", 100.0 * static_cast<double>(
                 component_summary["summary_ms"]["cumulative_fusion_advisory"]
                                  ["p50_ms"]) / component_p50},
        };

        workers_json.push_back({
            {"worker_count", worker_count},
            {"counter_only", std::move(counter_summary)},
            {"component_timed", std::move(component_summary)},
            {"component_timer_perturbation",
             {{"p50_delta_ms", component_p50 - counter_p50},
              {"p50_percent",
               100.0 * (component_p50 - counter_p50) / counter_p50},
              {"p95_delta_ms", component_p95 - counter_p95},
              {"p95_percent",
               100.0 * (component_p95 - counter_p95) / counter_p95}}},
        });
      }

      const std::string component_percentage_status =
          worker_one_component_timing_perturbs
              ? "PERTURBING_DIAGNOSTIC"
              : "COST_RANKING_DIAGNOSTIC";
      for (auto& worker : workers_json) {
        worker["component_percentage_status"] =
            component_percentage_status;
      }

      all_finite = all_finite && mode_finite;
      all_checksums_stable = all_checksums_stable && mode_checksums_stable;
      all_production_checksums_stable =
          all_production_checksums_stable &&
          mode_production_checksums_stable;
      all_counts_stable = all_counts_stable && mode_counts_stable;
      all_query_shapes_exact =
          all_query_shapes_exact && mode_query_shape_exact;
      all_phase_contracts_exact =
          all_phase_contracts_exact && mode_phase_contracts_exact;
      if (std::string(mode_definition.name) == "frozen_runtime") {
        frozen_horizon_invariant = mode_horizon_invariant;
      }

      modes.push_back({
          {"mode", mode_definition.name},
          {"production_label", mode_definition.production_label},
          {"current_production_contract",
           std::string(mode_definition.name) == "frozen_runtime"},
          {"gnss_local_occupancy_installed",
           mode_definition.occupancy != nullptr},
          {"gnss_visibility_model",
           mode_definition.occupancy == nullptr
               ? "elevation_mask_only_current_runtime"
               : "LocalOccupancyGrid ray-based LOS plus elevation mask"},
          {"scientific_checksum_fnv1a64", reference_checksum},
          {"production_result_checksum_fnv1a64",
           reference_production_checksum},
          {"horizon_scientific_fields_invariant",
           mode_horizon_invariant},
          {"validation",
           {{"finite_iterations", mode_finite},
            {"scientific_checksums_stable_across_phases_and_workers",
             mode_checksums_stable},
            {"production_result_checksums_stable_across_phases_and_workers",
             mode_production_checksums_stable},
            {"validity_source_flag_counts_stable_across_phases_and_workers",
             mode_counts_stable},
            {"query_and_conversion_shape_exact", mode_query_shape_exact},
            {"counter_vs_component_timing_contract_exact",
             mode_phase_contracts_exact}}},
          {"workers", std::move(workers_json)},
      });
    }

    const bool pass = all_finite && all_checksums_stable &&
                      all_production_checksums_stable &&
                      all_counts_stable && all_query_shapes_exact &&
                      all_phase_contracts_exact;
    const std::string horizon_semantic_status = frozen_horizon_invariant
        ? "MISSING_SIGMA_GROWTH"
        : "OBSERVED_HORIZON_VARIATION_REQUIRES_REVIEW";

    Json output = {
        {"schema_version", "p0_provider_offline_profile_v2"},
        {"diagnostic_execution_status", pass ? "PASS" : "FAIL"},
        {"p0_horizon_semantic_status", horizon_semantic_status},
        {"standards_conformance_status",
         "BLOCKED_MISSING_SIGMA_GROWTH_AND_PRODUCTION_MAP_LOS"},
        {"build_type", IAP_BUILD_TYPE},
        {"clock", "std::chrono::steady_clock"},
        {"cpu_count", std::thread::hardware_concurrency()},
        {"warmup_iterations", options.warmup_iterations},
        {"measured_iterations", options.measured_iterations},
        {"diagnostic_latency_budget_ms", 400.0},
        {"validation",
         {{"finite_iterations", all_finite},
          {"scientific_checksums_stable_per_mode",
           all_checksums_stable},
          {"production_result_checksums_stable_per_mode",
           all_production_checksums_stable},
          {"validity_source_flag_counts_stable_per_mode",
           all_counts_stable},
          {"query_and_conversion_shape_exact", all_query_shapes_exact},
          {"counter_vs_component_timing_contract_exact",
           all_phase_contracts_exact}}},
        {"workload",
         {{"grid_shape", {kGridX, kGridY, kGridZ}},
          {"resolution_m", kResolutionM},
          {"logical_position_count", kLogicalPositionCount},
          {"horizons_s", kHorizons},
          {"logical_query_count", kLogicalQueryCount},
          {"grouping", "spatial_position_then_all_six_horizons"},
          {"snapshot_satellite_count", 31},
          {"map_los_candidate_occupancy_point_count",
           occupancy_points.size()},
          {"lidar_fim_primitive_count", 704},
          {"lidar_map_point_count", 23309}}},
        {"frozen_runtime_contract",
         {{"source_mode", "fusion"},
          {"gnss_epoch_policy", "auto"},
          {"use_current_integrity_prior", true},
          {"conservative_max_with_gnss", false},
          {"lidar_legacy_observability", true},
          {"lidar_fim_radius_m", 12.0},
          {"freshness_max_odom_age_s", 1.0},
          {"freshness_max_integrity_age_s", 1.0},
          {"freshness_max_gnss_age_s", 2.0},
          {"freshness_max_snapshot_age_s", 1.0},
          {"production_gnss_local_occupancy_binding", false},
          {"production_result_conversion",
           "iap::makeRiskPredictionResult shared pure helper"}}},
        {"synthetic_inputs",
         {{"values_are_synthetic", true},
          {"gnss_epoch", "deterministic synthetic 31-satellite epoch"},
          {"lidar_fim_primitives",
           "deterministic synthetic 704-primitive model"},
          {"lidar_map_points",
           "deterministic synthetic 23309-point model"},
          {"map_los_occupancy",
           "deterministic synthetic 704-point occupancy model"}}},
        {"horizon_semantics",
         {{"frozen_scientific_fields_invariant",
           frozen_horizon_invariant},
          {"observation_is_not_conformance", true},
          {"required_semantics",
           "empirical covariance growth with horizon-dependent Sigma_pred and PL_pred"},
          {"whole_result_cross_horizon_reuse_prohibited", true},
          {"scientific_field_whitelist", scientific_field_whitelist()},
          {"metadata_fields_excluded",
           {"query_position_map", "query_time_s", "horizon_s", "frame_id"}},
          {"freshness_reference", "fixed snapshot.stamp"}}},
        {"production_gap",
         {{"map_based_gnss_occlusion_required_by_conventions", true},
          {"current_production_installs_gnss_local_occupancy", false},
          {"non_occupancy_inputs_and_parameters_identical_between_modes",
           true},
          {"map_los_candidate_repairs_product_behavior", false},
          {"map_los_candidate_absolute_latency_characterizes_icra005",
           false}}},
        {"retained_icra005_authority",
         {{"provider_p95_ms_approx", 639.377},
          {"total_refresh_p95_ms", 657.21388795},
          {"gate_limit_ms", 400.0},
          {"gate_status", "P0_PERFORMANCE_GATE_FAIL"}}},
        {"timing_relationships",
         {{"total_predictor_provider",
           "outer wall time; counter-only phase is diagnostic budget authority"},
          {"worker_metrics",
           "cumulative per-worker times; workers may overlap"},
          {"advisory_metrics",
           "component-timed cost ranking only; nested and not additive to outer wall"},
          {"result_materialization",
           "shared production makeRiskPredictionResult conversion"},
          {"scientific_validation_replay",
           "real identical-input replay after the provider timer; checksum/count validation only"}}},
        {"scientific_checksum_algorithm", "FNV-1a-64 over all scientific fields"},
        {"modes", std::move(modes)}};

    std::filesystem::create_directories(output_path.parent_path());
    std::ofstream stream(output_path);
    if (!stream) throw std::runtime_error("failed to open output file");
    stream << output.dump(2) << '\n';
    std::cout << output.dump(2) << '\n';
    return pass ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "iap_predictor_offline_profile: " << error.what() << '\n';
    return 2;
  }
}
