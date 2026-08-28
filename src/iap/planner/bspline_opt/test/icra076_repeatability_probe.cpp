#include <bspline_opt/p4_collision_guide.h>
#include <iap/planner/risk_grid_map.hpp>

#include <nlohmann/json.hpp>
#include <openssl/evp.h>

#include <Eigen/Core>

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

using json = nlohmann::json;

std::string sha256(const std::string & bytes)
{
  EVP_MD_CTX * context = EVP_MD_CTX_new();
  if (!context) {
    throw std::runtime_error("sha256_context_unavailable");
  }
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int digest_size = 0;
  const bool ok = EVP_DigestInit_ex(context, EVP_sha256(), nullptr) == 1 &&
    EVP_DigestUpdate(context, bytes.data(), bytes.size()) == 1 &&
    EVP_DigestFinal_ex(context, digest, &digest_size) == 1;
  EVP_MD_CTX_free(context);
  if (!ok) {
    throw std::runtime_error("sha256_failed");
  }
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < digest_size; ++index) {
    stream << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return stream.str();
}

std::vector<Eigen::Vector3d> pathFromJson(const json & value)
{
  if (!value.is_array() || value.size() < 2) {
    throw std::runtime_error("path_invalid");
  }
  std::vector<Eigen::Vector3d> path;
  for (const auto & point : value) {
    if (!point.is_array() || point.size() != 3) {
      throw std::runtime_error("path_point_invalid");
    }
    path.emplace_back(
      point.at(0).get<double>(), point.at(1).get<double>(),
      point.at(2).get<double>());
  }
  return path;
}

class FlatNullProvider final : public iap::RiskPredictionProvider
{
public:
  explicit FlatNullProvider(double provider_c_pi_m)
  : provider_c_pi_m_(provider_c_pi_m) {}

  bool batchQuery(
    const std::vector<iap::RiskPredictionQuery> & queries,
    std::vector<iap::RiskPredictionResult> * results) override
  {
    if (!results) {
      return false;
    }
    results->clear();
    results->reserve(queries.size());
    for (std::size_t index = 0; index < queries.size(); ++index) {
      iap::RiskPredictionResult result;
      result.available = true;
      result.valid = true;
      result.stale = false;
      result.hpl_pred = provider_c_pi_m_;
      result.vpl_pred = provider_c_pi_m_;
      result.reason = "ok";
      results->push_back(result);
    }
    return true;
  }

private:
  double provider_c_pi_m_;
};

class SerializedPathSearch final : public ego_planner::P4GuideSearch
{
public:
  SerializedPathSearch(
    std::vector<Eigen::Vector3d> original_path,
    std::vector<Eigen::Vector3d> risk_path)
  : original_path_(std::move(original_path)), risk_path_(std::move(risk_path)) {}

  ego_planner::P4GuideSearchOutcome searchOriginal(
    const ego_planner::P4GuideRequest &) override
  {
    ego_planner::P4GuideSearchOutcome outcome;
    outcome.success = true;
    outcome.path = original_path_;
    outcome.reason = "ok";
    return outcome;
  }

  ego_planner::P4GuideSearchOutcome searchRiskAware(
    const ego_planner::P4GuideRequest &) override
  {
    ego_planner::P4GuideSearchOutcome outcome;
    outcome.success = true;
    outcome.path = risk_path_;
    outcome.reason = "ok";
    return outcome;
  }

private:
  std::vector<Eigen::Vector3d> original_path_;
  std::vector<Eigen::Vector3d> risk_path_;
};

P4RiskAStarConfig p4Config(double query_speed_mps)
{
  P4RiskAStarConfig config;
  config.enable_risk_aware_astar = true;
  config.metrics_only = false;
  config.lambda_p4_risk = 0.2;
  config.risk_cost_max = 100.0;
  config.unknown_edge_penalty = 5.0;
  config.max_extra_path_ratio = 1.30;
  config.query_speed_mps = query_speed_mps;
  config.objective = P4RiskObjective::PROVIDER_BOTTLENECK_V2;
  return config;
}

int run(const std::string & snapshot_path, int invocation_index)
{
  std::ifstream input(snapshot_path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("snapshot_open_failed");
  }
  const std::string input_bytes(
    (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  const json serialized = json::parse(input_bytes);
  if (serialized.at("schema_version") !=
    "icra076_flat_null_serialized_snapshot_v1" ||
    serialized.at("provider").at("mode") != "FLAT_NULL" ||
    serialized.at("request").at("equal_arc_lattice_count") != 200 ||
    serialized.at("request").at("domain") !=
    "controllable_interior_b_equals_2r")
  {
    throw std::runtime_error("snapshot_contract_invalid");
  }

  iap::RiskGridMapParams params;
  params.frame_id = serialized.at("frame_id").get<std::string>();
  params.resolution_m = serialized.at("resolution_m").get<double>();
  params.size_x_m = serialized.at("size_m").at(0).get<double>();
  params.size_y_m = serialized.at("size_m").at(1).get<double>();
  params.size_z_m = serialized.at("size_m").at(2).get<double>();
  params.horizons_s = serialized.at("horizons_s").get<std::vector<double>>();
  params.stale_timeout_s = serialized.at("stale_timeout_s").get<double>();
  params.unknown_cost = serialized.at("unknown_cost").get<double>();
  params.cost_max = serialized.at("cost_max").get<double>();
  iap::RiskGridMap grid(params);
  FlatNullProvider provider(
    serialized.at("provider").at("c_pi_m").get<double>());
  const auto center = serialized.at("refresh_center_m");
  const Eigen::Vector3d refresh_center(
    center.at(0).get<double>(), center.at(1).get<double>(),
    center.at(2).get<double>());
  const double refresh_stamp_s =
    serialized.at("refresh_stamp_s").get<double>();
  std::string refresh_reason;
  if (!grid.refreshFromProvider(
      refresh_center, refresh_stamp_s, provider, &refresh_reason))
  {
    throw std::runtime_error("snapshot_refresh_failed:" + refresh_reason);
  }
  const auto snapshot = grid.acquireSnapshot();
  if (!snapshot) {
    throw std::runtime_error("snapshot_acquire_failed");
  }

  const auto request_json = serialized.at("request");
  const auto original_path = pathFromJson(request_json.at("original_path_m"));
  const auto risk_path = pathFromJson(request_json.at("risk_path_m"));
  SerializedPathSearch search(original_path, risk_path);
  ego_planner::P4CollisionGuidePlanner planner(search);
  const uint64_t occupancy_epoch =
    request_json.at("occupancy_epoch").get<uint64_t>();
  const ego_planner::P4GuideRequest request(
    request_json.at("planning_attempt_id").get<uint64_t>(),
    request_json.at("collision_segment_id").get<uint64_t>(),
    original_path.front(), original_path.back(), true, snapshot,
    request_json.at("query_base_time_s").get<double>(), occupancy_epoch,
    [occupancy_epoch]() {return occupancy_epoch;},
    p4Config(request_json.at("query_speed_mps").get<double>()));
  const auto decision = planner.planCollisionGuide(request);
  if (!decision.original.risk_profile.complete() ||
    !decision.risk.risk_profile.complete() ||
    decision.original.equal_arc_samples.size() != 200 ||
    decision.risk.equal_arc_samples.size() != 200)
  {
    throw std::runtime_error("production_profile_incomplete");
  }

  const double b_original = decision.original.risk_profile.max;
  const double b_risk = decision.risk.risk_profile.max;
  json output = {
    {"schema_version", "icra076_production_replay_measurement_v1"},
    {"status", "PASS"},
    {"unit", "m"},
    {"domain", "controllable_interior_b_equals_2r"},
    {"measurement_source", "P4GuideDecision.risk_profile.max"},
    {"invocation_index", invocation_index},
    {"snapshot_identity", {
        {"serialized_input_sha256", sha256(input_bytes)},
        {"snapshot_config_hash", request.snapshotConfigHash()},
        {"snapshot_generation", snapshot->generation_id()}}},
    {"sample_identity", {
        {"equal_arc_lattice_count", 200},
        {"original_lattice_hash", ego_planner::canonicalP4GuideHash(
            decision.original.equal_arc_samples)},
        {"risk_lattice_hash", ego_planner::canonicalP4GuideHash(
            decision.risk.equal_arc_samples)},
        {"original_controllable_sample_count",
          decision.original.risk_profile.sample_count},
        {"risk_controllable_sample_count",
          decision.risk.risk_profile.sample_count},
        {"original_valid_count", decision.original.risk_profile.valid_count},
        {"risk_valid_count", decision.risk.risk_profile.valid_count}}},
    {"B_original_m", b_original},
    {"B_risk_m", b_risk},
    {"D_peak_m", b_original - b_risk},
  };
  std::cout << output.dump() << '\n';
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    std::string snapshot_path;
    int invocation_index = 0;
    for (int index = 1; index < argc; ++index) {
      const std::string argument = argv[index];
      if (argument == "--snapshot" && index + 1 < argc) {
        snapshot_path = argv[++index];
      } else if (argument == "--invocation-index" && index + 1 < argc) {
        invocation_index = std::stoi(argv[++index]);
      } else {
        throw std::runtime_error("argument_invalid");
      }
    }
    if (snapshot_path.empty() || invocation_index <= 0) {
      throw std::runtime_error("required_argument_missing");
    }
    return run(snapshot_path, invocation_index);
  } catch (const std::exception & error) {
    std::cerr << "ICRA076_PROBE_ERROR:" << error.what() << '\n';
    return 2;
  }
}
