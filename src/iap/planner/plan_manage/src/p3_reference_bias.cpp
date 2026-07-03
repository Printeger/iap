#include <ego_planner/p3_reference_bias.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numeric>

#include <iap/planner/risk_grid_map.hpp>

namespace ego_planner
{
namespace
{

constexpr double kEps = 1.0e-9;

double lastHorizon(const iap::RiskGridSnapshot& snapshot)
{
  const auto& horizons = snapshot.params().horizons_s;
  return horizons.empty() ? 0.0 : horizons.back();
}

double queryTimeForDistance(const iap::RiskGridSnapshot& snapshot,
                            const double distance,
                            const double max_vel)
{
  const double speed = std::max(max_vel, 1.0e-3);
  const double tau = std::min(distance / speed, lastHorizon(snapshot));
  return snapshot.stamp_s() + std::max(0.0, tau);
}

double queryTimeForDistanceUnclamped(const iap::RiskGridSnapshot& snapshot,
                                     const double distance,
                                     const double max_vel)
{
  const double speed = std::max(max_vel, 1.0e-3);
  return snapshot.stamp_s() + std::max(0.0, distance / speed);
}

Eigen::Vector3d horizontalDirection(const Eigen::Vector3d& from,
                                    const Eigen::Vector3d& to)
{
  Eigen::Vector3d direction = to - from;
  direction.z() = 0.0;
  if (direction.norm() < 1.0e-6)
  {
    direction = Eigen::Vector3d::UnitX();
  }
  return direction.normalized();
}

Eigen::Vector3d lateralDirection(const Eigen::Vector3d& forward)
{
  Eigen::Vector3d lateral(-forward.y(), forward.x(), 0.0);
  if (lateral.norm() < 1.0e-6)
  {
    lateral = Eigen::Vector3d::UnitY();
  }
  return lateral.normalized();
}

bool queryRisk(const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
               const Eigen::Vector3d& point,
               const double query_time_s,
               double* cost)
{
  if (!snapshot || cost == nullptr)
  {
    return false;
  }
  iap::RiskCostSample sample;
  if (!snapshot->queryCost(point, query_time_s, &sample))
  {
    return false;
  }
  if (!std::isfinite(sample.cost))
  {
    return false;
  }
  *cost = sample.cost;
  return true;
}

void writeLocalCsv(const P3ReferenceBiasConfig& config,
                   const P3LocalBiasResult& result,
                   const double stamp_s,
                   const uint64_t batch_id)
{
  if (!config.debug_csv_enable || config.debug_csv_path.empty())
  {
    return;
  }
  std::ifstream existing(config.debug_csv_path);
  const bool write_header =
      !existing.good() || existing.peek() == std::ifstream::traits_type::eof();
  existing.close();

  std::ofstream csv(config.debug_csv_path, std::ios::app);
  if (!csv.good())
  {
    return;
  }
  if (write_header)
  {
    csv << "mode,stamp,batch_id,generation_id,nominal_score,best_score,"
           "improvement_ratio,selected,candidate_count,valid_count,"
           "unknown_count,occupied_count,out_of_map_count,corridor_valid_ratio,"
           "station_count,waypoint_count,detour_ratio,reason\n";
  }
  csv << "local," << stamp_s << ',' << batch_id << ','
      << result.snapshot_generation_id << ',' << result.nominal_score << ','
      << result.biased_score << ',' << result.improvement_ratio << ','
      << (result.used_bias ? 1 : 0) << ',' << result.candidate_count << ','
      << result.valid_count << ',' << result.unknown_count << ','
      << result.occupied_count << ',' << result.out_of_map_count
      << ",0,0,0,1," << result.reason << '\n';
}

void writeGlobalCsv(const P3ReferenceBiasConfig& config,
                    const P3GlobalBiasResult& result,
                    const double stamp_s,
                    const uint64_t batch_id)
{
  if (!config.debug_csv_enable || config.debug_csv_path.empty())
  {
    return;
  }
  std::ifstream existing(config.debug_csv_path);
  const bool write_header =
      !existing.good() || existing.peek() == std::ifstream::traits_type::eof();
  existing.close();

  std::ofstream csv(config.debug_csv_path, std::ios::app);
  if (!csv.good())
  {
    return;
  }
  if (write_header)
  {
    csv << "mode,stamp,batch_id,generation_id,nominal_score,best_score,"
           "improvement_ratio,selected,candidate_count,valid_count,"
           "unknown_count,occupied_count,out_of_map_count,corridor_valid_ratio,"
           "station_count,waypoint_count,detour_ratio,reason\n";
  }
  csv << "global," << stamp_s << ',' << batch_id << ','
      << result.snapshot_generation_id << ',' << result.nominal_score << ','
      << result.biased_score << ',' << result.improvement_ratio << ','
      << (result.used_bias ? 1 : 0) << ',' << result.candidate_count << ','
      << result.valid_count << ',' << result.unknown_count << ','
      << result.occupied_count << ",0," << result.corridor_valid_ratio << ','
      << result.station_count << ',' << result.biased_waypoints.size() << ','
      << result.detour_ratio << ',' << result.reason << '\n';
}

struct BeamNode
{
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  double score = 0.0;
  double path_length = 0.0;
  std::vector<Eigen::Vector3d> path;
};

}  // namespace

P3LocalBiasResult applyP3LocalReferenceBias(
    const P3LocalBiasInput& input,
    const P3ReferenceBiasConfig& config,
    std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
    const P3PositionSafetyFn& is_position_safe,
    const double stamp_s,
    const uint64_t batch_id)
{
  P3LocalBiasResult result;
  result.start_pt = input.start_pt;
  result.end_pt = input.end_pt;
  result.nominal_target = input.nominal_target;
  result.target = input.nominal_target;
  if (!config.enable_local_reference_bias)
  {
    result.reason = "disabled";
    return result;
  }
  if (!snapshot)
  {
    result.reason = "snapshot_unavailable";
    writeLocalCsv(config, result, stamp_s, batch_id);
    return result;
  }
  result.snapshot_generation_id = snapshot->generation_id();
  if (!is_position_safe || !input.start_pt.allFinite() ||
      !input.nominal_target.allFinite() || config.local_bias_radius_m <= 0.0)
  {
    result.reason = "invalid_input";
    writeLocalCsv(config, result, stamp_s, batch_id);
    return result;
  }

  const Eigen::Vector3d forward = horizontalDirection(input.start_pt, input.end_pt);
  const Eigen::Vector3d lateral = lateralDirection(forward);
  const double radius = config.local_bias_radius_m;
  const std::vector<Eigen::Vector3d> offsets = {
      Eigen::Vector3d::Zero(),
      0.5 * radius * forward,
      -0.5 * radius * forward,
      0.5 * radius * lateral,
      -0.5 * radius * lateral,
      radius * lateral,
      -radius * lateral,
      0.5 * radius * (forward + lateral).normalized(),
      0.5 * radius * (forward - lateral).normalized()};

  const double nominal_progress =
      (input.nominal_target - input.start_pt).dot(forward);
  const double nominal_query_time = queryTimeForDistance(
      *snapshot, (input.nominal_target - input.start_pt).norm(), input.max_vel);
  double nominal_cost = 0.0;
  if (!snapshot->isInMap(input.nominal_target) ||
      !queryRisk(snapshot, input.nominal_target, nominal_query_time,
                 &nominal_cost))
  {
    result.reason = "nominal_unavailable";
    writeLocalCsv(config, result, stamp_s, batch_id);
    return result;
  }

  result.nominal_score = config.w_risk * nominal_cost;
  result.biased_score = result.nominal_score;
  Eigen::Vector3d best_target = input.nominal_target;

  for (const auto& offset : offsets)
  {
    ++result.candidate_count;
    Eigen::Vector3d candidate = input.nominal_target + offset;
    candidate.z() = input.nominal_target.z();
    const double shift = (candidate - input.nominal_target).norm();
    if (shift > radius + 1.0e-6 ||
        (candidate - input.start_pt).dot(forward) + 1.0e-6 < nominal_progress)
    {
      ++result.out_of_map_count;
      continue;
    }
    if (!snapshot->isInMap(candidate))
    {
      ++result.out_of_map_count;
      continue;
    }
    if (!is_position_safe(candidate))
    {
      ++result.occupied_count;
      continue;
    }

    const double query_time = queryTimeForDistance(
        *snapshot, (candidate - input.start_pt).norm(), input.max_vel);
    double cost = 0.0;
    if (!queryRisk(snapshot, candidate, query_time, &cost))
    {
      ++result.unknown_count;
      continue;
    }
    ++result.valid_count;
    const double shift_ratio = shift / std::max(radius, kEps);
    const double score = config.w_risk * cost + config.w_detour * shift_ratio;
    if (score < result.biased_score)
    {
      result.biased_score = score;
      best_target = candidate;
    }
  }

  result.improvement_ratio =
      (result.nominal_score - result.biased_score) /
      std::max(std::abs(result.nominal_score), 1.0e-6);
  if ((best_target - input.nominal_target).norm() > 1.0e-6 &&
      result.improvement_ratio >= config.min_improvement_ratio)
  {
    result.used_bias = true;
    result.target = best_target;
    result.reason = "selected";
  }
  else
  {
    result.reason = result.valid_count > 0 ? "improvement_too_small"
                                           : "no_valid_candidates";
  }
  writeLocalCsv(config, result, stamp_s, batch_id);
  return result;
}

P3GlobalBiasResult computeP3GlobalReferenceBias(
    const P3GlobalBiasInput& input,
    const P3ReferenceBiasConfig& config,
    std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
    const P3PositionSafetyFn& is_position_safe,
    const double stamp_s,
    const uint64_t batch_id)
{
  P3GlobalBiasResult result;
  result.start_pos = input.start_pos;
  result.end_pos = input.end_pos;
  if (!config.enable_global_reference_bias)
  {
    result.reason = "disabled";
    return result;
  }
  if (!snapshot)
  {
    result.reason = "snapshot_unavailable";
    writeGlobalCsv(config, result, stamp_s, batch_id);
    return result;
  }
  result.snapshot_generation_id = snapshot->generation_id();
  if (!is_position_safe || !input.start_pos.allFinite() ||
      !input.end_pos.allFinite() || config.station_spacing_m <= 0.0 ||
      config.lateral_sample_step_m <= 0.0 ||
      config.lateral_sample_count_each_side < 0 || config.beam_width <= 0)
  {
    result.reason = "invalid_input";
    writeGlobalCsv(config, result, stamp_s, batch_id);
    return result;
  }

  const Eigen::Vector3d delta = input.end_pos - input.start_pos;
  const double straight_len = delta.norm();
  if (straight_len < 1.0e-6)
  {
    result.reason = "zero_length_corridor";
    writeGlobalCsv(config, result, stamp_s, batch_id);
    return result;
  }
  const Eigen::Vector3d forward = horizontalDirection(input.start_pos, input.end_pos);
  const Eigen::Vector3d lateral = lateralDirection(forward);
  const int segment_count =
      std::max(1, static_cast<int>(std::ceil(straight_len / config.station_spacing_m)));
  result.station_count = segment_count + 1;

  std::vector<std::vector<Eigen::Vector3d>> station_candidates;
  station_candidates.reserve(std::max(0, segment_count - 1));
  double nominal_score_sum = 0.0;
  int nominal_count = 0;

  for (int station = 1; station < segment_count; ++station)
  {
    const double alpha = static_cast<double>(station) /
                         static_cast<double>(segment_count);
    const Eigen::Vector3d center = input.start_pos + alpha * delta;
    std::vector<Eigen::Vector3d> candidates;
    for (int side = -config.lateral_sample_count_each_side;
         side <= config.lateral_sample_count_each_side; ++side)
    {
      Eigen::Vector3d candidate =
          center + static_cast<double>(side) *
                       config.lateral_sample_step_m * lateral;
      candidate.z() = center.z();
      ++result.candidate_count;
      const double station_distance = straight_len * alpha;
      const double query_time =
          queryTimeForDistanceUnclamped(*snapshot, station_distance, input.max_vel);
      double cost = 0.0;
      if (snapshot->isInMap(candidate) &&
          queryRisk(snapshot, candidate, query_time, &cost))
      {
        ++result.valid_count;
      }
      else
      {
        ++result.unknown_count;
      }
      if (side == 0)
      {
        nominal_score_sum +=
            queryRisk(snapshot, candidate, query_time, &cost)
                ? config.w_risk * cost
                : config.w_unknown;
        ++nominal_count;
      }
      candidates.push_back(candidate);
    }
    station_candidates.push_back(candidates);
  }

  if (result.candidate_count <= 0)
  {
    result.reason = "no_corridor_candidates";
    writeGlobalCsv(config, result, stamp_s, batch_id);
    return result;
  }
  result.corridor_valid_ratio =
      static_cast<double>(result.valid_count) /
      static_cast<double>(result.candidate_count);
  if (result.corridor_valid_ratio < config.min_corridor_valid_ratio)
  {
    result.reason = "corridor_coverage_insufficient";
    writeGlobalCsv(config, result, stamp_s, batch_id);
    return result;
  }

  std::vector<BeamNode> beam;
  beam.push_back(BeamNode{input.start_pos, 0.0, 0.0, {input.start_pos}});
  for (size_t station = 0; station < station_candidates.size(); ++station)
  {
    std::vector<BeamNode> next_beam;
    for (const auto& prev : beam)
    {
      for (const auto& candidate : station_candidates[station])
      {
        double cost = 0.0;
        const double station_distance =
            (candidate - input.start_pos).norm();
        const double query_time =
            queryTimeForDistanceUnclamped(*snapshot, station_distance, input.max_vel);
        const bool risk_ok =
            snapshot->isInMap(candidate) &&
            queryRisk(snapshot, candidate, query_time, &cost);
        if (!risk_ok)
        {
          ++result.unknown_count;
        }
        if (!is_position_safe(candidate))
        {
          ++result.occupied_count;
          continue;
        }
        const double edge_len = (candidate - prev.point).norm();
        BeamNode node;
        node.point = candidate;
        node.path = prev.path;
        node.path.push_back(candidate);
        node.path_length = prev.path_length + edge_len;
        node.score = prev.score +
                     (risk_ok ? config.w_risk * cost : config.w_unknown) +
                     config.w_detour *
                         (edge_len / std::max(config.station_spacing_m, kEps));
        next_beam.push_back(node);
      }
    }
    std::sort(next_beam.begin(), next_beam.end(),
              [](const BeamNode& a, const BeamNode& b) {
                return a.score < b.score;
              });
    if (static_cast<int>(next_beam.size()) > config.beam_width)
    {
      next_beam.resize(config.beam_width);
    }
    beam = next_beam;
    if (beam.empty())
    {
      result.reason = "all_beams_blocked";
      writeGlobalCsv(config, result, stamp_s, batch_id);
      return result;
    }
  }

  for (auto& node : beam)
  {
    node.path.push_back(input.end_pos);
    node.path_length += (input.end_pos - node.point).norm();
    node.score += config.w_detour *
                  ((input.end_pos - node.point).norm() /
                   std::max(config.station_spacing_m, kEps));
  }
  const auto best_it = std::min_element(
      beam.begin(), beam.end(),
      [](const BeamNode& a, const BeamNode& b) { return a.score < b.score; });
  if (best_it == beam.end())
  {
    result.reason = "no_beam_result";
    writeGlobalCsv(config, result, stamp_s, batch_id);
    return result;
  }

  result.nominal_score =
      nominal_count > 0 ? nominal_score_sum / static_cast<double>(nominal_count)
                        : 0.0;
  result.biased_score =
      best_it->score / std::max(1.0, static_cast<double>(station_candidates.size()));
  result.detour_ratio = best_it->path_length / std::max(straight_len, kEps);
  result.improvement_ratio =
      (result.nominal_score - result.biased_score) /
      std::max(std::abs(result.nominal_score), 1.0e-6);

  if (result.detour_ratio > config.max_detour_ratio)
  {
    result.reason = "detour_too_large";
    writeGlobalCsv(config, result, stamp_s, batch_id);
    return result;
  }
  if (result.improvement_ratio < config.min_improvement_ratio)
  {
    result.reason = "improvement_too_small";
    writeGlobalCsv(config, result, stamp_s, batch_id);
    return result;
  }

  result.used_bias = true;
  result.reason = "selected";
  for (size_t i = 1; i + 1 < best_it->path.size(); ++i)
  {
    result.biased_waypoints.push_back(best_it->path[i]);
  }
  if (result.biased_waypoints.empty())
  {
    result.used_bias = false;
    result.reason = "no_biased_waypoints";
  }
  writeGlobalCsv(config, result, stamp_s, batch_id);
  return result;
}

}  // namespace ego_planner
