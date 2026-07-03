#include <ego_planner/p2_candidate_ranking.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>

#include <bspline_opt/uniform_bspline.h>
#include <iap/planner/risk_grid_map.hpp>

namespace ego_planner
{
namespace
{

constexpr double kScoreEps = 1.0e-9;

int selectMinFinalCost(const std::vector<P2CandidateInput>& candidates)
{
  if (candidates.empty())
  {
    return -1;
  }
  int selected = 0;
  double best = candidates.front().final_cost;
  for (size_t i = 1; i < candidates.size(); ++i)
  {
    if (candidates[i].final_cost < best)
    {
      best = candidates[i].final_cost;
      selected = static_cast<int>(i);
    }
  }
  return selected;
}

int selectMinOriginalCost(const std::vector<P2CandidateInput>& candidates)
{
  if (candidates.empty())
  {
    return -1;
  }
  int selected = 0;
  double best = candidates.front().cost_breakdown.original_cost;
  for (size_t i = 1; i < candidates.size(); ++i)
  {
    if (candidates[i].cost_breakdown.original_cost < best)
    {
      best = candidates[i].cost_breakdown.original_cost;
      selected = static_cast<int>(i);
    }
  }
  return selected;
}

int selectMinCandidateScore(const std::vector<P2CandidateMetrics>& metrics)
{
  if (metrics.empty())
  {
    return -1;
  }
  int selected = 0;
  double best = metrics.front().candidate_score;
  for (size_t i = 1; i < metrics.size(); ++i)
  {
    if (metrics[i].candidate_score < best)
    {
      best = metrics[i].candidate_score;
      selected = static_cast<int>(i);
    }
  }
  return selected;
}

bool isStaleMiss(const iap::RiskCostSample& sample)
{
  if (sample.stale)
  {
    return true;
  }
  return sample.reason.find("stale") != std::string::npos;
}

void sampleCandidate(P2CandidateMetrics& metrics,
                     const P2CandidateInput& candidate,
                     const P2CandidateRankingConfig& config,
                     const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
                     const double query_base_time_s,
                     const double bspline_interval_s)
{
  metrics.candidate_id = candidate.candidate_id;
  metrics.total_cost = candidate.cost_breakdown.total_cost;
  metrics.original_cost = candidate.cost_breakdown.original_cost;
  metrics.integrity_cost = candidate.cost_breakdown.integrity_cost;
  metrics.metrics_only = config.metrics_only;
  metrics.snapshot_generation_id = snapshot ? snapshot->generation_id() : 0;

  if (!snapshot)
  {
    metrics.fallback_reason = "snapshot_unavailable";
    return;
  }
  if (!std::isfinite(config.sample_dt_s) || config.sample_dt_s <= 0.0 ||
      !std::isfinite(bspline_interval_s) || bspline_interval_s <= 0.0 ||
      candidate.control_points.rows() != 3 || candidate.control_points.cols() < 4)
  {
    metrics.fallback_reason = "invalid_sampling_config";
    return;
  }

  UniformBspline trajectory(candidate.control_points, 3, bspline_interval_s);
  const double duration = trajectory.getTimeSum();
  if (!std::isfinite(duration) || duration < 0.0)
  {
    metrics.fallback_reason = "invalid_trajectory_duration";
    return;
  }

  int sample_count = 0;
  int hit_count = 0;
  int unknown_count = 0;
  int stale_count = 0;
  double cost_sum = 0.0;
  double max_cost = 0.0;
  for (double t = 0.0; t <= duration + 1.0e-9; t += config.sample_dt_s)
  {
    ++sample_count;
    const Eigen::Vector3d p = trajectory.evaluateDeBoorT(std::min(t, duration));
    iap::RiskCostSample sample;
    const bool hit = snapshot->queryCost(p, query_base_time_s + std::min(t, duration), &sample);
    if (hit && std::isfinite(sample.cost))
    {
      ++hit_count;
      cost_sum += sample.cost;
      max_cost = std::max(max_cost, sample.cost);
    }
    else if (isStaleMiss(sample))
    {
      ++stale_count;
    }
    else
    {
      ++unknown_count;
    }

    if (t >= duration)
    {
      break;
    }
  }

  if (sample_count <= 0)
  {
    metrics.fallback_reason = "no_samples";
    return;
  }

  metrics.mean_cost = hit_count > 0 ? cost_sum / static_cast<double>(hit_count) : 0.0;
  metrics.max_cost = max_cost;
  metrics.valid_ratio = static_cast<double>(hit_count) / static_cast<double>(sample_count);
  metrics.unknown_ratio = static_cast<double>(unknown_count) / static_cast<double>(sample_count);
  metrics.stale_ratio = static_cast<double>(stale_count) / static_cast<double>(sample_count);
  metrics.fallback_reason = "ok";
}

void writeDebugCsv(const P2CandidateRankingConfig& config,
                   const P2CandidateRankingResult& result,
                   const double stamp_s,
                   const uint64_t batch_id)
{
  if (!config.enable_candidate_ranking || !config.debug_csv_enable ||
      config.debug_csv_path.empty())
  {
    return;
  }

  std::ifstream existing(config.debug_csv_path);
  const bool write_header = !existing.good() || existing.peek() == std::ifstream::traits_type::eof();
  existing.close();

  std::ofstream csv(config.debug_csv_path, std::ios::app);
  if (!csv.good())
  {
    return;
  }
  if (write_header)
  {
    csv << "stamp,batch_id,candidate_id,opt_success,total_cost,original_cost,"
           "integrity_cost,optimizer_score,mean_cost,max_cost,valid_ratio,"
           "unknown_ratio,stale_ratio,snapshot_generation_id,candidate_score,"
           "metrics_only,selected,fallback_reason\n";
  }
  for (const auto& metric : result.metrics)
  {
    csv << stamp_s << ',' << batch_id << ',' << metric.candidate_id << ','
        << (metric.opt_success ? 1 : 0) << ',' << metric.total_cost << ','
        << metric.original_cost << ',' << metric.integrity_cost << ','
        << metric.optimizer_score << ',' << metric.mean_cost << ','
        << metric.max_cost << ',' << metric.valid_ratio << ','
        << metric.unknown_ratio << ',' << metric.stale_ratio << ','
        << metric.snapshot_generation_id << ',' << metric.candidate_score << ','
        << (metric.metrics_only ? 1 : 0) << ','
        << (metric.selected ? 1 : 0) << ',' << metric.fallback_reason << '\n';
  }
}

}  // namespace

P2CandidateRankingResult rankP2Candidates(
    const std::vector<P2CandidateInput>& candidates,
    const P2CandidateRankingConfig& config,
    std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
    const double query_base_time_s,
    const double bspline_interval_s,
    const double stamp_s,
    const uint64_t batch_id)
{
  P2CandidateRankingResult result;
  if (candidates.empty())
  {
    result.fallback_reason = "no_successful_candidates";
    return result;
  }

  if (!config.enable_candidate_ranking)
  {
    result.selected_index = selectMinFinalCost(candidates);
    result.fallback = true;
    result.fallback_reason = "disabled";
    return result;
  }

  result.metrics.reserve(candidates.size());
  double min_original = std::numeric_limits<double>::infinity();
  double max_original = -std::numeric_limits<double>::infinity();
  for (const auto& candidate : candidates)
  {
    min_original = std::min(min_original, candidate.cost_breakdown.original_cost);
    max_original = std::max(max_original, candidate.cost_breakdown.original_cost);
  }

  for (const auto& candidate : candidates)
  {
    P2CandidateMetrics metric;
    sampleCandidate(metric, candidate, config, snapshot, query_base_time_s,
                    bspline_interval_s);
    metric.optimizer_score =
        (metric.original_cost - min_original) /
        (max_original - min_original + kScoreEps);
    const double integrity_score =
        metric.mean_cost +
        config.w_max_cost * metric.max_cost +
        config.w_unknown * metric.unknown_ratio +
        config.w_stale * metric.stale_ratio;
    metric.candidate_score =
        metric.optimizer_score + config.lambda_candidate_integrity * integrity_score;
    result.metrics.push_back(metric);
  }

  int selected = selectMinOriginalCost(candidates);
  result.fallback = true;
  result.fallback_reason = "metrics_only";

  if (!snapshot)
  {
    result.fallback_reason = "snapshot_unavailable";
  }
  else
  {
    bool any_valid = false;
    for (const auto& metric : result.metrics)
    {
      if (metric.valid_ratio >= config.min_valid_ratio)
      {
        any_valid = true;
        break;
      }
    }

    if (!any_valid)
    {
      result.fallback_reason = "valid_ratio_too_low";
    }
    else if (!config.metrics_only)
    {
      selected = selectMinCandidateScore(result.metrics);
      result.fallback = false;
      result.used_integrity_ranking = true;
      result.fallback_reason = "none";
    }
  }

  result.selected_index = selected;
  for (size_t i = 0; i < result.metrics.size(); ++i)
  {
    result.metrics[i].selected = static_cast<int>(i) == result.selected_index;
    result.metrics[i].fallback_reason = result.fallback_reason;
  }
  writeDebugCsv(config, result, stamp_s, batch_id);
  return result;
}

}  // namespace ego_planner
