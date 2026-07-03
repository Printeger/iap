#ifndef EGO_PLANNER_P2_CANDIDATE_RANKING_H_
#define EGO_PLANNER_P2_CANDIDATE_RANKING_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <bspline_opt/bspline_optimizer.h>

namespace iap
{
  class RiskGridSnapshot;
}

namespace ego_planner
{

struct P2CandidateRankingConfig
{
  bool enable_candidate_ranking = false;
  bool metrics_only = true;
  double sample_dt_s = 0.2;
  double lambda_candidate_integrity = 1.0;
  double w_max_cost = 0.25;
  double w_unknown = 5.0;
  double w_stale = 2.0;
  double min_valid_ratio = 0.3;
  bool debug_csv_enable = false;
  std::string debug_csv_path;
};

struct P2CandidateInput
{
  int candidate_id = 0;
  Eigen::MatrixXd control_points;
  double final_cost = 0.0;
  BsplineOptimizer::OptimizerCostBreakdown cost_breakdown;
};

struct P2CandidateMetrics
{
  int candidate_id = 0;
  bool opt_success = true;
  double total_cost = 0.0;
  double original_cost = 0.0;
  double integrity_cost = 0.0;
  double optimizer_score = 0.0;
  double mean_cost = 0.0;
  double max_cost = 0.0;
  double valid_ratio = 0.0;
  double unknown_ratio = 0.0;
  double stale_ratio = 0.0;
  uint64_t snapshot_generation_id = 0;
  double candidate_score = 0.0;
  bool metrics_only = true;
  bool selected = false;
  std::string fallback_reason = "not_evaluated";
};

struct P2CandidateRankingResult
{
  int selected_index = -1;
  bool used_integrity_ranking = false;
  bool fallback = true;
  std::string fallback_reason = "not_evaluated";
  std::vector<P2CandidateMetrics> metrics;
};

P2CandidateRankingResult rankP2Candidates(
    const std::vector<P2CandidateInput>& candidates,
    const P2CandidateRankingConfig& config,
    std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
    double query_base_time_s,
    double bspline_interval_s,
    double stamp_s,
    uint64_t batch_id);

}  // namespace ego_planner

#endif  // EGO_PLANNER_P2_CANDIDATE_RANKING_H_
