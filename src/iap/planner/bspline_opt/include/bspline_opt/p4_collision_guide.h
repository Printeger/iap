#ifndef BSPLINE_OPT__P4_COLLISION_GUIDE_H_
#define BSPLINE_OPT__P4_COLLISION_GUIDE_H_

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <path_searching/dyn_a_star.h>
#include <iap/planner/risk_grid_map.hpp>

namespace ego_planner
{

  inline constexpr char kP4GuideDecisionSchema[] =
    "p4_collision_guide_decision_v1";
  inline constexpr std::size_t kP4FinalGuideSampleCount = 200;
  inline constexpr double kP4PathLengthRatioHardCap = 1.30;

  enum class P4GuideDecisionStatus
  {
    ORIGINAL_SELECTED = 0,
    PLANNER_FAILURE,
    DECISION_INVALID_REPLAN_REQUIRED,
  };

  enum class P4GuideDecisionReason
  {
    NOT_EVALUATED = 0,
    METRICS_ONLY,
    RISK_DISABLED,
    SELECTION_NOT_AUTHORIZED,
    ORIGINAL_SEARCH_FAILED,
    ORIGINAL_SEARCH_TIMEOUT,
    SNAPSHOT_UNAVAILABLE,
    UNKNOWN_RISK,
    STALE_RISK,
    NON_FINITE_RISK,
    INCOMPLETE_PROFILE,
    RISK_SEARCH_FAILED,
    RISK_SEARCH_TIMEOUT,
    PATH_LENGTH_RATIO_EXCEEDED,
    OCCUPANCY_EPOCH_CHANGED,
    REQUEST_INVALID,
    REQUEST_IDENTITY_MISMATCH,
    ZERO_LENGTH_GEOMETRY,
  };

  const char * p4GuideDecisionStatusName(P4GuideDecisionStatus status);
  const char * p4GuideDecisionReasonName(P4GuideDecisionReason reason);

  struct P4GuideRiskProfile
  {
    std::size_t sample_count = 0;
    std::size_t valid_count = 0;
    std::size_t unknown_count = 0;
    std::size_t stale_count = 0;
    std::size_t non_finite_count = 0;
    std::string first_invalid_reason;
    double mean = std::numeric_limits < double > ::quiet_NaN();
    double max = std::numeric_limits < double > ::quiet_NaN();

    bool complete() const
    {
      return sample_count == kP4FinalGuideSampleCount &&
             valid_count == kP4FinalGuideSampleCount &&
             unknown_count == 0 && stale_count == 0 &&
             non_finite_count == 0 && std::isfinite(mean) &&
             std::isfinite(max);
    }
  };

  struct P4GuideRecord
  {
    bool returned = false;
    std::vector < Eigen::Vector3d > complete_path;
    std::vector < Eigen::Vector3d > equal_arc_samples;
    std::string canonical_hash;
    double length_m = 0.0;
    P4GuideRiskProfile risk_profile;
    struct SampleTrace
    {
      std::size_t sample_index = 0;
      Eigen::Vector3d point = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
      double query_time_s = std::numeric_limits<double>::quiet_NaN();
      iap::RiskCostSample sample;
      iap::RiskCostQueryTrace query;
    };
    std::vector<SampleTrace> sample_traces;
  };

  class P4GuideRequest
  {
public:
    using LiveOccupancyEpoch = std::function < uint64_t() >;

    P4GuideRequest(
      uint64_t planning_attempt_id, uint64_t collision_segment_id,
      Eigen::Vector3d start, Eigen::Vector3d end,
      bool scanner_verified_free_endpoints,
      std::shared_ptr < const iap::RiskGridSnapshot > snapshot,
      double query_base_time_s, uint64_t occupancy_epoch,
      LiveOccupancyEpoch live_occupancy_epoch,
      P4RiskAStarConfig config);

    uint64_t planningAttemptId() const {return planning_attempt_id_;}
    uint64_t collisionSegmentId() const {return collision_segment_id_;}
    const Eigen::Vector3d & start() const {return start_;}
    const Eigen::Vector3d & end() const {return end_;}
    bool scannerVerifiedFreeEndpoints() const
    {
      return scanner_verified_free_endpoints_;
    }
    const std::shared_ptr < const iap::RiskGridSnapshot > & snapshot() const
  {
      return snapshot_;
    }
    double queryBaseTimeS() const {return query_base_time_s_;}
    uint64_t occupancyEpoch() const {return occupancy_epoch_;}
    const LiveOccupancyEpoch & liveOccupancyEpoch() const
    {
      return live_occupancy_epoch_;
    }
    const P4RiskAStarConfig & config() const {return config_;}

    bool valid(std::string * reason = nullptr) const;
    std::string canonicalIdentityHash() const;

private:
    uint64_t planning_attempt_id_;
    uint64_t collision_segment_id_;
    Eigen::Vector3d start_;
    Eigen::Vector3d end_;
    bool scanner_verified_free_endpoints_;
    std::shared_ptr < const iap::RiskGridSnapshot > snapshot_;
    double query_base_time_s_;
    uint64_t occupancy_epoch_;
    LiveOccupancyEpoch live_occupancy_epoch_;
    P4RiskAStarConfig config_;
  };

  struct P4GuideSearchOutcome
  {
    bool success = false;
    bool timed_out = false;
    std::vector < Eigen::Vector3d > path;
    P4AStarMetrics metrics;
    std::string reason = "not_run";
  };

  class P4GuideSearch
  {
public:
    virtual ~P4GuideSearch() = default;
    virtual P4GuideSearchOutcome searchOriginal(
      const P4GuideRequest & request) = 0;
    virtual P4GuideSearchOutcome searchRiskAware(
      const P4GuideRequest & request) = 0;
  };

  class P4AStarGuideSearch final: public P4GuideSearch
  {
public:
    explicit P4AStarGuideSearch(AStar::Ptr a_star)
    : a_star_(std::move(a_star)) {
    }

    P4GuideSearchOutcome searchOriginal(
      const P4GuideRequest & request) override;
    P4GuideSearchOutcome searchRiskAware(
      const P4GuideRequest & request) override;

private:
    AStar::Ptr a_star_;
  };

  struct P4GuideDecision
  {
    std::string schema_version = kP4GuideDecisionSchema;
    P4GuideDecisionStatus status =
      P4GuideDecisionStatus::DECISION_INVALID_REPLAN_REQUIRED;
    P4GuideDecisionReason reason = P4GuideDecisionReason::NOT_EVALUATED;
    uint64_t planning_attempt_id = 0;
    uint64_t collision_segment_id = 0;
    Eigen::Vector3d segment_start = Eigen::Vector3d::Zero();
    Eigen::Vector3d segment_end = Eigen::Vector3d::Zero();
    uint64_t snapshot_generation = 0;
    double snapshot_stamp_s = std::numeric_limits < double > ::quiet_NaN();
    std::string snapshot_frame;
    double query_base_time_s = std::numeric_limits < double > ::quiet_NaN();
    uint64_t occupancy_epoch = 0;
    std::string request_hash;
    P4GuideRecord original;
    P4GuideRecord risk;
    P4GuideRecord selected;
    double original_search_latency_ms = 0.0;
    double risk_search_latency_ms = 0.0;
    double total_search_latency_ms = 0.0;
    double risk_original_length_ratio =
      std::numeric_limits < double > ::quiet_NaN();
    bool selection_applied = false;
    std::shared_ptr < const iap::RiskGridSnapshot > snapshot_owner;
  };

  class P4CollisionGuidePlanner
  {
public:
    explicit P4CollisionGuidePlanner(P4GuideSearch & search)
    : search_(search) {
    }

    P4GuideDecision planCollisionGuide(const P4GuideRequest & request);

private:
    P4GuideSearch & search_;
  };

  bool p4GuideDecisionReadyForInjection(
    const P4GuideDecision & decision,
    const P4GuideRequest & expected_request,
    P4GuideDecisionReason * reason = nullptr);

  std::string canonicalP4GuideHash(
    const std::vector < Eigen::Vector3d > &complete_path);

}  // namespace ego_planner

#endif  // BSPLINE_OPT__P4_COLLISION_GUIDE_H_
