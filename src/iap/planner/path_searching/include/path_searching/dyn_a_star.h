#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <cmath>
#include <cstdint>
#include <memory>
#include <queue>
#include <string>
#include <iap/planner/risk_grid_map.hpp>

enum class P4RiskObjective
{
	LEGACY_INTEGRAL_V1 = 0,
	PROVIDER_BOTTLENECK_V2,
};

struct P4V2LexicographicCost
{
	double bottleneck = 0.0;
	double integral = 0.0;
	double path_length = 0.0;
};

bool p4V2CostLess(const P4V2LexicographicCost &lhs,
				  const P4V2LexicographicCost &rhs);

struct P4RiskAStarConfig
{
	bool enable_risk_aware_astar = false;
	bool metrics_only = false;
	P4RiskObjective objective = P4RiskObjective::LEGACY_INTEGRAL_V1;
	double lambda_p4_risk = 0.05;
	double risk_cost_max = 100.0;
	double unknown_edge_penalty = 1.0;
	double max_extra_path_ratio = 1.3;
	bool fallback_to_original_when_risk_not_ready = true;
	bool debug_csv_enable = false;
	std::string debug_csv_path;
	bool profile_trace_enable = false;
	std::string profile_trace_path;
	double query_speed_mps = 1.0;
	iap::RiskCostQueryPolicy cost_query_policy =
		iap::RiskCostQueryPolicy::LEGACY_STRICT;
};

struct P4AStarMetrics
{
	double original_path_length = 0.0;
	double risk_path_length = 0.0;
	double path_length_ratio = 0.0;
	int risk_query_count = 0;
	int unknown_count = 0;
	int occupied_reject_count = 0;
	int expanded_nodes = 0;
	uint64_t snapshot_generation_id = 0;
	std::string fallback_reason = "not_evaluated";
	double elapsed_ms = 0.0;
	double path_mean_cost = 0.0;
	double path_max_cost = 0.0;
	bool risk_enabled = false;
	double provider_bottleneck = 0.0;
	double provider_integral = 0.0;
	int provider_incomplete_reject_count = 0;
	int time_state_count = 0;
};

constexpr double inf = 1 >> 20;
struct GridNode;
typedef GridNode *GridNodePtr;

struct GridNode
{
	enum enum_state
	{
		OPENSET = 1,
		CLOSEDSET = 2,
		UNDEFINED = 3
	};

	int rounds{0}; // Distinguish every call
	enum enum_state state
	{
		UNDEFINED
	};
	Eigen::Vector3i index;

	double gScore{inf}, fScore{inf};
	double travelDistance{0.0};
	GridNodePtr cameFrom{NULL};
};

class NodeComparator
{
public:
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->fScore > node2->fScore;
	}
};

class AStar
{
private:
	GridMap::Ptr grid_map_;

	inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
	double getManhHeu(GridNodePtr node1, GridNodePtr node2);
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);

	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

	inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
	inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	inline bool checkOccupancy(const Eigen::Vector3d &pos) { return (bool)grid_map_->getInflateOccupancy(pos); }

	std::vector<GridNodePtr> retrievePath(GridNodePtr current);
	bool astarSearchImpl(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool use_risk);
	bool astarSearchProviderBottleneckV2(const double step_size,
		Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
	bool queryProviderRiskForV2Edge(const Eigen::Vector3d &current_pos,
		const Eigen::Vector3d &next_pos, double travel_distance_m,
		double *provider_cost);
	double edgeCostWithRisk(const Eigen::Vector3d &current_pos, const Eigen::Vector3d &neighbor_pos, double geometric_cost, double query_time_s);
	double queryTimeForEdge(const GridNodePtr current, double geometric_cost) const;

	double step_size_{0.0}, inv_step_size_{0.0};
	Eigen::Vector3d center_;
	Eigen::Vector3d search_start_pt_ = Eigen::Vector3d::Zero();
	Eigen::Vector3i CENTER_IDX_ = Eigen::Vector3i::Zero(), POOL_SIZE_ = Eigen::Vector3i::Zero();
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	std::vector<GridNodePtr> gridPath_;
	std::vector<Eigen::Vector3d> v2_path_;

	GridNodePtr ***GridNodeMap_{nullptr};
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

	int rounds_{0};
	P4RiskAStarConfig p4_config_;
	P4AStarMetrics last_p4_metrics_;
	std::shared_ptr<const iap::RiskGridSnapshot> risk_snapshot_;
	double risk_query_base_time_s_{0.0};
	double p4_valid_cost_sum_{0.0};
	int p4_valid_cost_count_{0};
	double p4_v2_reference_path_length_m_{0.0};

public:
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

	bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
	bool AstarSearchOriginal(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
	bool AstarSearchRiskAware(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

	std::vector<Eigen::Vector3d> getPath();
	void setP4Config(const P4RiskAStarConfig &config) { p4_config_ = config; }
	const P4RiskAStarConfig &getP4Config() const { return p4_config_; }
	void setRiskSnapshot(std::shared_ptr<const iap::RiskGridSnapshot> snapshot, double query_base_time_s);
	void clearRiskSnapshot();
	bool hasRiskSnapshot() const { return static_cast<bool>(risk_snapshot_); }
	const P4AStarMetrics &getLastP4Metrics() const { return last_p4_metrics_; }
	void recordP4GuideMetrics(const P4AStarMetrics &metrics) { last_p4_metrics_ = metrics; }
	void setP4V2ReferencePathLength(double length_m)
	{
		p4_v2_reference_path_length_m_ = length_m;
	}
	double edgeCostWithRiskForTest(const Eigen::Vector3d &current_pos, const Eigen::Vector3d &neighbor_pos,
	                               double geometric_cost, double query_time_s)
	{
		return edgeCostWithRisk(current_pos, neighbor_pos, geometric_cost, query_time_s);
	}
	bool isOccupiedForTest(const Eigen::Vector3d &pos) { return checkOccupancy(pos); }
	bool queryProviderRiskForV2EdgeForTest(
		const Eigen::Vector3d &current_pos, const Eigen::Vector3d &next_pos,
		double travel_distance_m, double *provider_cost)
	{
		return queryProviderRiskForV2Edge(
			current_pos, next_pos, travel_distance_m, provider_cost);
	}
	double queryTimeFromCumulativeDistanceForTest(double distance_m) const
	{
		const double speed = std::isfinite(p4_config_.query_speed_mps) && p4_config_.query_speed_mps > 1.0e-3
		                         ? p4_config_.query_speed_mps
		                         : 1.0;
		return risk_query_base_time_s_ + distance_m / speed;
	}
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	return tie_breaker_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const
{
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

	if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
	{
		RCLCPP_ERROR(rclcpp::get_logger("Coord2Index"), "Ran out of pool, index=%d %d %d, POOL_SIZE=%d %d %d", idx(0), idx(1), idx(2),POOL_SIZE_(0), POOL_SIZE_(1), POOL_SIZE_(2));
		return false;
	}

	return true;
};

#endif
