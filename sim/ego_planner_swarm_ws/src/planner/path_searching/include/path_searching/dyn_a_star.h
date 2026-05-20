#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <functional>
#include <queue>
#include <string>

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

	double step_size_, inv_step_size_;
	Eigen::Vector3d center_;
	Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

		std::vector<GridNodePtr> gridPath_;

		GridNodePtr ***GridNodeMap_;
		std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

			int rounds_{0};
			bool use_integrity_cost_{false};
			bool use_grid_map_risk_overlay_{false};
			double lambda_integrity_cost_{0.0};
			double integrity_cost_max_{10.0};
			double search_time_limit_s_{0.2};
			std::function<bool(const Eigen::Vector3d &, double *)> integrity_cost_query_;
			std::shared_ptr<const RiskOverlaySnapshot> active_risk_overlay_snapshot_;
			std::shared_ptr<const RiskOverlaySnapshot> pinned_risk_overlay_snapshot_;
		int last_integrity_samples_used_{0};
		int last_integrity_samples_skipped_{0};
		int last_integrity_query_hit_count_{0};
		int last_integrity_query_unknown_count_{0};
		int last_integrity_query_stale_count_{0};
		double last_integrity_cost_sum_{0.0};
		double last_integrity_cost_max_{0.0};
		std::string last_risk_source_{"off"};
		int last_risk_overlay_generation_{-1};

		bool AstarSearchImpl(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt,
		                     bool use_integrity_cost, double search_time_limit_s);

	public:
		typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

		bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
		bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt,
		                 bool use_integrity_cost, double search_time_limit_s);

			void setIntegrityCostCallback(std::function<bool(const Eigen::Vector3d &, double *)> query);
			void setIntegrityCostParams(bool enabled, double lambda, double cost_max);
			void setGridMapRiskOverlayEnabled(bool enabled) { use_grid_map_risk_overlay_ = enabled; }
			void pinRiskOverlaySnapshot(std::shared_ptr<const RiskOverlaySnapshot> snapshot);
			void clearPinnedRiskOverlaySnapshot();
			bool integrityCostEnabled() const { return use_integrity_cost_; }
		int getLastIntegritySamplesUsed() const { return last_integrity_samples_used_; }
		int getLastIntegritySamplesSkipped() const { return last_integrity_samples_skipped_; }
		int getLastIntegrityQueryHitCount() const { return last_integrity_query_hit_count_; }
		int getLastIntegrityQueryUnknownCount() const { return last_integrity_query_unknown_count_; }
		int getLastIntegrityQueryStaleCount() const { return last_integrity_query_stale_count_; }
		double getLastIntegrityCostMean() const
		{
			return last_integrity_samples_used_ > 0 ? last_integrity_cost_sum_ / last_integrity_samples_used_ : 0.0;
		}
		double getLastIntegrityCostMax() const { return last_integrity_cost_max_; }
		const std::string &getLastRiskSource() const { return last_risk_source_; }
		int getLastRiskOverlayGeneration() const { return last_risk_overlay_generation_; }

		std::vector<Eigen::Vector3d> getPath();
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
