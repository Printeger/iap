#include <plan_env/planning_budget.h>
#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>
#include <functional>
#include <optional>
#include <unordered_map>
#include <array>
#include <limits>
#include <chrono>

constexpr double inf = std::numeric_limits<double>::infinity();
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

struct AStarQueueEntry { GridNodePtr node; double score; };
struct NodeComparator {
	bool operator()(const AStarQueueEntry& a, const AStarQueueEntry& b) const {
		return a.score > b.score;
	}
};

class AStar
{
public:
	enum class Failure {
		NONE, START_OUT_OF_POOL, END_OUT_OF_POOL, START_BLOCKED,
		END_UNOBSERVED, END_STALE, MAP_STALE, END_BLOCKED, CURRENT_MOTION,
		NO_PATH, NO_PATH_WITH_UNOBSERVED, TIME_BUDGET, ADVISORY_NO_PATH,
		END_OUT_OF_MAP, NO_VALID_REPAIR_ENTRY, NO_VALID_REPAIR_EXIT
	};
	struct Result {
		struct GoalEvidence { bool checked=false, eligible=false, reached=false; Eigen::Vector3d lattice=Eigen::Vector3d::Constant(NAN); Failure failure=Failure::NONE; };
		std::vector<GoalEvidence> goals; // Opt-in diagnostics; never affects search ordering.
		double endpoint_prepare_s=0., first_complete_path_s=NAN, heuristic_s=0., queue_s=0.;
		size_t edge_checks=0, segment_integrals=0;
		Failure failure = Failure::NONE;
		Eigen::Vector3d requested_start = Eigen::Vector3d::Zero();
		Eigen::Vector3d requested_end = Eigen::Vector3d::Zero();
        Eigen::Vector3d start_lattice = Eigen::Vector3d::Constant(NAN);
        Eigen::Vector3d end_lattice = Eigen::Vector3d::Constant(NAN);
        bool start_attachment_recovered = false;
		GridSearchCell start_cell, end_cell;
		bool has_first_rejection = false;
		Eigen::Vector3d first_rejection_position = Eigen::Vector3d::Zero();
		GridSearchCell first_rejection_cell;
		std::array<size_t, 10> rejected_execution{};
		size_t rejected_advisory = 0;
		size_t expanded = 0;
		bool exhausted = false;
		size_t selected_goal = std::numeric_limits<size_t>::max();
		size_t query_calls = 0;
		size_t cache_hits = 0;
		size_t queue_pushes = 0, queue_pops = 0, advisory_refresh_calls = 0;
		size_t advisory_query_calls = 0, risk_integration_calls = 0;
        size_t lower_bound_pruned_edges = 0;
		std::array<size_t, 3> sample_hits{}, sample_misses{}, cache_entries{}, cache_bytes{};
		double path_cost = 0.0; // Complete objective in metres.
        double path_length_m=0., risk_cost_m=0., terminal_cost_m=0.;
        bool optimality_proven=false, search_budget_exhausted=false, advisory_changed=false;
		double query_management_s = 0.0, edge_s = 0.0;
		double duration_s = 0.0;
		double occupancy_query_s = 0.0;
		double clearance_query_s = 0.0;
		double advisory_query_s = 0.0;
		double step_size_m = 0.0;
		Eigen::Vector3i pool_dimensions = Eigen::Vector3i::Zero();
		Eigen::Vector3d pool_center = Eigen::Vector3d::Zero();
		uint64_t occupancy_generation = 0;
		uint64_t live_generation_at_finish = 0;
		bool map_changed = false;
		bool performance_diagnostics = false;
	};
	// Offline component evidence uses the production traversal and edge authority.
	// Never installed by the online planner; it cannot authorize a trajectory.
	struct ComponentDiagnostic {
		struct Goal { bool eligible=false, reached=false; Eigen::Vector3d lattice=Eigen::Vector3d::Constant(NAN); Failure failure=Failure::NONE; };
		bool stop_on_first_goal=false;
		std::vector<Goal> goals;
		std::function<void(const Eigen::Vector3d&, const Eigen::Vector3d&)> visit;
		std::function<void(const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&, const char*)> boundary;
	};
private:
	ComponentDiagnostic* component_diagnostic_ = nullptr;
	GridMap::Ptr grid_map_;
    PlanningBudget::Ptr budget_;
    std::optional<Eigen::Vector3d> task_goal_;
    bool active_search_=false;
    PlanningBudget::Clock::time_point search_started_;
    PlanningBudget::Clock::time_point search_deadline_ = PlanningBudget::Clock::time_point::max();
    bool deadlineExpired() const { return PlanningBudget::Clock::now() >= search_deadline_ ||
        (budget_ && budget_->expired()); }
    std::shared_ptr<const FrozenOccupancyEpoch> frozen_epoch_;
	std::function<GridSearchCell(const Eigen::Vector3d&)> planning_query_;
	bool advisory_fallback_ = false;
	bool rejected_advisory_ = false;
	bool map_changed_ = false;
	Result result_;
	GridPlanningQueryStats query_stats_at_start_, advisory_stats_at_start_;
	bool performance_diagnostics_ = false;
	std::array<bool, 16> reported_failures_{};
	std::chrono::steady_clock::time_point last_failure_log_{};
	std::unordered_map<uint64_t, GridSearchCell> sample_cache_;
	std::function<GridPlanningRisk(const Eigen::Vector3d&)> advisory_query_;
    std::function<GridPlanningQueryStats()> advisory_statistics_;
    void recordFirstRejection(const Eigen::Vector3d& position, const GridSearchCell& cell);
	GridSearchCell querySample(uint64_t key, const Eigen::Vector3d& position, size_t kind);
	uint64_t latticeKey(const Eigen::Vector3i& doubled_index) const;
	uint64_t search_generation_ = 0;
	std::function<void(const Result&)> failure_observer_;
	std::function<uint64_t()> live_generation_provider_;
	GridSearchCell queryVoxelCenter(const Eigen::Vector3d& position);
	GridSearchCell queryLatticePoint(const Eigen::Vector3i& index);
	GridSearchCell timedPlanningQuery(const Eigen::Vector3d& position);
	void finishFailure(Failure failure, const rclcpp::Time& started);
	void recordMapAtFinish();
	void recordCacheStats();
	std::optional<double> segmentCost(const Eigen::Vector3d& from,
	                                  const Eigen::Vector3d& to, bool check_physical,
	                                  const std::array<GridSearchCell,3>* checked_samples = nullptr);
	std::optional<double> edgeCost(const Eigen::Vector3d& from,
	                                     const Eigen::Vector3d& to,
	                                     const Eigen::Vector3i& from_index,
	                                     const Eigen::Vector3i& to_index);

	inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
	double getManhHeu(GridNodePtr node1, GridNodePtr node2);
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);

	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

	inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
	inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	inline bool checkOccupancy(const Eigen::Vector3d &pos) {
		if (!planning_query_) return (bool)grid_map_->getInflateOccupancy(pos);
		const auto cell = planning_query_(pos);
		if (!cell.executable()) return true;
		return false;
	}

	std::vector<GridNodePtr> retrievePath(GridNodePtr current);

	double step_size_, inv_step_size_;
	Eigen::Vector3d center_;
	Eigen::Vector3i CENTER_IDX_, POOL_SIZE_ = Eigen::Vector3i::Zero();


	std::vector<GridNodePtr> gridPath_;

	GridNodePtr ***GridNodeMap_ = nullptr;
	std::priority_queue<AStarQueueEntry, std::vector<AStarQueueEntry>, NodeComparator> openSet_;

	int rounds_{0};

public:
    // Read-only offline attribution on a fresh checker. Uses the production
    // segment quadrature and physical queries; never authorizes execution.
    std::optional<double> diagnosticSegmentCost(const Eigen::Vector3d& from,
                                               const Eigen::Vector3d& to) {
        return segmentCost(from, to, true);
    }
	void setComponentDiagnostic(ComponentDiagnostic* diagnostic) { component_diagnostic_ = diagnostic; }
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);
	void setFrozenEpoch(std::shared_ptr<const FrozenOccupancyEpoch> epoch) { frozen_epoch_ = std::move(epoch); }
	void setSearchMap(GridMap::Ptr frozen_map) { grid_map_ = std::move(frozen_map); }
	void setLiveGenerationProvider(std::function<uint64_t()> provider) {
		live_generation_provider_ = std::move(provider);
	}
	void setPlanningQuery(std::function<GridSearchCell(const Eigen::Vector3d&)> query,
	                      bool advisory_fallback = false) {
		planning_query_ = std::move(query);
		advisory_fallback_ = advisory_fallback;
	}
	void setAdvisoryQuery(std::function<GridPlanningRisk(const Eigen::Vector3d&)> query,
                          std::function<GridPlanningQueryStats()> statistics = {}) {
        advisory_query_ = std::move(query);
        advisory_statistics_ = std::move(statistics);
    }
	void setPerformanceDiagnostics(bool enabled) { performance_diagnostics_ = enabled; }
	bool rejectedAdvisory() const { return rejected_advisory_; }
	const Result& lastResult() const { return result_; }
	void clearLastResult() { result_ = Result{}; }
	void recordPresearchFailure(Failure failure, const Eigen::Vector3d& start,
	                            const Eigen::Vector3d& end,
                                std::optional<GridSearchCell> start_cell = {}) {
		sample_cache_.clear();
		query_stats_at_start_ = grid_map_->planningQueryStats();
        advisory_stats_at_start_ = advisory_statistics_ ? advisory_statistics_() : GridPlanningQueryStats{};
		result_ = Result{};
		result_.failure = failure;
		result_.performance_diagnostics = performance_diagnostics_;
		result_.requested_start = start;
		result_.requested_end = end;
        if(start_cell) {
            result_.start_cell=*start_cell;
            if(!start_cell->executable()) {
                result_.first_rejection_position=start;
                result_.first_rejection_cell=*start_cell;
                result_.has_first_rejection=true;
            }
        }
		result_.step_size_m = 0.1;
		result_.pool_dimensions = POOL_SIZE_;
		result_.pool_center = (start + end) / 2.0;
		result_.occupancy_generation = grid_map_->occupancyGeneration();
		recordMapAtFinish();
	}
	void setFailureObserver(std::function<void(const Result&)> observer) {
		failure_observer_ = std::move(observer);
	}
	static const char* failureName(Failure failure);

	void setTaskGoal(std::optional<Eigen::Vector3d> goal) { task_goal_=std::move(goal); }
	void setPlanningBudget(PlanningBudget::Ptr budget) { budget_ = std::move(budget); }
	bool AstarSearch(const double step_size, Eigen::Vector3d start_pt,
	                 Eigen::Vector3d end_pt, double max_duration_s = -1.0,
	                 std::optional<Eigen::Vector3d> center_override = std::nullopt);
	bool AstarSearchGoals(double step_size, const Eigen::Vector3d& start,
	                     const std::vector<Eigen::Vector3d>& goals,
	                     double max_duration_s = -1.0,
	                     std::optional<Eigen::Vector3d> center_override = std::nullopt);

    enum class GoalSearchPurpose { Guide, CostProof };
    bool AstarSearchGoals(double step_size, const Eigen::Vector3d& start,
                         const std::vector<Eigen::Vector3d>& goals, double max_duration_s,
                         std::optional<Eigen::Vector3d> center_override, GoalSearchPurpose purpose);

	std::vector<Eigen::Vector3d> getPath();
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	return step_size_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const
{
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
	// Nearest lattice index must be symmetric around the pool center. Casting
	// negative fractions truncates toward zero and can shift a legal start by
	// an extra full step toward an obstacle.
	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5))
	          .array().floor().cast<int>().matrix() + CENTER_IDX_;

	if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
	{
		return false;
	}

	return true;
};

#endif
