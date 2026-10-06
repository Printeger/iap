#include "path_searching/dyn_a_star.h"

#include <chrono>

using namespace std;
using namespace Eigen;

const char* AStar::failureName(const Failure failure) {
    switch (failure) {
    case Failure::NONE: return "NONE";
    case Failure::START_OUT_OF_POOL: return "START_OUT_OF_POOL";
    case Failure::END_OUT_OF_POOL: return "END_OUT_OF_POOL";
    case Failure::START_BLOCKED: return "START_BLOCKED";
    case Failure::END_UNOBSERVED: return "END_UNOBSERVED";
    case Failure::END_OUT_OF_MAP: return "END_OUT_OF_MAP";
    case Failure::END_STALE: return "END_STALE";
    case Failure::MAP_STALE: return "MAP_STALE";
    case Failure::END_BLOCKED: return "END_BLOCKED";
    case Failure::CURRENT_MOTION: return "CURRENT_MOTION";
    case Failure::NO_PATH: return "NO_PATH";
    case Failure::NO_PATH_WITH_UNOBSERVED: return "NO_PATH_WITH_UNOBSERVED";
    case Failure::TIME_BUDGET: return "TIME_BUDGET";
    case Failure::ADVISORY_NO_PATH: return "ADVISORY_NO_PATH";
    case Failure::NO_VALID_REPAIR_ENTRY: return "NO_VALID_REPAIR_ENTRY";
    case Failure::NO_VALID_REPAIR_EXIT: return "NO_VALID_REPAIR_EXIT";
    }
    return "UNKNOWN";
}

GridSearchCell AStar::timedPlanningQuery(const Vector3d& position) {
    const auto started = performance_diagnostics_ ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    ++result_.query_calls;
    auto cell = planning_query_(position);
    recordFirstRejection(position, cell);
    if (performance_diagnostics_) result_.query_management_s += std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    return cell;
}

void AStar::recordFirstRejection(const Vector3d& position, const GridSearchCell& cell) {
    if (!result_.has_first_rejection && (!cell.executable() ||
        (!advisory_fallback_ && (cell.advisory_class == GridAdvisoryClass::AVOID ||
         cell.advisory_class == GridAdvisoryClass::PREDICTED_DEGRADED)))) {
        result_.has_first_rejection = true;
        result_.first_rejection_position = position;
        result_.first_rejection_cell = cell;
    }
}

void AStar::recordCacheStats() {
    const auto stats = grid_map_->planningQueryStats();
    result_.occupancy_query_s = stats.occupancy_s - query_stats_at_start_.occupancy_s;
    result_.clearance_query_s = stats.clearance_s - query_stats_at_start_.clearance_s;
    result_.advisory_query_s = stats.advisory_s - query_stats_at_start_.advisory_s;
    if (advisory_statistics_) {
        const auto advisory = advisory_statistics_();
        result_.advisory_query_s += advisory.advisory_s - advisory_stats_at_start_.advisory_s;
        result_.advisory_query_calls = advisory.queries - advisory_stats_at_start_.queries;
    }
    result_.cache_entries[0] = sample_cache_.size();
    result_.cache_bytes[0] = sample_cache_.size() *
        (sizeof(GridSearchCell) + sizeof(uint64_t) + 2 * sizeof(void*)) +
        sample_cache_.bucket_count() * sizeof(void*);
}

uint64_t AStar::latticeKey(const Vector3i& index) const {
    return (static_cast<uint64_t>(index.x()) * (2 * POOL_SIZE_.y()) +
        index.y()) * (2 * POOL_SIZE_.z()) + index.z();
}

GridSearchCell AStar::querySample(const uint64_t key,
                                 const Vector3d& position, const size_t kind) {
    GridSearchCell cell;
    if (grid_map_->occupancyGeneration() != search_generation_) {
        cell.execution_reason = GridExecutionReason::ENVIRONMENT_STALE;
        return cell;
    }
    auto found = sample_cache_.find(key);
    const bool cached = found != sample_cache_.end();
    if (cached) {
        ++result_.cache_hits;
        ++result_.sample_hits[kind];
        cell = found->second;
    } else {
        ++result_.sample_misses[kind];
        if (planning_query_) cell = timedPlanningQuery(position);
        else cell.execution_reason = grid_map_->getInflateOccupancy(position)
            ? GridExecutionReason::PHYSICAL_OBSTACLE : GridExecutionReason::OK;
        sample_cache_.emplace(key, cell);
    }
    // Spatial physical evidence is frozen. Advisory freshness is checked even
    // on hits; GridMap retains sole ownership of the versioned PL voxel cache.
    if (cached && cell.executable() && advisory_query_) {
        ++result_.advisory_refresh_calls;
        const auto risk = advisory_query_(position);
        cell.advisory_class = risk.classification;
        cell.cost_multiplier = risk.cost_multiplier;
    }
    recordFirstRejection(position, cell);
    return cell;
}

GridSearchCell AStar::queryVoxelCenter(const Vector3d& position) {
    if (!grid_map_->isInMap(position)) return {};
    Vector3i index;
    grid_map_->posToIndex(position, index);
    return querySample((uint64_t{1} << 63) | grid_map_->toAddress(index), position, 0);
}

GridSearchCell AStar::queryLatticePoint(const Vector3i& index) {
    return querySample(latticeKey(2 * index), Index2Coord(index), 1);
}

void AStar::recordMapAtFinish() {
    recordCacheStats();
    result_.live_generation_at_finish = live_generation_provider_
        ? live_generation_provider_() : grid_map_->occupancyGeneration();
    result_.map_changed = result_.live_generation_at_finish !=
        result_.occupancy_generation || map_changed_;
}

void AStar::finishFailure(const Failure failure, const rclcpp::Time& started) {
    // Termination and evidence freshness are independent. In particular an
    // online update must not erase a frozen search's TIME_BUDGET result.
    result_.failure = failure;
    recordMapAtFinish();
    result_.duration_s = (rclcpp::Clock().now() - started).seconds();
    const auto now = std::chrono::steady_clock::now();
    const auto reason_index = static_cast<size_t>(failure);
    if (!reported_failures_[reason_index] || now - last_failure_log_ >= std::chrono::seconds(1)) {
      reported_failures_[reason_index] = true;
      last_failure_log_ = now;
      RCLCPP_WARN(rclcpp::get_logger("AstarSearch"),
        "A* %s start=(%.2f %.2f %.2f) end=(%.2f %.2f %.2f) start_reason=%s end_reason=%s expanded=%zu queried=%zu cached=%zu rejected[out_map=%zu physical=%zu clearance=%zu unobserved=%zu stale=%zu motion_invalid=%zu motion_stale=%zu motion_budget=%zu advisory=%zu] elapsed=%.3fs occupancy=%.3fs clearance=%.3fs PL=%.3fs profile=%d map_changed=%d search_generation=%lu live_generation=%lu",
        failureName(result_.failure), result_.requested_start.x(), result_.requested_start.y(),
        result_.requested_start.z(), result_.requested_end.x(),
        result_.requested_end.y(), result_.requested_end.z(),
        gridExecutionReasonName(result_.start_cell.execution_reason),
        gridExecutionReasonName(result_.end_cell.execution_reason),
        result_.expanded, result_.query_calls, result_.cache_hits,
        result_.rejected_execution[static_cast<size_t>(GridExecutionReason::OUT_OF_MAP)],
        result_.rejected_execution[static_cast<size_t>(GridExecutionReason::PHYSICAL_OBSTACLE)],
        result_.rejected_execution[static_cast<size_t>(GridExecutionReason::INSUFFICIENT_CLEARANCE)],
        result_.rejected_execution[static_cast<size_t>(GridExecutionReason::ENVIRONMENT_UNOBSERVED)],
        result_.rejected_execution[static_cast<size_t>(GridExecutionReason::ENVIRONMENT_STALE)],
        result_.rejected_execution[static_cast<size_t>(GridExecutionReason::CURRENT_MOTION_UNAVAILABLE)],
        result_.rejected_execution[static_cast<size_t>(GridExecutionReason::CURRENT_MOTION_STALE)],
        result_.rejected_execution[static_cast<size_t>(GridExecutionReason::CURRENT_MOTION_BUDGET)],
        result_.rejected_advisory, result_.duration_s,
        result_.occupancy_query_s, result_.clearance_query_s,
        result_.advisory_query_s, result_.performance_diagnostics, result_.map_changed,
        static_cast<unsigned long>(result_.occupancy_generation),
        static_cast<unsigned long>(result_.live_generation_at_finish));
    }
    if (failure_observer_) failure_observer_(result_);
}

std::optional<double> AStar::edgeMultiplier(const Vector3d& from,
                                            const Vector3d& to,
                                            const Vector3i& from_index,
                                            const Vector3i& to_index)
{
    if (grid_map_->occupancyGeneration() != search_generation_) {
        map_changed_ = true;
        return std::nullopt;
    }
    if (!planning_query_) {
        return checkOccupancy(to) ? std::nullopt : std::optional<double>(1.0);
    }
    // Traverse the same GridMap voxel lattice as the physical and PL layers.
    RayCaster ray;
    const auto origin = grid_map_->getOrigin();
    const double resolution = grid_map_->getResolution();
    const auto begin = (from - origin) / resolution;
    const auto end = (to - origin) / resolution;
    const Vector3i first_voxel = begin.array().floor().cast<int>();
    const Vector3i last_voxel = end.array().floor().cast<int>();
    double multiplier = 1.0;
    auto examine = [&](const GridSearchCell& cell) {
        if (!cell.executable()) {
            ++result_.rejected_execution[static_cast<size_t>(cell.execution_reason)];
            if (cell.execution_reason == GridExecutionReason::ENVIRONMENT_STALE)
                map_changed_ = true;
            return false;
        }
        const auto cls = cell.advisory_class;
        if (cls == GridAdvisoryClass::AVOID ||
            cls == GridAdvisoryClass::PREDICTED_DEGRADED) {
            if (!advisory_fallback_) {
                rejected_advisory_ = true;
                ++result_.rejected_advisory;
                return false;
            }
            multiplier = std::max(multiplier, 3.0);
        } else {
            multiplier = std::max(multiplier, cell.cost_multiplier);
        }
        return true;
    };
    if (ray.setInput(begin, end)) {
        Vector3d voxel;
        do {
            const bool more = ray.step(voxel);
            const Vector3i index = voxel.cast<int>();
            // The centre of an endpoint voxel may be closer to an obstacle
            // than the actual search edge. Check the exact edge endpoints and
            // midpoint below, and retain centre checks for interior voxels.
            if (index != first_voxel && index != last_voxel &&
                !examine(queryVoxelCenter(
                    origin + (voxel.array() + 0.5).matrix() * resolution)))
                return std::nullopt;
            if (!more) break;
        } while (true);
    }
    if (!examine(queryLatticePoint(from_index))) return std::nullopt;
    const Vector3i mid_key = from_index + to_index;
    // Even half-lattice coordinates are exactly node samples; odd coordinates
    // are midpoints. Voxel centers use a separate key namespace. Arbitrary
    // requested endpoints and connectors never use these keys.
    const Vector3d midpoint = (mid_key.x() % 2 == 0 && mid_key.y() % 2 == 0 && mid_key.z() % 2 == 0)
        ? Index2Coord(mid_key / 2) : Vector3d((from + to) / 2.0);
    if (!examine(querySample(latticeKey(mid_key), midpoint, 2))) return std::nullopt;
    if (!examine(queryLatticePoint(to_index))) return std::nullopt;
    return multiplier;
}

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++) {
        for (int j = 0; j < POOL_SIZE_(1); j++) {
            for (int k = 0; k < POOL_SIZE_(2); k++) delete GridNodeMap_[i][j][k];
            delete[] GridNodeMap_[i][j];
        }
        delete[] GridNodeMap_[i];
    }
    delete[] GridNodeMap_;
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    grid_map_ = occ_map;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx)) {
        result_.failure = Failure::START_OUT_OF_POOL;
        return false;
    }
    if (!Coord2Index(end_pt, end_idx)) {
        result_.failure = Failure::END_OUT_OF_POOL;
        return false;
    }

    result_.start_cell = planning_query_ ? timedPlanningQuery(start_pt)
        : GridSearchCell{};
    result_.end_cell = planning_query_ ? timedPlanningQuery(end_pt)
        : GridSearchCell{};
    if (!planning_query_) {
        result_.start_cell.execution_reason = queryLatticePoint(start_idx).execution_reason;
        result_.end_cell.execution_reason = queryLatticePoint(end_idx).execution_reason;
    }
    if (!result_.start_cell.executable()) {
        result_.failure = Failure::START_BLOCKED;
        return false;
    }
    if (!result_.end_cell.executable()) {
        const auto reason = result_.end_cell.execution_reason;
        result_.failure = reason == GridExecutionReason::ENVIRONMENT_UNOBSERVED
            ? Failure::END_UNOBSERVED
            : reason == GridExecutionReason::OUT_OF_MAP
                ? Failure::END_OUT_OF_MAP
            : reason == GridExecutionReason::ENVIRONMENT_STALE
                ? Failure::END_STALE
                : reason == GridExecutionReason::CURRENT_MOTION_UNAVAILABLE ||
                  reason == GridExecutionReason::CURRENT_MOTION_STALE ||
                  reason == GridExecutionReason::CURRENT_MOTION_BUDGET
                    ? Failure::CURRENT_MOTION : Failure::END_BLOCKED;
        return false;
    }

    if (!queryLatticePoint(start_idx).executable()) {
        result_.failure = Failure::START_BLOCKED;
        return false;
    }
    if (!queryLatticePoint(end_idx).executable()) {
        // Rounding to the search lattice can place an otherwise valid target
        // in a blocked cell. Only accept a nearby lattice endpoint when the
        // connector back to the *requested* endpoint is fully executable.
        bool adjusted = false;
        const Vector3d direction = (start_pt - end_pt).normalized();
        if (direction.allFinite()) {
            for (double shift = step_size_; shift <= 1.0 + 1e-9;
                 shift += step_size_) {
                Vector3i candidate_index;
                const auto candidate = end_pt + direction * shift;
                if (!Coord2Index(candidate, candidate_index)) break;
                if (!queryLatticePoint(candidate_index).executable()) continue;
                const auto candidate_position = Index2Coord(candidate_index);
                const int samples = std::max(1, static_cast<int>(std::ceil(
                    (candidate_position - end_pt).norm() /
                    (grid_map_->getResolution() * 0.5))));
                bool connected = true;
                for (int i = 0; i <= samples; ++i) {
                    const auto p = end_pt + (candidate_position - end_pt) *
                        (static_cast<double>(i) / samples);
                    if (planning_query_ ? !timedPlanningQuery(p).executable()
                                        : grid_map_->getInflateOccupancy(p) != 0) {
                        connected = false;
                        break;
                    }
                }
                if (connected) {
                    end_idx = candidate_index;
                    adjusted = true;
                    break;
                }
            }
        }
        if (!adjusted) {
            result_.failure = Failure::END_BLOCKED;
            return false;
        }
    }

    const auto connector_ok = [this](const Vector3d& a, const Vector3d& b) {
        if (!planning_query_) return true;
        const int samples = std::max(1, static_cast<int>(std::ceil(
            (a - b).norm() / (grid_map_->getResolution() * 0.5))));
        for (int i = 0; i <= samples; ++i) {
            const auto point = a + (b - a) * (static_cast<double>(i) / samples);
            const auto cell = timedPlanningQuery(point);
            if (!cell.executable()) return false;
            const auto cls = cell.advisory_class;
            if (!advisory_fallback_ &&
                (cls == GridAdvisoryClass::AVOID ||
                 cls == GridAdvisoryClass::PREDICTED_DEGRADED)) {
                rejected_advisory_ = true;
                ++result_.rejected_advisory;
                return false;
            }
        }
        return true;
    };
    if (!connector_ok(start_pt, Index2Coord(start_idx))) {
        result_.failure = rejected_advisory_ ? Failure::ADVISORY_NO_PATH :
            Failure::START_BLOCKED;
        return false;
    }
    if (!connector_ok(Index2Coord(end_idx), end_pt)) {
        result_.failure = rejected_advisory_ ? Failure::ADVISORY_NO_PATH :
            Failure::END_BLOCKED;
        return false;
    }
    return true;
}

bool AStar::AstarSearch(const double step_size, Vector3d start_pt,
                        Vector3d end_pt, const double max_duration_s,
                        std::optional<Vector3d> center_override)
{
    rclcpp::Time time_1 = rclcpp::Clock().now();
    ++rounds_;
    rejected_advisory_ = false;
    map_changed_ = false;
    gridPath_.clear();
    sample_cache_.clear();
    result_ = Result{};
    result_.performance_diagnostics = performance_diagnostics_;
    result_.requested_start = start_pt;
    result_.requested_end = end_pt;
    query_stats_at_start_ = grid_map_->planningQueryStats();
    advisory_stats_at_start_ = advisory_statistics_ ? advisory_statistics_() : GridPlanningQueryStats{};
    search_generation_ = grid_map_->occupancyGeneration();
    result_.occupancy_generation = search_generation_;

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = center_override.value_or((start_pt + end_pt) / 2);
    result_.step_size_m = step_size;
    result_.pool_dimensions = POOL_SIZE_;
    result_.pool_center = center_;

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        finishFailure(result_.failure, time_1);
        return false;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<AStarQueueEntry, std::vector<AStarQueueEntry>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    endPtr->index = end_idx;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    ++result_.queue_pushes;
    openSet_.push({startPtr, startPtr->fScore});

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        if (grid_map_->occupancyGeneration() != search_generation_) {
            finishFailure(Failure::MAP_STALE, time_1);
            return false;
        }
        num_iter++;
        const auto entry = openSet_.top();
        openSet_.pop();
        ++result_.queue_pops;
        current = entry.node;
        if (current->state == GridNode::CLOSEDSET ||
            entry.score > current->fScore + 1e-9) continue;

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath_ = retrievePath(current);
            result_.path_cost = current->gScore;
            recordMapAtFinish();
            result_.duration_s = (rclcpp::Clock().now() - time_1).seconds();
            RCLCPP_DEBUG(rclcpp::get_logger("AstarSearch"),
                "A* path expanded=%zu queries=%zu cached=%zu elapsed=%.3fs occupancy=%.3fs clearance=%.3fs PL=%.3fs map_changed=%d search_generation=%lu live_generation=%lu",
                result_.expanded, result_.query_calls, result_.cache_hits,
                result_.duration_s, result_.occupancy_query_s,
                result_.clearance_query_s, result_.advisory_query_s,
                result_.map_changed,
                static_cast<unsigned long>(result_.occupancy_generation),
                static_cast<unsigned long>(result_.live_generation_at_finish));
            return true;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.
        ++result_.expanded;

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    Vector3i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;
                    neighborIdx(1) = (current->index)(1) + dy;
                    neighborIdx(2) = (current->index)(2) + dz;

                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                    {
                        continue;
                    }

                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                    neighborPtr->index = neighborIdx;

                    bool flag_explored = neighborPtr->rounds == rounds_;

                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    {
                        continue; //in closed set.
                    }

                    const auto edge_started = performance_diagnostics_ ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
                    const auto multiplier = edgeMultiplier(
                        Index2Coord(current->index), Index2Coord(neighborPtr->index),
                        current->index,
                        neighborIdx);
                    if (performance_diagnostics_) result_.edge_s += std::chrono::duration<double>(std::chrono::steady_clock::now() - edge_started).count();
                    if (!multiplier) {
                        if (map_changed_) {
                            finishFailure(Failure::MAP_STALE, time_1);
                            return false;
                        }
                        continue;
                    }

                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    tentative_gScore = current->gScore + static_cost * *multiplier;

                    if (!flag_explored)
                    {
                        // A rejected incoming edge does not discover a node.
                        // Otherwise a later legal edge can inherit scores or
                        // CLOSED state from a previous search round.
                        neighborPtr->rounds = rounds_;
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        ++result_.queue_pushes;
                        openSet_.push({neighborPtr, neighborPtr->fScore});
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    { //in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        ++result_.queue_pushes;
                        openSet_.push({neighborPtr, neighborPtr->fScore});
                    }
                }
        rclcpp::Time time_2 = rclcpp::Clock().now();
        const double limit = max_duration_s >= 0.0 ? max_duration_s :
            (planning_query_ ? 1.0 : 0.2);
        if ((time_2 - time_1).seconds() > limit)
        {
            finishFailure(Failure::TIME_BUDGET, time_1);
            return false;
        }
    }

    rclcpp::Time time_2 = rclcpp::Clock().now();

    if ((time_2 - time_1).seconds() > 0.1)
        RCLCPP_WARN(rclcpp::get_logger("AstarSearch"),
                    "Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).seconds(), num_iter);

    const bool unknown_rejected = result_.rejected_execution[
        static_cast<size_t>(GridExecutionReason::ENVIRONMENT_UNOBSERVED)] != 0;
    finishFailure(unknown_rejected ? Failure::NO_PATH_WITH_UNOBSERVED :
        result_.rejected_advisory != 0 && !advisory_fallback_
            ? Failure::ADVISORY_NO_PATH : Failure::NO_PATH, time_1);
    return false;
}

vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    if (!path.empty() && (path.front() - result_.requested_start).norm() > 1e-8)
        path.insert(path.begin(), result_.requested_start);
    if (!path.empty() && (path.back() - result_.requested_end).norm() > 1e-8)
        path.push_back(result_.requested_end);
    return path;
}
