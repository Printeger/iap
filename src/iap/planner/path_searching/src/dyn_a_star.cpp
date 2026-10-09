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
    if (deadlineExpired()) { GridSearchCell cell; cell.execution_reason=GridExecutionReason::ENVIRONMENT_STALE; return cell; }
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
    // Frozen samples use immutable geometry. The expanded-node boundary checks
    // live geometry/deadline; actual callbacks below also check the deadline.
    if (!frozen_epoch_ && grid_map_->occupancyGeneration() != search_generation_) {
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
        if(cell.advisory_class!=risk.classification || std::abs(cell.cost_multiplier-risk.cost_multiplier)>1e-12)
            result_.advisory_changed=true;
        cell.advisory_class = risk.classification;
        cell.cost_multiplier = risk.cost_multiplier;
    }
    recordFirstRejection(position, cell);
    return cell;
}

GridSearchCell AStar::queryVoxelCenter(const Vector3d& position) {
    Vector3i index;
    int address;
    if (frozen_epoch_) {
        const auto relative=position-frozen_epoch_->lattice_origin;
        if ((relative.array()<1e-4).any() ||
            (relative.array()>frozen_epoch_->extent_m.array()-1e-4).any()) return {};
        index=(relative*frozen_epoch_->resolution_inv).array().floor().cast<int>();
        if ((index.array()<0).any() || (index.array()>=frozen_epoch_->voxel_dimensions.array()).any()) return {};
        address=(index.x()*frozen_epoch_->voxel_dimensions.y()+index.y())*frozen_epoch_->voxel_dimensions.z()+index.z();
    } else {
        if (!grid_map_->isInMap(position)) return {};
        grid_map_->posToIndex(position,index); address=grid_map_->toAddress(index);
    }
    return querySample((uint64_t{1} << 63) | address,position,0);
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
    if (active_search_ && budget_) {
      auto& totals=budget_->searches;
      ++totals.calls; totals.seconds+=std::chrono::duration<double>(PlanningBudget::Clock::now()-search_started_).count();
      totals.queries+=result_.query_calls; totals.expanded+=result_.expanded;
      totals.pushes+=result_.queue_pushes; totals.pops+=result_.queue_pops;
      for(size_t i=0;i<3;++i) {
        totals.hits[i]+=result_.sample_hits[i]; totals.misses[i]+=result_.sample_misses[i];
        totals.peak_cache_bytes[i]=std::max(totals.peak_cache_bytes[i],result_.cache_bytes[i]);
      }
    }
    active_search_=false;

}

void AStar::finishFailure(const Failure failure, const rclcpp::Time& started) {
    // Termination and evidence freshness are independent. In particular an
    // online update must not erase a frozen search's TIME_BUDGET result.
    result_.failure = deadlineExpired() ? Failure::TIME_BUDGET : failure;
    recordMapAtFinish();
    result_.duration_s = std::chrono::duration<double>(PlanningBudget::Clock::now()-search_started_).count();
    const auto now = std::chrono::steady_clock::now();
    const auto reason_index = static_cast<size_t>(result_.failure);
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

std::optional<double> AStar::edgeCost(const Vector3d& from,
                                            const Vector3d& to,
                                            const Vector3i& from_index,
                                            const Vector3i& to_index)
{
    ++result_.edge_checks;
    if (!frozen_epoch_ && grid_map_->occupancyGeneration() != search_generation_) {
        map_changed_ = true;
        return std::nullopt;
    }
    if (!planning_query_) {
        return checkOccupancy(to) ? std::nullopt : std::optional<double>((to-from).norm());
    }
    // Traverse the same GridMap voxel lattice as the physical and PL layers.
    RayCaster ray;
    const auto origin = frozen_epoch_ ? frozen_epoch_->lattice_origin : grid_map_->getOrigin();
    const double resolution = frozen_epoch_ ? frozen_epoch_->resolution_m : grid_map_->getResolution();
    const auto begin = (from - origin) / resolution;
    const auto end = (to - origin) / resolution;
    const Vector3i first_voxel = begin.array().floor().cast<int>();
    const Vector3i last_voxel = end.array().floor().cast<int>();
    auto examine = [&](const GridSearchCell& cell, const Vector3d& sample) {
        if (!cell.executable()) {
            ++result_.rejected_execution[static_cast<size_t>(cell.execution_reason)];
            if (component_diagnostic_ && component_diagnostic_->boundary && !deadlineExpired())
                component_diagnostic_->boundary(from, to, sample, gridExecutionReasonName(cell.execution_reason));
            if (cell.execution_reason == GridExecutionReason::ENVIRONMENT_STALE && !deadlineExpired())
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
                    origin + (voxel.array() + 0.5).matrix() * resolution),
                    origin + (voxel.array() + 0.5).matrix() * resolution))
                return std::nullopt;
            if (!more) break;
        } while (true);
    }
    std::array<GridSearchCell,3> checked_samples;
    checked_samples[0] = queryLatticePoint(from_index);
    if (!examine(checked_samples[0], from)) return std::nullopt;
    const Vector3i mid_key = from_index + to_index;
    // Even half-lattice coordinates are exactly node samples; odd coordinates
    // are midpoints. Voxel centers use a separate key namespace. Arbitrary
    // requested endpoints and connectors never use these keys.
    const Vector3d midpoint = (mid_key.x() % 2 == 0 && mid_key.y() % 2 == 0 && mid_key.z() % 2 == 0)
        ? Index2Coord(mid_key / 2) : Vector3d((from + to) / 2.0);
    checked_samples[1] = querySample(latticeKey(mid_key), midpoint, 2);
    if (!examine(checked_samples[1], midpoint)) return std::nullopt;
    checked_samples[2] = queryLatticePoint(to_index);
    if (!examine(checked_samples[2], to)) return std::nullopt;
    return segmentCost(from, to, false, &checked_samples);
}

std::optional<double> AStar::segmentCost(const Vector3d& from, const Vector3d& to,
                                       const bool check_physical,
                                       const std::array<GridSearchCell,3>* checked_samples)
{
    ++result_.segment_integrals;
    // The same metric trapezoidal quadrature owns lattice and connector cost.
    // Complete edge physics is checked separately before advisory-only samples.
    const double length = (to - from).norm();
    const double resolution = frozen_epoch_ ? frozen_epoch_->resolution_m : grid_map_->getResolution();
    // Share the actual-curve audit's fine spatial scale, including short
    // diagonal corner crossings that a half-voxel midpoint can miss.
    const int count = std::max(1, static_cast<int>(std::ceil(length / (resolution * .25))));
    double sum = 0.;
    for (int i = 0; i <= count; ++i) {
        if (deadlineExpired()) return std::nullopt;
        const Vector3d position = from + (to - from) * (double(i) / count);
        GridSearchCell cell;
        if (checked_samples && (i == 0 || i == count || 2*i == count)) {
            // These exact points were just checked/refreshed on this edge.
            cell = (*checked_samples)[i == 0 ? 0 : i == count ? 2 : 1];
        } else if (!planning_query_) {
            if (check_physical && grid_map_->getInflateOccupancy(position)) return std::nullopt;
            cell.execution_reason = GridExecutionReason::OK;
            cell.cost_multiplier = 1.;
        } else if (!check_physical && advisory_query_) {
            ++result_.risk_integration_calls;
            const auto risk = advisory_query_(position);
            cell.execution_reason = GridExecutionReason::OK;
            cell.advisory_class = risk.classification;
            cell.cost_multiplier = risk.cost_multiplier;
        } else {
            cell = timedPlanningQuery(position);
        }
        if (!cell.executable()) {
            ++result_.rejected_execution[static_cast<size_t>(cell.execution_reason)];
            if (cell.execution_reason == GridExecutionReason::ENVIRONMENT_STALE && !deadlineExpired())
                map_changed_ = true;
            return std::nullopt;
        }
        const bool warning = cell.advisory_class == GridAdvisoryClass::AVOID ||
            cell.advisory_class == GridAdvisoryClass::PREDICTED_DEGRADED;
        if (warning && !advisory_fallback_) {
            rejected_advisory_ = true;
            ++result_.rejected_advisory;
            return std::nullopt;
        }
        const double multiplier = gridAdvisoryCostMultiplier(cell.advisory_class,cell.cost_multiplier);
        sum += (i == 0 || i == count ? .5 : 1.) * multiplier;
    }
    return length * sum / count;
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

    const auto connector_ok = [this](const Vector3d& a, const Vector3d& b) {
        if (!planning_query_) return true;
        const int samples = std::max(1, static_cast<int>(std::ceil(
            (a - b).norm() / ((frozen_epoch_ ? frozen_epoch_->resolution_m : grid_map_->getResolution()) * 0.5))));
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
    result_.start_lattice = Index2Coord(start_idx);
    result_.end_lattice = Index2Coord(end_idx);
    if (!queryLatticePoint(start_idx).executable() ||
        !connector_ok(start_pt, Index2Coord(start_idx))) {
        // Recover the search lattice attachment, never the measured real start.
        // Use the same existing 1 m endpoint bound, deterministic nearest first,
        // and the complete physical connector under the shared search deadline.
        struct Attachment { Vector3i index; double distance2; };
        std::vector<Attachment> attachments;
        const int radius = std::min(static_cast<int>(std::ceil(1.0 / step_size_)), POOL_SIZE_.maxCoeff());
        for (int x=-radius; x<=radius; ++x) {
          if (deadlineExpired()) { result_.failure=Failure::TIME_BUDGET; return false; }
          for (int y=-radius; y<=radius; ++y) for (int z=-radius; z<=radius; ++z) {
            const Vector3i index=start_idx+Vector3i(x,y,z);
            if((index.array()<0).any() || (index.array()>=POOL_SIZE_.array()).any()) continue;
            const double distance2=(Index2Coord(index)-start_pt).squaredNorm();
            if(distance2<=1.0+1e-12) attachments.push_back({index,distance2});
          }
        }
        std::sort(attachments.begin(),attachments.end(),[](const Attachment& a,const Attachment& b) {
          if(a.distance2!=b.distance2) return a.distance2<b.distance2;
          for(int i=0;i<3;++i) if(a.index[i]!=b.index[i]) return a.index[i]<b.index[i];
          return false;
        });
        bool attached=false;
        for(const auto& candidate:attachments) {
          if(deadlineExpired()) { result_.failure=Failure::TIME_BUDGET; return false; }
          if(!queryLatticePoint(candidate.index).executable() ||
             !connector_ok(start_pt,Index2Coord(candidate.index))) continue;
          start_idx=candidate.index; attached=true; break;
        }
        if(!attached) { result_.failure=Failure::START_BLOCKED; return false; }
        result_.start_lattice=Index2Coord(start_idx);
        result_.start_attachment_recovered=true;
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
                    ((frozen_epoch_ ? frozen_epoch_->resolution_m : grid_map_->getResolution()) * 0.5))));
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
                    result_.end_lattice = candidate_position;
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
    return AstarSearchGoals(step_size, start_pt, {end_pt}, max_duration_s, center_override);
}

bool AStar::AstarSearchGoals(double step_size, const Vector3d& start_pt,
                            const std::vector<Vector3d>& requested_goals, double max_duration_s,
                            std::optional<Vector3d> center_override)
{
    std::vector<Vector3d> diagnostic_goals;
    if (component_diagnostic_) {
        diagnostic_goals = requested_goals;
        component_diagnostic_->goals.assign(requested_goals.size(), {});
        // Auxiliary self endpoint prepares the real start even when every target is invalid.
        diagnostic_goals.push_back(start_pt);
    }
    const auto& goals = component_diagnostic_ ? diagnostic_goals : requested_goals;
    if (goals.empty()) { recordPresearchFailure(Failure::END_BLOCKED, start_pt, start_pt); return false; }
    const Vector3d end_pt = goals.front();
    const auto steady_start = PlanningBudget::Clock::now();
    search_started_=steady_start; active_search_=true;
    const double limit = max_duration_s >= 0.0 ? max_duration_s : (planning_query_ ? 1.0 : .2);
    search_deadline_ = steady_start + std::chrono::duration_cast<PlanningBudget::Clock::duration>(
        std::chrono::duration<double>(limit));
    rclcpp::Time time_1 = rclcpp::Clock().now();
    ++rounds_;
    rejected_advisory_ = false;
    map_changed_ = false;
    gridPath_.clear();
    sample_cache_.clear();
    result_ = Result{};
    result_.performance_diagnostics = performance_diagnostics_;
    if(performance_diagnostics_) result_.goals.resize(requested_goals.size());
    result_.requested_start = start_pt;
    result_.requested_end = end_pt;
    query_stats_at_start_ = grid_map_->planningQueryStats();
    advisory_stats_at_start_ = advisory_statistics_ ? advisory_statistics_() : GridPlanningQueryStats{};
    search_generation_ = frozen_epoch_ ? frozen_epoch_->generation : grid_map_->occupancyGeneration();
    result_.occupancy_generation = search_generation_;

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = center_override.value_or((start_pt + end_pt) / 2);
    result_.step_size_m = step_size;
    result_.pool_dimensions = POOL_SIZE_;
    result_.pool_center = center_;

    if (deadlineExpired()) { finishFailure(Failure::TIME_BUDGET, time_1); return false; }
    Vector3i start_idx, end_idx;
    struct Goal { size_t original; Vector3i index; GridSearchCell cell; double terminal; };
    std::vector<Goal> valid_goals;
    // Prepare physical connectors without making advisory an endpoint authority.
    // Advisory on a connector is checked when the goal is actually reached.
    const bool preference_fallback = advisory_fallback_;
    advisory_fallback_ = true;
    for (size_t i = 0; i < goals.size(); ++i) {
        if (deadlineExpired()) { advisory_fallback_ = preference_fallback;
            if(performance_diagnostics_) result_.endpoint_prepare_s=std::chrono::duration<double>(PlanningBudget::Clock::now()-search_started_).count();
            finishFailure(Failure::TIME_BUDGET, time_1); return false; }
        const bool eligible = ConvertToIndexAndAdjustStartEndPoints(start_pt, goals[i], start_idx, end_idx);
        if(performance_diagnostics_ && i<requested_goals.size()) {
            auto& evidence=result_.goals[i];evidence.eligible=eligible;
            evidence.checked=!deadlineExpired();
            evidence.failure=eligible ? Failure::NONE : result_.failure;
            evidence.lattice=eligible ? result_.end_lattice : Vector3d::Constant(NAN);
        }
        if (component_diagnostic_ && i < requested_goals.size()) {
            auto& evidence = component_diagnostic_->goals[i];
            evidence.eligible = eligible; evidence.failure = eligible ? Failure::NONE : result_.failure;
            evidence.lattice = eligible ? result_.end_lattice : Vector3d::Constant(NAN);
        }
        if (eligible) valid_goals.push_back({i, end_idx, result_.end_cell,task_goal_ ? (goals[i]-*task_goal_).norm() : 0.});
        else if (result_.failure == Failure::START_BLOCKED || result_.failure == Failure::START_OUT_OF_POOL ||
                 result_.failure == Failure::CURRENT_MOTION || result_.failure == Failure::END_STALE) break;
    }
    advisory_fallback_ = preference_fallback;
    if(performance_diagnostics_) result_.endpoint_prepare_s=std::chrono::duration<double>(PlanningBudget::Clock::now()-search_started_).count();
    if (valid_goals.empty() || result_.failure == Failure::START_BLOCKED ||
        result_.failure == Failure::START_OUT_OF_POOL || result_.failure == Failure::CURRENT_MOTION ||
        result_.failure == Failure::END_STALE) {
        finishFailure(result_.failure, time_1); return false;
    }
    result_.failure = Failure::NONE;
    // Discovery priority is separate from the objective/proof lower bound.
    // With advisory costs, unit-distance A* spends the cold prediction allowance
    // flooding a ball before it reaches any terminal. Use the existing warning
    // multiplier only to order discovery, then reopen every discovered node
    // under the original admissible bound as soon as a complete guide exists.
    // This priority never changes measured edge/risk/terminal costs or grants
    // optimality, and component diagnostics keep their original ordering.
    bool discovering = planning_query_ && !component_diagnostic_;
    std::vector<GridNodePtr> discovered;
    const auto heuristic = [&](GridNodePtr node) {
        if (component_diagnostic_ && !component_diagnostic_->stop_on_first_goal) return 0.0;
        const auto began=performance_diagnostics_ ? PlanningBudget::Clock::now() : PlanningBudget::Clock::time_point{};
        double best = inf;
        for (const auto& goal : valid_goals) {
            GridNode terminal; terminal.index = goal.index;
            const double distance=getHeu(node, &terminal)+(Index2Coord(goal.index)-goals[goal.original]).norm();
            best = std::min(best, (discovering ? gridAdvisoryCostMultiplier(GridAdvisoryClass::AVOID, 1.) : 1.)*distance+goal.terminal);
        }
        if(performance_diagnostics_) result_.heuristic_s+=std::chrono::duration<double>(PlanningBudget::Clock::now()-began).count();
        return best;
    };
    const auto connector_cost = [&](const Vector3d& a, const Vector3d& b) {
        return segmentCost(a, b, true);
    };
    const auto start_connector=connector_cost(start_pt,Index2Coord(start_idx));
    if(!start_connector) {
        result_.exhausted=!deadlineExpired() && rejected_advisory_;
        finishFailure(deadlineExpired() ? Failure::TIME_BUDGET :
            result_.exhausted ? Failure::ADVISORY_NO_PATH : Failure::START_BLOCKED,time_1);
        return false;
    }
    double incumbent=inf;
    const auto finish_success=[&](bool proven) {
        result_.failure=Failure::NONE;result_.optimality_proven=proven && !result_.advisory_changed;
        result_.search_budget_exhausted=deadlineExpired();recordMapAtFinish();
        result_.duration_s=std::chrono::duration<double>(PlanningBudget::Clock::now()-search_started_).count();
        const auto path=getPath();result_.path_length_m=0.;
        for(size_t i=1;i<path.size();++i)result_.path_length_m+=(path[i]-path[i-1]).norm();
        result_.risk_cost_m=std::max(0.,result_.path_cost-result_.terminal_cost_m-result_.path_length_m);
        return true;
    };
    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];

    std::priority_queue<AStarQueueEntry, std::vector<AStarQueueEntry>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = *start_connector;
    startPtr->fScore = *start_connector+heuristic(startPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    if(discovering) discovered.push_back(startPtr);
    ++result_.queue_pushes;
    openSet_.push({startPtr, startPtr->fScore});

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        if (deadlineExpired()) { if(!gridPath_.empty()) return finish_success(false); finishFailure(Failure::TIME_BUDGET,time_1); return false; }
        if ((frozen_epoch_ ? !grid_map_->geometryMatches(*frozen_epoch_) : grid_map_->occupancyGeneration() != search_generation_)) {
            finishFailure(Failure::MAP_STALE, time_1);
            return false;
        }
        num_iter++;
        const auto queue_began=performance_diagnostics_ ? PlanningBudget::Clock::now() : PlanningBudget::Clock::time_point{};
        const auto entry = openSet_.top();
        openSet_.pop();
        if(performance_diagnostics_) result_.queue_s+=std::chrono::duration<double>(PlanningBudget::Clock::now()-queue_began).count();
        ++result_.queue_pops;
        current = entry.node;
        if (current->state == GridNode::CLOSEDSET ||
            entry.score > current->fScore + 1e-9) continue;

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if(!discovering && !component_diagnostic_ && std::isfinite(incumbent) && current->fScore>=incumbent-1e-9)
            return finish_success(true);
        for(const auto& goal:valid_goals) {
            if((component_diagnostic_ && goal.original>=requested_goals.size()) || current->index!=goal.index) continue;
            const auto connector=connector_cost(Index2Coord(current->index),goals[goal.original]);
            if(!connector) continue;
            if(performance_diagnostics_ && goal.original<result_.goals.size()) result_.goals[goal.original].reached=true;
            if(component_diagnostic_) component_diagnostic_->goals[goal.original].reached=true;
            const double complete=current->gScore+*connector+goal.terminal;
            if(complete<incumbent-1e-9) {
                if(!std::isfinite(incumbent)) result_.first_complete_path_s=std::chrono::duration<double>(PlanningBudget::Clock::now()-search_started_).count();
                incumbent=complete;gridPath_=retrievePath(current);
                result_.selected_goal=goal.original;result_.requested_end=goals[goal.original];
                result_.end_lattice=Index2Coord(goal.index);result_.end_cell=goal.cell;
                result_.path_cost=complete;result_.terminal_cost_m=goal.terminal;
            }
        }
        if (component_diagnostic_ && component_diagnostic_->stop_on_first_goal && !gridPath_.empty())
            return finish_success(false); // Explicit diagnostic early exit never claims optimality.
        if(deadlineExpired()) {if(!gridPath_.empty()) return finish_success(false);finishFailure(Failure::TIME_BUDGET,time_1);return false;}
        if(discovering && !gridPath_.empty()) {
            discovering=false;
            decltype(openSet_) proof_queue;
            for(auto node:discovered) {
                if(deadlineExpired()) return finish_success(false);
                node->state=GridNode::OPENSET;
                node->fScore=node->gScore+heuristic(node);
                proof_queue.push({node,node->fScore});
                ++result_.queue_pushes;
            }
            openSet_.swap(proof_queue);
            // Reopening includes CLOSED nodes: discovery scores are not an
            // admissible settled-cost proof and cannot suppress a cheaper route.
            continue;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.
        ++result_.expanded;
        if (component_diagnostic_ && component_diagnostic_->visit)
            component_diagnostic_->visit(Index2Coord(current->index), current->cameFrom ?
                Index2Coord(current->cameFrom->index) : start_pt);

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
                        if (component_diagnostic_ && component_diagnostic_->boundary)
                            component_diagnostic_->boundary(Index2Coord(current->index), Index2Coord(neighborIdx),
                                Index2Coord(neighborIdx), "POOL_BOUNDARY");
                        continue;
                    }

                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                    neighborPtr->index = neighborIdx;

                    bool flag_explored = neighborPtr->rounds == rounds_;

                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    {
                        continue; //in closed set.
                    }

                    // Every admitted edge multiplier is at least one. An OPEN
                    // neighbor already has a completely checked incoming path;
                    // this edge cannot improve it even at its geometric lower
                    // bound. Keep diagnostic boundary enumeration exhaustive.
                    const double edge_length=(Index2Coord(current->index)-Index2Coord(neighborIdx)).norm();
                    if(!component_diagnostic_ && flag_explored &&
                        current->gScore+edge_length>=neighborPtr->gScore) {
                        ++result_.lower_bound_pruned_edges;
                        continue;
                    }
                    const auto edge_started = performance_diagnostics_ ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
                    const auto multiplier = edgeCost(
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

                    tentative_gScore = current->gScore + *multiplier;

                    if (!flag_explored)
                    {
                        // A rejected incoming edge does not discover a node.
                        // Otherwise a later legal edge can inherit scores or
                        // CLOSED state from a previous search round.
                        neighborPtr->rounds = rounds_;
                        if(discovering) discovered.push_back(neighborPtr);
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + heuristic(neighborPtr);
                        ++result_.queue_pushes;
                        openSet_.push({neighborPtr, neighborPtr->fScore});
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    { //in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + heuristic(neighborPtr);
                        ++result_.queue_pushes;
                        openSet_.push({neighborPtr, neighborPtr->fScore});
                    }
                }
        if (deadlineExpired())
        {
            if(!gridPath_.empty()) return finish_success(false);
            finishFailure(Failure::TIME_BUDGET, time_1);return false;
        }
    }

    rclcpp::Time time_2 = rclcpp::Clock().now();

    if ((time_2 - time_1).seconds() > 0.1)
        RCLCPP_WARN(rclcpp::get_logger("AstarSearch"),
                    "Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).seconds(), num_iter);

    if (component_diagnostic_ && !component_diagnostic_->stop_on_first_goal && !deadlineExpired())
        result_.exhausted = true; // A fully enumerated component can also contain a legal goal.
    if(!gridPath_.empty()) return finish_success(!deadlineExpired());
    if (deadlineExpired()) { finishFailure(Failure::TIME_BUDGET, time_1); return false; }
    result_.exhausted = true;
    const bool unknown_rejected = result_.rejected_execution[
        static_cast<size_t>(GridExecutionReason::ENVIRONMENT_UNOBSERVED)] != 0;
    // Unknown cells bound every partially observed component. They must not
    // hide strict advisory rejections on its frontier: the existing bounded
    // high-cost retry can still discover a fully observed route. This is no
    // proof of advisory causality; preserve all physical rejection counters.
    finishFailure(result_.rejected_advisory != 0 && !advisory_fallback_
        ? Failure::ADVISORY_NO_PATH : unknown_rejected
            ? Failure::NO_PATH_WITH_UNOBSERVED : Failure::NO_PATH, time_1);
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
