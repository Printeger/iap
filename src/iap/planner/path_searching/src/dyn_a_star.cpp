#include "path_searching/dyn_a_star.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <utility>

#include <iap/planner/risk_grid_map.hpp>

using namespace std;
using namespace Eigen;

bool p4V2CostLess(const P4V2LexicographicCost &lhs,
                  const P4V2LexicographicCost &rhs)
{
    return std::tie(lhs.bottleneck, lhs.integral, lhs.path_length) <
           std::tie(rhs.bottleneck, rhs.integral, rhs.path_length);
}

AStar::~AStar()
{
    if (!GridNodeMap_)
        return;
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
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

void AStar::setRiskSnapshot(std::shared_ptr<const iap::RiskGridSnapshot> snapshot, double query_base_time_s)
{
    risk_snapshot_ = std::move(snapshot);
    risk_query_base_time_s_ = query_base_time_s;
}

void AStar::clearRiskSnapshot()
{
    risk_snapshot_.reset();
    risk_query_base_time_s_ = 0.0;
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
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    if (checkOccupancy(Index2Coord(start_idx)))
    {
        // RCLCPP_WARN(rclcpp::get_logger("ConvertToIndexAndAdjustStartEndPoints"), "Start point is insdide an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            if (!Coord2Index(start_pt, start_idx))
                return false;
        } while (checkOccupancy(Index2Coord(start_idx)));
    }

    if (checkOccupancy(Index2Coord(end_idx)))
    {
        // RCLCPP_WARN(rclcpp::get_logger("ConvertToIndexAndAdjustStartEndPoints"), "End point is insdide an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            if (!Coord2Index(end_pt, end_idx))
                return false;
        } while (checkOccupancy(Index2Coord(end_idx)));
    }

    return true;
}

bool AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    if (p4_config_.enable_risk_aware_astar && risk_snapshot_)
        return AstarSearchRiskAware(step_size, start_pt, end_pt);
    return AstarSearchOriginal(step_size, start_pt, end_pt);
}

bool AStar::AstarSearchOriginal(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    v2_path_.clear();
    return astarSearchImpl(step_size, start_pt, end_pt, false);
}

bool AStar::AstarSearchRiskAware(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    if (p4_config_.objective == P4RiskObjective::PROVIDER_BOTTLENECK_V2 &&
        p4_config_.enable_risk_aware_astar && risk_snapshot_)
        return astarSearchProviderBottleneckV2(step_size, start_pt, end_pt);
    v2_path_.clear();
    return astarSearchImpl(step_size, start_pt, end_pt, p4_config_.enable_risk_aware_astar && risk_snapshot_);
}

bool AStar::astarSearchProviderBottleneckV2(
    const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    struct Key
    {
        int x = 0, y = 0, z = 0, time_bin = 0;
        bool operator==(const Key &other) const
        {
            return x == other.x && y == other.y && z == other.z &&
                   time_bin == other.time_bin;
        }
    };
    struct KeyHash
    {
        std::size_t operator()(const Key &key) const
        {
            std::size_t value = static_cast<std::size_t>(key.x + 4099);
            value = value * 8191u + static_cast<std::size_t>(key.y + 4099);
            value = value * 8191u + static_cast<std::size_t>(key.z + 4099);
            return value * 65537u + static_cast<std::size_t>(key.time_bin);
        }
    };
    struct Label
    {
        Key key;
        Vector3i index = Vector3i::Zero();
        P4V2LexicographicCost cost;
        std::shared_ptr<const Label> parent;
        uint64_t serial = 0;
    };
    struct LabelWorse
    {
        bool operator()(const std::shared_ptr<const Label> &lhs,
                        const std::shared_ptr<const Label> &rhs) const
        {
            if (p4V2CostLess(rhs->cost, lhs->cost)) return true;
            if (p4V2CostLess(lhs->cost, rhs->cost)) return false;
            return std::tie(lhs->key.time_bin, lhs->key.x, lhs->key.y,
                            lhs->key.z, lhs->serial) >
                   std::tie(rhs->key.time_bin, rhs->key.x, rhs->key.y,
                            rhs->key.z, rhs->serial);
        }
    };

    const rclcpp::Time started = rclcpp::Clock().now();
    last_p4_metrics_ = P4AStarMetrics{};
    last_p4_metrics_.risk_enabled = true;
    last_p4_metrics_.snapshot_generation_id = risk_snapshot_->generation_id();
    last_p4_metrics_.fallback_reason = "provider_bottleneck_v2_search";
    v2_path_.clear();
    gridPath_.clear();
    step_size_ = step_size;
    inv_step_size_ = 1.0 / step_size;
    center_ = (start_pt + end_pt) / 2.0;
    search_start_pt_ = start_pt;

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(
            start_pt, end_pt, start_idx, end_idx))
    {
        last_p4_metrics_.fallback_reason = "invalid_start_or_end";
        return false;
    }
    const double speed = p4_config_.query_speed_mps;
    const double time_bin_width_s = step_size / speed;
    const double endpoint_buffer_m = 2.0 * risk_snapshot_->params().resolution_m;
    const double reference_length = p4_v2_reference_path_length_m_ > 0.0
                                        ? p4_v2_reference_path_length_m_
                                        : (Index2Coord(end_idx) -
                                           Index2Coord(start_idx)).norm();
    const double max_path_length =
        reference_length * p4_config_.max_extra_path_ratio;

    std::priority_queue<std::shared_ptr<const Label>,
                        std::vector<std::shared_ptr<const Label>>, LabelWorse>
        open;
    std::unordered_map<Key, P4V2LexicographicCost, KeyHash> best;
    uint64_t serial = 0;
    auto start = std::make_shared<Label>();
    start->key = Key{start_idx.x(), start_idx.y(), start_idx.z(), 0};
    start->index = start_idx;
    start->serial = serial++;
    open.push(start);
    best.emplace(start->key, start->cost);

    while (!open.empty())
    {
        if ((rclcpp::Clock().now() - started).seconds() > 0.2)
        {
            last_p4_metrics_.elapsed_ms =
                (rclcpp::Clock().now() - started).seconds() * 1000.0;
            last_p4_metrics_.fallback_reason = "timeout";
            return false;
        }
        const auto current = open.top();
        open.pop();
        const auto best_it = best.find(current->key);
        if (best_it == best.end() ||
            p4V2CostLess(best_it->second, current->cost))
            continue;
        ++last_p4_metrics_.expanded_nodes;
        if (current->index == end_idx)
        {
            for (auto label = current; label; label = label->parent)
                v2_path_.push_back(Index2Coord(label->index));
            std::reverse(v2_path_.begin(), v2_path_.end());
            last_p4_metrics_.provider_bottleneck = current->cost.bottleneck;
            last_p4_metrics_.provider_integral = current->cost.integral;
            last_p4_metrics_.risk_path_length = current->cost.path_length;
            last_p4_metrics_.time_state_count = static_cast<int>(best.size());
            last_p4_metrics_.elapsed_ms =
                (rclcpp::Clock().now() - started).seconds() * 1000.0;
            last_p4_metrics_.fallback_reason = "provider_bottleneck_v2";
            return true;
        }

        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz)
                {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    const Vector3i next_index =
                        current->index + Vector3i(dx, dy, dz);
                    if (next_index.x() < 1 || next_index.x() >= POOL_SIZE_.x() - 1 ||
                        next_index.y() < 1 || next_index.y() >= POOL_SIZE_.y() - 1 ||
                        next_index.z() < 1 || next_index.z() >= POOL_SIZE_.z() - 1)
                        continue;
                    const Vector3d next_position = Index2Coord(next_index);
                    // Occupancy authority is evaluated before any P0 risk query.
                    if (checkOccupancy(next_position))
                    {
                        ++last_p4_metrics_.occupied_reject_count;
                        continue;
                    }
                    const double edge_length =
                        step_size * std::sqrt(dx * dx + dy * dy + dz * dz);
                    const double travel = current->cost.path_length + edge_length;
                    if (travel > max_path_length + 1.0e-12) continue;

                    P4V2LexicographicCost next_cost = current->cost;
                    next_cost.path_length = travel;
                    const double remaining =
                        (Index2Coord(end_idx) - next_position).norm();
                    if (travel >= endpoint_buffer_m &&
                        remaining >= endpoint_buffer_m)
                    {
                        double provider = 0.0;
                        if (!queryProviderRiskForV2Edge(
                                Index2Coord(current->index), next_position,
                                travel, &provider))
                            continue;
                        next_cost.bottleneck =
                            std::max(next_cost.bottleneck, provider);
                        next_cost.integral += provider * edge_length;
                    }
                    const int time_bin = static_cast<int>(std::floor(
                        (travel / speed) / time_bin_width_s + 1.0e-12));
                    const Key key{next_index.x(), next_index.y(),
                                  next_index.z(), time_bin};
                    const auto incumbent = best.find(key);
                    if (incumbent != best.end() &&
                        !p4V2CostLess(next_cost, incumbent->second))
                        continue;
                    best[key] = next_cost;
                    auto next = std::make_shared<Label>();
                    next->key = key;
                    next->index = next_index;
                    next->cost = next_cost;
                    next->parent = current;
                    next->serial = serial++;
                    open.push(next);
                }
    }
    last_p4_metrics_.elapsed_ms =
        (rclcpp::Clock().now() - started).seconds() * 1000.0;
    last_p4_metrics_.time_state_count = static_cast<int>(best.size());
    last_p4_metrics_.fallback_reason =
        last_p4_metrics_.provider_incomplete_reject_count > 0
            ? "provider_support_incomplete"
            : "no_path";
    return false;
}

bool AStar::queryProviderRiskForV2Edge(
    const Eigen::Vector3d &current_pos, const Eigen::Vector3d &next_pos,
    const double travel_distance_m, double *provider_cost)
{
    if (!provider_cost || !risk_snapshot_)
        return false;
    const Eigen::Vector3d query_position = 0.5 * (current_pos + next_pos);
    // This check is deliberately before both the diagnostic query counter and
    // the provider-decomposition call so occupancy retains hard authority.
    if (checkOccupancy(query_position))
    {
        ++last_p4_metrics_.occupied_reject_count;
        return false;
    }
    ++last_p4_metrics_.risk_query_count;
    const double speed = p4_config_.query_speed_mps;
    const double query_time =
        risk_query_base_time_s_ + travel_distance_m / speed;
    iap::RiskCostDecomposition decomposition;
    if (!risk_snapshot_->queryRiskCostDecomposition(
            query_position, query_time, &decomposition))
    {
        if (decomposition.occupied_support_weight > 0.0)
            ++last_p4_metrics_.occupied_reject_count;
        else
        {
            ++last_p4_metrics_.unknown_count;
            ++last_p4_metrics_.provider_incomplete_reject_count;
        }
        return false;
    }
    *provider_cost = decomposition.provider_c_pi;
    return true;
}

double AStar::queryTimeForEdge(const GridNodePtr current, double geometric_cost) const
{
    const double speed = std::isfinite(p4_config_.query_speed_mps) && p4_config_.query_speed_mps > 1.0e-3
                             ? p4_config_.query_speed_mps
                             : 1.0;
    const double distance_m = current ? current->travelDistance +
                                            geometric_cost * step_size_
                                      : geometric_cost * step_size_;
    return risk_query_base_time_s_ + distance_m / speed;
}

double AStar::edgeCostWithRisk(const Vector3d &current_pos, const Vector3d &neighbor_pos,
                               double geometric_cost, double query_time_s)
{
    if (!p4_config_.enable_risk_aware_astar || !risk_snapshot_)
        return geometric_cost;

    ++last_p4_metrics_.risk_query_count;
    iap::RiskCostSample sample;
    const Vector3d query_pos = 0.5 * (current_pos + neighbor_pos);
    if (!risk_snapshot_->queryCost(
            query_pos, query_time_s, &sample, p4_config_.cost_query_policy) ||
        !sample.valid || sample.stale || !std::isfinite(sample.cost))
    {
        ++last_p4_metrics_.unknown_count;
        return geometric_cost + p4_config_.unknown_edge_penalty;
    }

    const double risk_cost = std::clamp(sample.cost, 0.0, p4_config_.risk_cost_max);
    p4_valid_cost_sum_ += risk_cost;
    ++p4_valid_cost_count_;
    last_p4_metrics_.path_max_cost = std::max(last_p4_metrics_.path_max_cost, risk_cost);
    return geometric_cost + p4_config_.lambda_p4_risk * geometric_cost * risk_cost;
}

bool AStar::astarSearchImpl(const double step_size, Vector3d start_pt, Vector3d end_pt, bool use_risk)
{
    rclcpp::Time time_1 = rclcpp::Clock().now();
    ++rounds_;
    last_p4_metrics_ = P4AStarMetrics{};
    last_p4_metrics_.risk_enabled = use_risk;
    last_p4_metrics_.snapshot_generation_id = (use_risk && risk_snapshot_) ? risk_snapshot_->generation_id() : 0;
    last_p4_metrics_.fallback_reason = use_risk ? "risk_search" : "original_search";
    p4_valid_cost_sum_ = 0.0;
    p4_valid_cost_count_ = 0;

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;
    search_start_pt_ = start_pt;

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        RCLCPP_ERROR(rclcpp::get_logger("AstarSearch"), "Unable to handle the initial or end point, force return!");
        last_p4_metrics_.fallback_reason = "invalid_start_or_end";
        return false;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->travelDistance = 0.0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); //put start in open set

    endPtr->index = end_idx;

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        last_p4_metrics_.expanded_nodes = num_iter;
        current = openSet_.top();
        openSet_.pop();

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath_ = retrievePath(current);
            rclcpp::Time time_2 = rclcpp::Clock().now();
            last_p4_metrics_.elapsed_ms = (time_2 - time_1).seconds() * 1000.0;
            if (p4_valid_cost_count_ > 0)
                last_p4_metrics_.path_mean_cost = p4_valid_cost_sum_ / static_cast<double>(p4_valid_cost_count_);
            return true;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

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

                    neighborPtr->rounds = rounds_;

                    if (checkOccupancy(Index2Coord(neighborPtr->index)))
                    {
                        ++last_p4_metrics_.occupied_reject_count;
                        continue;
                    }

                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    const double edge_cost = use_risk
                                                 ? edgeCostWithRisk(Index2Coord(current->index),
                                                                    Index2Coord(neighborPtr->index),
                                                                    static_cost,
                                                                    queryTimeForEdge(current, static_cost))
                                                 : static_cost;
                    tentative_gScore = current->gScore + edge_cost;

                    if (!flag_explored)
                    {
                        //discover a new node
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->travelDistance = current->travelDistance + static_cost * step_size_;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr); //put neighbor in open set and record it.
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    { //in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->travelDistance = current->travelDistance + static_cost * step_size_;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    }
                }
        rclcpp::Time time_2 = rclcpp::Clock().now();
        if ((time_2 - time_1).seconds() > 0.2)
        {
            RCLCPP_WARN(rclcpp::get_logger("AstarSearch"), "Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            last_p4_metrics_.elapsed_ms = (time_2 - time_1).seconds() * 1000.0;
            last_p4_metrics_.fallback_reason = "timeout";
            return false;
        }
    }

    rclcpp::Time time_2 = rclcpp::Clock().now();

    if ((time_2 - time_1).seconds() > 0.1)
        RCLCPP_WARN(rclcpp::get_logger("AstarSearch"),
                    "Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).seconds(), num_iter);

    last_p4_metrics_.elapsed_ms = (time_2 - time_1).seconds() * 1000.0;
    last_p4_metrics_.fallback_reason = "no_path";
    return false;
}

vector<Vector3d> AStar::getPath()
{
    if (!v2_path_.empty())
        return v2_path_;
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}
