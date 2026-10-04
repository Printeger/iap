#include "path_searching/dyn_a_star.h"

using namespace std;
using namespace Eigen;

std::optional<double> AStar::edgeMultiplier(const Vector3d& from,
                                            const Vector3d& to)
{
    if (!planning_query_) {
        return checkOccupancy(to) ? std::nullopt : std::optional<double>(1.0);
    }
    // Traverse the same GridMap voxel lattice as the physical and PL layers.
    RayCaster ray;
    const auto origin = grid_map_->getOrigin();
    const double resolution = grid_map_->getResolution();
    const auto begin = (from - origin) / resolution;
    const auto end = (to - origin) / resolution;
    double multiplier = 1.0;
    auto examine = [&](const Vector3d& position) {
        const auto cell = planning_query_(position);
        if (!cell.executable()) return false;
        const auto cls = cell.advisory.classification;
        if (cls == GridAdvisoryClass::AVOID ||
            cls == GridAdvisoryClass::PREDICTED_DEGRADED) {
            if (!advisory_fallback_) {
                rejected_advisory_ = true;
                return false;
            }
            multiplier = std::max(multiplier, 3.0);
        } else {
            multiplier = std::max(multiplier, cell.advisory.cost_multiplier);
        }
        return true;
    };
    if (ray.setInput(begin, end)) {
        Vector3d voxel;
        do {
            const bool more = ray.step(voxel);
            if (!examine(origin + (voxel.array() + 0.5).matrix() * resolution))
                return std::nullopt;
            if (!more) break;
        } while (true);
    }
    if (!examine(to)) return std::nullopt;
    return multiplier;
}

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
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
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    // A warning at the requested endpoint is not a physical obstacle to
    // move the endpoint through. Let the normal attempt fail so the bounded
    // advisory fallback can search to the actual local target.
    if (planning_query_ && !advisory_fallback_) {
        const auto end_cell = planning_query_(Index2Coord(end_idx));
        if (end_cell.executable() &&
            (end_cell.advisory.classification == GridAdvisoryClass::AVOID ||
             end_cell.advisory.classification ==
                 GridAdvisoryClass::PREDICTED_DEGRADED)) {
            rejected_advisory_ = true;
            return false;
        }
    }

    const auto blocked_lattice_cell = [this](const Vector3i& search_index) {
        const auto position = Index2Coord(search_index);
        if (checkOccupancy(position)) return true;
        if (!planning_query_) return false;
        Vector3i map_index;
        grid_map_->posToIndex(position, map_index);
        if (!grid_map_->isInMap(map_index)) return true;
        Vector3d map_center;
        grid_map_->indexToPos(map_index, map_center);
        return checkOccupancy(map_center);
    };

    if (blocked_lattice_cell(start_idx))
    {
        // RCLCPP_WARN(rclcpp::get_logger("ConvertToIndexAndAdjustStartEndPoints"), "Start point is insdide an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            if (!Coord2Index(start_pt, start_idx))
                return false;
        } while (blocked_lattice_cell(start_idx));
    }

    if (blocked_lattice_cell(end_idx))
    {
        // RCLCPP_WARN(rclcpp::get_logger("ConvertToIndexAndAdjustStartEndPoints"), "End point is insdide an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            if (!Coord2Index(end_pt, end_idx))
                return false;
        } while (blocked_lattice_cell(end_idx));
    }

    return true;
}

bool AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    rclcpp::Time time_1 = rclcpp::Clock().now();
    ++rounds_;
    rejected_advisory_ = false;
    gridPath_.clear();

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        RCLCPP_ERROR(rclcpp::get_logger("AstarSearch"), "Unable to handle the initial or end point, force return!");
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
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push({startPtr, startPtr->fScore});

    endPtr->index = end_idx;

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        const auto entry = openSet_.top();
        openSet_.pop();
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

                    const auto multiplier = edgeMultiplier(
                        Index2Coord(current->index), Index2Coord(neighborPtr->index));
                    if (!multiplier) {
                        continue;
                    }

                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    tentative_gScore = current->gScore + static_cost * *multiplier;

                    if (!flag_explored)
                    {
                        //discover a new node
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push({neighborPtr, neighborPtr->fScore});
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    { //in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push({neighborPtr, neighborPtr->fScore});
                    }
                }
        rclcpp::Time time_2 = rclcpp::Clock().now();
        if ((time_2 - time_1).seconds() > (planning_query_ ? 1.0 : 0.2))
        {
            RCLCPP_WARN(rclcpp::get_logger("AstarSearch"), "A* time budget exceeded");
            return false;
        }
    }

    rclcpp::Time time_2 = rclcpp::Clock().now();

    if ((time_2 - time_1).seconds() > 0.1)
        RCLCPP_WARN(rclcpp::get_logger("AstarSearch"),
                    "Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).seconds(), num_iter);

    return false;
}

vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}
