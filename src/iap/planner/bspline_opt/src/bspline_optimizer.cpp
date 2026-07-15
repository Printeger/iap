#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/gradient_descent_optimizer.h"
#include <iap/planner/risk_grid_map.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <utility>
// using namespace std;

namespace ego_planner
{
  namespace
  {
    constexpr int kP1AcceptedProfileSampleCount = 200;
    constexpr const char *kP1AcceptedProfileCsvName =
        "planner_p1_accepted_trajectory_risk_profile.csv";

    double pathLength(const std::vector<Eigen::Vector3d> &path)
    {
      double length = 0.0;
      for (size_t i = 1; i < path.size(); ++i)
        length += (path[i] - path[i - 1]).norm();
      return length;
    }

    std::string siblingPath(const std::string &path, const std::string &filename)
    {
      const auto slash = path.find_last_of("/\\");
      if (slash == std::string::npos)
      {
        return filename;
      }
      return path.substr(0, slash + 1) + filename;
    }

    void writeP4Csv(const P4RiskAStarConfig &config,
                    const P4AStarMetrics &metrics,
                    double stamp,
                    uint64_t astar_call_id,
                    int segment_id)
    {
      if (!config.enable_risk_aware_astar || !config.debug_csv_enable || config.debug_csv_path.empty())
        return;
      std::ifstream existing(config.debug_csv_path);
      const bool write_header =
          !existing.good() || existing.peek() == std::ifstream::traits_type::eof();
      existing.close();

      std::ofstream csv(config.debug_csv_path, std::ios::app);
      if (!csv.good())
        return;
      if (write_header)
      {
        csv << "stamp,astar_call_id,segment_id,risk_enabled,snapshot_generation_id,"
               "expanded_nodes,risk_query_count,unknown_count,occupied_reject_count,"
               "original_path_length,risk_path_length,path_length_ratio,path_mean_cost,"
               "path_max_cost,elapsed_ms,fallback_reason\n";
      }
      csv << stamp << ',' << astar_call_id << ',' << segment_id << ','
          << (metrics.risk_enabled ? 1 : 0) << ','
          << metrics.snapshot_generation_id << ','
          << metrics.expanded_nodes << ','
          << metrics.risk_query_count << ','
          << metrics.unknown_count << ','
          << metrics.occupied_reject_count << ','
          << metrics.original_path_length << ','
          << metrics.risk_path_length << ','
          << metrics.path_length_ratio << ','
          << metrics.path_mean_cost << ','
          << metrics.path_max_cost << ','
          << metrics.elapsed_ms << ','
          << metrics.fallback_reason << '\n';
    }
  } // namespace


  void BsplineOptimizer::setParam(rclcpp::Node::SharedPtr node)
  {

    node->declare_parameter("optimization/lambda_smooth", -1.0);
    node->declare_parameter("optimization/lambda_collision", -1.0);
    node->declare_parameter("optimization/lambda_feasibility", -1.0);
    node->declare_parameter("optimization/lambda_fitness", -1.0);

    node->declare_parameter("optimization/dist0", -1.0);
    node->declare_parameter("optimization/swarm_clearance", -1.0);
    node->declare_parameter("optimization/max_vel", -1.0);
    node->declare_parameter("optimization/max_acc", -1.0);

    node->declare_parameter("optimization/order", 3);
    node->declare_parameter("p1.use_integrity_cost", false);
    node->declare_parameter("p1.metrics_only", true);
    node->declare_parameter("p1.lambda_integrity", 0.0);
    node->declare_parameter("p1.sample_dt_min_s", 0.1);
    node->declare_parameter("p1.sample_dt_scale", 1.0);
    node->declare_parameter("p1.max_samples_per_eval", 30);
    node->declare_parameter("p1.integrity_cost_max", 100.0);
    node->declare_parameter("p1.integrity_grad_norm_max", 0.1);
    node->declare_parameter("p1.unknown_policy", std::string("skip"));
    node->declare_parameter("p1.unknown_soft_penalty", 1.0);
    node->declare_parameter("p1.debug_csv_enable", false);
    node->declare_parameter("p1.debug_csv_path", std::string(""));
    node->declare_parameter("p4.enable_risk_aware_astar", false);
    node->declare_parameter("p4.lambda_p4_risk", 0.05);
    node->declare_parameter("p4.risk_cost_max", 100.0);
    node->declare_parameter("p4.unknown_edge_penalty", 1.0);
    node->declare_parameter("p4.max_extra_path_ratio", 1.3);
    node->declare_parameter("p4.fallback_to_original_when_risk_not_ready", true);
    node->declare_parameter("p4.debug_csv_enable", false);
    node->declare_parameter("p4.debug_csv_path", std::string(""));

    node->get_parameter("optimization/lambda_smooth", lambda1_);
    node->get_parameter("optimization/lambda_collision", lambda2_);
    node->get_parameter("optimization/lambda_feasibility", lambda3_);
    node->get_parameter("optimization/lambda_fitness", lambda4_);

    node->get_parameter("optimization/dist0", dist0_);
    node->get_parameter("optimization/swarm_clearance", swarm_clearance_);
    node->get_parameter("optimization/max_vel", max_vel_);
    node->get_parameter("optimization/max_acc", max_acc_);

    node->get_parameter("optimization/order", order_);
    node->get_parameter("p1.use_integrity_cost", p1_config_.use_integrity_cost);
    node->get_parameter("p1.metrics_only", p1_config_.metrics_only);
    node->get_parameter("p1.lambda_integrity", p1_config_.lambda_integrity);
    node->get_parameter("p1.sample_dt_min_s", p1_config_.sample_dt_min_s);
    node->get_parameter("p1.sample_dt_scale", p1_config_.sample_dt_scale);
    node->get_parameter("p1.max_samples_per_eval", p1_config_.max_samples_per_eval);
    node->get_parameter("p1.integrity_cost_max", p1_config_.integrity_cost_max);
    node->get_parameter("p1.integrity_grad_norm_max", p1_config_.integrity_grad_norm_max);
    node->get_parameter("p1.unknown_policy", p1_config_.unknown_policy);
    node->get_parameter("p1.unknown_soft_penalty", p1_config_.unknown_soft_penalty);
    node->get_parameter("p1.debug_csv_enable", p1_config_.debug_csv_enable);
    node->get_parameter("p1.debug_csv_path", p1_config_.debug_csv_path);
    node->get_parameter("p4.enable_risk_aware_astar", p4_config_.enable_risk_aware_astar);
    node->get_parameter("p4.lambda_p4_risk", p4_config_.lambda_p4_risk);
    node->get_parameter("p4.risk_cost_max", p4_config_.risk_cost_max);
    node->get_parameter("p4.unknown_edge_penalty", p4_config_.unknown_edge_penalty);
    node->get_parameter("p4.max_extra_path_ratio", p4_config_.max_extra_path_ratio);
    node->get_parameter("p4.fallback_to_original_when_risk_not_ready", p4_config_.fallback_to_original_when_risk_not_ready);
    node->get_parameter("p4.debug_csv_enable", p4_config_.debug_csv_enable);
    node->get_parameter("p4.debug_csv_path", p4_config_.debug_csv_path);
    p4_config_.query_speed_mps = std::isfinite(max_vel_) && max_vel_ > 1.0e-3 ? max_vel_ : 1.0;
  }

  void BsplineOptimizer::setEnvironment(const GridMap::Ptr &map)
  {
    this->grid_map_ = map;
  }

  void BsplineOptimizer::setEnvironment(const GridMap::Ptr &map, const fast_planner::ObjPredictor::Ptr mov_obj)
  {
    this->grid_map_ = map;
    this->moving_objs_ = mov_obj;
  }

  void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.points = points;
  }

  void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

  void BsplineOptimizer::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr) { swarm_trajs_ = swarm_trajs_ptr; }

  void BsplineOptimizer::setDroneId(const int drone_id) { drone_id_ = drone_id; }

  void BsplineOptimizer::setRiskSnapshot(std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
                                         double query_base_time_s)
  {
    risk_snapshot_ = std::move(snapshot);
    risk_query_base_time_s_ = query_base_time_s;
  }

  void BsplineOptimizer::clearRiskSnapshot()
  {
    risk_snapshot_.reset();
    risk_query_base_time_s_ = 0.0;
  }

  void BsplineOptimizer::setP4RiskSnapshot(std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
                                           double query_base_time_s)
  {
    if (!a_star_)
      return;
    p4_config_.query_speed_mps = std::isfinite(max_vel_) && max_vel_ > 1.0e-3 ? max_vel_ : 1.0;
    a_star_->setP4Config(p4_config_);
    a_star_->setRiskSnapshot(std::move(snapshot), query_base_time_s);
  }

  void BsplineOptimizer::clearP4RiskSnapshot()
  {
    if (!a_star_)
      return;
    a_star_->clearRiskSnapshot();
  }

  // 返回多个安全的控制点集
  std::vector<ControlPoints> BsplineOptimizer::distinctiveTrajs(vector<std::pair<int, int>> segments)
  {
    if (segments.size() == 0) // will be invoked again later.
    {
      std::vector<ControlPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    constexpr int MAX_TRAJS = 8;                                                                            // 最多的轨迹数量
    constexpr int VARIS = 2;                                                                                // 允许的变化种类数
    int seg_upbound = std::min((int)segments.size(), static_cast<int>(floor(log(MAX_TRAJS) / log(VARIS)))); // 允许变换的片段数量上限
    std::vector<ControlPoints> control_pts_buf;
    control_pts_buf.reserve(MAX_TRAJS);
    const double RESOLUTION = grid_map_->getResolution();
    const double CTRL_PT_DIST = (cps_.points.col(0) - cps_.points.col(cps_.size - 1)).norm() / (cps_.size - 1); // 计算控制点间的平均距离

    // Step 1. Find the opposite vectors and base points for every segment.
    std::vector<std::pair<ControlPoints, ControlPoints>> RichInfoSegs;
    // 初始化两套控制点信息
    for (int i = 0; i < seg_upbound; i++)
    {
      std::pair<ControlPoints, ControlPoints> RichInfoOneSeg;
      ControlPoints RichInfoOneSeg_temp;
      // 获取指定片段的控制点信息
      cps_.segment(RichInfoOneSeg_temp, segments[i].first, segments[i].second);
      RichInfoOneSeg.first = RichInfoOneSeg_temp;
      RichInfoOneSeg.second = RichInfoOneSeg_temp;
      RichInfoSegs.push_back(RichInfoOneSeg);

      // cout << "RichInfoOneSeg_temp, out" << endl;
      // cout << "RichInfoSegs[" << i << "].first" << endl;
      // for ( int k=0; k<RichInfoOneSeg_temp.size; k++ )
      //   if ( RichInfoOneSeg_temp.base_point[k].size() > 0 )
      //   {
      //     cout << "###" << RichInfoOneSeg_temp.points.col(k).transpose() << endl;
      //     for (int k2 = 0; k2 < RichInfoOneSeg_temp.base_point[k].size(); k2++)
      //     {
      //       cout << "      " << RichInfoOneSeg_temp.base_point[k][k2].transpose() << " @ " << RichInfoOneSeg_temp.direction[k][k2].transpose() << endl;
      //     }
      //   }
    }

    for (int i = 0; i < seg_upbound; i++)
    {

      // 1.1 Find the start occupied point id and the last occupied point id
      if (RichInfoSegs[i].first.size > 1)
      {
        int occ_start_id = -1, occ_end_id = -1;
        Eigen::Vector3d occ_start_pt, occ_end_pt;
        for (int j = 0; j < RichInfoSegs[i].first.size - 1; j++)
        {
          // cout << "A *" << j << "*" << endl;
          //  遍历每个控制点及其后一个点，在两点间通过线性插值生成采样点
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j + 1)).norm() / 2;
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j + 1));
            // cout << " " << grid_map_->getInflateOccupancy(pt) << " pt=" << pt.transpose() << endl;
            //  如果检测到在障碍物内，则存储对应数据
            if (grid_map_->getInflateOccupancy(pt))
            {
              occ_start_id = j;
              occ_start_pt = pt;
              goto exit_multi_loop1;
            }
          }
        }
      exit_multi_loop1:;
        // 查找片段与最后一个障碍物的交点
        for (int j = RichInfoSegs[i].first.size - 1; j >= 1; j--)
        {
          // cout << "j=" << j << endl;
          // cout << "B *" << j << "*" << endl;
          ;
          // 和上面同样的采样，然后检测是否在障碍物中
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j - 1)).norm();
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j - 1));
            // cout << " " << grid_map_->getInflateOccupancy(pt) << " pt=" << pt.transpose() << endl;
            ;
            if (grid_map_->getInflateOccupancy(pt))
            {
              occ_end_id = j;
              occ_end_pt = pt;
              goto exit_multi_loop2;
            }
          }
        }
      exit_multi_loop2:;

        // double check
        // 如果片段的起点或者终点在障碍物中，将会被移除
        if (occ_start_id == -1 || occ_end_id == -1)
        {
          // It means that the first or the last control points of one segment are in obstacles, which is not allowed.
          // ROS_WARN("What? occ_start_id=%d, occ_end_id=%d", occ_start_id, occ_end_id);

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;

          continue;

          // cout << "RichInfoSegs[" << i << "].first" << endl;
          // for (int k = 0; k < RichInfoSegs[i].first.size; k++)
          // {
          //   if (RichInfoSegs[i].first.base_point.size() > 0)
          //   {
          //     cout << "###" << RichInfoSegs[i].first.points.col(k).transpose() << endl;
          //     for (int k2 = 0; k2 < RichInfoSegs[i].first.base_point[k].size(); k2++)
          //     {
          //       cout << "      " << RichInfoSegs[i].first.base_point[k][k2].transpose() << " @ " << RichInfoSegs[i].first.direction[k][k2].transpose() << endl;
          //     }
          //   }
          // }
        }

        // 1.2 Reverse the vector and find new base points from occ_start_id to occ_end_id.
        for (int j = occ_start_id; j <= occ_end_id; j++)
        {
          Eigen::Vector3d base_pt_reverse, base_vec_reverse;
          // 检查控制点的base point是否为1
          if (RichInfoSegs[i].first.base_point[j].size() != 1)
          {
            cout << "RichInfoSegs[" << i << "].first.base_point[" << j << "].size()=" << RichInfoSegs[i].first.base_point[j].size() << endl;
            RCLCPP_ERROR(rclcpp::get_logger("distinctiveTrajs"), "Wrong number of base_points!!! Should not be happen!.");

            cout << setprecision(5);
            cout << "cps_" << endl;
            cout << " clearance=" << cps_.clearance << " cps.size=" << cps_.size << endl;
            // 输出错误信息
            for (int temp_i = 0; temp_i < cps_.size; temp_i++)
            {
              if (cps_.base_point[temp_i].size() > 1 && cps_.base_point[temp_i].size() < 1000)
              {
                RCLCPP_ERROR(rclcpp::get_logger("distinctiveTrajs"), "Should not happen!!!");
                cout << "######" << cps_.points.col(temp_i).transpose() << endl;
                for (size_t temp_j = 0; temp_j < cps_.base_point[temp_i].size(); temp_j++)
                  cout << "      " << cps_.base_point[temp_i][temp_j].transpose() << " @ " << cps_.direction[temp_i][temp_j].transpose() << endl;
              }
            }

            std::vector<ControlPoints> blank;
            return blank;
          }

          // 通过取反获得相反方向的向量
          base_vec_reverse = -RichInfoSegs[i].first.direction[j][0];

          // The start and the end case must get taken special care of.
          // 若当前控制点为片段的起始点 occ_start_id，则将障碍物交点 occ_start_pt 直接设为 base_pt_reverse
          if (j == occ_start_id)
          {
            base_pt_reverse = occ_start_pt;
          }
          // 若当前控制点为片段的终止点 occ_end_id，则将终点交点 occ_end_pt 设为 base_pt_reverse
          else if (j == occ_end_id)
          {
            base_pt_reverse = occ_end_pt;
          }
          // 对于片段中的中间控制点，将基准点 base_pt_reverse 设置为当前控制点 points.col(j) 沿反向向量 base_vec_reverse 方向延伸的某一距离位置
          else
          {
            base_pt_reverse = RichInfoSegs[i].first.points.col(j) + base_vec_reverse * (RichInfoSegs[i].first.base_point[j][0] - RichInfoSegs[i].first.points.col(j)).norm();
          }

          // 检查base_pt_reverse是否在障碍物中
          if (grid_map_->getInflateOccupancy(base_pt_reverse)) // Search outward.
          {
            // 最大搜索范围
            double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
            double l = RESOLUTION;
            for (; l <= l_upbound; l += RESOLUTION)
            {
              // 不断将控制点向外移动，寻找不在障碍物中的控制点
              Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
              // cout << base_pt_temp.transpose() << endl;
              if (!grid_map_->getInflateOccupancy(base_pt_temp))
              {
                RichInfoSegs[i].second.base_point[j][0] = base_pt_temp;
                RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
                break;
              }
            }
            // 如果找不到则删除这一段
            if (l > l_upbound)
            {
              RCLCPP_WARN(rclcpp::get_logger("distinctiveTrajs"), "Can't find the new base points at the opposite within the threshold. i=%d, j=%d", i, j);

              segments.erase(segments.begin() + i);
              RichInfoSegs.erase(RichInfoSegs.begin() + i);
              seg_upbound--;
              i--;

              goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
            }
          }
          // 如果距离控制点足够远且不再障碍物中则无需继续搜索
          else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(j)).norm() >= RESOLUTION) // Unnecessary to search.
          {
            RichInfoSegs[i].second.base_point[j][0] = base_pt_reverse;
            RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
          }
          // 基点和控制点太近则删除这一段
          else
          {
            RCLCPP_WARN(rclcpp::get_logger("distinctiveTrajs"), "base_point and control point are too close!");
            cout << "base_point=" << RichInfoSegs[i].first.base_point[j][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(j).transpose() << endl;

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;

            goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
          }
        }

        // 1.3 Assign the base points to control points within [0, occ_start_id) and (occ_end_id, RichInfoSegs[i].first.size()-1].
        if (RichInfoSegs[i].second.size)
        {
          // 为片段起点之前和终点之后的控制点设置统一的基准点和方向，使得这些控制点在障碍物影响范围外时能够保持一致的路径属性
          for (int j = occ_start_id - 1; j >= 0; j--)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_start_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_start_id][0];
          }
          for (int j = occ_end_id + 1; j < RichInfoSegs[i].second.size; j++)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_end_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_end_id][0];
          }
        }

      exit_multi_loop3:;
      }
      // 片段只有一个控制点的情况
      else if (RichInfoSegs[i].first.size == 1)
      {
        cout << "i=" << i << " RichInfoSegs.size()=" << RichInfoSegs.size() << endl;
        cout << "RichInfoSegs[i].first.size=" << RichInfoSegs[i].first.size << endl;
        cout << "RichInfoSegs[i].first.direction.size()=" << RichInfoSegs[i].first.direction.size() << endl;
        cout << "RichInfoSegs[i].first.direction[0].size()=" << RichInfoSegs[i].first.direction[0].size() << endl;
        cout << "RichInfoSegs[i].first.points.cols()=" << RichInfoSegs[i].first.points.cols() << endl;
        cout << "RichInfoSegs[i].first.base_point.size()=" << RichInfoSegs[i].first.base_point.size() << endl;
        cout << "RichInfoSegs[i].first.base_point[0].size()=" << RichInfoSegs[i].first.base_point[0].size() << endl;
        Eigen::Vector3d base_vec_reverse = -RichInfoSegs[i].first.direction[0][0];
        Eigen::Vector3d base_pt_reverse = RichInfoSegs[i].first.points.col(0) + base_vec_reverse * (RichInfoSegs[i].first.base_point[0][0] - RichInfoSegs[i].first.points.col(0)).norm();

        if (grid_map_->getInflateOccupancy(base_pt_reverse)) // Search outward.
        {
          double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
          double l = RESOLUTION;
          for (; l <= l_upbound; l += RESOLUTION)
          {
            Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
            // cout << base_pt_temp.transpose() << endl;
            if (!grid_map_->getInflateOccupancy(base_pt_temp))
            {
              RichInfoSegs[i].second.base_point[0][0] = base_pt_temp;
              RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
              break;
            }
          }
          if (l > l_upbound)
          {
            RCLCPP_WARN(rclcpp::get_logger("distinctiveTrajs"), 
                        "Can't find the new base points at the opposite within the threshold, 2. i=%d", i);

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;
          }
        }
        else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(0)).norm() >= RESOLUTION) // Unnecessary to search.
        {
          RichInfoSegs[i].second.base_point[0][0] = base_pt_reverse;
          RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
        }
        else
        {
          RCLCPP_WARN(rclcpp::get_logger("distinctiveTrajs"), 
                        "base_point and control point are too close!, 2");
          cout << "base_point=" << RichInfoSegs[i].first.base_point[0][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(0).transpose() << endl;

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;
        }
      }
      else
      {
        segments.erase(segments.begin() + i);
        RichInfoSegs.erase(RichInfoSegs.begin() + i);
        seg_upbound--;
        i--;
      }
    }
    // cout << "A3" << endl;

    // Step 2. Assemble each segment to make up the new control point sequence.
    // 将每个分段组合起来，组成新的控制点序列
    if (seg_upbound == 0) // After the erase operation above, segment legth will decrease to 0 again.
    {
      std::vector<ControlPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    // 初始化选择向量
    std::vector<int> selection(seg_upbound);
    std::fill(selection.begin(), selection.end(), 0);
    selection[0] = -1; // init
    // 计算最大组合数
    int max_traj_nums = static_cast<int>(pow(VARIS, seg_upbound));
    for (int i = 0; i < max_traj_nums; i++)
    {
      // 2.1 Calculate the selection table.
      int digit_id = 0;
      selection[digit_id]++;
      // 生成一个选择表
      while (digit_id < seg_upbound && selection[digit_id] >= VARIS)
      {
        selection[digit_id] = 0;
        digit_id++;
        if (digit_id >= seg_upbound)
        {
          RCLCPP_ERROR(rclcpp::get_logger("distinctiveTrajs"), 
                        "Should not happen!!! digit_id=%d, seg_upbound=%d", digit_id, seg_upbound);
          
        }
        selection[digit_id]++;
      }

      // 2.2 Assign params according to the selection table.
      ControlPoints cpsOneSample;
      cpsOneSample.resize(cps_.size);
      cpsOneSample.clearance = cps_.clearance;
      int cp_id = 0, seg_id = 0, cp_of_seg_id = 0;
      // 遍历所有控制点
      while (/*seg_id < RichInfoSegs.size() ||*/ cp_id < cps_.size)
      {
        // cout << "A ";
        //  if ( seg_id >= RichInfoSegs.size() )
        //  {
        //    cout << "seg_id=" << seg_id << " RichInfoSegs.size()=" << RichInfoSegs.size() << endl;
        //  }
        //  if ( cp_id >= cps_.base_point.size() )
        //  {
        //    cout << "cp_id=" << cp_id << " cps_.base_point.size()=" << cps_.base_point.size() << endl;
        //  }
        //  if ( cp_of_seg_id >= RichInfoSegs[seg_id].first.base_point.size() )
        //  {
        //    cout << "cp_of_seg_id=" << cp_of_seg_id << " RichInfoSegs[seg_id].first.base_point.size()=" << RichInfoSegs[seg_id].first.base_point.size() << endl;
        //  }
        //  判断控制点是否在当前控制范围内
        //  如果不在则直接从原始控制点集中复制数据
        if (seg_id >= seg_upbound || cp_id < segments[seg_id].first || cp_id > segments[seg_id].second)
        {
          cpsOneSample.points.col(cp_id) = cps_.points.col(cp_id);
          cpsOneSample.base_point[cp_id] = cps_.base_point[cp_id];
          cpsOneSample.direction[cp_id] = cps_.direction[cp_id];
        }
        // 如果 cp_id 位于当前片段范围内，根据 selection[seg_id] 的值选择片段的第一套或第二套基准点和方向
        else if (cp_id >= segments[seg_id].first && cp_id <= segments[seg_id].second)
        {
          if (!selection[seg_id]) // zx-todo
          {
            cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].first.points.col(cp_of_seg_id);
            cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].first.base_point[cp_of_seg_id];
            cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].first.direction[cp_of_seg_id];
            cp_of_seg_id++;
          }
          else
          {
            if (RichInfoSegs[seg_id].second.size)
            {
              cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].second.points.col(cp_of_seg_id);
              cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].second.base_point[cp_of_seg_id];
              cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].second.direction[cp_of_seg_id];
              cp_of_seg_id++;
            }
            else
            {
              // Abandon this trajectory.
              goto abandon_this_trajectory;
            }
          }

          // 当遍历到片段的最后一个控制点时，将 cp_of_seg_id 重置为 0，并将 seg_id 指向下一个片段
          if (cp_id == segments[seg_id].second)
          {
            cp_of_seg_id = 0;
            seg_id++;
          }
        }
        else
        {
          RCLCPP_ERROR(rclcpp::get_logger("distinctiveTrajs"), 
                    "Shold not happen!!!!, cp_id=%d, seg_id=%d, segments.front().first=%d, segments.back().second=%d, segments[seg_id].first=%d, segments[seg_id].second=%d",
                    cp_id, seg_id, segments.front().first, segments.back().second, segments[seg_id].first, segments[seg_id].second);
        }

        cp_id++;
      }

      control_pts_buf.push_back(cpsOneSample);

    abandon_this_trajectory:;
    }

    return control_pts_buf;
  } // namespace ego_planner

  /* This function is very similar to check_collision_and_rebound().
   * It was written separately, just because I did it once and it has been running stably since March 2020.
   * But I will merge then someday.*/
  // 初始化控制点
  std::vector<std::pair<int, int>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
  {
    last_p4_guides_.clear();

    if (flag_first_init)
    {
      cps_.clearance = dist0_;
      cps_.resize(init_points.cols());
      cps_.points = init_points;
    }

    /*** Segment the initial trajectory according to obstacles ***/
    // 进入或离开障碍物稳定的时间间隔
    constexpr int ENOUGH_INTERVAL = 2;
    // 障碍物检测的步长
    double step_size = grid_map_->getResolution() / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 1.5;
    int in_id = -1, out_id = -1;
    vector<std::pair<int, int>> segment_ids;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    // 标识片段的起点和终点是否找到
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    int i_end = (int)init_points.cols() - order_ - ((int)init_points.cols() - 2 * order_) / 3; // only check closed 2/3 points.
    // 遍历所有点
    for (int i = order_; i <= i_end; ++i)
    {
      // cout << " *" << i-1 << "*" ;
      //  相邻两个点之间进行线性插值并检测障碍物
      for (double a = 1.0; a > 0.0; a -= step_size)
      {
        // TODO:没搞懂这是干嘛的
        occ = grid_map_->getInflateOccupancy(a * init_points.col(i - 1) + (1 - a) * init_points.col(i));
        // cout << " " << occ;
        //  cout << setprecision(5);
        //  cout << (a * init_points.col(i-1) + (1-a) * init_points.col(i)).transpose() << " occ1=" << occ << endl;

        // 进入障碍物
        if (occ && !last_occ)
        {
          if (same_occ_state_times > ENOUGH_INTERVAL || i == order_)
          {
            in_id = i - 1;
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false; // terminate in advance
        }
        // 离开障碍物
        else if (!occ && last_occ)
        {
          out_id = i;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        // 如果状态没发生变化
        else
        {
          ++same_occ_state_times;
        }

        // 如果已经离开障碍物，则结束
        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == (int)init_points.cols() - order_)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;

        // 重置标志位并存储信息
        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        }
      }
    }
    // cout << endl;

    // for (size_t i = 0; i < segment_ids.size(); i++)
    // {
    //   cout << "segment_ids=" << segment_ids[i].first << " ~ " << segment_ids[i].second << endl;
    // }

    // return in advance
    if (segment_ids.size() == 0)
    {
      vector<std::pair<int, int>> blank_ret;
      return blank_ret;
    }

    /*** a star search ***/
    // 在每个无障碍片段 segment_ids 的起点和终点之间寻找一条路径
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      // cout << "in=" << in.transpose() << " out=" << out.transpose() << endl;
      Eigen::Vector3d in(init_points.col(segment_ids[i].first)), out(init_points.col(segment_ids[i].second));
      if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in, out))
      {
        a_star_pathes.push_back(a_star_->getPath());
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("initControlPoints"), "a star error, force return!");
        vector<std::pair<int, int>> blank_ret;
        return blank_ret;
      }
    }

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    // 遍历每一段，设置边界
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

      if (i == 0) // first segment
      {
        id_low_bound = order_;
        if (segment_ids.size() > 1)
        {
          // 取当前片段结束点 segment_ids[0].second 和下一个片段起始点 segment_ids[1].first 的中间位置，向下取整后作为高边界
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          // 距末尾 order_ + 1 个点的位置
          id_up_bound = init_points.cols() - order_ - 1;
        }
      }
      // 末尾的边界
      else if (i == segment_ids.size() - 1) // last segment, i != 0 here
      {
        // 尾段的低边界为当前片段起点和上一个片段终点的中间位置，向上取整
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        // 距末尾点 order_ + 1 个点
        id_up_bound = init_points.cols() - order_ - 1;
      }
      else
      {
        // 低边界：为当前片段起点和前一片段终点的中间位置，向上取整
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        // 高边界：为当前片段终点和下一片段起点的中间位置，向下取整
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    // cout << "+++++++++" << endl;
    // for ( int j=0; j<bounds.size(); ++j )
    // {
    //   cout << bounds[j].first << "  " << bounds[j].second << endl;
    // }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
    // 控制点的最小比例
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient force
    // 控制点的最小数量
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust segment length ***/
      // 获取当前片段的点数
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      // cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
      if (num_points < minimum_points)
      {
        // 如果点数不够则在两侧扩展点，确保不能超过边界
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        adjusted_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;

        adjusted_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
      }
      else
      {
        adjusted_segment_ids[i].first = segment_ids[i].first;
        adjusted_segment_ids[i].second = segment_ids[i].second;
      }

      // cout << "final:" << "i = " << i << " first = " << adjusted_segment_ids[i].first << " second = " << adjusted_segment_ids[i].second << endl;
    }
    // 避免重叠
    for (size_t i = 1; i < adjusted_segment_ids.size(); i++) // Avoid overlap
    {
      if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first)
      {
        double middle = (double)(adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) / 2.0;
        adjusted_segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
        adjusted_segment_ids[i].first = static_cast<int>(middle + 1.1);
      }
    }

    // Used for return
    vector<std::pair<int, int>> final_segment_ids;

    /*** Assign data to each segment ***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      // step 1
      // 遍历该段所有控制点的id，并将标志位置为false
      for (int j = adjusted_segment_ids[i].first; j <= adjusted_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      // step 2
      // 初始化交点标记
      int got_intersection_id = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        // 计算控制点的方向向量
        Eigen::Vector3d ctrl_pts_law(init_points.col(j + 1) - init_points.col(j - 1)), intersection_point;
        // A*路径的中点
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        // 路径点和控制点之间的点积，判断当前路径点在控制点方向 ctrl_pts_law 的哪一侧
        double val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law), last_val = val;
        while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
        {
          last_Astar_id = Astar_id;

          // 根据 val 的正负决定 Astar_id 的移动方向
          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            // 寻找可能的交点，TODO:还没搞懂
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(init_points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            // cout << "i=" << i << " j=" << j << " Astar_id=" << Astar_id << " last_Astar_id=" << last_Astar_id << " intersection_point = " << intersection_point.transpose() << endl;

            got_intersection_id = j;
            break;
          }
        }

        if (got_intersection_id >= 0)
        {
          // 计算交点到控制点之间的距离
          double length = (intersection_point - init_points.col(j)).norm();
          if (length > 1e-5)
          {
            cps_.flag_temp[j] = true;
            // 逐步进行采样
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
            {
              // 通过线性插值计算采样点位置
              occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * init_points.col(j));

              if (occ || a < grid_map_->getResolution())
              {
                if (occ)
                  a += grid_map_->getResolution();
                // 记录基点并计算到控制点的方向
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * init_points.col(j));
                cps_.direction[j].push_back((intersection_point - init_points.col(j)).normalized());
                // cout << "A " << j << endl;
                break;
              }
            }
          }
          else
          {
            got_intersection_id = -1;
          }
        }
      }

      /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      // 当轨迹片段只有两个控制点（即长度过短）时，A* 路径的方向可能与控制点的方向不一致，导致梯度方向相反
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        // 计算控制点方向向量和中点
        Eigen::Vector3d ctrl_pts_law(init_points.col(segment_ids[i].second) - init_points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (init_points.col(segment_ids[i].second) + init_points.col(segment_ids[i].first)) / 2;
        // 计算A*中点并判断方向，同上
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), last_val = val;
        while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
        {
          last_Astar_id = Astar_id;

          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          // 和上面不一样，这里减去的是中点
          val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            // 计算交点
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            // 满足距离要求则存储相关信息
            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(init_points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      // step 3
      if (got_intersection_id >= 0)
      {
        // 遍历交点之后的控制点
        for (int j = got_intersection_id + 1; j <= adjusted_segment_ids[i].second; ++j)
          // 如果没被标记则将前一个控制点的信息给该点
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
            // cout << "AAA " << j << endl;
          }

        // 遍历交点前的控制点，如果没被标记用后一个控制点的数据赋值
        for (int j = got_intersection_id - 1; j >= adjusted_segment_ids[i].first; --j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
            // cout << "AAAA " << j << endl;
          }

        final_segment_ids.push_back(adjusted_segment_ids[i]);
      }
      else
      {
        // Just ignore, it does not matter ^_^.
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    return final_segment_ids;
  }

  // 急停情况下提前退出
  int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    // cout << "k=" << k << endl;
    // cout << "opt->flag_continue_to_optimize_=" << opt->flag_continue_to_optimize_ << endl;
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  // 利用combineCostRebound计算损失
  double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRebound(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

  // 利用combineCostRefine计算优化后的损失
  double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRefine(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

  // 几个计算损失的函数
  void BsplineOptimizer::calcSwarmCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_ - (double)(q.cols() - 2 * order_) * 1.0 / 3.0; // Only check the first 2/3 points
    const double CLEARANCE = swarm_clearance_ * 2;
    double t_now = rclcpp::Clock().now().seconds();
    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

    for (int i = order_; i < end_idx; i++)
    {
      double glb_time = t_now + ((double)(order_ - 1) / 2 + (i - order_ + 1)) * bspline_interval_;

      for (size_t id = 0; id < swarm_trajs_->size(); id++)
      {
        if ((swarm_trajs_->at(id).drone_id != (int)id) || swarm_trajs_->at(id).drone_id == drone_id_)
        {
          continue;
        }

        double traj_i_satrt_time = swarm_trajs_->at(id).start_time_.seconds();
        if (glb_time < traj_i_satrt_time + swarm_trajs_->at(id).duration_ - 0.1)
        {
          /* def cost=(c-sqrt([Q-O]'D[Q-O]))^2, D=[1/b^2,0,0;0,1/b^2,0;0,0,1/a^2] */
          Eigen::Vector3d swarm_prid = swarm_trajs_->at(id).position_traj_.evaluateDeBoorT(glb_time - traj_i_satrt_time);
          Eigen::Vector3d dist_vec = cps_.points.col(i) - swarm_prid;
          double ellip_dist = sqrt(dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2);
          double dist_err = CLEARANCE - ellip_dist;

          Eigen::Vector3d dist_grad = cps_.points.col(i) - swarm_prid;
          Eigen::Vector3d Coeff;
          Coeff(0) = -2 * (CLEARANCE / ellip_dist - 1) * inv_b2;
          Coeff(1) = Coeff(0);
          Coeff(2) = -2 * (CLEARANCE / ellip_dist - 1) * inv_a2;

          if (dist_err < 0)
          {
            /* do nothing */
          }
          else
          {
            cost += pow(dist_err, 2);
            gradient.col(i) += (Coeff.array() * dist_grad.array()).matrix();
          }

          if (min_ellip_dist_ > dist_err)
          {
            min_ellip_dist_ = dist_err;
          }
        }
      }
    }
  }

  void BsplineOptimizer::calcMovingObjCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_;
    constexpr double CLEARANCE = 1.5;
    double t_now = rclcpp::Clock().now().seconds();

    for (int i = order_; i < end_idx; i++)
    {
      double time = ((double)(order_ - 1) / 2 + (i - order_ + 1)) * bspline_interval_;

      for (int id = 0; id < moving_objs_->getObjNums(); id++)
      {
        Eigen::Vector3d obj_prid = moving_objs_->evaluateConstVel(id, t_now + time);
        double dist = (cps_.points.col(i) - obj_prid).norm();
        // cout /*<< "cps_.points.col(i)=" << cps_.points.col(i).transpose()*/ << " moving_objs_=" << obj_prid.transpose() << " dist=" << dist << endl;
        double dist_err = CLEARANCE - dist;
        Eigen::Vector3d dist_grad = (cps_.points.col(i) - obj_prid).normalized();

        if (dist_err < 0)
        {
          /* do nothing */
        }
        else
        {
          cost += pow(dist_err, 2);
          gradient.col(i) += -2.0 * dist_err * dist_grad;
        }
      }
      // cout << "time=" << time << " i=" << i << " order_=" << order_ << " end_idx=" << end_idx << endl;
      // cout << "--" << endl;
    }
    // cout << "---------------" << endl;
  }

  void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                 Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_;
    double demarcation = cps_.clearance;
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

    force_stop_type_ = DONT_STOP;
    if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1) // 0.1 is an experimental value that indicates the trajectory is smooth enough.
    {
      check_collision_and_rebound();
    }

    /*** calculate distance cost and gradient ***/
    for (auto i = order_; i < end_idx; ++i)
    {
      for (size_t j = 0; j < cps_.direction[i].size(); ++j)
      {
        double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
        double dist_err = cps_.clearance - dist;
        Eigen::Vector3d dist_grad = cps_.direction[i][j];

        if (dist_err < 0)
        {
          /* do nothing */
        }
        else if (dist_err < demarcation)
        {
          cost += pow(dist_err, 3);
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        }
        else
        {
          cost += a * dist_err * dist_err + b * dist_err + c;
          gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
        }
      }
    }
  }

  void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {

    cost = 0.0;

    int end_idx = q.cols() - order_;

    // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
    double a2 = 25, b2 = 1;
    for (auto i = order_ - 1; i < end_idx + 1; ++i)
    {
      Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
      Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();

      double xdotv = x.dot(v);
      Eigen::Vector3d xcrossv = x.cross(v);

      double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
      cost += f;

      Eigen::Matrix3d m;
      m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
      Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

      gradient.col(i - 1) += df_dx / 6;
      gradient.col(i) += 4 * df_dx / 6;
      gradient.col(i + 1) += df_dx / 6;
    }
  }

  void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
  {

    cost = 0.0;

    if (falg_use_jerk)
    {
      Eigen::Vector3d jerk, temp_j;

      for (int i = 0; i < q.cols() - 3; i++)
      {
        /* evaluate jerk */
        jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
        cost += jerk.squaredNorm();
        temp_j = 2.0 * jerk;
        /* jerk gradient */
        gradient.col(i + 0) += -temp_j;
        gradient.col(i + 1) += 3.0 * temp_j;
        gradient.col(i + 2) += -3.0 * temp_j;
        gradient.col(i + 3) += temp_j;
      }
    }
    else
    {
      Eigen::Vector3d acc, temp_acc;

      for (int i = 0; i < q.cols() - 2; i++)
      {
        /* evaluate acc */
        acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
        cost += acc.squaredNorm();
        temp_acc = 2.0 * acc;
        /* acc gradient */
        gradient.col(i + 0) += temp_acc;
        gradient.col(i + 1) += -2.0 * temp_acc;
        gradient.col(i + 2) += temp_acc;
      }
    }
  }

  void BsplineOptimizer::calcTerminalCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
    cost = 0.0;

    // zero cost and gradient in hard constraints
    Eigen::Vector3d q_3, q_2, q_1, dq;
    q_3 = q.col(q.cols() - 3);
    q_2 = q.col(q.cols() - 2);
    q_1 = q.col(q.cols() - 1);

    dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - local_target_pt_;
    cost += dq.squaredNorm();

    gradient.col(q.cols() - 3) += 2 * dq * (1 / 6.0);
    gradient.col(q.cols() - 2) += 2 * dq * (4 / 6.0);
    gradient.col(q.cols() - 1) += 2 * dq * (1 / 6.0);
  }

  void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                             Eigen::MatrixXd &gradient)
  {

    // #define SECOND_DERIVATIVE_CONTINOUS

#ifdef SECOND_DERIVATIVE_CONTINOUS

    cost = 0.0;
    double demarcation = 1.0; // 1m/s, 1m/s/s
    double ar = 3 * demarcation, br = -3 * pow(demarcation, 2), cr = pow(demarcation, 3);
    double al = ar, bl = -br, cl = cr;

    /* abbreviation */
    double ts, ts_inv2, ts_inv3;
    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;
    ts_inv3 = 1 / ts / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_ + demarcation)
        {
          double diff = vi(j) - max_vel_;
          cost += (ar * diff * diff + br * diff + cr) * ts_inv3; // multiply ts_inv3 to make vel and acc has similar magnitude

          double grad = (2.0 * ar * diff + br) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) > max_vel_)
        {
          double diff = vi(j) - max_vel_;
          cost += pow(diff, 3) * ts_inv3;
          ;

          double grad = 3 * diff * diff / ts * ts_inv3;
          ;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -(max_vel_ + demarcation))
        {
          double diff = vi(j) + max_vel_;
          cost += (al * diff * diff + bl * diff + cl) * ts_inv3;

          double grad = (2.0 * al * diff + bl) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -max_vel_)
        {
          double diff = vi(j) + max_vel_;
          cost += -pow(diff, 3) * ts_inv3;

          double grad = -3 * diff * diff / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_ + demarcation)
        {
          double diff = ai(j) - max_acc_;
          cost += ar * diff * diff + br * diff + cr;

          double grad = (2.0 * ar * diff + br) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) > max_acc_)
        {
          double diff = ai(j) - max_acc_;
          cost += pow(diff, 3);

          double grad = 3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -(max_acc_ + demarcation))
        {
          double diff = ai(j) + max_acc_;
          cost += al * diff * diff + bl * diff + cl;

          double grad = (2.0 * al * diff + bl) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -max_acc_)
        {
          double diff = ai(j) + max_acc_;
          cost += -pow(diff, 3);

          double grad = -3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

#else

    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      // cout << "temp_v * vi=" ;
      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_)
        {
          // cout << "zx-todo VEL" << endl;
          // cout << vi(j) << endl;
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      // cout << "temp_a * ai=" ;
      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_)
        {
          // cout << "zx-todo ACC" << endl;
          // cout << ai(j) << endl;
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
      // cout << endl;
    }

#endif
  }
  /*                    上面七个都是计算损失的函数           */

  // 检查是否有障碍物？
  bool BsplineOptimizer::check_collision_and_rebound(void)
  {

    int end_idx = cps_.size - order_;

    /*** Check and segment the initial trajectory according to obstacles ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    int i_end = end_idx - (end_idx - order_) / 3;
    for (int i = order_ - 1; i <= i_end; ++i)
    {

      bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));

      /*** check if the new collision will be valid ***/
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          cout.precision(2);
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
          {
            occ = false; // Not really takes effect, just for better hunman understanding.
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;

        int j;
        for (j = i - 1; j >= 0; --j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));
          if (!occ)
          {
            in_id = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          RCLCPP_ERROR(rclcpp::get_logger("check_collision_and_rebound"), "ERROR! the drone is in obstacle. This should not happen.");
          in_id = 0;
        }

        for (j = i + 1; j < cps_.size; ++j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        if (j >= cps_.size) // fail to get the obs free point
        {
          RCLCPP_WARN(rclcpp::get_logger("check_collision_and_rebound"), 
                      "WARN! terminal point of the current trajectory is in obstacle, skip this planning.");

          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }

    if (flag_new_obs_valid)
    {
      static uint64_t p4_astar_call_id = 0;
      vector<vector<Eigen::Vector3d>> a_star_pathes;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        /*** a star search ***/
        Eigen::Vector3d in(cps_.points.col(segment_ids[i].first)), out(cps_.points.col(segment_ids[i].second));
        const uint64_t astar_call_id = ++p4_astar_call_id;
        const bool original_success = a_star_->AstarSearchOriginal(/*(in-out).norm()/10+0.05*/ 0.1, in, out);
        if (original_success)
        {
          const std::vector<Eigen::Vector3d> original_path = a_star_->getPath();
          std::vector<Eigen::Vector3d> risk_path;
          std::vector<Eigen::Vector3d> selected_path = original_path;
          bool selected_risk_path = false;
          P4AStarMetrics metrics = a_star_->getLastP4Metrics();
          metrics.original_path_length = pathLength(original_path);
          metrics.risk_path_length = 0.0;
          metrics.path_length_ratio = 1.0;
          metrics.risk_enabled = false;

          if (!p4_config_.enable_risk_aware_astar)
          {
            metrics.fallback_reason = "p4_disabled";
          }
          else if (!a_star_->hasRiskSnapshot())
          {
            metrics.fallback_reason = p4_config_.fallback_to_original_when_risk_not_ready
                                          ? "snapshot_unavailable"
                                          : "snapshot_unavailable";
          }
          else
          {
            const bool risk_success = a_star_->AstarSearchRiskAware(0.1, in, out);
            P4AStarMetrics risk_metrics = a_star_->getLastP4Metrics();
            risk_metrics.original_path_length = metrics.original_path_length;
            risk_metrics.risk_enabled = true;
            if (risk_success)
            {
              risk_path = a_star_->getPath();
              risk_metrics.risk_path_length = pathLength(risk_path);
              risk_metrics.path_length_ratio =
                  metrics.original_path_length > 1.0e-9
                      ? risk_metrics.risk_path_length / metrics.original_path_length
                      : 1.0;
              if (risk_metrics.path_length_ratio > p4_config_.max_extra_path_ratio)
              {
                risk_metrics.fallback_reason = "path_length_ratio_exceeded";
              }
              else
              {
                selected_path = risk_path;
                selected_risk_path = true;
                risk_metrics.fallback_reason = "risk_path_selected";
              }
            }
            else
            {
              risk_metrics.risk_path_length = 0.0;
              risk_metrics.path_length_ratio = 0.0;
              risk_metrics.fallback_reason = "risk_search_failed";
            }
            metrics = risk_metrics;
          }
          a_star_->recordP4GuideMetrics(metrics);
          P4GuideViz guide;
          guide.original_path = original_path;
          guide.risk_path = risk_path;
          guide.selected_path = selected_path;
          guide.segment_start = in;
          guide.segment_end = out;
          guide.metrics = metrics;
          guide.risk_selected = selected_risk_path;
          last_p4_guides_.push_back(guide);
          writeP4Csv(p4_config_, metrics, rclcpp::Clock().now().seconds(), astar_call_id, static_cast<int>(i));
          a_star_pathes.push_back(selected_path);
        }
        else
        {
          RCLCPP_ERROR(rclcpp::get_logger("check_collision_and_rebound"), "a star error");
          segment_ids.erase(segment_ids.begin() + i);
          i--;
        }
      }

      for (size_t i = 1; i < segment_ids.size(); i++) // Avoid overlap
      {
        if (segment_ids[i - 1].second >= segment_ids[i].first)
        {
          double middle = (double)(segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
          segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
          segment_ids[i].first = static_cast<int>(middle + 1.1);
        }
      }

      /*** Assign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        // step 1
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
          while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
          {
            last_Astar_id = Astar_id;

            if (val >= 0)
              --Astar_id;
            else
              ++Astar_id;

            val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            // cout << val << endl;

            if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
            {
              intersection_point =
                  a_star_pathes[i][Astar_id] +
                  ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                  );

              got_intersection_id = j;
              break;
            }
          }

          if (got_intersection_id >= 0)
          {
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              cps_.flag_temp[j] = true;
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

                if (occ || a < grid_map_->getResolution())
                {
                  if (occ)
                    a += grid_map_->getResolution();
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                  break;
                }
              }
            }
            else
            {
              got_intersection_id = -1;
            }
          }
        }

        // step 3
        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          RCLCPP_WARN(rclcpp::get_logger("check_collision_and_rebound"), "Failed to generate direction. It doesn't matter.");
      }

      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;
  }

  // 设置时间间隔ts，调用rebound_optimize(final_cost)将轨迹推出障碍物，得到最优的无碰撞轨迹，并将其控制点赋值给optimal_points
  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
  {
    setBsplineInterval(ts);

    double final_cost;
    bool flag_success = rebound_optimize(final_cost);

    optimal_points = cps_.points;

    return flag_success;
  }

  // 设置初始控制点control_points、时间间隔ts，调用rebound_optimize(final_cost)将轨迹推出障碍物，
  // 得到最优的无碰撞轨迹，并将其控制点赋值给optimal_points
  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double &final_cost, const ControlPoints &control_points, double ts)
  {
    // 将时间间隔存储到成员变量
    setBsplineInterval(ts);

    cps_ = control_points;

    bool flag_success = rebound_optimize(final_cost);

    optimal_points = cps_.points;

    return flag_success;
  }

  // 设置初始控制点init_points、时间间隔ts，调用refine_optimize()重新分配时间，得到最优的动力学可行轨迹，并将其控制点赋值给optimal_points
  bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
  {

    // 将控制点数据存储到成员变量
    setControlPoints(init_points);
    setBsplineInterval(ts);

    bool flag_success = refine_optimize();

    optimal_points = cps_.points;

    return flag_success;
  }

  // 使用L-BFGS方法对目标函数进行优化，得到光滑、无碰撞、动力学可行、与其他无人机碰撞、结束项的轨迹。
  bool BsplineOptimizer::rebound_optimize(double &final_cost)
  {
    iter_num_ = 0;
    int start_id = order_;
    // int end_id = this->cps_.size - order_; //Fixed end
    int end_id = this->cps_.size; // Free end
    // 变量个数
    variable_num_ = 3 * (end_id - start_id);

    rclcpp::Time t0 = rclcpp::Clock().now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;
    ;
    bool flag_force_return, flag_occ, success;
    new_lambda2_ = lambda2_;
    constexpr int MAX_RESART_NUMS_SET = 3;
    do
    {
      /* ---------- prepare ---------- */
      min_cost_ = std::numeric_limits<double>::max();
      min_ellip_dist_ = INIT_min_ellip_dist_;
      iter_num_ = 0;
      flag_force_return = false;
      flag_occ = false;
      success = false;

      // 控制点数组初始化
      double q[variable_num_];
      memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

      // 初始化L-BFGS算法的参数
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16; // 算法保存的历史优化步数
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.01;

      /* ---------- optimize ---------- */
      t1 = rclcpp::Clock().now();
      // 执行优化
      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
      t2 = rclcpp::Clock().now();
      double time_ms = (t2 - t1).seconds() * 1000;
      double total_time_ms = (t2 - t0).seconds() * 1000;

      /* ---------- success temporary, check collision again ---------- */
      // 收敛、达到最大迭代次数、已达到最小值或被停止，则进入碰撞检测阶段
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        // ROS_WARN("Solver error in planning!, return = %s", lbfgs::lbfgs_strerror(result));
        flag_force_return = false;

        /*** collision check, phase 1 ***/
        if ((min_ellip_dist_ != INIT_min_ellip_dist_) && (min_ellip_dist_ > swarm_clearance_))
        {
          success = false;
          restart_nums++;
          initControlPoints(cps_.points, false);
          new_lambda2_ *= 2; // 提高规避权重

          printf("\033[32miter(+1)=%d,time(ms)=%5.3f, swarm too close, keep optimizing\n\033[0m", iter_num_, time_ms);

          continue;
        }

        /*** collision check, phase 2 ***/
        // 创建均匀的B样条曲线
        UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
        // 开始时间，结束时间
        double tm, tmp;
        traj.getTimeSpan(tm, tmp);
        // 计算时间步长
        double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution());
        // 遍历轨迹的前2/3部分进行障碍物检测
        for (double t = tm; t < tmp * 2 / 3; t += t_step) // Only check the closest 2/3 partition of the whole trajectory.
        {
          flag_occ = grid_map_->getInflateOccupancy(traj.evaluateDeBoorT(t));
          if (flag_occ)
          {
            // cout << "hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

            // 如果在前三个控制点范围内检测到了碰撞则视为不可行
            if (t <= bspline_interval_) // First 3 control points in obstacles!
            {
              // cout << cps_.points.col(1).transpose() << "\n"
              //      << cps_.points.col(2).transpose() << "\n"
              //      << cps_.points.col(3).transpose() << "\n"
              //      << cps_.points.col(4).transpose() << endl;
              RCLCPP_WARN(rclcpp::get_logger("rebound_optimize"), "First 3 control points in obstacles! return false, t=%f", t);
              return false;
            }

            break;
          }
        }

        // cout << "XXXXXX" << ((cps_.points.col(cps_.points.cols()-1) + 4*cps_.points.col(cps_.points.cols()-2) + cps_.points.col(cps_.points.cols()-3))/6 - local_target_pt_).norm() << endl;

        /*** collision check, phase 3 ***/
// #define USE_SECOND_CLEARENCE_CHECK
#ifdef USE_SECOND_CLEARENCE_CHECK
        bool flag_cls_xyp, flag_cls_xyn, flag_cls_zp, flag_cls_zn;
        Eigen::Vector3d start_end_vec = traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm);
        Eigen::Vector3d offset_xy(-start_end_vec(0), start_end_vec(1), 0);
        offset_xy.normalize();
        Eigen::Vector3d offset_z = start_end_vec.cross(offset_xy);
        offset_z.normalize();
        offset_xy *= cps_.clearance / 2;
        offset_z *= cps_.clearance / 2;

        Eigen::MatrixXd check_pts(cps_.points.rows(), cps_.points.cols());
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts.col(i) = cps_.points.col(i);
          check_pts(0, i) += offset_xy(0);
          check_pts(1, i) += offset_xy(1);
          check_pts(2, i) += offset_xy(2);
        }
        flag_cls_xyp = initControlPoints(check_pts, false).size() > 0;
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts(0, i) -= 2 * offset_xy(0);
          check_pts(1, i) -= 2 * offset_xy(1);
          check_pts(2, i) -= 2 * offset_xy(2);
        }
        flag_cls_xyn = initControlPoints(check_pts, false).size() > 0;
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts(0, i) += offset_xy(0) + offset_z(0);
          check_pts(1, i) += offset_xy(1) + offset_z(1);
          check_pts(2, i) += offset_xy(2) + offset_z(2);
        }
        flag_cls_zp = initControlPoints(check_pts, false).size() > 0;
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts(0, i) -= 2 * offset_z(0);
          check_pts(1, i) -= 2 * offset_z(1);
          check_pts(2, i) -= 2 * offset_z(2);
        }
        flag_cls_zn = initControlPoints(check_pts, false).size() > 0;
        if ((flag_cls_xyp ^ flag_cls_xyn) || (flag_cls_zp ^ flag_cls_zn))
          flag_occ = true;
#endif

        // 如果没有检测到碰撞视为优化成功
        if (!flag_occ)
        {
          printf("\033[32miter(+1)=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
          success = true;
        }
        // 如果有碰撞则重新初始化
        else // restart
        {
          restart_nums++;
          initControlPoints(cps_.points, false);
          new_lambda2_ *= 2;

          printf("\033[32miter(+1)=%d,time(ms)=%5.3f, collided, keep optimizing\n\033[0m", iter_num_, time_ms);
        }
      }
      // 如果优化被强制取消
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        flag_force_return = true;
        rebound_times++;
        cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." << endl;
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("rebound_optimize"), 
                                        "Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
        // while (rclcpp::ok());
      }

    } while (
        ((flag_occ || ((min_ellip_dist_ != INIT_min_ellip_dist_) && (min_ellip_dist_ > swarm_clearance_))) && restart_nums < MAX_RESART_NUMS_SET) ||
        (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

    return success;
  }

  // 使用L-BFGS方法对目标函数进行优化，得到重新分配时间后，光滑、拟合较好、动力学可行的轨迹。
  bool BsplineOptimizer::refine_optimize()
  {
    iter_num_ = 0;
    int start_id = order_;
    int end_id = this->cps_.points.cols() - order_;
    variable_num_ = 3 * (end_id - start_id);

    double q[variable_num_];
    double final_cost;

    memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

    double origin_lambda4 = lambda4_;
    bool flag_safe = true;
    int iter_count = 0;
    do
    {
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.001;

      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        // pass
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("refine_optimize"), 
                                        "Solver error in refining!, return = %d, %s", result, lbfgs::lbfgs_strerror(result));
      }

      // 使用优化后的控制点创建新的轨迹
      UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
      double tm, tmp;
      traj.getTimeSpan(tm, tmp);
      double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution()); // Step size is defined as the maximum size that can passes throgth every gird.
      for (double t = tm; t < tmp * 2 / 3; t += t_step)
      {
        if (grid_map_->getInflateOccupancy(traj.evaluateDeBoorT(t)))
        {
          // cout << "Refined traj hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

          // 将ref_pts存储为矩阵形式
          Eigen::MatrixXd ref_pts(ref_pts_.size(), 3);
          for (size_t i = 0; i < ref_pts_.size(); i++)
          {
            ref_pts.row(i) = ref_pts_[i].transpose();
          }

          flag_safe = false;
          break;
        }
      }

      // 如果存在碰撞则调整参数并重新迭代
      if (!flag_safe)
        lambda4_ *= 2;

      iter_count++;
    } while (!flag_safe && iter_count <= 0);

    lambda4_ = origin_lambda4;

    // cout << "iter_num_=" << iter_num_ << endl;

    return flag_safe;
  }

  bool BsplineOptimizer::cubicBasisForTime(double t, int control_point_count,
                                           int &first_control_point,
                                           double weights[4]) const
  {
    if (order_ != 3 || control_point_count < 4 || bspline_interval_ <= 0.0 ||
        !std::isfinite(t))
    {
      return false;
    }

    const double duration = static_cast<double>(control_point_count - order_) * bspline_interval_;
    if (duration <= 0.0)
    {
      return false;
    }

    const double clamped_t = std::min(std::max(0.0, t), duration);
    int k = order_;
    double s = 0.0;
    if (clamped_t >= duration)
    {
      k = control_point_count - 1;
      s = 1.0;
    }
    else
    {
      const double span = std::floor(clamped_t / bspline_interval_);
      k = order_ + static_cast<int>(span);
      k = std::min(std::max(order_, k), control_point_count - 1);
      s = (clamped_t - static_cast<double>(k - order_) * bspline_interval_) / bspline_interval_;
      s = std::min(std::max(0.0, s), 1.0);
    }

    first_control_point = k - order_;
    if (first_control_point < 0 || first_control_point + 3 >= control_point_count)
    {
      return false;
    }

    const double s2 = s * s;
    const double s3 = s2 * s;
    weights[0] = (1.0 - 3.0 * s + 3.0 * s2 - s3) / 6.0;
    weights[1] = (4.0 - 6.0 * s2 + 3.0 * s3) / 6.0;
    weights[2] = (1.0 + 3.0 * s + 3.0 * s2 - 3.0 * s3) / 6.0;
    weights[3] = s3 / 6.0;
    return true;
  }

  void BsplineOptimizer::calcIntegrityTrajectoryCost(const Eigen::MatrixXd &q, double &cost,
                                                     Eigen::MatrixXd &gradient,
                                                     P1IntegrityMetrics &metrics)
  {
    cost = 0.0;
    gradient.setZero();
    metrics = P1IntegrityMetrics{};
    last_p1_viz_samples_.clear();

    if (!risk_snapshot_)
    {
      metrics.fallback_reason = "snapshot_unavailable";
      return;
    }

    metrics.snapshot_generation_id = risk_snapshot_->generation_id();

    if (order_ != 3)
    {
      metrics.fallback_reason = "unsupported_order";
      return;
    }

    if (q.cols() < order_ + 1 || bspline_interval_ <= 0.0)
    {
      metrics.fallback_reason = "invalid_trajectory";
      return;
    }

    const double duration = static_cast<double>(q.cols() - order_) * bspline_interval_;
    if (duration <= 0.0 || !std::isfinite(duration))
    {
      metrics.fallback_reason = "invalid_duration";
      return;
    }

    const double raw_dt = std::max(p1_config_.sample_dt_min_s,
                                   bspline_interval_ * std::max(0.0, p1_config_.sample_dt_scale));
    if (!(raw_dt > 0.0) || !std::isfinite(raw_dt))
    {
      metrics.fallback_reason = "invalid_sample_dt";
      return;
    }

    int sample_count = static_cast<int>(std::ceil(duration / raw_dt));
    sample_count = std::max(1, sample_count);
    if (p1_config_.max_samples_per_eval > 0)
    {
      sample_count = std::min(sample_count, p1_config_.max_samples_per_eval);
    }
    metrics.sample_count = sample_count;

    UniformBspline traj(q, order_, bspline_interval_);
    const double denom = sample_count > 1 ? static_cast<double>(sample_count - 1) : 1.0;
    const bool small_penalty = p1_config_.unknown_policy == "small_penalty";
    const double cost_max = std::max(0.0, p1_config_.integrity_cost_max);
    const double grad_max = std::max(0.0, p1_config_.integrity_grad_norm_max);

    for (int sample_id = 0; sample_id < sample_count; ++sample_id)
    {
      const double t = sample_count > 1 ? duration * static_cast<double>(sample_id) / denom : 0.0;
      const Eigen::Vector3d p = traj.evaluateDeBoorT(t);
      P1IntegrityVizSample viz;
      viz.position = p;
      viz.t_s = t;

      iap::RiskCostSample sample;
      const bool hit = risk_snapshot_->queryCost(p, risk_query_base_time_s_ + t, &sample);
      if (!hit || !sample.valid || !std::isfinite(sample.cost))
      {
        metrics.miss_count++;
        if (sample.stale)
        {
          metrics.stale_count++;
        }
        viz.hit = false;
        viz.stale = sample.stale;
        viz.unknown = true;
        viz.reason = hit ? sample.reason : "query_miss";
        if (small_penalty)
        {
          viz.cost = std::max(0.0, p1_config_.unknown_soft_penalty);
        }
        last_p1_viz_samples_.push_back(viz);
        if (small_penalty)
        {
          cost += std::max(0.0, p1_config_.unknown_soft_penalty);
        }
        continue;
      }

      metrics.hit_count++;
      if (sample.stale)
      {
        metrics.stale_count++;
      }

      double sample_cost = std::min(std::max(0.0, sample.cost), cost_max);
      Eigen::Vector3d sample_grad = sample.grad;
      if (!sample_grad.allFinite())
      {
        sample_grad.setZero();
      }
      const double grad_norm = sample_grad.norm();
      if (grad_max > 0.0 && grad_norm > grad_max)
      {
        sample_grad *= grad_max / grad_norm;
        metrics.clipped_grad_count++;
      }

      cost += sample_cost;
      viz.hit = true;
      viz.stale = sample.stale;
      viz.unknown = false;
      viz.cost = sample_cost;
      viz.grad = sample_grad;
      viz.push = -p1_config_.lambda_integrity * sample_grad;
      viz.reason = sample.reason;
      last_p1_viz_samples_.push_back(viz);

      int first_control_point = 0;
      double weights[4] = {0.0, 0.0, 0.0, 0.0};
      if (cubicBasisForTime(t, static_cast<int>(q.cols()), first_control_point, weights))
      {
        for (int i = 0; i < 4; ++i)
        {
          gradient.col(first_control_point + i) += weights[i] * sample_grad;
        }
      }
      else
      {
        metrics.miss_count++;
      }
    }

    if (sample_count > 0)
    {
      const double inv_count = 1.0 / static_cast<double>(sample_count);
      cost *= inv_count;
      gradient *= inv_count;
      metrics.miss_ratio = static_cast<double>(metrics.miss_count) / static_cast<double>(sample_count);
      metrics.stale_ratio = static_cast<double>(metrics.stale_count) / static_cast<double>(sample_count);
    }

    metrics.f_integrity = cost;
    metrics.weighted_f_integrity = p1_config_.lambda_integrity * cost;
    metrics.grad_norm_integrity = gradient.norm();
    metrics.weighted_grad_integrity_norm = std::abs(p1_config_.lambda_integrity) * metrics.grad_norm_integrity;
    metrics.fallback_reason = metrics.hit_count > 0 || small_penalty ? "ok" : "no_valid_samples";
  }

  void BsplineOptimizer::writeP1DebugCsv(const P1IntegrityMetrics &metrics) const
  {
    if (!p1_config_.debug_csv_enable || p1_config_.debug_csv_path.empty())
    {
      return;
    }

    std::ifstream existing(p1_config_.debug_csv_path);
    const bool write_header = !existing.good() || existing.peek() == std::ifstream::traits_type::eof();
    existing.close();

    std::ofstream out(p1_config_.debug_csv_path, std::ios::app);
    if (!out.good())
    {
      return;
    }
    if (write_header)
    {
      out << "stamp,lbfgs_iter,snapshot_generation_id,query_base_time_s,"
             "sample_count,hit_count,miss_count,stale_count,miss_ratio,stale_ratio,"
             "f_integrity,weighted_f_integrity,grad_norm_integrity,grad_norm_original,"
             "grad_ratio,clipped_grad_count,fallback_reason,applied_to_objective\n";
    }

    out << rclcpp::Clock().now().seconds() << ','
        << iter_num_ << ','
        << metrics.snapshot_generation_id << ','
        << risk_query_base_time_s_ << ','
        << metrics.sample_count << ','
        << metrics.hit_count << ','
        << metrics.miss_count << ','
        << metrics.stale_count << ','
        << metrics.miss_ratio << ','
        << metrics.stale_ratio << ','
        << metrics.f_integrity << ','
        << metrics.weighted_f_integrity << ','
        << metrics.grad_norm_integrity << ','
        << metrics.grad_norm_original << ','
        << metrics.grad_ratio << ','
        << metrics.clipped_grad_count << ','
        << metrics.fallback_reason << ','
        << (metrics.applied_to_objective ? 1 : 0) << '\n';
  }

  std::string BsplineOptimizer::p1AcceptedTrajectoryRiskProfilePath() const
  {
    if (p1_config_.debug_csv_path.empty())
    {
      return kP1AcceptedProfileCsvName;
    }
    return siblingPath(p1_config_.debug_csv_path, kP1AcceptedProfileCsvName);
  }

  bool BsplineOptimizer::writeP1AcceptedTrajectoryRiskProfile(
      UniformBspline trajectory,
      const uint64_t profile_seq,
      const uint64_t trajectory_id,
      const double stamp_s) const
  {
    if (!p1_config_.debug_csv_enable || p1_config_.debug_csv_path.empty() ||
        !risk_snapshot_)
    {
      return false;
    }

    const double duration = trajectory.getTimeSum();
    if (!std::isfinite(duration) || duration < 0.0)
    {
      return false;
    }

    const std::string profile_path = p1AcceptedTrajectoryRiskProfilePath();
    std::ifstream existing(profile_path);
    const bool write_header =
        !existing.good() || existing.peek() == std::ifstream::traits_type::eof();
    existing.close();

    std::ofstream out(profile_path, std::ios::app);
    if (!out.good())
    {
      return false;
    }
    out << std::setprecision(17);
    if (write_header)
    {
      out << "profile_seq,stamp,trajectory_id,applied_to_objective,metrics_only,"
             "lambda_integrity,snapshot_generation_id,query_base_time_s,"
             "sample_index,arc_fraction,t_s,x,y,z,hit,valid,stale,c_pi,reason\n";
    }

    const bool applied_to_objective =
        p1_config_.use_integrity_cost && !p1_config_.metrics_only &&
        p1_config_.lambda_integrity != 0.0;
    const int denom = std::max(1, kP1AcceptedProfileSampleCount - 1);
    for (int sample_index = 0; sample_index < kP1AcceptedProfileSampleCount;
         ++sample_index)
    {
      const double arc_fraction =
          static_cast<double>(sample_index) / static_cast<double>(denom);
      const double t_s = duration * arc_fraction;
      const Eigen::Vector3d p = trajectory.evaluateDeBoorT(t_s);
      iap::RiskCostSample sample;
      const bool hit =
          risk_snapshot_->queryCost(p, risk_query_base_time_s_ + t_s, &sample);
      const bool c_pi_finite =
          hit && sample.valid && !sample.stale && std::isfinite(sample.cost);
      std::string reason = sample.reason.empty() ? "query_miss" : sample.reason;
      if (!hit && reason == "not_evaluated")
      {
        reason = "query_miss";
      }

      out << profile_seq << ','
          << stamp_s << ','
          << trajectory_id << ','
          << (applied_to_objective ? 1 : 0) << ','
          << (p1_config_.metrics_only ? 1 : 0) << ','
          << p1_config_.lambda_integrity << ','
          << risk_snapshot_->generation_id() << ','
          << risk_query_base_time_s_ << ','
          << sample_index << ','
          << arc_fraction << ','
          << t_s << ','
          << p.x() << ','
          << p.y() << ','
          << p.z() << ','
          << (hit ? 1 : 0) << ','
          << (sample.valid ? 1 : 0) << ','
          << (sample.stale ? 1 : 0) << ',';
      if (c_pi_finite)
      {
        out << sample.cost;
      }
      out << ',' << reason << '\n';
    }
    return true;
  }

  bool BsplineOptimizer::evaluateReboundCostForTest(const Eigen::MatrixXd &control_points,
                                                    double ts, double &cost,
                                                    Eigen::MatrixXd &gradient)
  {
    if (control_points.rows() != 3 || control_points.cols() <= order_)
    {
      return false;
    }

    setBsplineInterval(ts);
    cps_.resize(static_cast<int>(control_points.cols()));
    cps_.points = control_points;
    cps_.clearance = dist0_;
    min_ellip_dist_ = INIT_min_ellip_dist_;
    iter_num_ = 0;
    new_lambda2_ = lambda2_;
    variable_num_ = 3 * (cps_.size - order_);

    std::vector<double> x(variable_num_, 0.0);
    std::vector<double> grad(variable_num_, 0.0);
    memcpy(x.data(), cps_.points.data() + 3 * order_, variable_num_ * sizeof(double));
    combineCostRebound(x.data(), grad.data(), cost, variable_num_);

    gradient = Eigen::MatrixXd::Zero(3, cps_.size);
    memcpy(gradient.data() + 3 * order_, grad.data(), variable_num_ * sizeof(double));
    return true;
  }

  // 计算损失
  void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
  {
    // cout << "drone_id_=" << drone_id_ << endl;
    // cout << "cps_.points.size()=" << cps_.points.size() << endl;
    // cout << "n=" << n << endl;
    // cout << "sizeof(x[0])=" << sizeof(x[0]) << endl;

    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_distance, f_feasibility /*, f_mov_objs*/, f_swarm, f_terminal;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);
    // Eigen::MatrixXd g_mov_objs = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_swarm = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_terminal = Eigen::MatrixXd::Zero(3, cps_.size);

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);
    // calcMovingObjCost(cps_.points, f_mov_objs, g_mov_objs);
    calcSwarmCost(cps_.points, f_swarm, g_swarm);
    calcTerminalCost(cps_.points, f_terminal, g_terminal);

    f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility + new_lambda2_ * f_swarm + lambda2_ * f_terminal;
    const double f_original = f_combine;
    // f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility + new_lambda2_ * f_mov_objs;
    // printf("origin %f %f %f %f\n", f_smoothness, f_distance, f_feasibility, f_combine);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility + new_lambda2_ * g_swarm + lambda2_ * g_terminal;
    // Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility + new_lambda2_ * g_mov_objs;

    last_p1_metrics_ = P1IntegrityMetrics{};
    last_p1_viz_samples_.clear();
    last_optimizer_cost_breakdown_ = OptimizerCostBreakdown{};
    last_optimizer_cost_breakdown_.original_cost = f_original;
    last_optimizer_cost_breakdown_.total_cost = f_original;
    const bool p1_should_evaluate =
        risk_snapshot_ && (p1_config_.metrics_only || p1_config_.use_integrity_cost || p1_config_.debug_csv_enable);
    if (p1_should_evaluate)
    {
      double f_integrity = 0.0;
      Eigen::MatrixXd g_integrity = Eigen::MatrixXd::Zero(3, cps_.size);
      P1IntegrityMetrics metrics;
      calcIntegrityTrajectoryCost(cps_.points, f_integrity, g_integrity, metrics);
      metrics.grad_norm_original = grad_3D.norm();
      metrics.weighted_f_integrity = p1_config_.lambda_integrity * f_integrity;
      metrics.weighted_grad_integrity_norm = std::abs(p1_config_.lambda_integrity) * g_integrity.norm();
      if (metrics.grad_norm_original > 1.0e-12)
      {
        metrics.grad_ratio = metrics.weighted_grad_integrity_norm / metrics.grad_norm_original;
      }
      metrics.applied_to_objective =
          p1_config_.use_integrity_cost && !p1_config_.metrics_only && p1_config_.lambda_integrity != 0.0;

      if (metrics.applied_to_objective)
      {
        f_combine += p1_config_.lambda_integrity * f_integrity;
        grad_3D += p1_config_.lambda_integrity * g_integrity;
        last_optimizer_cost_breakdown_.integrity_cost = metrics.weighted_f_integrity;
      }

      last_p1_metrics_ = metrics;
      writeP1DebugCsv(last_p1_metrics_);
    }
    last_optimizer_cost_breakdown_.total_cost = f_combine;
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

  // 计算优化后的损失
  void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
  {

    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_fitness, f_feasibility;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

    // time_satrt = rclcpp::Clock().now();

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcFitnessCost(cps_.points, f_fitness, g_fitness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

    /* ---------- convert to solver format...---------- */
    f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility;
    // printf("origin %f %f %f %f\n", f_smoothness, f_fitness, f_feasibility, f_combine);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + lambda4_ * g_fitness + lambda3_ * g_feasibility;
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

} // namespace ego_planner
