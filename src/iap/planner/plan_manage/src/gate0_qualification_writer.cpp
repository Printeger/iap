#include <ego_planner/gate0_qualification_writer.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <utility>

namespace ego_planner
{
namespace
{

void prepareFile(const std::string &path, const std::string &header)
{
  const std::filesystem::path file(path);
  if (!file.parent_path().empty())
  {
    std::filesystem::create_directories(file.parent_path());
  }
  if (!std::filesystem::exists(file) || std::filesystem::file_size(file) == 0)
  {
    std::ofstream output(path, std::ios::app);
    output << header << '\n';
  }
}

}  // namespace

Gate0QualificationWriter::Gate0QualificationWriter(
  Gate0QualificationConfig config)
: config_(std::move(config))
{
}

std::string Gate0QualificationWriter::eventCsvHeader()
{
  return "schema_version,run_id,evidence_manifest_path,event,stamp_s,"
         "planning_attempt_id,candidate_id,collision_segment_count,"
         "base_generated_count,optimizer_input_count,optimizer_success,"
         "optimizer_success_count,original_candidate_id,selected_candidate_id,"
         "entered_refinement,ego_feasible,refinement_success,update_traj_info,"
         "bspline_publish_count,degree,ts,rows,cols,original_cost,final_cost,reason";
}

std::string Gate0QualificationWriter::controlPointCsvHeader()
{
  return "schema_version,run_id,evidence_manifest_path,stage,stamp_s,"
         "planning_attempt_id,candidate_id,degree,ts,rows,cols,original_cost,"
         "final_cost,point_row,point_col,value";
}

std::string Gate0QualificationWriter::csv(const std::string &value)
{
  if (value.find_first_of(",\"\r\n") == std::string::npos)
  {
    return value;
  }
  std::string escaped;
  escaped.reserve(value.size() + 2);
  escaped.push_back('"');
  for (const char character : value)
  {
    if (character == '"')
    {
      escaped.push_back('"');
    }
    escaped.push_back(character);
  }
  escaped.push_back('"');
  return escaped;
}

void Gate0QualificationWriter::appendEvent(
  const Gate0QualificationEvent &event)
{
  if (!config_.enabled || config_.candidate_events_path.empty())
  {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  prepareFile(config_.candidate_events_path, eventCsvHeader());
  std::ofstream output(config_.candidate_events_path, std::ios::app);
  output << std::setprecision(17)
         << "gate0_qualification_v1," << csv(config_.run_id) << ','
         << csv(config_.evidence_manifest_path) << ',' << csv(event.event) << ','
         << event.stamp_s << ',' << event.planning_attempt_id << ','
         << event.candidate_id << ',' << event.collision_segment_count << ','
         << event.base_generated_count << ',' << event.optimizer_input_count << ','
         << event.optimizer_success << ',' << event.optimizer_success_count << ','
         << event.original_candidate_id << ',' << event.selected_candidate_id << ','
         << event.entered_refinement << ',' << event.ego_feasible << ','
         << event.refinement_success << ',' << event.update_traj_info << ','
         << event.bspline_publish_count << ',' << event.degree << ',' << event.ts << ','
         << event.rows << ',' << event.cols << ',' << event.original_cost << ','
         << event.final_cost << ',' << csv(event.reason) << '\n';
}

void Gate0QualificationWriter::appendControlPoints(
  const Gate0ControlPointEvidence &evidence)
{
  if (!config_.enabled || config_.control_points_path.empty())
  {
    return;
  }
  const Eigen::MatrixXd points = evidence.control_points;
  std::lock_guard<std::mutex> lock(mutex_);
  prepareFile(config_.control_points_path, controlPointCsvHeader());
  std::ofstream output(config_.control_points_path, std::ios::app);
  output << std::setprecision(17);
  for (Eigen::Index row = 0; row < points.rows(); ++row)
  {
    for (Eigen::Index column = 0; column < points.cols(); ++column)
    {
      output << "gate0_qualification_v1," << csv(config_.run_id) << ','
             << csv(config_.evidence_manifest_path) << ',' << csv(evidence.stage)
             << ',' << evidence.stamp_s << ',' << evidence.planning_attempt_id
             << ',' << evidence.candidate_id << ',' << evidence.degree << ','
             << evidence.ts << ',' << points.rows() << ',' << points.cols() << ','
             << evidence.original_cost << ',' << evidence.final_cost << ','
             << row << ',' << column << ',' << points(row, column) << '\n';
    }
  }
}

}  // namespace ego_planner
