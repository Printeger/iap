#ifndef EGO_PLANNER__GATE0_QUALIFICATION_WRITER_H_
#define EGO_PLANNER__GATE0_QUALIFICATION_WRITER_H_

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <mutex>
#include <string>

namespace ego_planner
{

struct Gate0QualificationConfig
{
  bool enabled = false;
  std::string candidate_events_path;
  std::string control_points_path;
  std::string run_id;
  std::string evidence_manifest_path;
};

struct Gate0QualificationEvent
{
  std::string event;
  double stamp_s = std::numeric_limits<double>::quiet_NaN();
  uint64_t planning_attempt_id = 0;
  int candidate_id = 0;
  int collision_segment_count = -1;
  int base_generated_count = -1;
  int optimizer_input_count = -1;
  int optimizer_success = -1;
  int optimizer_success_count = -1;
  int original_candidate_id = 0;
  int selected_candidate_id = 0;
  int entered_refinement = -1;
  int ego_feasible = -1;
  int refinement_success = -1;
  int update_traj_info = -1;
  int bspline_publish_count = -1;
  int degree = -1;
  double ts = std::numeric_limits<double>::quiet_NaN();
  int rows = -1;
  int cols = -1;
  double original_cost = std::numeric_limits<double>::quiet_NaN();
  double final_cost = std::numeric_limits<double>::quiet_NaN();
  std::string reason;
};

struct Gate0ControlPointEvidence
{
  std::string stage;
  double stamp_s = std::numeric_limits<double>::quiet_NaN();
  uint64_t planning_attempt_id = 0;
  int candidate_id = 0;
  int degree = -1;
  double ts = std::numeric_limits<double>::quiet_NaN();
  double original_cost = std::numeric_limits<double>::quiet_NaN();
  double final_cost = std::numeric_limits<double>::quiet_NaN();
  Eigen::MatrixXd control_points;
};

// A diagnostic-only append adapter. It owns no planner state and receives
// copied scalar/matrix evidence after planner decisions have already happened.
class Gate0QualificationWriter
{
public:
  explicit Gate0QualificationWriter(Gate0QualificationConfig config);

  bool enabled() const noexcept {return config_.enabled;}
  void appendEvent(const Gate0QualificationEvent &event);
  void appendControlPoints(const Gate0ControlPointEvidence &evidence);

  static std::string eventCsvHeader();
  static std::string controlPointCsvHeader();

private:
  Gate0QualificationConfig config_;
  std::mutex mutex_;

  static std::string csv(const std::string &value);
};

}  // namespace ego_planner

#endif  // EGO_PLANNER__GATE0_QUALIFICATION_WRITER_H_
