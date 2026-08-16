#include <ego_planner/gate0_qualification_writer.h>

#include <gtest/gtest.h>

#include <atomic>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>
#include <vector>
#include <unistd.h>

namespace ego_planner
{
namespace
{

std::filesystem::path testDirectory(const std::string &name)
{
  const auto path = std::filesystem::temp_directory_path() /
      ("iap_gate0_writer_" + name + "_" +
       std::to_string(static_cast<unsigned long>(::getpid())));
  std::filesystem::remove_all(path);
  std::filesystem::create_directories(path);
  return path;
}

std::vector<std::string> lines(const std::filesystem::path &path)
{
  std::ifstream input(path);
  std::vector<std::string> result;
  for (std::string line; std::getline(input, line);)
  {
    result.push_back(line);
  }
  return result;
}

TEST(Gate0QualificationWriter, DisabledIsNoOp)
{
  const auto directory = testDirectory("disabled");
  Gate0QualificationConfig config;
  config.enabled = false;
  config.candidate_events_path = (directory / "events.csv").string();
  config.control_points_path = (directory / "points.csv").string();

  Gate0QualificationWriter writer(config);
  Gate0QualificationEvent event;
  event.event = "attempt_start";
  event.planning_attempt_id = 7;
  writer.appendEvent(event);

  Gate0ControlPointEvidence points;
  points.stage = "generated";
  points.control_points = Eigen::MatrixXd::Ones(3, 2);
  writer.appendControlPoints(points);

  EXPECT_FALSE(std::filesystem::exists(config.candidate_events_path));
  EXPECT_FALSE(std::filesystem::exists(config.control_points_path));
}

TEST(Gate0QualificationWriter, WritesStableSchemaAndEveryControlPoint)
{
  const auto directory = testDirectory("schema");
  Gate0QualificationConfig config;
  config.enabled = true;
  config.run_id = "primary-r1";
  config.evidence_manifest_path = "/evidence/manifest.json";
  config.candidate_events_path = (directory / "events.csv").string();
  config.control_points_path = (directory / "points.csv").string();
  Gate0QualificationWriter writer(config);

  Gate0QualificationEvent event;
  event.event = "optimizer_result";
  event.stamp_s = 12.5;
  event.planning_attempt_id = 9;
  event.candidate_id = 2;
  event.base_generated_count = 3;
  event.optimizer_input_count = 3;
  event.optimizer_success = 1;
  event.original_cost = 4.25;
  event.final_cost = 5.5;
  event.reason = "ok";
  writer.appendEvent(event);

  Gate0ControlPointEvidence points;
  points.stage = "optimized";
  points.stamp_s = 12.5;
  points.planning_attempt_id = 9;
  points.candidate_id = 2;
  points.degree = 3;
  points.ts = 0.4;
  points.original_cost = 4.25;
  points.final_cost = 5.5;
  points.control_points.resize(3, 2);
  points.control_points << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  writer.appendControlPoints(points);

  const auto event_lines = lines(config.candidate_events_path);
  ASSERT_EQ(event_lines.size(), 2u);
  EXPECT_EQ(event_lines.front(), Gate0QualificationWriter::eventCsvHeader());
  EXPECT_NE(event_lines.back().find("primary-r1"), std::string::npos);
  EXPECT_NE(event_lines.back().find("optimizer_result"), std::string::npos);

  const auto point_lines = lines(config.control_points_path);
  ASSERT_EQ(point_lines.size(), 7u);
  EXPECT_EQ(point_lines.front(), Gate0QualificationWriter::controlPointCsvHeader());
  EXPECT_NE(point_lines[1].find("optimized"), std::string::npos);
  EXPECT_NE(point_lines[1].find(",0,0,1"), std::string::npos);
  EXPECT_NE(point_lines[6].find(",2,1,6"), std::string::npos);
}

TEST(Gate0QualificationWriter, ConcurrentAppendsKeepOneHeaderAndCompleteRows)
{
  const auto directory = testDirectory("concurrent");
  Gate0QualificationConfig config;
  config.enabled = true;
  config.run_id = "concurrent";
  config.candidate_events_path = (directory / "events.csv").string();
  config.control_points_path = (directory / "points.csv").string();
  Gate0QualificationWriter writer(config);

  constexpr int thread_count = 8;
  constexpr int rows_per_thread = 25;
  std::vector<std::thread> threads;
  for (int thread = 0; thread < thread_count; ++thread)
  {
    threads.emplace_back([thread, &writer]() {
      for (int row = 0; row < rows_per_thread; ++row)
      {
        Gate0QualificationEvent event;
        event.event = "optimizer_input";
        event.planning_attempt_id = static_cast<uint64_t>(thread + 1);
        event.candidate_id = row + 1;
        writer.appendEvent(event);
      }
    });
  }
  for (auto &thread : threads)
  {
    thread.join();
  }

  const auto event_lines = lines(config.candidate_events_path);
  ASSERT_EQ(event_lines.size(),
            1u + static_cast<std::size_t>(thread_count * rows_per_thread));
  EXPECT_EQ(event_lines.front(), Gate0QualificationWriter::eventCsvHeader());
  EXPECT_EQ(std::count(event_lines.begin(), event_lines.end(),
                       Gate0QualificationWriter::eventCsvHeader()),
            1);
}

}  // namespace
}  // namespace ego_planner
