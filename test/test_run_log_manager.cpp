#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include <unistd.h>

#include <iap/util/config.hpp>
#include <iap/util/run_log_manager.hpp>

TEST(RunLogManagerTest, CreatesRunLayoutAndIsIdempotent) {
  const auto unique = std::to_string(::getpid());
  const std::filesystem::path temp_root =
      std::filesystem::temp_directory_path() / ("iap_run_log_manager_test_" + unique);
  const std::filesystem::path config_dir = temp_root / "config";
  const std::filesystem::path log_root = temp_root / "logs";

  std::filesystem::create_directories(config_dir);

  std::ofstream config(config_dir / "config.json");
  ASSERT_TRUE(config.is_open());
  config << "{\n"
         << "  \"global\": {\n"
         << "    \"config_path\": \"\"\n"
         << "  },\n"
         << "  \"logging\": {\n"
         << "    \"log_dir\": \"" << log_root.string() << "\",\n"
         << "    \"save_logs\": true,\n"
         << "    \"rotate_logs\": false\n"
         << "  }\n"
         << "}\n";
  config.close();

  glim::GlobalConfig::instance(config_dir.string(), true);
  auto& first = glim::RunLogManager::initialize("test_run_log_manager", config_dir.string());
  auto& second = glim::RunLogManager::initialize("ignored_second_call", config_dir.string());

  EXPECT_EQ(&first, &second);
  EXPECT_TRUE(std::filesystem::exists(first.run_dir()));
  EXPECT_TRUE(std::filesystem::exists(first.runtime_path("")));
  EXPECT_TRUE(std::filesystem::exists(first.profiling_path("")));
  EXPECT_TRUE(std::filesystem::exists(first.export_path("")));
  EXPECT_TRUE(std::filesystem::exists(first.metadata_path("")));
  EXPECT_EQ(first.runtime_path("iap_main.log").parent_path(), first.run_dir() / "runtime");
  EXPECT_EQ(first.profiling_path("iap_timing.csv").parent_path(), first.run_dir() / "profiling");
  EXPECT_EQ(first.export_path("iap_icp.csv").parent_path(), first.run_dir() / "export");
  EXPECT_EQ(first.metadata_path("run_info.json").parent_path(), first.run_dir() / "metadata");

  const auto latest = first.log_root() / "latest";
  if (std::filesystem::exists(latest) || std::filesystem::is_symlink(latest)) {
    EXPECT_TRUE(std::filesystem::is_symlink(latest));
  }
}
