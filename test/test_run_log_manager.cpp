#include <gtest/gtest.h>

#include <filesystem>
#include <cstdlib>

#include <unistd.h>

#include <iap/util/run_log_manager.hpp>

namespace {
struct ScopedDirectory {
  explicit ScopedDirectory(std::filesystem::path value) : path(std::move(value)) {}
  ~ScopedDirectory() { std::filesystem::remove_all(path); }
  std::filesystem::path path;
};
}  // namespace

TEST(RunLogManagerTest, AdoptsOwnerRunWithoutAllocatingNestedTimestamp) {
  const auto unique = std::to_string(::getpid());
  const std::filesystem::path temp_root =
      std::filesystem::temp_directory_path() / ("iap_run_log_manager_test_" + unique);
  const ScopedDirectory cleanup(temp_root);
  const std::filesystem::path config_dir = temp_root / "config";
  const std::filesystem::path run_dir = temp_root / "20260929T120000Z_001";

  std::filesystem::create_directories(config_dir);
  std::filesystem::create_directories(run_dir);
  ASSERT_EQ(::setenv("IAP_RUN_DIR", run_dir.c_str(), 1), 0);

  auto& first = glim::RunLogManager::initialize("test_run_log_manager", config_dir.string());
  auto& second = glim::RunLogManager::initialize("ignored_second_call", config_dir.string());

  EXPECT_EQ(&first, &second);
  EXPECT_EQ(first.run_dir(), std::filesystem::canonical(run_dir));
  EXPECT_EQ(first.log_root(), std::filesystem::canonical(temp_root));
  EXPECT_TRUE(std::filesystem::exists(first.runtime_path("")));
  EXPECT_TRUE(std::filesystem::exists(first.profiling_path("")));
  EXPECT_TRUE(std::filesystem::exists(first.export_path("")));
  EXPECT_TRUE(std::filesystem::exists(first.metadata_path("")));
  EXPECT_EQ(first.runtime_path("iap_main.log").parent_path(), first.run_dir() / "runtime");
  EXPECT_EQ(first.profiling_path("iap_timing.csv").parent_path(), first.run_dir() / "profiling");
  EXPECT_EQ(first.export_path("iap_icp.csv").parent_path(), first.run_dir() / "export");
  EXPECT_EQ(first.metadata_path("run_info.json").parent_path(), first.run_dir() / "metadata");
  EXPECT_FALSE(std::filesystem::exists(run_dir / "20260929T120000Z_001"));
  EXPECT_FALSE(std::filesystem::exists(temp_root / "latest"));
  EXPECT_THROW(first.export_path("../escape.csv"), std::invalid_argument);
  EXPECT_THROW(first.runtime_path("/tmp/escape.log"), std::invalid_argument);
}
