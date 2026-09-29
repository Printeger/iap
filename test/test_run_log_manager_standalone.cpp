#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>

#include <unistd.h>

#include <iap/util/run_log_manager.hpp>

namespace {
struct ScopedDirectory {
  explicit ScopedDirectory(std::filesystem::path value) : path(std::move(value)) {}
  ~ScopedDirectory() { std::filesystem::remove_all(path); }
  std::filesystem::path path;
};
}  // namespace

TEST(RunLogManagerStandaloneTest, AllocatesDirectlyBelowConfiguredRoot) {
  const auto unique = std::to_string(::getpid());
  const std::filesystem::path root =
      std::filesystem::temp_directory_path() / ("iap_run_log_standalone_" + unique);
  const ScopedDirectory cleanup(root);
  ASSERT_EQ(::unsetenv("IAP_RUN_DIR"), 0);
  ASSERT_EQ(::setenv("IAP_RUN_ROOT", root.c_str(), 1), 0);

  auto& logs = glim::RunLogManager::initialize("standalone");

  EXPECT_EQ(logs.log_root(), std::filesystem::canonical(root));
  EXPECT_EQ(logs.run_dir().parent_path(), std::filesystem::canonical(root));
  EXPECT_TRUE(std::filesystem::exists(logs.run_dir() / "runtime"));
  EXPECT_TRUE(std::filesystem::exists(logs.run_dir() / "profiling"));
  EXPECT_TRUE(std::filesystem::exists(logs.run_dir() / "export"));
  EXPECT_TRUE(std::filesystem::exists(logs.run_dir() / "metadata"));
  EXPECT_TRUE(std::filesystem::is_regular_file(
      logs.run_dir() / "metadata" / "run_manifest.json"));
  EXPECT_FALSE(std::filesystem::exists(logs.run_dir() / "metadata" / "run_info.json"));
  ASSERT_TRUE(std::filesystem::is_symlink(root / "latest"));
  EXPECT_EQ(std::filesystem::canonical(root / "latest"), logs.run_dir());
}
