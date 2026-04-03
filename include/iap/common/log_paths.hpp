#pragma once

#include <filesystem>
#include <string>

namespace iap {

class LogPaths {
public:
  static LogPaths& instance();
  static void reset();

  const std::filesystem::path& package_root() const { return package_root_; }
  const std::filesystem::path& root_dir() const { return root_dir_; }
  const std::filesystem::path& run_dir() const { return run_dir_; }
  const std::filesystem::path& runtime_dir() const { return runtime_dir_; }
  const std::filesystem::path& profiling_dir() const { return profiling_dir_; }
  const std::filesystem::path& export_dir() const { return export_dir_; }
  const std::filesystem::path& metadata_dir() const { return metadata_dir_; }

  std::filesystem::path runtime_main_log_path() const;
  std::filesystem::path runtime_module_log_path(const std::string& module_name) const;
  std::filesystem::path warnings_log_path() const;
  std::filesystem::path profiling_path(const std::string& file_name) const;
  std::filesystem::path export_path(const std::string& file_name) const;
  std::filesystem::path metadata_path(const std::string& file_name) const;

private:
  LogPaths();
  void initialize();

  std::filesystem::path package_root_;
  std::filesystem::path root_dir_;
  std::filesystem::path run_dir_;
  std::filesystem::path runtime_dir_;
  std::filesystem::path profiling_dir_;
  std::filesystem::path export_dir_;
  std::filesystem::path metadata_dir_;
};

}  // namespace iap
