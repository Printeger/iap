#pragma once

#include <filesystem>
#include <map>
#include <string>

namespace glim {

class RunLogManager {
public:
  static RunLogManager& initialize(const std::string& process_name,
                                   const std::string& config_dir_or_empty = std::string());
  static RunLogManager& instance();
  static RunLogManager* get_if_initialized();

  const std::filesystem::path& log_root() const;
  const std::filesystem::path& run_dir() const;

  std::filesystem::path runtime_path(const std::string& name) const;
  std::filesystem::path profiling_path(const std::string& name) const;
  std::filesystem::path export_path(const std::string& name) const;
  std::filesystem::path metadata_path(const std::string& name = std::string()) const;

  void write_run_info(const std::map<std::string, std::string>& extra_fields = {}) const;

private:
  RunLogManager(std::string process_name, std::string config_dir_or_empty);

  std::filesystem::path category_path(const std::string& category,
                                      const std::string& name) const;
  std::filesystem::path resolve_log_root() const;
  void create_layout();
  void update_latest_symlink() const;
  std::map<std::string, std::string> collect_run_info_fields() const;

  std::string process_name_;
  std::string config_dir_;
  std::string start_timestamp_;
  std::string start_timestamp_iso_;
  std::filesystem::path log_root_;
  std::filesystem::path run_dir_;
};

}  // namespace glim
