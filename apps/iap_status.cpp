/**
 * @file iap_status.cpp
 * @brief Minimal IAP smoke-test executable.  (IAP-RQ-002)
 *
 * Usage (after colcon build):
 *   source install/setup.bash
 *   ros2 run iap iap_status [config_dir]
 *   # or directly:
 *   ./install/iap/lib/iap/iap_status [config_dir]
 *
 * Loads IAP config, prints library info, and exits with 0 on success.
 * Suitable as a CI smoke test and as a clangd compile-commands anchor.
 */

#include <iostream>
#include <string>
#include <iap/util/config.hpp>
#include <iap/util/logging.hpp>

int main(int argc, char** argv) {
  const std::string config_dir = (argc > 1) ? argv[1] : "config";
  glim::GlobalConfig::instance(config_dir, true);

  auto logger = glim::create_module_logger("iap_status");

  logger->info("=== IAP status check (v{}) ===", IAP_VERSION);
  logger->info("Config dir : {}", config_dir);

  // Try loading the base config (non-fatal if not found)
  try {
    glim::Config config(config_dir + "/config.json");
    logger->info("config.json loaded OK");
  } catch (const std::exception& e) {
    logger->warn("Config not loaded: {}  (pass config path as first arg)", e.what());
  }

  logger->info("=== IAP library OK ===");
  std::cout << "iap_status: OK" << std::endl;
  return 0;
}
