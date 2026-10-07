#pragma once
#include <iap/odometry/gpu_evidence/match_residual.hpp>
#include <array>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>

namespace glim { class RunLogManager; }
namespace iap {
struct GpuMatchEvidencePacket {
  std::uint64_t request_id=0;
  bool capture_requested=false;
  GpuMatchOwner owner;
  GpuMatchCapture capture;
  std::array<std::uint64_t,2> factor_keys{};
  std::size_t key_count=0;
  Eigen::Matrix4d T_world_source=Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_world_target=Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_lidar_imu=Eigen::Matrix4d::Identity();
  std::int64_t gnss_frame_id=-1;
  std::uint64_t gnss_update_sequence=0, gnss_epoch_identity=0;
  double gnss_state_stamp=0., gnss_epoch_stamp=0.;
  bool gnss_owner_matches=false;
  std::string used_constellations;
};

// The single odometry producer records a small request ledger; large raw
// serialization and writes belong to one bounded worker. No source admission.
class GpuMatchEvidenceWriter {
public:
  explicit GpuMatchEvidenceWriter(const glim::RunLogManager& logs);
  ~GpuMatchEvidenceWriter();
  GpuMatchEvidenceWriter(const GpuMatchEvidenceWriter&)=delete;
  GpuMatchEvidenceWriter& operator=(const GpuMatchEvidenceWriter&)=delete;
  void submit(GpuMatchEvidencePacket packet);
private:
  void drain();
  static constexpr std::size_t kQueueLimit=16;
  std::filesystem::path raw_path_, ledger_path_, manifest_path_;
  std::string runtime_identity_;
  std::ofstream raw_, ledger_;
  std::deque<GpuMatchEvidencePacket> queue_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool stopping_=false;
  std::thread worker_;
  std::uint64_t entries_=0, requests_=0, available_=0, queue_dropped_=0, written_=0, write_failures_=0;
  std::uint64_t samples_written_=0;
};
}
