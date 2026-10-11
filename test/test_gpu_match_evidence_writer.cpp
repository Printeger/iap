#include <gtest/gtest.h>
#include <iap/odometry/gpu_evidence/evidence_writer.hpp>
#include <iap/util/run_log_manager.hpp>
#include <nlohmann/json.hpp>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>

TEST(GpuMatchEvidenceWriter, BoundedPressureRetainsEveryRequestAndDrainsOwnedRaw) {
  std::string pattern=(std::filesystem::temp_directory_path()/"iap_gpu_writer_XXXXXX").string();
  auto* allocated=::mkdtemp(pattern.data());ASSERT_NE(allocated,nullptr);
  struct Cleanup { std::filesystem::path path;~Cleanup(){std::filesystem::remove_all(path);} } cleanup{allocated};
  const auto run=cleanup.path/"run";std::filesystem::create_directory(run);
  ASSERT_EQ(setenv("IAP_RUN_DIR",run.c_str(),1),0);
  auto& logs=glim::RunLogManager::initialize("gpu_writer_regression");
  constexpr int count=2000;
  {
    iap::GpuMatchEvidenceWriter writer(logs);
    iap::GpuMatchEvidencePacket packet;packet.capture_requested=true;packet.capture.available=true;
    packet.capture.samples.resize(128);packet.capture.original_source_count=128;
    packet.capture.original_inlier_count=128;packet.capture.sampling_stride=1;
    packet.capture.sample_limit=128;packet.capture.source_slots=128;
    packet.capture.source_geometry_available=true;
    packet.owner.target_frame_id=9;packet.owner.level_id=0;
    for(int i=0;i<count;++i) {
      packet.owner.source_frame_id=i;packet.owner.source_stamp=1000.+i*.1;
      if(i==0) {
        packet.capture.samples.resize(iap::kGpuMatchDenseEvidenceLimit);
        packet.capture.sample_limit=iap::kGpuMatchDenseEvidenceLimit;
        packet.capture.original_source_count=packet.capture.original_inlier_count=packet.capture.source_slots=iap::kGpuMatchDenseEvidenceLimit;
      } else {
        packet.capture.samples.resize(128);packet.capture.sample_limit=128;
        packet.capture.original_source_count=packet.capture.original_inlier_count=packet.capture.source_slots=128;
      }
      packet.capture.source_samples.resize(packet.capture.source_slots);
      for(int slot=0;slot<packet.capture.source_slots;++slot) {
        packet.capture.source_samples[slot].source_index=slot;
        packet.capture.source_samples[slot].mean_m[0]=slot*.1f;
      }
      writer.submit(packet);
    }
    auto invalid=packet;invalid.capture.samples.resize(129);
    EXPECT_THROW(writer.submit(invalid),std::invalid_argument);
    invalid=packet;invalid.capture.source_samples.resize(129);
    EXPECT_THROW(writer.submit(invalid),std::invalid_argument);
    invalid=packet;invalid.capture.source_geometry_available=false;
    EXPECT_THROW(writer.submit(invalid),std::invalid_argument);
  }
  using Json=nlohmann::json;
  Json manifest;std::ifstream(logs.metadata_path("manifests/gpu_match_evidence.json"))>>manifest;
  EXPECT_EQ(manifest["entries"],count);EXPECT_EQ(manifest["requested"],count);
  EXPECT_EQ(manifest["available"],count);EXPECT_EQ(manifest["write_failures"],0);
  EXPECT_FALSE(manifest["runtime_identity_available"]);EXPECT_TRUE(manifest["runtime_identity"].is_null());
  EXPECT_GT(manifest["queue_dropped"].get<int>(),0);
  EXPECT_EQ(manifest["written"].get<int>()+manifest["queue_dropped"].get<int>(),count);
  std::set<std::uint64_t> accepted,exported;
  std::ifstream ledger(logs.export_path("glio/gpu_match_requests.csv"));std::string line;
  ASSERT_TRUE(std::getline(ledger,line));int rows=0;
  while(std::getline(ledger,line)) {
    std::vector<std::string> fields;std::stringstream stream(line);std::string value;
    while(std::getline(stream,value,',')) fields.push_back(value);
    ASSERT_EQ(fields.size(),32u);++rows;
    EXPECT_EQ(std::stoull(fields[0]),rows);EXPECT_EQ(std::stoll(fields[1]),rows-1);
    if(fields[8]=="1") accepted.insert(std::stoull(fields[0]));
  }
  EXPECT_EQ(rows,count);
  std::ifstream raw(logs.export_path("glio/gpu_match_residuals.jsonl"));
  while(std::getline(raw,line)) {
    const auto packet=Json::parse(line);const auto id=packet["request_id"].get<std::uint64_t>();
    EXPECT_TRUE(exported.insert(id).second);EXPECT_EQ(packet["source_frame_id"],id-1);
    EXPECT_EQ(packet["samples"].size(),id==1 ? iap::kGpuMatchDenseEvidenceLimit : 128u);
    EXPECT_EQ(packet["schema"],"iap_gpu_match_residual_v2");
    EXPECT_TRUE(packet["source_geometry_available"].get<bool>());
    EXPECT_EQ(packet["source_samples"].size(),packet["source_slots"].get<std::size_t>());
    for(std::size_t slot=0;slot<packet["source_samples"].size();++slot) {
      EXPECT_EQ(packet["source_samples"][slot]["source_index"],slot);
      EXPECT_FLOAT_EQ(packet["source_samples"][slot]["mean_m"][0].get<float>(),slot*.1f);
    }
  }
  EXPECT_EQ(exported,accepted);EXPECT_EQ(exported.size(),manifest["written"].get<std::size_t>());
  EXPECT_THROW(iap::GpuMatchEvidenceWriter duplicate(logs),std::runtime_error);
}
