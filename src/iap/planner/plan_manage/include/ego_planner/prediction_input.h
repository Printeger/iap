#pragma once
#include <plan_env/grid_map.h>
#include <iap/predictor/predictor_module.hpp>
#include <atomic>
namespace ego_planner {
// Read-only display transport and planner binding share this input. Display
// results never return to GridMap's advisory cache or execution authority.
struct PredictionInput {
  std::shared_ptr<const FrozenOccupancyEpoch> occupancy;
  iap::IntegritySnapshot integrity;
  iap::PredictorParams params;
  double reference_time_s = 0.0;
  double validity_s = .5;
};
GridRiskContext makeRiskPrediction(const PredictionInput& input,
    std::shared_ptr<std::atomic<uint64_t>> calls = {});
std::vector<uint8_t> encodePredictionInput(const PredictionInput& input);
PredictionInput decodePredictionInput(const std::vector<uint8_t>& payload);
}
