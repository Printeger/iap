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
// Shared input authority. This only controls the Advisory posterior proxy;
// Current Monitor fields and GLIO/FGO state are never changed.
void setAdvisoryPosteriorPrior(iap::IntegritySnapshot& snapshot, bool enabled);
GridRiskContext makeRiskPrediction(const PredictionInput& input,
    std::shared_ptr<std::atomic<uint64_t>> calls = {},
    std::string* rejection_reason = nullptr);
// Same frozen map/support/primitive derivation as the production binding.
// Direct queries are diagnostic; callers must retain the binding's rejection.
iap::PredictorQueryInput frozenPredictionQuery(const PredictionInput& input, const Eigen::Vector3d& center);
iap::PredictorModule makeFrozenPredictor(const PredictionInput& input);
GridRiskVoxel predictionRiskVoxel(const iap::PredictorQueryResult& result);
std::vector<uint8_t> encodePredictionInput(const PredictionInput& input);
PredictionInput decodePredictionInput(const std::vector<uint8_t>& payload);
}
