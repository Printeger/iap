#include <iap/planner/predictor_risk_conversion.hpp>

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace {

TEST(PredictorRiskConversion, MapsEveryProductionField) {
  iap::PredictorQueryResult prediction;
  prediction.available = true;
  prediction.valid = false;
  prediction.fallback = true;
  prediction.fallback_reason = "stale_current_prior";
  prediction.source_flags = 0xA5u;
  prediction.fused.hpl = 12.5;
  prediction.fused.vpl = 7.25;

  const auto result = iap::makeRiskPredictionResult(prediction);

  EXPECT_TRUE(result.available);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.stale);
  EXPECT_DOUBLE_EQ(result.hpl_pred, 12.5);
  EXPECT_DOUBLE_EQ(result.vpl_pred, 7.25);
  EXPECT_EQ(result.source_flags, 0xA5u);
  EXPECT_EQ(result.reason, "stale_current_prior");
}

TEST(PredictorRiskConversion, UsesOkAndDoesNotInferStaleWithoutFallback) {
  iap::PredictorQueryResult prediction;
  prediction.available = true;
  prediction.valid = true;
  prediction.fallback = false;
  prediction.fallback_reason.clear();
  prediction.fused.hpl = std::numeric_limits<double>::infinity();
  prediction.fused.vpl = -1.0;

  const auto result = iap::makeRiskPredictionResult(prediction);

  EXPECT_FALSE(result.stale);
  EXPECT_EQ(result.reason, "ok");
  EXPECT_TRUE(std::isinf(result.hpl_pred));
  EXPECT_DOUBLE_EQ(result.vpl_pred, -1.0);
}

}  // namespace
