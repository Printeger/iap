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
  prediction.gnss.available = true;
  prediction.gnss.valid = true;
  prediction.gnss.hpl = 11.0;
  prediction.gnss.vpl = 21.0;
  prediction.gnss.lambda_trace = 3.0;
  prediction.fused.lidar_only_hpl = 2.0;
  prediction.fused.lidar_only_vpl = 3.0;
  prediction.fused.lambda_lidar_trace = 5.0;
  prediction.fused.prior_only_hpl = 4.0;
  prediction.fused.prior_only_vpl = 6.0;
  prediction.fused.lambda_prior_trace = 7.0;
  prediction.fused.pre_conservative_hpl = 5.0;
  prediction.fused.pre_conservative_vpl = 8.0;
  prediction.fused.floor_source_h = "gnss";
  prediction.fused.floor_source_v = "gnss";
  prediction.fused.floor_increment_h = 7.5;
  prediction.fused.floor_increment_v = 0.0;

  const auto result = iap::makeRiskPredictionResult(prediction);

  EXPECT_TRUE(result.available);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.stale);
  EXPECT_DOUBLE_EQ(result.hpl_pred, 12.5);
  EXPECT_DOUBLE_EQ(result.vpl_pred, 7.25);
  EXPECT_EQ(result.source_flags, 0xA5u);
  EXPECT_EQ(result.reason, "stale_current_prior");
  EXPECT_DOUBLE_EQ(result.gnss.hpl, 11.0);
  EXPECT_DOUBLE_EQ(result.gnss.risk_ratio, 1.1);
  EXPECT_DOUBLE_EQ(result.lidar.risk_ratio, 0.2);
  EXPECT_DOUBLE_EQ(result.prior.risk_ratio, 0.4);
  EXPECT_DOUBLE_EQ(result.fim_fused.risk_ratio, 0.5);
  EXPECT_DOUBLE_EQ(result.safety_fused.risk_ratio, 1.25);
  EXPECT_EQ(result.floor_source_h, "gnss");
  EXPECT_DOUBLE_EQ(result.floor_increment_h, 7.5);
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
