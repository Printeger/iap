#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <iap/planner/pi_cost_adapter.hpp>

TEST(PICostAdapterTest, SafeBandHasZeroCost) {
  iap::PICostAdapter adapter;
  const auto result = adapter.evaluate(10.0, 12.0, 6.0, 7.0);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.margin_h, 4.0);
  EXPECT_DOUBLE_EQ(result.margin_v, 5.0);
  EXPECT_DOUBLE_EQ(result.cost_total, 0.0);
  EXPECT_EQ(result.risk_band, "SAFE_PI");
  EXPECT_EQ(result.risk_band_code, 1);
  EXPECT_EQ(result.dominant_axis, "horizontal");
}

TEST(PICostAdapterTest, MarginalBandUsesMinimumMargin) {
  iap::PICostAdapter adapter;
  const auto result = adapter.evaluate(10.0, 12.0, 9.25, 8.0);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.margin_h, 0.75);
  EXPECT_DOUBLE_EQ(result.cost_h, 0.0625);
  EXPECT_DOUBLE_EQ(result.cost_v, 0.0);
  EXPECT_DOUBLE_EQ(result.cost_total, 0.0625);
  EXPECT_EQ(result.risk_band, "MARGINAL_PI");
  EXPECT_EQ(result.dominant_axis, "horizontal");
}

TEST(PICostAdapterTest, UnsafeBandUsesAxisCostBreakdown) {
  iap::PICostAdapter::Params params;
  params.weight_h = 2.0;
  params.weight_v = 3.0;
  iap::PICostAdapter adapter(params);

  const auto result = adapter.evaluate(10.0, 8.0, 12.0, 11.0);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.margin_h, -2.0);
  EXPECT_DOUBLE_EQ(result.margin_v, -3.0);
  EXPECT_DOUBLE_EQ(result.cost_h, 18.0);
  EXPECT_DOUBLE_EQ(result.cost_v, 48.0);
  EXPECT_DOUBLE_EQ(result.cost_total, 66.0);
  EXPECT_EQ(result.risk_band, "UNSAFE_PI");
  EXPECT_EQ(result.risk_band_code, 3);
  EXPECT_EQ(result.dominant_axis, "vertical");
}

TEST(PICostAdapterTest, NonFiniteInputsAreUnknownAndZeroCost) {
  iap::PICostAdapter adapter;
  const auto result =
      adapter.evaluate(10.0, 8.0, std::numeric_limits<double>::quiet_NaN(), 4.0);

  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.cost_total, 0.0);
  EXPECT_EQ(result.risk_band, "UNKNOWN_PI");
  EXPECT_EQ(result.risk_band_code, 0);
  EXPECT_EQ(result.dominant_axis, "unknown");
}

TEST(PICostAdapterTest, EvaluateWithGradientKeepsFiniteGradient) {
  iap::PICostAdapter adapter;
  const auto result =
      adapter.evaluate_with_gradient(10.0, 8.0, 12.0, 9.0, 1.0, -2.0, 0.5);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.grad_x, 1.0);
  EXPECT_DOUBLE_EQ(result.grad_y, -2.0);
  EXPECT_DOUBLE_EQ(result.grad_z, 0.5);
  EXPECT_EQ(result.risk_band, "UNSAFE_PI");
}

TEST(PICostAdapterTest, EvaluateWithGradientRejectsNonFiniteGradient) {
  iap::PICostAdapter adapter;
  const auto result = adapter.evaluate_with_gradient(
      10.0, 8.0, 12.0, 9.0, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);

  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.grad_x, 0.0);
  EXPECT_DOUBLE_EQ(result.grad_y, 0.0);
  EXPECT_DOUBLE_EQ(result.grad_z, 0.0);
  EXPECT_EQ(result.risk_band, "UNKNOWN_PI");
}

TEST(PICostAdapterTest, Stage3HingeActivatesInsideAlertLimitByMargin) {
  iap::PICostAdapter::Params params;
  params.use_unified_advisory_pl = true;
  params.margin_h_m = 2.0;
  params.margin_v_m = 1.0;
  params.lambda_pi = 3.0;
  iap::PICostAdapter adapter(params);

  const auto result = adapter.evaluate(10.0, 12.0, 9.0, 8.0);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.input_valid);
  EXPECT_DOUBLE_EQ(result.margin_h, 1.0);
  EXPECT_DOUBLE_EQ(result.cost_h, 3.0);
  EXPECT_DOUBLE_EQ(result.cost_v, 0.0);
  EXPECT_DOUBLE_EQ(result.hinge_cost, 3.0);
  EXPECT_DOUBLE_EQ(result.ratio_cost, 0.0);
  EXPECT_DOUBLE_EQ(result.cost_total, 3.0);
  EXPECT_EQ(result.risk_band, "MARGINAL_PI");
}

TEST(PICostAdapterTest, Stage3RatioTermDisabledByDefault) {
  iap::PICostAdapter::Params params;
  params.use_unified_advisory_pl = true;
  params.margin_h_m = 0.0;
  params.margin_v_m = 0.0;
  params.lambda_pi = 1.0;
  params.mu_ratio = 100.0;
  iap::PICostAdapter adapter(params);

  const auto result = adapter.evaluate(10.0, 10.0, 5.0, 5.0);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.hinge_cost, 0.0);
  EXPECT_DOUBLE_EQ(result.ratio_cost, 0.0);
  EXPECT_DOUBLE_EQ(result.cost_total, 0.0);
}

TEST(PICostAdapterTest, Stage3UnknownAdvisoryCanBePenalized) {
  iap::PICostAdapter::Params params;
  params.use_unified_advisory_pl = true;
  params.penalize_unknown_advisory = true;
  params.max_cost = 42.0;
  iap::PICostAdapter adapter(params);

  const auto result =
      adapter.evaluate(10.0, 8.0, std::numeric_limits<double>::quiet_NaN(), 4.0);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.input_valid);
  EXPECT_DOUBLE_EQ(result.unknown_penalty, 42.0);
  EXPECT_DOUBLE_EQ(result.cost_total, 42.0);
  EXPECT_EQ(result.risk_band, "UNKNOWN_PI");
}

TEST(PICostAdapterTest, Stage3SentinelAdvisoryCanBePenalized) {
  iap::PICostAdapter::Params params;
  params.use_unified_advisory_pl = true;
  params.penalize_unknown_advisory = true;
  params.max_cost = 100.0;
  iap::PICostAdapter adapter(params);

  const auto result = adapter.evaluate(10.0, 8.0, 1.0e9, 4.0);

  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.cost_total, 100.0);
}

TEST(PICostAdapterTest, LegacyDefaultBehaviorRemainsUnchanged) {
  iap::PICostAdapter::Params params;
  params.margin_h_m = 10.0;
  params.margin_v_m = 10.0;
  params.lambda_pi = 100.0;
  params.penalize_unknown_advisory = true;
  iap::PICostAdapter adapter(params);

  const auto result = adapter.evaluate(10.0, 12.0, 9.25, 8.0);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.margin_h, 0.75);
  EXPECT_DOUBLE_EQ(result.cost_h, 0.0625);
  EXPECT_DOUBLE_EQ(result.cost_v, 0.0);
  EXPECT_DOUBLE_EQ(result.cost_total, 0.0625);
  EXPECT_EQ(result.risk_band, "MARGINAL_PI");
}
