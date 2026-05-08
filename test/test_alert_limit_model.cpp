#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <iap/planner/alert_limit_model.hpp>

TEST(AlertLimitModelTest, FixedAlertLimitUsesMissionLimits) {
  iap::AlertLimitModelParams params;
  params.model = "fixed_alert_limit";
  params.hal_m = 30.0;
  params.val_m = 60.0;
  params.drone_radius_m = 0.35;
  params.safety_buffer_m = 0.20;

  const auto al = iap::evaluate_alert_limit(params, 2.0, 3.0, 4.0);

  EXPECT_DOUBLE_EQ(al.hal_m, 30.0);
  EXPECT_DOUBLE_EQ(al.val_m, 60.0);
  EXPECT_DOUBLE_EQ(al.al_m, 30.0);
  EXPECT_EQ(al.source, "fixed_alert_limit");
  EXPECT_DOUBLE_EQ(al.collision_clearance_h_m, 1.45);
  EXPECT_DOUBLE_EQ(al.collision_clearance_v_m, 3.0);

  const double hpl = 18.5;
  const double vpl = 42.0;
  EXPECT_DOUBLE_EQ(al.hal_m - hpl, 11.5);
  EXPECT_DOUBLE_EQ(al.val_m - vpl, 18.0);
}

TEST(AlertLimitModelTest, CloudClearanceKeepsLegacyDynamicAl) {
  iap::AlertLimitModelParams params;
  params.model = "cloud_clearance";
  params.drone_radius_m = 0.35;
  params.safety_buffer_m = 0.20;
  params.gamma_h = 0.8;
  params.gamma_v = 0.5;

  const auto al = iap::evaluate_alert_limit(params, 5.0, 2.0, 7.0);

  EXPECT_DOUBLE_EQ(al.collision_clearance_h_m, 4.45);
  EXPECT_DOUBLE_EQ(al.collision_clearance_v_m, 2.0);
  EXPECT_DOUBLE_EQ(al.hal_m, 3.56);
  EXPECT_DOUBLE_EQ(al.val_m, 1.0);
  EXPECT_DOUBLE_EQ(al.al_m, 1.0);
  EXPECT_EQ(al.source, "cloud_clearance");
}

TEST(AlertLimitModelTest, UnknownModelProducesUnknownAlButKeepsDiagnostics) {
  iap::AlertLimitModelParams params;
  params.model = "not_a_model";

  const auto al = iap::evaluate_alert_limit(params, 6.0, 4.0, 5.0);

  EXPECT_FALSE(std::isfinite(al.hal_m));
  EXPECT_FALSE(std::isfinite(al.val_m));
  EXPECT_FALSE(std::isfinite(al.al_m));
  EXPECT_EQ(al.source, "unknown_al_model");
  EXPECT_DOUBLE_EQ(al.collision_clearance_h_m, 5.45);
  EXPECT_DOUBLE_EQ(al.collision_clearance_v_m, 4.0);
}
