#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <iap/sim/demo11_publication_stamp_authority.hpp>

namespace {

using iap::sim::Demo11PublicationStampAuthority;
using iap::sim::StampUpdateResult;

builtin_interfaces::msg::Time stamp(const std::int32_t sec,
                                    const std::uint32_t nanosec) {
  builtin_interfaces::msg::Time value;
  value.sec = sec;
  value.nanosec = nanosec;
  return value;
}

void expect_stamp(const sensor_msgs::msg::PointCloud2& cloud,
                  const std::int32_t expected_sec,
                  const std::uint32_t expected_nanosec) {
  EXPECT_EQ(cloud.header.stamp.sec, expected_sec);
  EXPECT_EQ(cloud.header.stamp.nanosec, expected_nanosec);
}

TEST(Demo11PublicationStampAuthorityTest,
     ProductionVariadicFanoutWaitsForAuthorityAndStampsAllSevenClouds) {
  Demo11PublicationStampAuthority authority;
  sensor_msgs::msg::PointCloud2 global_cloud;
  sensor_msgs::msg::PointCloud2 local_cloud;
  sensor_msgs::msg::PointCloud2 trunk_cloud;
  sensor_msgs::msg::PointCloud2 canopy_cloud;
  sensor_msgs::msg::PointCloud2 terminal_wall_cloud;
  sensor_msgs::msg::PointCloud2 p0_fixture_cloud;
  sensor_msgs::msg::PointCloud2 p1_fixture_cloud;

  EXPECT_FALSE(iap::sim::stamp_demo11_publication(
      authority, global_cloud, local_cloud, trunk_cloud, canopy_cloud,
      terminal_wall_cloud, p0_fixture_cloud, p1_fixture_cloud));
  EXPECT_FALSE(authority.snapshot().has_value());

  ASSERT_EQ(authority.update(stamp(1657065601, 123456789U)),
            StampUpdateResult::kAccepted);
  ASSERT_TRUE(iap::sim::stamp_demo11_publication(
      authority, global_cloud, local_cloud, trunk_cloud, canopy_cloud,
      terminal_wall_cloud, p0_fixture_cloud, p1_fixture_cloud));

  expect_stamp(global_cloud, 1657065601, 123456789U);
  expect_stamp(local_cloud, 1657065601, 123456789U);
  expect_stamp(trunk_cloud, 1657065601, 123456789U);
  expect_stamp(canopy_cloud, 1657065601, 123456789U);
  expect_stamp(terminal_wall_cloud, 1657065601, 123456789U);
  expect_stamp(p0_fixture_cloud, 1657065601, 123456789U);
  expect_stamp(p1_fixture_cloud, 1657065601, 123456789U);
}

TEST(Demo11PublicationStampAuthorityTest,
     InvalidUpdatesRetainAcceptedStampBeforeMonotonicSevenCloudAdvance) {
  Demo11PublicationStampAuthority authority;
  ASSERT_EQ(authority.update(stamp(10, 20)), StampUpdateResult::kAccepted);

  const auto expect_rejected_without_replacement =
      [&authority](const builtin_interfaces::msg::Time& rejected,
                   const StampUpdateResult expected_result) {
        EXPECT_EQ(authority.update(rejected), expected_result);
        const auto retained = authority.snapshot();
        ASSERT_TRUE(retained.has_value());
        EXPECT_EQ(retained->sec, 10);
        EXPECT_EQ(retained->nanosec, 20U);
      };

  expect_rejected_without_replacement(stamp(0, 0),
                                      StampUpdateResult::kNonPositive);
  expect_rejected_without_replacement(stamp(-1, 1),
                                      StampUpdateResult::kMalformed);
  expect_rejected_without_replacement(stamp(11, 1000000000U),
                                      StampUpdateResult::kMalformed);
  expect_rejected_without_replacement(stamp(9, 999999999U),
                                      StampUpdateResult::kRegressed);

  sensor_msgs::msg::PointCloud2 retained_global;
  sensor_msgs::msg::PointCloud2 retained_local;
  sensor_msgs::msg::PointCloud2 retained_trunk;
  sensor_msgs::msg::PointCloud2 retained_canopy;
  sensor_msgs::msg::PointCloud2 retained_terminal_wall;
  sensor_msgs::msg::PointCloud2 retained_p0_fixture;
  sensor_msgs::msg::PointCloud2 retained_p1_fixture;
  ASSERT_TRUE(iap::sim::stamp_demo11_publication(
      authority, retained_global, retained_local, retained_trunk,
      retained_canopy, retained_terminal_wall, retained_p0_fixture,
      retained_p1_fixture));
  expect_stamp(retained_global, 10, 20U);
  expect_stamp(retained_local, 10, 20U);
  expect_stamp(retained_trunk, 10, 20U);
  expect_stamp(retained_canopy, 10, 20U);
  expect_stamp(retained_terminal_wall, 10, 20U);
  expect_stamp(retained_p0_fixture, 10, 20U);
  expect_stamp(retained_p1_fixture, 10, 20U);

  ASSERT_EQ(authority.update(stamp(10, 21)), StampUpdateResult::kAccepted);
  sensor_msgs::msg::PointCloud2 next_global;
  sensor_msgs::msg::PointCloud2 next_local;
  sensor_msgs::msg::PointCloud2 next_trunk;
  sensor_msgs::msg::PointCloud2 next_canopy;
  sensor_msgs::msg::PointCloud2 next_terminal_wall;
  sensor_msgs::msg::PointCloud2 next_p0_fixture;
  sensor_msgs::msg::PointCloud2 next_p1_fixture;
  ASSERT_TRUE(iap::sim::stamp_demo11_publication(
      authority, next_global, next_local, next_trunk, next_canopy,
      next_terminal_wall, next_p0_fixture, next_p1_fixture));
  expect_stamp(next_global, 10, 21U);
  expect_stamp(next_local, 10, 21U);
  expect_stamp(next_trunk, 10, 21U);
  expect_stamp(next_canopy, 10, 21U);
  expect_stamp(next_terminal_wall, 10, 21U);
  expect_stamp(next_p0_fixture, 10, 21U);
  expect_stamp(next_p1_fixture, 10, 21U);
}

}  // namespace
