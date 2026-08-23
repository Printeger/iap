#include <array>

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

TEST(Demo11PublicationStampAuthorityTest,
     RejectsZeroAndMalformedAuthorityBeforePublication) {
  Demo11PublicationStampAuthority authority;
  std::array<sensor_msgs::msg::PointCloud2, 7> clouds;

  EXPECT_EQ(authority.update(stamp(0, 0)), StampUpdateResult::kNonPositive);
  EXPECT_EQ(authority.update(stamp(-1, 1)), StampUpdateResult::kMalformed);
  EXPECT_EQ(authority.update(stamp(1, 1000000000U)),
            StampUpdateResult::kMalformed);
  EXPECT_FALSE(iap::sim::stamp_demo11_publication(authority, clouds));
  EXPECT_FALSE(authority.snapshot().has_value());
}

TEST(Demo11PublicationStampAuthorityTest,
     RetainsLastAcceptedStampAcrossRegressionAndAdvancesMonotonically) {
  Demo11PublicationStampAuthority authority;

  EXPECT_EQ(authority.update(stamp(10, 20)), StampUpdateResult::kAccepted);
  EXPECT_EQ(authority.update(stamp(9, 999999999U)),
            StampUpdateResult::kRegressed);
  ASSERT_TRUE(authority.snapshot().has_value());
  EXPECT_EQ(authority.snapshot()->sec, 10);
  EXPECT_EQ(authority.snapshot()->nanosec, 20U);

  EXPECT_EQ(authority.update(stamp(10, 21)), StampUpdateResult::kAccepted);
  ASSERT_TRUE(authority.snapshot().has_value());
  EXPECT_EQ(authority.snapshot()->sec, 10);
  EXPECT_EQ(authority.snapshot()->nanosec, 21U);
}

TEST(Demo11PublicationStampAuthorityTest,
     OnePublicationUsesBitIdenticalStampForEveryCloud) {
  Demo11PublicationStampAuthority authority;
  ASSERT_EQ(authority.update(stamp(1657065601, 123456789U)),
            StampUpdateResult::kAccepted);
  std::array<sensor_msgs::msg::PointCloud2, 7> clouds;
  for (std::size_t i = 0; i < clouds.size(); ++i) {
    clouds[i].header.stamp = stamp(static_cast<std::int32_t>(i + 1), 7U);
  }

  ASSERT_TRUE(iap::sim::stamp_demo11_publication(authority, clouds));
  for (const auto& cloud : clouds) {
    EXPECT_EQ(cloud.header.stamp.sec, 1657065601);
    EXPECT_EQ(cloud.header.stamp.nanosec, 123456789U);
  }
}

}  // namespace
