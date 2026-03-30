#include <gtest/gtest.h>

#include <iap/odometry/shared_target_handle.hpp>

#include <gtsam_points/ann/ivox.hpp>

TEST(SharedTargetHandle, RetainsIdentityRevisionAndMetadata) {
  auto target = std::make_shared<gtsam_points::iVox>(0.5);
  std::shared_ptr<const gtsam_points::NearestNeighborSearch> tree = target;

  iap::SharedTargetHandle handle(
    iap::SharedTargetHandleMode::GlobalReference,
    42,
    target,
    tree,
    3,
    128,
    3,
    128,
    0.2,
    true,
    1.5);

  EXPECT_EQ(handle.mode(), iap::SharedTargetHandleMode::GlobalReference);
  EXPECT_EQ(handle.revision(), 42u);
  EXPECT_EQ(handle.identity(), target.get());
  EXPECT_EQ(handle.target_snapshot().get(), target.get());
  EXPECT_EQ(handle.target_tree().get(), tree.get());
  EXPECT_EQ(handle.contributing_frames(), 3u);
  EXPECT_EQ(handle.point_count(), 128u);
  EXPECT_EQ(handle.snapshot_frame_count(), 3u);
  EXPECT_EQ(handle.snapshot_point_count(), 128u);
  EXPECT_DOUBLE_EQ(handle.snapshot_span_sec(), 0.2);
  EXPECT_TRUE(handle.snapshot_policy_accepted());
  EXPECT_DOUBLE_EQ(handle.build_ms(), 1.5);
}
