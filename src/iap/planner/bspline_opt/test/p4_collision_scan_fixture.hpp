#ifndef BSPLINE_OPT_TEST_P4_COLLISION_SCAN_FIXTURE_HPP_
#define BSPLINE_OPT_TEST_P4_COLLISION_SCAN_FIXTURE_HPP_

#include <array>
#include <cstddef>
#include <limits>
#include <string_view>

namespace p4_collision_fixture
{

constexpr std::size_t kSampleCount = 15;
constexpr std::size_t kMaxExpectedSegments = 2;

enum class CollisionScanStatus
{
  kNoCollision,
  kClosedSegments,
  kOpenEndedCollision,
  kInvalidInput,
};

enum class SeedShape
{
  kValid,
  kEmpty,
  kNonFinite,
  kStructurallyInvalid,
};

struct Sample
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  bool occupied = false;
};

struct Segment
{
  int free_start_index = -1;
  int free_end_index = -1;

  friend bool operator==(const Segment & lhs, const Segment & rhs)
  {
    return lhs.free_start_index == rhs.free_start_index &&
           lhs.free_end_index == rhs.free_end_index;
  }
};

struct CollisionCase
{
  std::string_view name;
  std::array<Sample, kSampleCount> samples{};
  std::size_t sample_count = kSampleCount;
  SeedShape seed_shape = SeedShape::kValid;
  bool occupancy_truth_available = true;
  CollisionScanStatus expected_status = CollisionScanStatus::kNoCollision;
  std::array<Segment, kMaxExpectedSegments> expected_segments{};
  std::size_t expected_segment_count = 0;
};

constexpr std::string_view statusName(const CollisionScanStatus status)
{
  switch (status) {
    case CollisionScanStatus::kNoCollision:
      return "NO_COLLISION";
    case CollisionScanStatus::kClosedSegments:
      return "CLOSED_SEGMENTS";
    case CollisionScanStatus::kOpenEndedCollision:
      return "OPEN_ENDED_COLLISION";
    case CollisionScanStatus::kInvalidInput:
      return "INVALID_INPUT";
  }
  return "INVALID_INPUT";
}

constexpr std::array<Sample, kSampleCount> samplesWithOccupancy(
  const std::array<bool, kSampleCount> & occupancy)
{
  std::array<Sample, kSampleCount> samples{};
  for (std::size_t index = 0; index < samples.size(); ++index) {
    samples[index] = Sample{static_cast<double>(index), 0.0, 0.0,
      occupancy[index]};
  }
  return samples;
}

constexpr std::array<bool, kSampleCount> occupiedIndices(
  const std::array<int, kSampleCount> & indices, const std::size_t count)
{
  std::array<bool, kSampleCount> occupancy{};
  for (std::size_t offset = 0; offset < count; ++offset) {
    occupancy[static_cast<std::size_t>(indices[offset])] = true;
  }
  return occupancy;
}

constexpr CollisionCase makeCase(
  const std::string_view name,
  const std::array<bool, kSampleCount> & occupancy,
  const CollisionScanStatus expected_status,
  const std::array<Segment, kMaxExpectedSegments> & expected_segments = {},
  const std::size_t expected_segment_count = 0)
{
  return CollisionCase{name,
    samplesWithOccupancy(occupancy),
    kSampleCount,
    SeedShape::kValid,
    true,
    expected_status,
    expected_segments,
    expected_segment_count};
}

constexpr std::array<bool, kSampleCount> kAllFree{};
constexpr auto kOneClosedOccupancy = occupiedIndices(
  {4, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 2);
constexpr auto kLateExitOccupancy = occupiedIndices(
  {8, 9, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 3);
constexpr auto kOpenEndedOccupancy = occupiedIndices(
  {7, 8, 9, 10, 11, 12, 13, 14, 0, 0, 0, 0, 0, 0, 0}, 8);
constexpr auto kMultipleClosedOccupancy = occupiedIndices(
  {3, 4, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 3);
constexpr auto kClosedThenOpenOccupancy = occupiedIndices(
  {3, 4, 7, 8, 9, 10, 11, 12, 13, 14, 0, 0, 0, 0, 0}, 10);

constexpr CollisionCase kNoCollision = makeCase(
    "no_collision", kAllFree, CollisionScanStatus::kNoCollision);
constexpr CollisionCase kOneClosed = makeCase(
    "one_closed", kOneClosedOccupancy,
    CollisionScanStatus::kClosedSegments, {Segment{3, 6}}, 1);
constexpr CollisionCase kLateExitClosed = makeCase(
    "entry_before_two_thirds_exit_after_two_thirds", kLateExitOccupancy,
    CollisionScanStatus::kClosedSegments, {Segment{7, 11}}, 1);
constexpr CollisionCase kOpenEnded = makeCase(
    "open_ended", kOpenEndedOccupancy,
    CollisionScanStatus::kOpenEndedCollision);
constexpr CollisionCase kMultipleClosed = makeCase(
    "multiple_closed", kMultipleClosedOccupancy,
    CollisionScanStatus::kClosedSegments,
  {Segment{2, 5}, Segment{6, 8}}, 2);
constexpr CollisionCase kClosedThenOpen = makeCase(
    "closed_then_open", kClosedThenOpenOccupancy,
    CollisionScanStatus::kOpenEndedCollision);

constexpr CollisionCase invalidCase(
  const std::string_view name,
  const SeedShape shape,
  const bool occupancy_truth_available = true)
{
  CollisionCase fixture = makeCase(
      name, kAllFree, CollisionScanStatus::kInvalidInput);
  fixture.seed_shape = shape;
  fixture.occupancy_truth_available = occupancy_truth_available;
  if (shape == SeedShape::kEmpty) {
    fixture.sample_count = 0;
  }
  if (shape == SeedShape::kNonFinite) {
    fixture.samples[7].x = std::numeric_limits<double>::quiet_NaN();
  }
  return fixture;
}

constexpr CollisionCase kEmptySeed = invalidCase(
    "empty_seed", SeedShape::kEmpty);
constexpr CollisionCase kNonFiniteSeed = invalidCase(
    "non_finite_seed", SeedShape::kNonFinite);
constexpr CollisionCase kStructurallyInvalidSeed = invalidCase(
    "structurally_invalid_seed", SeedShape::kStructurallyInvalid);
constexpr CollisionCase kUnavailableOccupancy = invalidCase(
    "unavailable_occupancy", SeedShape::kValid, false);

constexpr std::array<CollisionCase, 10> kAllCases = {
  kNoCollision,
  kOneClosed,
  kLateExitClosed,
  kOpenEnded,
  kEmptySeed,
  kNonFiniteSeed,
  kStructurallyInvalidSeed,
  kUnavailableOccupancy,
  kMultipleClosed,
  kClosedThenOpen,
};

}  // namespace p4_collision_fixture

#endif  // BSPLINE_OPT_TEST_P4_COLLISION_SCAN_FIXTURE_HPP_
