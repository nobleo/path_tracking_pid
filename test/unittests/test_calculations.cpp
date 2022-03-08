#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "../../src/calculations.hpp"

namespace
{

using path_tracking_pid::deltas_of_plan;
using path_tracking_pid::distances_to_goal;
using path_tracking_pid::distSquared;
using path_tracking_pid::inverse_turning_radiuses;
using path_tracking_pid::is_in_direction_of_target;

// Create a transform (with an identity basis) based on the given coordinates.
tf2::Transform create_transform(double x, double y, double z)
{
  tf2::Transform result;
  result.getBasis().setIdentity();
  result.setOrigin({x, y, z});
  return result;
}

TEST(PathTrackingPidCalculations, DeltasOfPlan_Empty)
{
  const auto plan = std::vector<tf2::Transform>{};
  const auto result = deltas_of_plan(plan);

  EXPECT_TRUE(result.empty());
}

TEST(PathTrackingPidCalculations, DeltasOfPlan_OnlyOne)
{
  const auto plan = std::vector<tf2::Transform>{create_transform(1, 1, 1)};
  const auto result = deltas_of_plan(plan);

  EXPECT_TRUE(result.empty());
}

TEST(PathTrackingPidCalculations, DeltasOfPlan)
{
  const auto plan = std::vector<tf2::Transform>{
    create_transform(0, 0, 0), create_transform(1, 1, 1), create_transform(0, 0, 4)};
  const auto ref =
    std::vector<tf2::Transform>{create_transform(1, 1, 1), create_transform(-1, -1, 3)};
  const auto result = deltas_of_plan(plan);

  EXPECT_EQ(ref, result);
}

TEST(PathTrackingPidCalculations, DistancesToGoal_Empty)
{
  const auto deltas = std::vector<tf2::Transform>{};
  const auto ref = std::vector<double>{0.0};
  const auto result = distances_to_goal(deltas);

  EXPECT_EQ(ref, result);
}

TEST(PathTrackingPidCalculations, DistancesToGoal_Single)
{
  const auto deltas = std::vector<tf2::Transform>{create_transform(3, 4, 0)};
  const auto ref = std::vector<double>{5.0, 0.0};
  const auto result = distances_to_goal(deltas);

  EXPECT_EQ(ref, result);
}

TEST(PathTrackingPidCalculations, DistancesToGoal)
{
  const auto deltas = std::vector<tf2::Transform>{
    create_transform(3, 4, 0), create_transform(-2, 0, 0), create_transform(0, 1, 0)};
  const auto ref = std::vector<double>{8.0, 3.0, 1.0, 0.0};
  const auto result = distances_to_goal(deltas);

  EXPECT_EQ(ref, result);
}

TEST(PathTrackingPidCalculations, DistancesToGoal_IgnoreZ)
{
  const auto deltas = std::vector<tf2::Transform>{
    create_transform(3, 4, 1), create_transform(-2, 0, 2), create_transform(0, 1, 3)};
  const auto ref = std::vector<double>{8.0, 3.0, 1.0, 0.0};
  const auto result = distances_to_goal(deltas);

  EXPECT_EQ(ref, result);
}

TEST(PathTrackingPidCalculations, InverseTurningRadiuses_Empty)
{
  const auto deltas = std::vector<tf2::Transform>{};
  const auto ref = std::vector<double>{0.0};
  const auto result = inverse_turning_radiuses(deltas);

  EXPECT_EQ(ref, result);
}

TEST(PathTrackingPidCalculations, InverseTurningRadiuses_SingleTooSmall)
{
  const auto deltas = std::vector<tf2::Transform>{create_transform(0, 0, 0)};
  const auto ref = std::vector<double>{std::numeric_limits<double>::infinity(), 0.0};
  const auto result = inverse_turning_radiuses(deltas);

  EXPECT_EQ(ref, result);
}

TEST(PathTrackingPidCalculations, InverseTurningRadiuses_Single)
{
  const auto deltas = std::vector<tf2::Transform>{create_transform(3, 4, 0)};
  const auto ref = std::vector<double>{8.0 / 25.0, 0.0};
  const auto result = inverse_turning_radiuses(deltas);

  EXPECT_EQ(ref, result);
}

TEST(PathTrackingPidCalculations, InverseTurningRadiuses)
{
  const auto deltas = std::vector<tf2::Transform>{
    create_transform(3, 4, 0), create_transform(2, 0, 0), create_transform(0, 2, 0)};
  const auto ref = std::vector<double>{8.0 / 25.0, 0.0, 1.0, 0.0};
  const auto result = inverse_turning_radiuses(deltas);

  EXPECT_EQ(ref, result);
}

TEST(PathTrackingPidCalculations, InverseTurningRadiuses_IgnoreZ)
{
  const auto deltas = std::vector<tf2::Transform>{
    create_transform(3, 4, 1), create_transform(2, 0, 2), create_transform(0, 2, 3)};
  const auto ref = std::vector<double>{8.0 / 25.0, 0.0, 1.0, 0.0};
  const auto result = inverse_turning_radiuses(deltas);

  EXPECT_EQ(ref, result);
}

// Parameters for parameterized tests of is_in_direction_of_target().
struct IsInDirectionOfTargetTestParams
{
  bool result = false;
  tf2::Transform current;
  tf2::Vector3 target;
  double velocity = 0.0;
};

// Fixture for tests of is_in_direction_of_target().
class IsInDirectionOfTargetTestFixture
: public ::testing::TestWithParam<IsInDirectionOfTargetTestParams>
{
};

TEST_P(IsInDirectionOfTargetTestFixture, IsInDirectionOfTarget)
{
  const auto & [result, current, target, velocity] = GetParam();

  EXPECT_EQ(result, is_in_direction_of_target(current, target, velocity));
  EXPECT_EQ(!result, is_in_direction_of_target(current, target, -velocity));
}

// Input for parameterized tests of is_in_direction_of_target().
static const auto is_in_direction_of_target_params = std::vector<IsInDirectionOfTargetTestParams>({
  // base tests: x delta
  {true, tf2::Transform{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {2, 3, 4}}, {3, 3, 4}, 1.0},
  // 0 velocity
  {true, tf2::Transform{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {2, 3, 4}}, {3, 3, 4}, 0.0},
  // already at target
  {true, tf2::Transform{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {3, 3, 4}}, {3, 3, 4}, 1.0},
  // past target
  {false, tf2::Transform{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {4, 3, 4}}, {3, 3, 4}, 1.0},
  // y delta
  {true, tf2::Transform{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {3, 2, 4}}, {3, 3, 4}, 1.0},
  // z delta
  {true, tf2::Transform{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {3, 3, 3}}, {3, 3, 4}, 1.0},
  // xyz delta
  {true, tf2::Transform{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {2, 2, 3}}, {3, 3, 4}, 1.0},
  // 0 basis
  {true, tf2::Transform{{0, 0, 0, 0, 0, 0, 0, 0, 0}, {2, 3, 4}}, {3, 4, 5}, 1.0},
  // inspired by actual values from integration tests
  {false, tf2::Transform{{-1, -0.5, 0, 0.5, -1, 0, 0, 0, 1}, {-1, 2, 0}}, {-0.5, 2, 0}, 0.5},
  {false, tf2::Transform{{-1, 0.5, 0, -0.5, -1, 0, 0, 0, 1}, {-0.6, 2, 0}}, {-0.5, 2, 0}, 0.5},
  {true, tf2::Transform{{-0.5, -1, 0, 1, -0.5, 0, 0, 0, 1}, {12, 1.5, 0}}, {11, 1.8, 0}, 0.5},
  {true, tf2::Transform{{-1, -0.5, 0, 0.5, -1, 0, 0, 0, 1}, {11, 2.3, 0}}, {10, 2.1, 0}, 0.7},
  {true, tf2::Transform{{-1, 0.5, 0, -0.5, -1, 0, 0, 0, 1}, {9, 2.2, 0}}, {8, 2, 0}, 0.5},
  {true, tf2::Transform{{0.1, -1, 0, 1, 0.1, 0, 0, 0, 1}, {12, 1, 0}}, {11, 2, 0}, 0.5},
  {true, tf2::Transform{{1, -0.5, 0, 0.5, 1, 0, 0, 0, 1}, {11.2, 0.1, 0}}, {11.1, 0.7, 0}, 0.5},
});

INSTANTIATE_TEST_CASE_P(
  PathTrackingPidCalculations, IsInDirectionOfTargetTestFixture,
  ::testing::ValuesIn(is_in_direction_of_target_params),
  [](const auto & info) { return std::to_string(info.index); });

TEST(PathTrackingPidCalculations, DistSquared)
{
  EXPECT_EQ(14, distSquared(create_transform(1, 2, 3), create_transform(2, 4, 6)));
  EXPECT_EQ(45, distSquared(create_transform(6, 2, 0), create_transform(2, 4, -5)));
  EXPECT_EQ(0, distSquared(create_transform(1, 2, 3), create_transform(1, 2, 3)));
}

}  // namespace
