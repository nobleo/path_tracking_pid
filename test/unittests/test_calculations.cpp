#include <gtest/gtest.h>

#include <vector>

#include "../../src/calculations.hpp"

namespace
{

using path_tracking_pid::deltas_of_plan;
using path_tracking_pid::distances_to_goal;
using path_tracking_pid::inverse_turning_radiuses;

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

}  // namespace
