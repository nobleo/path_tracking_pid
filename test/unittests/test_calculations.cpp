#include <gtest/gtest.h>

#include <vector>

#include "../../src/calculations.hpp"

namespace
{

using path_tracking_pid::deltas_of_plan;

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

}  // namespace
