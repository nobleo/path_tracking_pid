#include <gtest/gtest.h>

#include <path_tracking_pid/details/derivative.hpp>
#include <vector>

using path_tracking_pid::details::Derivative;

constexpr double eps = 1e-6;

TEST(Derivative, StepResponse)
{
  double dt = 0.1;

  Derivative filter;

  std::vector<double> expected_response = {10, 0, 0};
  for (int i = 0; i < static_cast<int>(expected_response.size()); ++i) {
    SCOPED_TRACE(i);

    auto result = filter.filter(1, dt);
    EXPECT_NEAR(result, expected_response[i], eps);
  }
}

TEST(Derivative, Reset)
{
  double dt = 0.1;

  Derivative filter;

  EXPECT_NEAR(filter.filter(1, dt), 10, eps);
  filter.reset();
  EXPECT_NEAR(filter.filter(0, dt), 0, eps);
}
