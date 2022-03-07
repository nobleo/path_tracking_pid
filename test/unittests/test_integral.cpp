#include <gtest/gtest.h>

#include <path_tracking_pid/details/integral.hpp>
#include <vector>

using path_tracking_pid::details::Integral;

constexpr double eps = 1e-6;

TEST(Integral, StepResponse)
{
  double dt = 0.1;
  double windup_limit = 0.5;

  Integral filter{windup_limit};

  std::vector<double> expected_response = {0.05, 0.15, 0.25, 0.35, 0.45, 0.5, 0.5};
  for (int i = 0; i < static_cast<int>(expected_response.size()); ++i) {
    SCOPED_TRACE(i);

    auto result = filter.filter(1, dt);
    EXPECT_NEAR(result, expected_response[i], eps);
  }
}

TEST(Integral, Reset)
{
  double dt = 0.1;
  double windup_limit = 0.5;

  Integral filter{windup_limit};

  EXPECT_NEAR(filter.filter(1, dt), 0.05, eps);
  EXPECT_NEAR(filter.filter(1, dt), 0.15, eps);
  filter.reset();
  EXPECT_NEAR(filter.filter(1, dt), 0.05, eps);
  EXPECT_NEAR(filter.filter(1, dt), 0.15, eps);
}

TEST(Integral, Configure)
{
  double dt = 0.1;
  double windup_limit = 0.2;

  Integral filter{windup_limit};

  EXPECT_NEAR(filter.filter(1, dt), 0.05, eps);
  EXPECT_NEAR(filter.filter(1, dt), 0.15, eps);
  EXPECT_NEAR(filter.filter(1, dt), 0.20, eps);
  filter.configure(0.35);
  EXPECT_NEAR(filter.filter(1, dt), 0.30, eps);
  EXPECT_NEAR(filter.filter(1, dt), 0.35, eps);
}
