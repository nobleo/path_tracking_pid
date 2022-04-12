#include <gtest/gtest.h>

#include <cmath>
#include <path_tracking_pid/details/second_order_lowpass.hpp>
#include <vector>

using path_tracking_pid::details::SecondOrderLowpass;

constexpr double eps = 1e-6;

TEST(SecondOrderLowpass, StepResponse)
{
  double dt = 0.1;
  double cutoff = 1 / dt / 4;
  double damping = sqrt(2);

  SecondOrderLowpass filter(cutoff, damping);

  std::vector<double> expected_response = {0.16071,  0.514214, 0.770813, 0.877725, 0.939488,
                                           0.968659, 0.984211, 0.991911, 0.995898, 0.997907};
  for (int i = 0; i < static_cast<int>(expected_response.size()); ++i) {
    SCOPED_TRACE(i);

    auto result = filter.filter(1, dt);
    EXPECT_NEAR(result, expected_response[i], eps);
  }
}

TEST(SecondOrderLowpass, Disable)
{
  double dt = 0.1;
  double cutoff = 0;
  double damping = sqrt(2);

  SecondOrderLowpass filter(cutoff, damping);
  EXPECT_NEAR(filter.filter(0, dt), 0, eps);
  EXPECT_NEAR(filter.filter(1, dt), 1, eps);
  EXPECT_NEAR(filter.filter(5, dt), 5, eps);
}

TEST(SecondOrderLowpass, StepResponseCutoff)
{
  double dt = 0.1;
  double cutoff = 1 / dt / 8;  // lower cutoff so slower response
  double damping = sqrt(2);

  SecondOrderLowpass filter(cutoff, damping);

  std::vector<double> expected_response = {
    0.068087, 0.255112, 0.461572, 0.612177, 0.720691,
    0.798844, 0.855129, 0.895665, 0.924859, 0.945884,
  };
  for (int i = 0; i < static_cast<int>(expected_response.size()); ++i) {
    SCOPED_TRACE(i);

    auto result = filter.filter(1, dt);
    EXPECT_NEAR(result, expected_response[i], eps);
  }
}

TEST(SecondOrderLowpass, StepResponseDamping)
{
  double dt = 0.1;
  double cutoff = 1 / dt / 4;
  double damping = sqrt(2) * 2;  // more damping

  SecondOrderLowpass filter(cutoff, damping);

  std::vector<double> expected_response = {
    0.101795, 0.318258, 0.494899, 0.618187, 0.716157,
    0.786043, 0.84057,  0.880057, 0.91048,  0.932743,
  };
  for (int i = 0; i < static_cast<int>(expected_response.size()); ++i) {
    SCOPED_TRACE(i);

    auto result = filter.filter(1, dt);
    EXPECT_NEAR(result, expected_response[i], eps);
  }
}

TEST(SecondOrderLowpass, Reset)
{
  double dt = 0.1;
  double cutoff = 1 / dt / 4;
  double damping = sqrt(2);

  SecondOrderLowpass filter(cutoff, damping);

  std::vector<double> expected_response = {0.16071, 0.514214, 0.770813, 0.877725, 0.939488};
  for (int i = 0; i < static_cast<int>(expected_response.size()); ++i) {
    SCOPED_TRACE(i);

    auto result = filter.filter(1, dt);
    EXPECT_NEAR(result, expected_response[i], eps);
  }

  filter.reset();

  for (int i = 0; i < static_cast<int>(expected_response.size()); ++i) {
    SCOPED_TRACE(i);

    auto result = filter.filter(1, dt);
    EXPECT_NEAR(result, expected_response[i], eps);
  }
}

TEST(SecondOrderLowpass, Configure)
{
  double dt = 0.1;
  double cutoff = 1 / dt / 4;
  double damping = sqrt(2);

  SecondOrderLowpass filter(cutoff, damping);

  std::vector<double> expected_response = {0.16071, 0.514214, 0.770813, 0.877725, 0.939488};
  for (int i = 0; i < static_cast<int>(expected_response.size()); ++i) {
    SCOPED_TRACE(i);

    auto result = filter.filter(1, dt);
    EXPECT_NEAR(result, expected_response[i], eps);
  }

  filter.configure(cutoff / 2, damping);

  // no configure step response is {0.968659, 0.984211, 0.991911, 0.995898, 0.997907}
  expected_response = {0.957154, 0.969162, 0.977792, 0.984006, 0.988481};
  for (int i = 0; i < static_cast<int>(expected_response.size()); ++i) {
    SCOPED_TRACE(i);

    auto result = filter.filter(1, dt);
    EXPECT_NEAR(result, expected_response[i], eps);
  }
}
