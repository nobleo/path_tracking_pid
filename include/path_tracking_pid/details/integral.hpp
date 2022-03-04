#pragma once

#include <path_tracking_pid/details/fifo_array.hpp>

namespace path_tracking_pid::details
{
/**
 * @brief Discrete time integral filter
 */
class Integral
{
  constexpr static auto NaN = std::numeric_limits<double>::quiet_NaN();

public:
  Integral() = default;

  /**
   * @brief Construct an integral filter
   * @param windup_limit Integral windup limit
   */
  explicit Integral(double windup_limit);

  /**
   * @brief Change the parameters of the filter
   * @param windup_limit Integral windup limit
   */
  void configure(double windup_limit);

  /**
   * @brief Filter one sample of a signal
   * @param u Signal to be filtered
   * @param step_size Time step from previous sample
   * @return Integral of the signal
   */
  double filter(double u, double step_size);

  /**
   * @brief Reset the signal buffers
   */
  void reset();

private:
  FifoArray<double, 2> u_ = {};
  FifoArray<double, 2> y_ = {};
  double windup_limit_ = NaN;
};

}  // namespace path_tracking_pid::details
