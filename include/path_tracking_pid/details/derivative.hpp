#pragma once

#include <path_tracking_pid/details/fifo_array.hpp>

namespace path_tracking_pid::details
{
/**
 * @brief Discrete time derivative filter
 */
class Derivative
{
public:
  /**
   * @brief Filter one sample of a signal
   * @param u Signal to be filtered
   * @param step_size Time step from previous sample
   * @return Derivative of the signal
   */
  double filter(double u, double step_size);

  /**
   * @brief Reset the signal buffers
   */
  void reset();

private:
  FifoArray<double, 2> u_ = {};
};

}  // namespace path_tracking_pid::details
