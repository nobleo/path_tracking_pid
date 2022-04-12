#pragma once

#include <limits>
#include <path_tracking_pid/details/fifo_array.hpp>

namespace path_tracking_pid::details
{
/**
 * @brief Discrete time second order lowpass filter
 */
class SecondOrderLowpass
{
  constexpr static auto NaN = std::numeric_limits<double>::quiet_NaN();

public:
  /**
   * @brief Construct a SecondOrderLowpass instance with NaNs
   */
  SecondOrderLowpass() = default;

  /**
   * @brief Construct a SecondOrderLowpass instance
   * @param cutoff frequency in Hz, 0 disables the filter
   * @param damping frequency in Hz
   */
  SecondOrderLowpass(double cutoff, double damping);

  /**
   * @brief Change the parameters of the filter
   * @param cutoff frequency in Hz, 0 disables the filter
   * @param damping frequency in Hz
   */
  void configure(double cutoff, double damping);

  /**
   * @brief Filter one sample of a signal
   * @param u Signal to be filtered
   * @param step_size Time step from previous sample
   * @return Lowpass-filtered signal
   */
  double filter(double u, double step_size);

  /**
   * @brief Reset the signal buffers
   */
  void reset();

private:
  FifoArray<double, 3> u_ = {};
  FifoArray<double, 3> y_ = {};
  double cutoff_ = NaN;
  double damping_ = NaN;
};

}  // namespace path_tracking_pid::details
