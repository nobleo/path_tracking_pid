#pragma once

#include <path_tracking_pid/details/filtered_error_tracker.hpp>

#include <chrono>

namespace path_tracking_pid::details
{

// Error tracker for the last 3 error, derivative error and their filtered values.
class DerivativeFilteredErrorTracker
{
public:
  // Pushes the given value to the errors FIFO buffer. A corresponding filtered error value is calculated and pushed
  // to the filtered errors FIFO buffer. The given time delta is used to calculate the derivative error, which is
  // pushed to the derivative errors FIFO buffer. A corresponding filtered derivative error value is calculated and
  // pushed to the filtered derivative errors FIFO buffer.
  void push(double value, std::chrono::duration<double> td);

  // Resets all FIFO buffers.
  void reset();

  // Retrieves the current error value.
  double current_error() const;

  // Retrieves the current filtered error value.
  double current_filtered_error() const;

  // Retrieves the current derivative error value.
  double current_derivative_error() const;

  // Retrieves the current filtered derivative error value.
  double current_filtered_derivative_error() const;

private:
  FilteredErrorTracker errors_;
  FilteredErrorTracker derivative_errors_;
};

}  // namespace path_tracking_pid::details
