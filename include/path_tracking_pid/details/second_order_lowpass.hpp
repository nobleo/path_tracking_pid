#pragma once

#include <path_tracking_pid/details/fifo_array.hpp>

namespace path_tracking_pid::details
{

// Error tracker for the last 3 error and filtered error values.
class SecondOrderLowpass
{
public:
  // Pushes the given value to the errors FIFO buffer. A corresponding filtered error value is calculated and pushed
  // to the filtered errors FIFO buffer.
  void push(double value);

  // Resets both errors and filtered errors FIFO buffers.
  void reset();

  // Read-only access to the errors FIFO buffer.
  const FifoArray<double, 3>& errors() const;

  // Read-only access to the filtered errors FIFO buffer.
  const FifoArray<double, 3>& filtered_errors() const;

private:
  FifoArray<double, 3> errors_;
  FifoArray<double, 3> filtered_errors_;
};

}  // namespace path_tracking_pid::details
