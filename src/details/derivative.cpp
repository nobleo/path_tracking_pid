#include <path_tracking_pid/details/derivative.hpp>

namespace path_tracking_pid::details
{
double Derivative::filter(double u, double step_size)
{
  // save history
  u_.push(u);
  return (u_[0] - u_[1]) / step_size;
}

void Derivative::reset() { u_ = {}; }
}  // namespace path_tracking_pid::details
