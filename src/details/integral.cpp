#include <path_tracking_pid/details/integral.hpp>

namespace path_tracking_pid::details
{
Integral::Integral(double windup_limit) : windup_limit_(windup_limit) {}

void Integral::configure(double windup_limit) { windup_limit_ = windup_limit; }

double Integral::filter(double u, double step_size)
{
  // save history
  u_.push(u);
  y_.push(u);  // increase index so the math below looks correct

  // A continous time integrator was discretized with Tustin's method. For a mathematical
  // explanation, see doc/integral_tustin.ipynb
  auto T = step_size;
  y_[0] = T / 2 * (u_[0] + u_[1]) + y_[1];
  y_[0] = std::clamp(y_[0], -windup_limit_, windup_limit_);
  return y_[0];
}

void Integral::reset()
{
  u_ = {};
  y_ = {};
}
}  // namespace path_tracking_pid::details
