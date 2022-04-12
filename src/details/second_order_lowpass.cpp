#include <cmath>
#include <path_tracking_pid/details/second_order_lowpass.hpp>

namespace path_tracking_pid::details
{
SecondOrderLowpass::SecondOrderLowpass(double cutoff, double damping)
: cutoff_(cutoff), damping_(damping)
{
}

void SecondOrderLowpass::configure(double cutoff, double damping)
{
  cutoff_ = cutoff;
  damping_ = damping;
}

double SecondOrderLowpass::filter(double u, double step_size)
{
  // save history
  u_.push(u);
  y_.push(u);  // increase index so the math below looks correct

  if (cutoff_ == 0) {
    return u;
  }

  // A continous time second order lowpass was discretized with Tustin's method. For a mathematical
  // explanation, see doc/second_order_lowpass_tustin.ipynb
  auto c = cutoff_;
  auto d = damping_;
  auto T = step_size;
  auto a = 2 * M_PI * c;
  auto b = T * a;
  y_[0] = ((pow(b, 2)) * u_[0] + (2 * pow(b, 2)) * u_[1] + (pow(b, 2)) * u_[2] -
           (2 * pow(b, 2) - 8) * y_[1] - (pow(b, 2) - 4 * T * a * d + 4) * y_[2]) /
          (pow(b, 2) + 4 * T * a * d + 4);
  return y_[0];
}

void SecondOrderLowpass::reset()
{
  u_ = {};
  y_ = {};
}
}  // namespace path_tracking_pid::details
