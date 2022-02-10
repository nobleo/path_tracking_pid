#include <path_tracking_pid/details/filtered_error_tracker.hpp>

#include <cmath>

namespace path_tracking_pid::details
{

namespace
{

// Used in filter calculations. Default 1.0 corresponds to a cutoff frequency at 1/4 of the sample rate.
constexpr double cutoff = 1.;

}  // namespace

void FilteredErrorTracker::push(double value)
{
  errors_.push(value);

  filtered_errors_.push((1 / (1 + cutoff * cutoff + M_SQRT2 * cutoff)) *
                        (errors_.at<2>() + 2 * errors_.at<1>() + errors_.at<0>() -
                         (cutoff * cutoff - M_SQRT2 * cutoff + 1) * filtered_errors_.at<1>() -
                         (-2 * cutoff * cutoff + 2) * filtered_errors_.at<0>()));
}

void FilteredErrorTracker::reset()
{
  errors_.reset();
  filtered_errors_.reset();
}

const FifoArray<double, 3>& FilteredErrorTracker::errors() const
{
  return errors_;
}
const FifoArray<double, 3>& FilteredErrorTracker::filtered_errors() const
{
  return filtered_errors_;
}

}  // namespace path_tracking_pid::details
