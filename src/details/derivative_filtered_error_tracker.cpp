#include <path_tracking_pid/details/derivative_filtered_error_tracker.hpp>

namespace path_tracking_pid::details
{

void DerivativeFilteredErrorTracker::push(double value, std::chrono::duration<double> td)
{
  errors_.push(value);
  derivative_errors_.push((errors_.errors().at<0>() - errors_.errors().at<1>()) / td.count());
}

void DerivativeFilteredErrorTracker::reset()
{
  errors_.reset();
  derivative_errors_.reset();
}

double DerivativeFilteredErrorTracker::current_error() const
{
  return errors_.errors().at<0>();
}

double DerivativeFilteredErrorTracker::current_filtered_error() const
{
  return errors_.filtered_errors().at<0>();
}

double DerivativeFilteredErrorTracker::current_derivative_error() const
{
  return derivative_errors_.errors().at<0>();
}

double DerivativeFilteredErrorTracker::current_filtered_derivative_error() const
{
  return derivative_errors_.filtered_errors().at<0>();
}

}  // namespace path_tracking_pid::details
