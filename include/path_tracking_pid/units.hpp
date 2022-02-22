#pragma once

#include <boost/units/cmath.hpp>
#include <boost/units/systems/si.hpp>

namespace path_tracking_pid::units
{

// Distance in meter.
using distance_t = boost::units::quantity<boost::units::si::length, double>;

// Distance squared in square meter.
using distance_squared_t = boost::units::quantity<boost::units::si::area, double>;

// Duration in second.
using duration_t = boost::units::quantity<boost::units::si::time, double>;

// Velocity in meter per second.
using velocity_t = boost::units::quantity<boost::units::si::velocity, double>;

// Acceleration in meter per second squared.
using acceleration_t = boost::units::quantity<boost::units::si::acceleration, double>;

// The following units can be used to create quantities from raw values. Please note that the type
// of the raw value is used for the value type of the quantity.
// Example:
// ```cpp
// auto v1 = 23.0 * square_meter;  // type of v1 is distance_squared_t
// auto v2 = 5 * meter_per_second; // type of v1 is boost::units::quantity<boost::units::si::velocity, int>
// ```
inline const auto meter = boost::units::si::meter;
inline const auto meter_per_second = boost::units::si::meter_per_second;
inline const auto meter_per_second_squared = boost::units::si::meter_per_second_squared;
inline const auto second = boost::units::si::second;
inline const auto square_meter = boost::units::si::square_meter;

}  // namespace path_tracking_pid::units
