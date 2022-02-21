#pragma once

#include <boost/units/systems/si.hpp>
#include <type_traits>

namespace path_tracking_pid
{

inline constexpr auto VELOCITY_EPS = 1e-3 * boost::units::si::meter_per_second;  // Neglegible velocity

// Converts an enumeration to its underlying type.
template <typename enum_type>
constexpr std::underlying_type_t<enum_type> to_underlying(enum_type value) noexcept
{
  return static_cast<std::underlying_type_t<enum_type>>(value);
}

}  // namespace path_tracking_pid
