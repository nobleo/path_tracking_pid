#pragma once

#include <path_tracking_pid/units.hpp>
#include <type_traits>

namespace path_tracking_pid
{

inline constexpr auto VELOCITY_EPS = units::Speed{1e-3};  // Neglegible velocity

// Converts an enumeration to its underlying type.
template <typename enum_type>
constexpr std::underlying_type_t<enum_type> to_underlying(enum_type value) noexcept
{
  return static_cast<std::underlying_type_t<enum_type>>(value);
}

}  // namespace path_tracking_pid
