#pragma once

#include <tf2/convert.h>

#include <type_traits>

namespace path_tracking_pid
{
inline constexpr double VELOCITY_EPS = 1e-3;  // Neglegible velocity

// Converts an enumeration to its underlying type.
template <typename enum_type>
constexpr std::underlying_type_t<enum_type> to_underlying(enum_type value) noexcept
{
  return static_cast<std::underlying_type_t<enum_type>>(value);
}

/**
 * Converts input (of input_type) to result_type. Like tf2::fromMsg() but using a return value
 * instead of an output parameter.
 * 
 * @tparam result_type Resulting type of the conversion. Should be a tf2 type.
 * @tparam input_type Input type for the conversion. Should be a message type.
 * @param[in] input Input object to convert.
 * @return Converted object.
 */
template <
  typename result_type, typename input_type,
  typename = std::enable_if_t<!std::is_same_v<result_type, input_type>>>
result_type tf2_convert(const input_type & input)
{
  result_type result;
  tf2::fromMsg(input, result);
  return result;
}

}  // namespace path_tracking_pid
