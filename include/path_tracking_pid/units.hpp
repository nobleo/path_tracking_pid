#pragma once

#include <ros/duration.h>

#include <chrono>
#include <cmath>

namespace path_tracking_pid::units
{

namespace tags
{

struct SpeedTag;
struct AccelerationTag;
struct DistanceTag;
struct AreaTag;
struct DurationTag;

}  // namespace tags

template <typename value_type, typename tag_type>
struct Unit
{
  explicit constexpr Unit(value_type value) : value_{value} {}

  template <
    typename T = tag_type, typename = std::enable_if_t<std::is_same_v<T, tags::DurationTag>>>
  constexpr Unit(std::chrono::duration<double> value) : value_{value.count()}
  {
  }

  template <
    typename T = tag_type, typename = std::enable_if_t<std::is_same_v<T, tags::DurationTag>>>
  Unit(ros::Duration value) : value_{value.toSec()}
  {
  }

  template <
    typename T = tag_type, typename = std::enable_if_t<std::is_same_v<T, tags::DurationTag>>>
  operator std::chrono::duration<double>() const
  {
    return std::chrono::duration<double>{value_};
  }

  template <
    typename T = tag_type, typename = std::enable_if_t<std::is_same_v<T, tags::DurationTag>>>
  operator ros::Duration() const
  {
    return ros::Duration{value_};
  }

  constexpr value_type value() const { return value_; }

private:
  value_type value_;
};

using Speed = Unit<double, tags::SpeedTag>;                // m/s
using Acceleration = Unit<double, tags::AccelerationTag>;  // m/(s*s)
using Distance = Unit<double, tags::DistanceTag>;          // m
using Area = Unit<double, tags::AreaTag>;                  // m*m
using Duration = Unit<double, tags::DurationTag>;          // s

namespace literals
{

inline constexpr Speed operator"" _meters_per_second(long double value)
{
  return Speed{static_cast<double>(value)};
}

inline constexpr Acceleration operator"" _meters_per_second_squared(long double value)
{
  return Acceleration{static_cast<double>(value)};
}

inline constexpr Distance operator"" _meters(long double value)
{
  return Distance{static_cast<double>(value)};
}

inline constexpr Area operator"" _meters_squared(long double value)
{
  return Area{static_cast<double>(value)};
}

inline constexpr Duration operator"" _seconds(long double value)
{
  return Duration{static_cast<double>(value)};
}

}  // namespace literals

template <typename V, typename T>
inline constexpr Unit<V, T> operator+(Unit<V, T> lhs, Unit<V, T> rhs)
{
  return Unit<V, T>{lhs.value() + rhs.value()};
}

template <typename V, typename T>
inline constexpr Unit<V, T> operator-(Unit<V, T> lhs, Unit<V, T> rhs)
{
  return Unit<V, T>{lhs.value() - rhs.value()};
}

template <typename V, typename T>
inline constexpr Unit<V, T> operator+=(Unit<V, T> & lhs, Unit<V, T> rhs)
{
  lhs = Unit<V, T>{lhs.value() + rhs.value()};
  return lhs;
}

template <typename V, typename T>
inline constexpr Unit<V, T> operator-(Unit<V, T> u)
{
  return Unit<V, T>{-u.value()};
}

template <typename V, typename T>
inline constexpr bool operator==(Unit<V, T> lhs, Unit<V, T> rhs)
{
  return lhs.value() == rhs.value();
}

template <typename V, typename T>
inline constexpr bool operator!=(Unit<V, T> lhs, Unit<V, T> rhs)
{
  return lhs.value() != rhs.value();
}

template <typename V, typename T>
inline constexpr bool operator<(Unit<V, T> lhs, Unit<V, T> rhs)
{
  return lhs.value() < rhs.value();
}

template <typename V, typename T>
inline constexpr bool operator<=(Unit<V, T> lhs, Unit<V, T> rhs)
{
  return lhs.value() <= rhs.value();
}

template <typename V, typename T>
inline constexpr bool operator>(Unit<V, T> lhs, Unit<V, T> rhs)
{
  return lhs.value() > rhs.value();
}

template <typename V, typename T>
inline constexpr bool operator>=(Unit<V, T> lhs, Unit<V, T> rhs)
{
  return lhs.value() >= rhs.value();
}

template <typename V, typename T>
inline constexpr V operator/(Unit<V, T> lhs, Unit<V, T> rhs)
{
  return lhs.value() / rhs.value();
}

template <
  typename V, typename T, typename V2, typename = std::enable_if_t<std::is_arithmetic_v<V2>>>
inline constexpr Unit<V, T> operator/(Unit<V, T> lhs, V2 rhs)
{
  return Unit<V, T>{lhs.value() / rhs};
}

template <
  typename V, typename T, typename V2, typename = std::enable_if_t<std::is_arithmetic_v<V2>>>
inline constexpr Unit<V, T> operator*(V2 lhs, Unit<V, T> rhs)
{
  return Unit<V, T>{lhs * rhs.value()};
}

template <
  typename V, typename T, typename V2, typename = std::enable_if_t<std::is_arithmetic_v<V2>>>
inline constexpr Unit<V, T> operator*(Unit<V, T> lhs, V2 rhs)
{
  return Unit<V, T>{lhs.value() * rhs};
}

template <typename V, typename T>
inline constexpr Unit<V, T> abs(Unit<V, T> u)
{
  return Unit<V, T>{std::abs(u.value())};
}

template <typename V, typename T>
inline constexpr Unit<V, T> fmin(Unit<V, T> lhs, Unit<V, T> rhs)
{
  return Unit<V, T>{std::fmin(lhs.value(), rhs.value())};
}

template <typename V, typename T>
inline constexpr Unit<V, T> fmax(Unit<V, T> lhs, Unit<V, T> rhs)
{
  return Unit<V, T>{std::fmax(lhs.value(), rhs.value())};
}

// TODO LDN could probably be made more general
template <typename V, typename T, typename V2, typename T2>
inline constexpr Unit<V, T> copysign(Unit<V, T> lhs, Unit<V2, T2> rhs)
{
  return Unit<V, T>{std::copysign(lhs.value(), rhs.value())};
}

inline Distance sqrt(const Area & d) { return Distance{std::sqrt(d.value())}; }

inline constexpr Duration operator/(Speed lhs, Acceleration rhs)
{
  return Duration{lhs.value() / rhs.value()};
}

inline constexpr Acceleration operator/(Speed lhs, Duration rhs)
{
  return Acceleration{lhs.value() / rhs.value()};
}

inline constexpr Speed operator*(Acceleration lhs, Duration rhs)
{
  return Speed{lhs.value() * rhs.value()};
}

inline constexpr Speed operator*(Duration lhs, Acceleration rhs)
{
  return Speed{lhs.value() * rhs.value()};
}

inline constexpr Duration operator/(Distance lhs, Speed rhs)
{
  return Duration{lhs.value() / rhs.value()};
}

inline constexpr Distance operator*(Speed lhs, Duration rhs)
{
  return Distance{lhs.value() * rhs.value()};
}

inline constexpr Distance operator*(Duration lhs, Speed rhs)
{
  return Distance{lhs.value() * rhs.value()};
}

}  // namespace path_tracking_pid::units
