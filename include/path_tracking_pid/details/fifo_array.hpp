#pragma once

#include <algorithm>
#include <array>

namespace path_tracking_pid::details
{
// Fixed-size array-like FIFO buffer. All elements are value initialized upon construction.
template <typename value_type, std::size_t size>
class FifoArray
{
public:
  // Pushes the given value to the front of the buffer (index = 0). All elements already in the buffer are moved
  // towards the end of the buffer (index = size - 1). The last element is removed from the buffer.
  constexpr void push(const value_type & value)
  {
    std::copy_backward(data_.begin(), std::prev(data_.end()), data_.end());
    data_[0] = value;
  }

  // Value initializes all elements in the buffer.
  constexpr void reset() { data_ = {}; }

  // Read-only access to the element at the given index.
  constexpr const value_type & operator[](std::size_t index) const { return data_[index]; }

  // Read-write access to the element at the given index.
  value_type & operator[](std::size_t index) { return data_[index]; }

  // Read-only access to the element at the given index (with compile-time range check).
  template <std::size_t index>
  constexpr const value_type & at() const
  {
    static_assert(index < size);
    return data_[index];
  }

private:
  std::array<value_type, size> data_{};
};

}  // namespace path_tracking_pid::details
