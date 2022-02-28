#include "calculations.hpp"

#include <algorithm>
#include <vector>

namespace path_tracking_pid
{

std::vector<tf2::Transform> deltas_of_plan(const std::vector<tf2::Transform> & input)
{
  auto result = std::vector<tf2::Transform>{};

  if (input.size() < 2) {
    return result;
  }

  result.reserve(input.size() - 1);
  std::transform(
    input.cbegin(), input.cend() - 1, input.cbegin() + 1, std::back_inserter(result),
    [](const tf2::Transform & a, const tf2::Transform & b) { return a.inverseTimes(b); });

  return result;
}

}  // namespace path_tracking_pid
