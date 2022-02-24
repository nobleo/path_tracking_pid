#pragma once

#include <tf2/LinearMath/Transform.h>

#include <vector>

namespace path_tracking_pid
{

/**
 * Determine the deltas between consecutive poses in the given plan. Each delta is the inverse of a
 * pose times the next pose. If the plan contains fewer than 2 poses, the output is empty.
 *
 * @param[in] plan Plan to process.
 * @return Deltas between consecutive poses.
 */
std::vector<tf2::Transform> deltas_of_plan(const std::vector<tf2::Transform> & plan);

}  // namespace path_tracking_pid
