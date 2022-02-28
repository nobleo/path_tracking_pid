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

/**
 * Determine the distances to the goal for each pose of a plan, based on the given deltas of that
 * plan. The 2D distance is calculated; the z component is ignored.
 *
 * @param[in] deltas Deltas between consecutive poses of a plan. (Result of deltas_of_plan().)
 * @return Distances to the goal.
 */
std::vector<double> distances_to_goal(const std::vector<tf2::Transform> & deltas);

}  // namespace path_tracking_pid
