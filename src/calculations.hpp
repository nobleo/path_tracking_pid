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

/**
 * Determine the inverse turning radiuses for each pose of a plan, based on the given deltas of that
 * plan. The 2D radius is calculated; the z component is ignored.
 * 
 * @param[in] deltas Deltas between consecutive poses of a plan. (Result of deltas_of_plan().)
 * @return Inverse turning radiuses.
 */
std::vector<double> inverse_turning_radiuses(const std::vector<tf2::Transform> & deltas);

/**
 * Determine if the given current pose is in the direction of the given target position taking the
 * given velocity into account.
 *
 * @param[in] current Current pose.
 * @param[in] target Target position.
 * @param[in] velocity Forward velocity in m/s.
 * @return True if it is in the direction of the target, false otherwise.
 */
bool is_in_direction_of_target(
  const tf2::Transform & current, const tf2::Vector3 & target, double velocity);

}  // namespace path_tracking_pid
