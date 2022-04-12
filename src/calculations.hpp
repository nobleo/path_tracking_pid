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

/**
 * Returns the square distance between two poses.
 * 
 * @param[in] a Pose A.
 * @param[in] b Pose B.
 * @return Distance between A and B.
 */
double distSquared(const tf2::Transform & a, const tf2::Transform & b);

/**
 * @brief Closest pose between a line segment and a point
 *
 * Calculate the closest pose between the line segment bounded by `segment_start` and `segment_end`
 * and point `point`.
 *
 * @param[in] point               The point
 * @param[in] segment_start       Start of the line segment
 * @param[in] segment_end         End of the line segment
 * @param[in] estimate_pose_angle Indicates if the pose angle should be estimated from the line
 *                                segment (true) or if the pose angle from segment_start should be
 *                                used.
 * @return The pose projection of the closest point.
 */
tf2::Transform closestPoseOnSegment(
  const tf2::Transform & point, const tf2::Transform & segment_start,
  const tf2::Transform & segment_end, bool estimate_pose_angle);

/**
 * Determine the control point pose based on the given pose and control distance.
 *
 * @param[in] pose Pose to transform.
 * @param[in] control_distance Control distance to use.
 * @return Control point pose.
 */
tf2::Transform getControlPointPose(const tf2::Transform & pose, double control_distance);

}  // namespace path_tracking_pid
