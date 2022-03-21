#include "calculations.hpp"

#include <tf2/utils.h>

#include <algorithm>
#include <limits>
#include <numeric>
#include <vector>

namespace path_tracking_pid
{
std::vector<tf2::Transform> deltas_of_plan(const std::vector<tf2::Transform> & plan)
{
  auto result = std::vector<tf2::Transform>{};

  if (plan.size() < 2) {
    return result;
  }

  result.reserve(plan.size() - 1);
  std::transform(
    plan.cbegin(), plan.cend() - 1, plan.cbegin() + 1, std::back_inserter(result),
    [](const tf2::Transform & a, const tf2::Transform & b) { return a.inverseTimes(b); });

  return result;
}

std::vector<double> distances_to_goal(const std::vector<tf2::Transform> & deltas)
{
  auto result = std::vector<double>{};

  result.reserve(deltas.size() + 1);
  std::transform(
    deltas.cbegin(), deltas.cend(), std::back_inserter(result), [](const tf2::Transform & d) {
      const auto & origin = d.getOrigin();
      return std::hypot(origin.x(), origin.y());
    });
  result.push_back(0.0);

  std::partial_sum(result.crbegin(), result.crend(), result.rbegin());

  return result;
}

std::vector<double> inverse_turning_radiuses(const std::vector<tf2::Transform> & deltas)
{
  auto result = std::vector<double>{};

  result.reserve(deltas.size() + 1);
  std::transform(
    deltas.cbegin(), deltas.cend(), std::back_inserter(result), [](const tf2::Transform & d) {
      const auto & origin = d.getOrigin();
      const auto dpX = origin.x();
      const auto dpY = origin.y();
      const auto dpXY2 = std::pow(dpX, 2) + std::pow(dpY, 2);
      if (dpXY2 < FLT_EPSILON) {
        return std::numeric_limits<double>::infinity();
      }
      return (2 * dpY) / dpXY2;
    });
  result.push_back(0.0);

  return result;
}

bool is_in_direction_of_target(
  const tf2::Transform & current, const tf2::Vector3 & target, double velocity)
{
  const auto delta = target - current.getOrigin();
  const auto projection = current.getBasis().tdotx(delta);

  return !std::signbit(projection * velocity);
}

double distSquared(const tf2::Transform & a, const tf2::Transform & b)
{
  return a.getOrigin().distance2(b.getOrigin());
}

tf2::Transform closestPoseOnSegment(
  const tf2::Transform & point, const tf2::Transform & segment_start,
  const tf2::Transform & segment_end, bool estimate_pose_angle)
{
  const double l2 = distSquared(segment_start, segment_end);
  if (l2 == 0) {
    return segment_end;
  }

  tf2::Transform result;

  const double t = std::clamp(
    ((point.getOrigin().x() - segment_start.getOrigin().x()) *
       (segment_end.getOrigin().x() - segment_start.getOrigin().x()) +
     (point.getOrigin().y() - segment_start.getOrigin().y()) *
       (segment_end.getOrigin().y() - segment_start.getOrigin().y())) /
      l2,
    0.0, 1.0);
  result.setOrigin(tf2::Vector3(
    segment_start.getOrigin().x() +
      t * (segment_end.getOrigin().x() - segment_start.getOrigin().x()),
    segment_start.getOrigin().y() +
      t * (segment_end.getOrigin().y() - segment_start.getOrigin().y()),
    0.0));

  const auto yaw = estimate_pose_angle
                     ? atan2(
                         segment_end.getOrigin().y() - segment_start.getOrigin().y(),
                         segment_end.getOrigin().x() - segment_start.getOrigin().x())
                     : tf2::getYaw(segment_start.getRotation());
  tf2::Quaternion pose_quaternion;
  pose_quaternion.setRPY(0.0, 0.0, yaw);
  result.setRotation(pose_quaternion);

  return result;
}

tf2::Transform getControlPointPose(const tf2::Transform & pose, double control_distance)
{
  return tf2::Transform{pose.getBasis(), pose * tf2::Vector3{control_distance, 0, 0}};
}

}  // namespace path_tracking_pid
