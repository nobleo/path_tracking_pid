//
// Created by nobleo on 11-9-18.
//

#include <angles/angles.h>
#include <tf2/utils.h>

#include <algorithm>
#include <limits>
#include <path_tracking_pid/controller.hpp>
#include <vector>

#include "common.hpp"

namespace path_tracking_pid
{
namespace
{
constexpr double RADIUS_EPS = 0.001;        // Smallest relevant radius [m]
constexpr double LONG_DURATION = 31556926;  // A year (ros::Duration cannot be inf)

// Upper and lower saturation limits
constexpr double lat_upper_limit = 100.0;
constexpr double lat_lower_limit = -100.0;

constexpr double ang_upper_limit = 100.0;
constexpr double ang_lower_limit = -100.0;

// Anti-windup term. Limits the absolute value of the integral term.
constexpr double windup_limit = 1000.0;

// Indicates if both values have the same sign.
template <typename T>
bool have_same_sign(T val1, T val2)
{
  return std::signbit(val1) == std::signbit(val2);
}

// Floating-point aware version of std::clamp(). fclamp() is to std::clamp() what std::fmin() and
// std::fmax() are to std::min() and std::max().
template <typename T>
T fclamp(T value, T low, T high)
{
  return std::fmax(low, std::fmin(value, high));
}

// Converts a pose to the corresponding transform.
tf2::Transform to_transform(const geometry_msgs::Pose & pose)
{
  tf2::Transform result;
  tf2::fromMsg(pose, result);
  return result;
}

// Returns the square distance between two points
double distSquared(const tf2::Transform & a, const tf2::Transform & b)
{
  return a.getOrigin().distance2(b.getOrigin());
}

// Return the square distance between two points.
double distSquared(const geometry_msgs::Pose & a, const geometry_msgs::Pose & b)
{
  return std::pow(a.position.x - b.position.x, 2) + std::pow(a.position.y - b.position.y, 2);
}

// Indicates if the angle of the cur pose is obtuse (with respect to the prev and next poses).
bool is_pose_angle_obtuse(
  const geometry_msgs::Pose & prev, const geometry_msgs::Pose & cur,
  const geometry_msgs::Pose & next)
{
  return distSquared(prev, next) > (distSquared(prev, cur) + distSquared(cur, next));
}

// Filters (and converts) the given plan. The first and last poses are always accepted (if they
// exist). Intermediate poses are only accepted if the angle (with respect to the previous and next
// poses) is obtuse. All accepted poses are converted to transforms.
std::vector<tf2::Transform> filter_plan(const std::vector<geometry_msgs::PoseStamped> & plan)
{
  const auto plan_size = plan.size();
  auto result = std::vector<tf2::Transform>{};

  if (plan_size > 0) {
    result.push_back(to_transform(plan.front().pose));
  }

  for (int pose_idx = 1; pose_idx < static_cast<int>(plan_size) - 1; ++pose_idx) {
    const auto prev_pose = plan[pose_idx - 1].pose;
    const auto pose = plan[pose_idx].pose;
    const auto next_pose = plan[pose_idx + 1].pose;
    if (is_pose_angle_obtuse(prev_pose, pose, next_pose)) {
      result.push_back(to_transform(pose));
    }
  }

  if (plan_size > 1) {
    result.push_back(to_transform(plan.back().pose));
  }

  return result;
}

}  // namespace

void Controller::setHolonomic(bool holonomic)
{
  // Set configuration parameters
  ROS_WARN_COND(holonomic, "Holonomic mode is unmaintained. Expect bugs with y-direction");
  holonomic_robot_enable_ = holonomic;
}

void Controller::setEstimatePoseAngle(bool estimate_pose_angle)
{
  // Set configuration parameters
  estimate_pose_angle_enabled_ = estimate_pose_angle;
}

void Controller::setTricycleModel(
  bool tricycle_model_enabled, const geometry_msgs::Transform & tf_base_to_steered_wheel)
{
  // Set tricycle model
  use_tricycle_model_ = tricycle_model_enabled;
  tf_base_to_steered_wheel_ = tf_base_to_steered_wheel;
  const double wheel_x = tf_base_to_steered_wheel_.translation.x;
  const double wheel_y = tf_base_to_steered_wheel_.translation.y;

  const double distance_base_to_steered_wheel = hypot(wheel_x, wheel_y);
  const double wheel_theta = atan2(wheel_y, wheel_x);
  inverse_kinematics_matrix_[0][0] = 1;
  inverse_kinematics_matrix_[0][1] = -distance_base_to_steered_wheel * sin(wheel_theta);
  inverse_kinematics_matrix_[1][0] = 0;
  inverse_kinematics_matrix_[1][1] = -distance_base_to_steered_wheel * cos(wheel_theta);

  const double determinant = inverse_kinematics_matrix_[0][0] * inverse_kinematics_matrix_[1][1] -
                             inverse_kinematics_matrix_[0][1] * inverse_kinematics_matrix_[1][0];

  if (determinant == 0) {
    ROS_ERROR("Steered wheel at base_link. Invalid for tricycle model, it will be disabled.");
    use_tricycle_model_ = false;
    return;
  }

  forward_kinematics_matrix_[0][0] = inverse_kinematics_matrix_[1][1] / determinant;
  forward_kinematics_matrix_[0][1] = -inverse_kinematics_matrix_[0][1] / determinant;
  forward_kinematics_matrix_[1][0] = -inverse_kinematics_matrix_[1][0] / determinant;
  forward_kinematics_matrix_[1][1] = inverse_kinematics_matrix_[0][0] / determinant;

  controller_state_.previous_steering_angle = tf2::getYaw(tf_base_to_steered_wheel_.rotation);
}

geometry_msgs::Twist Controller::computeTricycleModelForwardKinematics(
  double x_vel, double steering_angle)
{
  geometry_msgs::Twist estimated_base_twist;
  const double x_alpha = x_vel * cos(steering_angle);
  const double y_alpha = x_vel * sin(steering_angle);

  estimated_base_twist.linear.x =
    forward_kinematics_matrix_[0][0] * x_alpha + forward_kinematics_matrix_[0][1] * y_alpha;
  estimated_base_twist.angular.z =
    forward_kinematics_matrix_[1][0] * x_alpha + forward_kinematics_matrix_[1][1] * y_alpha;

  return estimated_base_twist;
}

TricycleSteeringCmdVel Controller::computeTricycleModelInverseKinematics(
  const geometry_msgs::Twist & cmd_vel)
{
  TricycleSteeringCmdVel steering_cmd_vel;
  const double x_alpha = inverse_kinematics_matrix_[0][0] * cmd_vel.linear.x +
                         inverse_kinematics_matrix_[0][1] * cmd_vel.angular.z;
  const double y_alpha = inverse_kinematics_matrix_[1][0] * cmd_vel.linear.x +
                         inverse_kinematics_matrix_[1][1] * cmd_vel.angular.z;

  steering_cmd_vel.steering_angle = atan2(y_alpha, x_alpha);
  steering_cmd_vel.speed = hypot(x_alpha, y_alpha);

  return steering_cmd_vel;
}

void Controller::setPlan(
  const geometry_msgs::Transform & current_tf, const geometry_msgs::Twist & odom_twist,
  const std::vector<geometry_msgs::PoseStamped> & global_plan)
{
  ROS_DEBUG("TrackingPidLocalPlanner::setPlan(%d)", (int)global_plan.size());
  ROS_DEBUG("Plan is defined in frame '%s'", global_plan.at(0).header.frame_id.c_str());

  tf2::Transform current_tf2;
  tf2::convert(current_tf, current_tf2);

  global_plan_tf_ = filter_plan(global_plan);
  if (global_plan_tf_.size() != global_plan.size()) {
    ROS_WARN(
      "Not all poses of path are used since not all poses were in the expected direction of the "
      "path!");
  }

  if (!config_.track_base_link) {
    // Add carrot length to plan using goal pose (we assume the last pose contains correct angle)
    tf2::Transform carrotTF(
      tf2::Matrix3x3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
      tf2::Vector3(config_.l, 0.0, 0.0));
    global_plan_tf_.push_back(global_plan_tf_.back() * carrotTF);
  }

  // Whenever a new path is recieved, computed the closest pose to
  // the current carrot pose
  controller_state_.current_global_plan_index = 0;

  // find closest current position to global plan
  double minimum_distance_to_path = 1e3;
  double dist_to_segment;
  double iterative_dist_to_goal = 0.0;
  distance_to_goal_vector_.clear();
  distance_to_goal_vector_.resize(global_plan_tf_.size());
  distance_to_goal_vector_[global_plan_tf_.size() - 1] = 0.0;
  turning_radius_inv_vector_.clear();
  turning_radius_inv_vector_.resize(global_plan_tf_.size());
  turning_radius_inv_vector_[global_plan_tf_.size() - 1] = 0.0;
  tf2::Transform deltaPlan;
  // We define segment0 to be the segment connecting pose0 and pose1.
  // Hence, when picking the starting path's pose, we mean to start at the segment connecting that and the next pose.
  for (int idx_path = static_cast<int>(global_plan_tf_.size() - 2); idx_path >= 0; --idx_path) {
    /* Get distance to segment to determine if this is the segment to start at */
    dist_to_segment =
      distToSegmentSquared(current_tf2, global_plan_tf_[idx_path], global_plan_tf_[idx_path + 1])
        .distance2_to_p;
    // Calculate 3D distance, since current_tf2 might have significant z-offset and roll/pitch values w.r.t. path-pose
    // When not doing this, we're brutely projecting in robot's frame and might snap to another segment!
    if (dist_to_segment < minimum_distance_to_path) {
      minimum_distance_to_path = dist_to_segment;
      controller_state_.current_global_plan_index = idx_path;
    }

    /* Create distance and turning radius vectors once for usage later */
    deltaPlan = global_plan_tf_[idx_path].inverseTimes(global_plan_tf_[idx_path + 1]);
    const double dpX = deltaPlan.getOrigin().x();
    const double dpY = deltaPlan.getOrigin().y();
    iterative_dist_to_goal += hypot(dpX, dpY);
    distance_to_goal_vector_[idx_path] = iterative_dist_to_goal;
    // compute turning radius based on trigonometric analysis
    // radius such that next pose is connected from current pose with a semi-circle
    const double dpXY2 = dpY * dpY + dpX * dpX;
    if (dpXY2 < FLT_EPSILON) {
      turning_radius_inv_vector_[idx_path] = std::numeric_limits<double>::infinity();
    } else {
      //  0.5*dpY*( 1 + dpX*dpX/(dpY*dPY) );
      // turning_radius_vector[idx_path] = 0.5*(1/dpY)*( dpY*dpY + dpX*dpX );
      turning_radius_inv_vector_[idx_path] = 2 * dpY / dpXY2;
    }
    ROS_DEBUG("turning_radius_inv_vector[%d] = %f", idx_path, turning_radius_inv_vector_[idx_path]);
  }

  // Set initial velocity
  switch (config_.init_vel_method) {
    case Pid_Zero:
      reset();
      break;
    case Pid_Odom:
      reset();
      controller_state_.current_x_vel = odom_twist.linear.x;
      controller_state_.current_yaw_vel = odom_twist.angular.z;
      ROS_INFO(
        "Resuming on odom velocity x: %f, yaw: %f", odom_twist.linear.x, odom_twist.angular.z);
      break;
    default:
      ROS_DEBUG("Internal controller_state stays valid");
      break;
  }

  // When velocity error is too big reset current_x_vel
  if (fabs(odom_twist.linear.x - controller_state_.current_x_vel) > config_.max_error_x_vel) {
    // TODO(clopez/mcfurry/nobleo): Give feedback to higher level software here
    ROS_WARN(
      "Large control error. Current_x_vel %f / odometry %f", controller_state_.current_x_vel,
      odom_twist.linear.x);
  }
  controller_state_.end_phase_enabled = false;
  controller_state_.end_reached = false;
}

void Controller::setPlan(
  const geometry_msgs::Transform & current_tf, const geometry_msgs::Twist & odom_twist,
  const geometry_msgs::Transform & tf_base_to_steered_wheel,
  const geometry_msgs::Twist & /* steering_odom_twist */,
  const std::vector<geometry_msgs::PoseStamped> & global_plan)
{
  setPlan(current_tf, odom_twist, global_plan);
  controller_state_.previous_steering_angle = tf2::getYaw(tf_base_to_steered_wheel.rotation);
  // TODO(clopez) use steering_odom_twist to check if setpoint is being followed
}

Controller::DistToSegmentSquaredResult Controller::distToSegmentSquared(
  const tf2::Transform & pose_p, const tf2::Transform & pose_v, const tf2::Transform & pose_w) const
{
  const double l2 = distSquared(pose_v, pose_w);
  if (l2 == 0) {
    DistToSegmentSquaredResult result;

    result.pose_projection = pose_w;
    result.distance_to_w = 0.0;
    result.distance2_to_p = distSquared(pose_p, pose_w);

    return result;
  }

  DistToSegmentSquaredResult result;

  const double t = fclamp(
    ((pose_p.getOrigin().x() - pose_v.getOrigin().x()) *
       (pose_w.getOrigin().x() - pose_v.getOrigin().x()) +
     (pose_p.getOrigin().y() - pose_v.getOrigin().y()) *
       (pose_w.getOrigin().y() - pose_v.getOrigin().y())) /
      l2,
    0.0, 1.0);
  result.pose_projection.setOrigin(tf2::Vector3(
    pose_v.getOrigin().x() + t * (pose_w.getOrigin().x() - pose_v.getOrigin().x()),
    pose_v.getOrigin().y() + t * (pose_w.getOrigin().y() - pose_v.getOrigin().y()), 0.0));

  const auto yaw = estimate_pose_angle_enabled_ ? atan2(
                                                    pose_w.getOrigin().y() - pose_v.getOrigin().y(),
                                                    pose_w.getOrigin().x() - pose_v.getOrigin().x())
                                                : tf2::getYaw(pose_v.getRotation());
  tf2::Quaternion pose_quaternion;
  pose_quaternion.setRPY(0.0, 0.0, yaw);
  result.pose_projection.setRotation(pose_quaternion);

  result.distance_to_w = sqrt(distSquared(result.pose_projection, pose_w));
  result.distance2_to_p = distSquared(pose_p, result.pose_projection);

  return result;
}

tf2::Transform Controller::findPositionOnPlan(
  const geometry_msgs::Transform & current_tf, ControllerState * controller_state_ptr,
  size_t & path_pose_idx)
{
  tf2::Transform current_tf2;
  tf2::convert(current_tf, current_tf2);
  // 'Project' current_tf by removing z-component
  tf2::Vector3 originProj = current_tf2.getOrigin();
  originProj.setZ(0.0);
  current_tf2.setOrigin(originProj);

  // Computed the closest pose to the current provided pose
  // by looking on the surroundings of the last known pose

  // find closest current position to global plan
  double minimum_distance_to_path = FLT_MAX;
  double distance_to_path;
  tf2::Transform error;

  // We define segment0 to be the segment connecting pose0 and pose1.
  // Hence, when idx_path==i we are currently tracking the section connection pose i and pose i+1

  // First look in current position and in front
  for (auto idx_path = controller_state_ptr->current_global_plan_index;
       idx_path < global_plan_tf_.size(); idx_path++) {
    error = current_tf2.inverseTimes(global_plan_tf_[idx_path]);
    // Calculate 3D distance, since current_tf2 might have significant z-offset and roll/pitch values w.r.t. path-pose
    // When not doing this, we're brutely projecting in robot's frame and might snap to another segment!
    distance_to_path = hypot(error.getOrigin().x(), error.getOrigin().y(), error.getOrigin().z());

    if (distance_to_path <= minimum_distance_to_path) {
      minimum_distance_to_path = distance_to_path;
      controller_state_ptr->current_global_plan_index = idx_path;
    } else {
      break;
    }
  }

  // Then look backwards
  for (auto idx_path = controller_state_ptr->current_global_plan_index; idx_path > 0; --idx_path) {
    error = current_tf2.inverseTimes(global_plan_tf_[idx_path - 1]);
    // Calculate 3D distance, since current_tf2 might have significant z-offset and roll/pitch values w.r.t. path-pose
    // When not doing this, we're brutely projecting in robot's frame and might snap to another segment!
    distance_to_path = hypot(error.getOrigin().x(), error.getOrigin().y(), error.getOrigin().z());

    if (distance_to_path < minimum_distance_to_path) {
      minimum_distance_to_path = distance_to_path;
      controller_state_ptr->current_global_plan_index = idx_path - 1;
    } else {
      break;
    }
  }
  ROS_DEBUG(
    "progress: %lu of %lu", controller_state_ptr->current_global_plan_index,
    global_plan_tf_.size() - 1);
  // To finalize, compute the indexes of the start and end points of
  // the closest line segment to the current carrot
  tf2::Transform current_goal_local;
  current_goal_local = global_plan_tf_[controller_state_ptr->current_global_plan_index];

  if (controller_state_ptr->current_global_plan_index == 0) {
    const auto dist_result =
      distToSegmentSquared(current_tf2, global_plan_tf_[0], global_plan_tf_[1]);

    current_goal_local = dist_result.pose_projection;
    distance_to_goal_ = distance_to_goal_vector_[1] + dist_result.distance_to_w;
    controller_state_ptr->last_visited_pose_index = 0;
    path_pose_idx = controller_state_ptr->current_global_plan_index;
  } else if (controller_state_ptr->current_global_plan_index == global_plan_tf_.size() - 1) {
    const auto dist_result = distToSegmentSquared(
      current_tf2, global_plan_tf_[controller_state_ptr->current_global_plan_index - 1],
      global_plan_tf_[controller_state_ptr->current_global_plan_index]);

    current_goal_local = dist_result.pose_projection;
    distance_to_goal_ = dist_result.distance_to_w;
    controller_state_ptr->last_visited_pose_index = global_plan_tf_.size() - 2;
    path_pose_idx = controller_state_ptr->current_global_plan_index - 1;
  } else {
    const auto dist_result_ahead = distToSegmentSquared(
      current_tf2, global_plan_tf_[controller_state_ptr->current_global_plan_index],
      global_plan_tf_[controller_state_ptr->current_global_plan_index + 1]);
    const auto dist_result_behind = distToSegmentSquared(
      current_tf2, global_plan_tf_[controller_state_ptr->current_global_plan_index - 1],
      global_plan_tf_[controller_state_ptr->current_global_plan_index]);

    if (dist_result_ahead.distance2_to_p < dist_result_behind.distance2_to_p) {
      current_goal_local = dist_result_ahead.pose_projection;
      distance_to_goal_ =
        distance_to_goal_vector_[controller_state_ptr->current_global_plan_index + 1] +
        dist_result_ahead.distance_to_w;
      controller_state_ptr->last_visited_pose_index =
        controller_state_ptr->current_global_plan_index;
    } else {
      current_goal_local = dist_result_behind.pose_projection;
      distance_to_goal_ =
        distance_to_goal_vector_[controller_state_ptr->current_global_plan_index] +
        dist_result_behind.distance_to_w;
      controller_state_ptr->last_visited_pose_index =
        controller_state_ptr->current_global_plan_index - 1;
    }
    path_pose_idx = controller_state_ptr->current_global_plan_index;
  }
  return current_goal_local;
}

Controller::UpdateResult Controller::update(
  double target_x_vel, double target_end_x_vel, const geometry_msgs::Transform & current_tf,
  const geometry_msgs::Twist & odom_twist, ros::Duration dt)
{
  UpdateResult result;

  const double current_x_vel = controller_state_.current_x_vel;
  const double current_yaw_vel = controller_state_.current_yaw_vel;

  // Compute location of the point to be controlled
  const double theda_rp = tf2::getYaw(current_tf.rotation);
  tf2::Vector3 current_with_carrot_origin;
  current_with_carrot_origin.setX(current_tf.translation.x + config_.l * cos(theda_rp));
  current_with_carrot_origin.setY(current_tf.translation.y + config_.l * sin(theda_rp));
  current_with_carrot_origin.setZ(0);

  current_with_carrot_.setOrigin(current_with_carrot_origin);
  tf2::Quaternion cur_rot(
    current_tf.rotation.x, current_tf.rotation.y, current_tf.rotation.z, current_tf.rotation.w);
  current_with_carrot_.setRotation(cur_rot);

  size_t path_pose_idx;
  if (config_.track_base_link) {
    // Find closes robot position to path and then project carrot on goal
    current_pos_on_plan_ = current_goal_ =
      findPositionOnPlan(current_tf, &controller_state_, path_pose_idx);
    // To track the base link the goal is then transform to the control point goal
    double theda_rp = tf2::getYaw(current_goal_.getRotation());
    tf2::Vector3 newControlOrigin;
    newControlOrigin.setX(current_goal_.getOrigin().x() + config_.l * cos(theda_rp));
    newControlOrigin.setY(current_goal_.getOrigin().y() + config_.l * sin(theda_rp));
    newControlOrigin.setZ(0);
    current_goal_.setOrigin(newControlOrigin);
  } else {
    // find position of current position with projected carrot
    geometry_msgs::Transform current_with_carrot_g;
    tf2::convert(current_with_carrot_, current_with_carrot_g);
    current_pos_on_plan_ = current_goal_ =
      findPositionOnPlan(current_with_carrot_g, &controller_state_, path_pose_idx);
  }

  result.progress = 1.0 - distance_to_goal_ / distance_to_goal_vector_[0];

  // Compute errorPose between controlPose and currentGoalPose
  tf2::Transform error = current_with_carrot_.inverseTimes(current_goal_);

  //***** Feedback control *****//
  if (!((config_.Kp_lat <= 0. && config_.Ki_lat <= 0. && config_.Kd_lat <= 0.) ||
        (config_.Kp_lat >= 0. && config_.Ki_lat >= 0. &&
         config_.Kd_lat >= 0.)))  // All 3 gains should have the same sign
  {
    ROS_WARN("All three gains (Kp, Ki, Kd) should have the same sign for stability.");
  }
  if (!((config_.Kp_ang <= 0. && config_.Ki_ang <= 0. && config_.Kd_ang <= 0.) ||
        (config_.Kp_ang >= 0. && config_.Ki_ang >= 0. &&
         config_.Kd_ang >= 0.)))  // All 3 gains should have the same sign
  {
    ROS_WARN("All three gains (Kp, Ki, Kd) should have the same sign for stability.");
  }

  controller_state_.error_lat.push(error.getOrigin().y());
  controller_state_.error_ang.push(angles::normalize_angle(tf2::getYaw(error.getRotation())));

  // tracking error for diagnostic purposes
  // Transform current pose into local-path-frame to get tracked-frame-error
  tf2::Quaternion path_quat;
  path_quat.setEuler(
    0.0, 0.0,
    atan2(
      global_plan_tf_[path_pose_idx + 1].getOrigin().y() -
        global_plan_tf_[path_pose_idx].getOrigin().y(),
      global_plan_tf_[path_pose_idx + 1].getOrigin().x() -
        global_plan_tf_[path_pose_idx].getOrigin().x()));
  tf2::Transform path_segmen_tf = tf2::Transform(
    path_quat, tf2::Vector3(
                 global_plan_tf_[path_pose_idx].getOrigin().x(),
                 global_plan_tf_[path_pose_idx].getOrigin().y(),
                 global_plan_tf_[path_pose_idx].getOrigin().z()));

  tf2::Vector3 current_tracking_err =
    -(path_segmen_tf.inverse() *
      tf2::Vector3(current_tf.translation.x, current_tf.translation.y, current_tf.translation.z));

  // trackin_error here represents the error between tracked link and position on plan
  controller_state_.tracking_error_lat = current_tracking_err.y();
  controller_state_.tracking_error_ang = controller_state_.error_ang.errors().at<0>();

  // integrate the error
  controller_state_.error_integral_lat += controller_state_.error_lat.errors().at<0>() * dt.toSec();
  controller_state_.error_integral_ang += controller_state_.error_ang.errors().at<0>() * dt.toSec();

  // Apply windup limit to limit the size of the integral term
  controller_state_.error_integral_lat =
    std::clamp(controller_state_.error_integral_lat, -windup_limit, windup_limit);
  controller_state_.error_integral_ang =
    std::clamp(controller_state_.error_integral_ang, -windup_limit, windup_limit);

  // Take derivative of error, first the raw unfiltered data:
  controller_state_.error_deriv_lat.push(
    (controller_state_.error_lat.errors().at<0>() - controller_state_.error_lat.errors().at<1>()) /
    dt.toSec());
  controller_state_.error_deriv_ang.push(
    (controller_state_.error_ang.errors().at<0>() - controller_state_.error_ang.errors().at<1>()) /
    dt.toSec());

  // calculate the control effort
  const auto proportional_lat =
    config_.Kp_lat * controller_state_.error_lat.filtered_errors().at<0>();
  const auto integral_lat = config_.Ki_lat * controller_state_.error_integral_lat;
  const auto derivative_lat =
    config_.Kd_lat * controller_state_.error_deriv_lat.filtered_errors().at<0>();

  const auto proportional_ang =
    config_.Kp_ang * controller_state_.error_ang.filtered_errors().at<0>();
  const auto integral_ang = config_.Ki_ang * controller_state_.error_integral_ang;
  const auto derivative_ang =
    config_.Kd_ang * controller_state_.error_deriv_ang.filtered_errors().at<0>();

  /***** Compute forward velocity *****/
  // Apply acceleration limits and end velocity
  double t_end_phase_current;
  double d_end_phase;

  // Compute time to reach end velocity from current velocity
  // Compute estimate overall distance during end_phase
  // The estimates are done a bit conservative to account that robot will take longer
  // to de-accelerate and thus avoid abrupt velocity changes at the end of the trajectory
  // The sample time plays an important role on how good these estimates are.
  // Thus We add a distance to the end phase distance estimation depending on the sample time
  if (
    (current_target_x_vel_ > 0.0 && current_x_vel > target_end_x_vel) ||
    (current_target_x_vel_ < 0.0 && current_x_vel < target_end_x_vel)) {
    t_end_phase_current = (target_end_x_vel - current_x_vel) / (-config_.target_x_decc);
    d_end_phase = current_x_vel * t_end_phase_current -
                  0.5 * (config_.target_x_decc) * t_end_phase_current * t_end_phase_current +
                  target_x_vel * 2.0 * dt.toSec();
  } else {
    t_end_phase_current = (target_end_x_vel - current_x_vel) / (config_.target_x_acc);
    d_end_phase = current_x_vel * t_end_phase_current +
                  0.5 * (config_.target_x_acc) * t_end_phase_current * t_end_phase_current +
                  target_x_vel * 2.0 * dt.toSec();
  }
  ROS_DEBUG("t_end_phase_current: %f", t_end_phase_current);
  ROS_DEBUG("d_end_phase: %f", d_end_phase);
  ROS_DEBUG("distance_to_goal: %f", distance_to_goal_);

  // Get 'angle' towards current_goal
  tf2::Transform robot_pose;
  tf2::convert(current_tf, robot_pose);
  tf2::Transform base_to_goal = robot_pose.inverseTimes(current_goal_);
  const double angle_to_goal = atan2(base_to_goal.getOrigin().x(), -base_to_goal.getOrigin().y());

  // If we are as close to our goal or closer then we need to reach end velocity, enable end_phase.
  // However, if robot is not facing to the same direction as the local velocity target vector, don't enable end_phase.
  // This is to avoid skipping paths that start with opposite velocity.
  if ((distance_to_goal_ <= fabs(d_end_phase)) && have_same_sign(target_x_vel, angle_to_goal)) {
    // This state will be remebered to avoid jittering on target_x_vel
    controller_state_.end_phase_enabled = true;
  }

  if (controller_state_.end_phase_enabled && fabs(target_x_vel) > VELOCITY_EPS) {
    current_target_x_vel_ = target_end_x_vel;
  } else {
    controller_state_.end_phase_enabled = false;
    current_target_x_vel_ = target_x_vel;
  }

  // Determine if we need to accelerate, decelerate or maintain speed
  double current_target_acc = 0;                    // Assume maintaining speed
  if (fabs(current_target_x_vel_) <= VELOCITY_EPS)  // Zero velocity requested
  {
    if (current_x_vel > current_target_x_vel_) {
      current_target_acc = -config_.target_x_decc;
    } else {
      current_target_acc = config_.target_x_decc;
    }
  } else if (current_target_x_vel_ > 0)  // Positive velocity requested
  {
    if (current_x_vel > current_target_x_vel_) {
      current_target_acc = -config_.target_x_decc;
    } else {
      current_target_acc = config_.target_x_acc;
    }
  } else  // Negative velocity requested
  {
    if (current_x_vel > current_target_x_vel_) {
      current_target_acc = -config_.target_x_acc;
    } else {
      current_target_acc = config_.target_x_decc;
    }
  }

  const double acc_desired = (current_target_x_vel_ - current_x_vel) / dt.toSec();
  const double acc_abs = fmin(fabs(acc_desired), fabs(current_target_acc));
  const auto acc = copysign(acc_abs, current_target_acc);

  double new_x_vel = current_x_vel + acc * dt.toSec();

  // For low target_end_x_vel we have a minimum velocity to ensure the goal is reached
  double min_vel = copysign(1.0, config_.l) * config_.abs_minimum_x_vel;
  if (
    !controller_state_.end_reached && controller_state_.end_phase_enabled &&
    fabs(target_end_x_vel) <= fabs(min_vel) + VELOCITY_EPS &&
    fabs(new_x_vel) <= fabs(min_vel) + VELOCITY_EPS) {
    new_x_vel = min_vel;
  }

  // When velocity error is too big reset current_x_vel
  if (
    fabs(odom_twist.linear.x) < fabs(current_target_x_vel_) &&
    fabs(odom_twist.linear.x - new_x_vel) > config_.max_error_x_vel) {
    // TODO(clopez/mcfurry/nobleo): Give feedback to higher level software here
    ROS_WARN_THROTTLE(
      1.0, "Large tracking error. Current_x_vel %f / odometry %f", new_x_vel, odom_twist.linear.x);
  }

  // Force target_end_x_vel at the very end of the path
  // Or when the end velocity is reached.
  // Warning! If target_end_x_vel == 0 and min_vel = 0 then the robot might not reach end pose
  if (
    (distance_to_goal_ == 0.0 && target_end_x_vel >= VELOCITY_EPS) ||
    (controller_state_.end_phase_enabled && new_x_vel >= target_end_x_vel - VELOCITY_EPS &&
     new_x_vel <= target_end_x_vel + VELOCITY_EPS)) {
    controller_state_.end_reached = true;
    controller_state_.end_phase_enabled = false;
    result.progress = 1.0;
    result.eda = 0.0;
    enabled_ = false;
  } else {
    controller_state_.end_reached = false;
    // eda (Estimated duration of arrival) estimation
    if (fabs(target_x_vel) > VELOCITY_EPS) {
      const double t_const =
        (copysign(distance_to_goal_, target_x_vel) - d_end_phase) / target_x_vel;
      result.eda = fmin(fmax(t_end_phase_current, 0.0) + fmax(t_const, 0.0), LONG_DURATION);
    } else {
      result.eda = LONG_DURATION;
    }
  }
  /******* end calculation of forward velocity ********/

  //***** Overall control *****//
  // Controller logic && overall control effort
  control_effort_long_ = new_x_vel;
  control_effort_lat_ = 0.0;
  control_effort_ang_ = 0.0;

  if (config_.feedback_lat) {
    control_effort_lat_ = proportional_lat + integral_lat + derivative_lat;
  }
  if (config_.feedback_ang) {
    control_effort_ang_ = proportional_ang + integral_ang + derivative_ang;
  }

  //***** Feedforward control *****//
  if (config_.feedforward_lat) {
    feedforward_lat_ = 0.0;  // Not implemented
    control_effort_lat_ = control_effort_lat_ + feedforward_lat_;
  } else {
    feedforward_lat_ = 0.0;
  }

  if (config_.feedforward_ang) {
    feedforward_ang_ =
      turning_radius_inv_vector_[controller_state_.last_visited_pose_index] * control_effort_long_;
    ROS_DEBUG(
      "turning_radius_inv_vector[%lu] = %f", controller_state_.last_visited_pose_index,
      turning_radius_inv_vector_[controller_state_.last_visited_pose_index]);

    control_effort_ang_ = control_effort_ang_ + feedforward_ang_;
  } else {
    feedforward_ang_ = 0.0;
  }

  // Apply saturation limits
  control_effort_lat_ = std::clamp(control_effort_lat_, lat_lower_limit, lat_upper_limit);
  control_effort_ang_ = std::clamp(control_effort_ang_, ang_lower_limit, ang_upper_limit);

  // Populate debug output
  // Error topic containing the 'control' error on which the PID acts
  result.pid_debug.control_error.linear.x = 0.0;
  result.pid_debug.control_error.linear.y = controller_state_.error_lat.errors().at<0>();
  result.pid_debug.control_error.angular.z = controller_state_.error_ang.errors().at<0>();
  // Error topic containing the 'tracking' error, i.e. the real error between path and tracked link
  result.pid_debug.tracking_error.linear.x = 0.0;
  result.pid_debug.tracking_error.linear.y = controller_state_.tracking_error_lat;
  result.pid_debug.tracking_error.angular.z = controller_state_.tracking_error_ang;

  result.pid_debug.proportional.linear.x = 0.0;
  result.pid_debug.proportional.linear.y = proportional_lat;
  result.pid_debug.proportional.angular.z = proportional_ang;

  result.pid_debug.integral.linear.x = 0.0;
  result.pid_debug.integral.linear.y = integral_lat;
  result.pid_debug.integral.angular.z = integral_ang;

  result.pid_debug.derivative.linear.x = 0.0;
  result.pid_debug.derivative.linear.y = derivative_lat;
  result.pid_debug.derivative.angular.z = derivative_ang;

  result.pid_debug.feedforward.linear.x = new_x_vel;
  result.pid_debug.feedforward.linear.y = feedforward_lat_;
  result.pid_debug.feedforward.angular.z = feedforward_ang_;

  // Generate twist message
  if (holonomic_robot_enable_) {
    result.velocity_command.linear.x = control_effort_long_;
    result.velocity_command.linear.y = control_effort_lat_;
    result.velocity_command.linear.z = 0;
    result.velocity_command.angular.x = 0;
    result.velocity_command.angular.y = 0;
    result.velocity_command.angular.z = control_effort_ang_;
    result.velocity_command.angular.z =
      std::clamp(result.velocity_command.angular.z, -config_.max_yaw_vel, config_.max_yaw_vel);
  } else {
    result.velocity_command.linear.x = control_effort_long_;
    result.velocity_command.linear.y = 0;
    result.velocity_command.linear.z = 0;
    result.velocity_command.angular.x = 0;
    result.velocity_command.angular.y = 0;
    result.velocity_command.angular.z =
      copysign(1.0, config_.l) * control_effort_lat_ +
      control_effort_ang_;  // Take the sign of l for the lateral control effort
    result.velocity_command.angular.z =
      std::clamp(result.velocity_command.angular.z, -config_.max_yaw_vel, config_.max_yaw_vel);
    // For non-holonomic robots apply saturation based on minimum turning radius
    double max_ang_twist_tr;
    if (config_.min_turning_radius < RADIUS_EPS) {
      // Rotation in place is allowed
      // minimum_turning_radius = RADIUS_EPS; // This variable is not used anymore so it does not matter
      // do not restrict angular velocity. Thus use the biggets number possible
      max_ang_twist_tr = std::numeric_limits<double>::infinity();
    } else {
      max_ang_twist_tr = fabs(result.velocity_command.linear.x / config_.min_turning_radius);
    }
    result.velocity_command.angular.z =
      std::clamp(result.velocity_command.angular.z, -max_ang_twist_tr, max_ang_twist_tr);
  }
  // Apply max acceleration limit to yaw
  const double yaw_acc = std::clamp(
    (result.velocity_command.angular.z - current_yaw_vel) / dt.toSec(), -config_.max_yaw_acc,
    config_.max_yaw_acc);
  const double new_yaw_vel = current_yaw_vel + (yaw_acc * dt.toSec());
  result.velocity_command.angular.z = new_yaw_vel;

  // Transform velocity commands at base_link to steer when using tricycle model
  if (use_tricycle_model_) {
    geometry_msgs::Twist output_steering;
    TricycleSteeringCmdVel steering_cmd =
      computeTricycleModelInverseKinematics(result.velocity_command);
    if (result.velocity_command.linear.x < 0.0 && steering_cmd.speed > 0.0) {
      steering_cmd.speed = -steering_cmd.speed;
      if (steering_cmd.steering_angle > 0) {
        steering_cmd.steering_angle = steering_cmd.steering_angle - M_PI;
      } else {
        steering_cmd.steering_angle = steering_cmd.steering_angle + M_PI;
      }
    }
    // Apply limits to steering commands
    steering_cmd.steering_angle = std::clamp(
      steering_cmd.steering_angle, -config_.max_steering_angle, config_.max_steering_angle);
    const double steering_yaw_vel = std::clamp(
      (steering_cmd.steering_angle - controller_state_.previous_steering_angle) / dt.toSec(),
      -config_.max_steering_yaw_vel, config_.max_steering_yaw_vel);
    const double steering_angle_acc = std::clamp(
      (steering_yaw_vel - controller_state_.previous_steering_yaw_vel) / dt.toSec(),
      -config_.max_steering_yaw_acc, config_.max_steering_yaw_acc);
    steering_cmd.steering_angle_velocity =
      controller_state_.previous_steering_yaw_vel + (steering_angle_acc * dt.toSec());
    steering_cmd.steering_angle = controller_state_.previous_steering_angle +
                                  (steering_cmd.steering_angle_velocity * dt.toSec());

    steering_cmd.speed =
      std::clamp(steering_cmd.speed, -config_.max_steering_x_vel, config_.max_steering_x_vel);
    steering_cmd.acceleration = std::clamp(
      (steering_cmd.speed - controller_state_.previous_steering_x_vel) / dt.toSec(),
      -config_.max_steering_x_acc, config_.max_steering_x_acc);
    steering_cmd.speed =
      controller_state_.previous_steering_x_vel + (steering_cmd.acceleration * dt.toSec());

    controller_state_.previous_steering_angle = steering_cmd.steering_angle;
    controller_state_.previous_steering_yaw_vel = steering_cmd.steering_angle_velocity;
    controller_state_.previous_steering_x_vel = steering_cmd.speed;

    // Compute velocities back to base_link and update controller state
    output_steering =
      computeTricycleModelForwardKinematics(steering_cmd.speed, steering_cmd.steering_angle);
    controller_state_.current_x_vel = output_steering.linear.x;
    controller_state_.current_yaw_vel = output_steering.angular.z;

    result.pid_debug.steering_angle = steering_cmd.steering_angle;
    result.pid_debug.steering_yaw_vel = steering_cmd.steering_angle_velocity;
    result.pid_debug.steering_x_vel = steering_cmd.speed;

    result.velocity_command = output_steering;
  }

  // Publish control effort if controller enabled
  if (!enabled_)  // Do nothing reset integral action and all filters
  {
    controller_state_.error_integral_lat = 0.0;
    controller_state_.error_integral_ang = 0.0;
  }

  controller_state_.current_x_vel = new_x_vel;
  controller_state_.current_yaw_vel = new_yaw_vel;

  return result;
}

Controller::UpdateResult Controller::update_with_limits(
  const geometry_msgs::Transform & current_tf, const geometry_msgs::Twist & odom_twist,
  ros::Duration dt)
{
  // All limits are absolute
  double max_x_vel = std::abs(config_.target_x_vel);

  // Apply external limit
  max_x_vel = std::min(max_x_vel, vel_max_external_);

  // Apply obstacle limit
  max_x_vel = std::min(max_x_vel, vel_max_obstacle_);

  // Apply mpc limit (last because less iterations required if max vel is already limited)
  double vel_max_mpc = std::numeric_limits<double>::infinity();
  if (config_.use_mpc) {
    vel_max_mpc = std::abs(
      mpc_based_max_vel(std::copysign(max_x_vel, config_.target_x_vel), current_tf, odom_twist));
    max_x_vel = std::min(max_x_vel, vel_max_mpc);
  }

  // Some logging:
  ROS_DEBUG(
    "max_x_vel=%.3f, target_x_vel=%.3f, vel_max_external=%.3f, vel_max_obstacle=%.3f, "
    "vel_max_mpc=%.3f",
    max_x_vel, config_.target_x_vel, vel_max_external_, vel_max_obstacle_, vel_max_mpc);
  if (max_x_vel != config_.target_x_vel) {
    if (max_x_vel == vel_max_external_) {
      ROS_WARN_THROTTLE(5.0, "External velocity limit active %.2fm/s", vel_max_external_);
    } else if (max_x_vel == vel_max_obstacle_) {
      ROS_WARN_THROTTLE(5.0, "Obstacle velocity limit active %.2fm/s", vel_max_obstacle_);
    } else if (max_x_vel == vel_max_mpc) {
      ROS_WARN_THROTTLE(5.0, "MPC velocity limit active %.2fm/s", vel_max_mpc);
    }
  }

  // The end velocity is bound by the same limits to avoid accelerating above the limit in the end phase
  double max_end_x_vel = std::min(
    {std::abs(config_.target_end_x_vel), vel_max_external_, vel_max_obstacle_, vel_max_mpc});
  max_end_x_vel = std::copysign(max_end_x_vel, config_.target_end_x_vel);

  // Update the controller with the new setting
  max_x_vel = std::copysign(max_x_vel, config_.target_x_vel);
  return update(max_x_vel, max_end_x_vel, current_tf, odom_twist, dt);
}

// output updated velocity command: (Current position, current measured velocity, closest point index, estimated
// duration of arrival, debug info)
double Controller::mpc_based_max_vel(
  double target_x_vel, const geometry_msgs::Transform & current_tf,
  const geometry_msgs::Twist & odom_twist)
{
  // Temporary save global data
  ControllerState controller_state_saved;
  controller_state_saved = controller_state_;

  // Bisection optimisation parameters
  double target_x_vel_prev = 0.0;  // Previous iteration velocity command
  int mpc_vel_optimization_iter = 0;

  // MPC parameters
  int mpc_fwd_iter = 0;  // Reset MPC iterations

  // Create predicted position vector
  geometry_msgs::Transform predicted_tf = current_tf;
  geometry_msgs::Twist pred_twist = odom_twist;

  double new_nominal_x_vel = target_x_vel;  // Start off from the current velocity

  // Loop MPC
  while (mpc_fwd_iter < config_.mpc_max_fwd_iterations &&
         mpc_vel_optimization_iter <= config_.mpc_max_vel_optimization_iterations) {
    mpc_fwd_iter += 1;

    // Check if robot stays within bounds for all iterations, if the new_nominal_x_vel is smaller than
    // max_target_x_vel we can increase it
    if (
      mpc_fwd_iter == config_.mpc_max_fwd_iterations &&
      fabs(controller_state_.error_lat.errors().at<0>()) <= config_.mpc_max_error_lat &&
      fabs(new_nominal_x_vel) < abs(target_x_vel)) {
      mpc_vel_optimization_iter += 1;

      // When we reach the maximum allowed mpc optimization iterations, do not change velocity anymore
      if (mpc_vel_optimization_iter > config_.mpc_max_vel_optimization_iterations) {
        break;
      }

      // Increase speed
      target_x_vel_prev = std::exchange(
        new_nominal_x_vel,
        copysign(1.0, new_nominal_x_vel) * abs(target_x_vel_prev - new_nominal_x_vel) / 2 +
          new_nominal_x_vel);

      // Reset variables
      controller_state_ = controller_state_saved;

      predicted_tf = current_tf;
      pred_twist = odom_twist;
      mpc_fwd_iter = 0;
    }
    // If the robot gets out of bounds earlier we decrease the velocity
    else if (abs(controller_state_.error_lat.errors().at<0>()) >= config_.mpc_max_error_lat) {
      mpc_vel_optimization_iter += 1;

      // Lower speed
      target_x_vel_prev = std::exchange(
        new_nominal_x_vel,
        -copysign(1.0, new_nominal_x_vel) * abs(target_x_vel_prev - new_nominal_x_vel) / 2 +
          new_nominal_x_vel);

      // Reset variables
      controller_state_ = controller_state_saved;

      predicted_tf = current_tf;
      pred_twist = odom_twist;
      mpc_fwd_iter = 0;

      // Warning if new_nominal_x_vel becomes really low
      if (abs(new_nominal_x_vel) < 0.01) {
        ROS_WARN_THROTTLE(5.0, "Lowering velocity did not decrease the lateral error enough.");
      }
    } else if (mpc_fwd_iter != config_.mpc_max_fwd_iterations) {
      // Run controller
      // Output: pred_twist.[linear.x, linear.y, linear.z, angular.x, angular.y, angular.z]
      pred_twist = Controller::update(
                     new_nominal_x_vel, config_.target_end_x_vel, predicted_tf, pred_twist,
                     ros::Duration(config_.mpc_simulation_sample_time))
                     .velocity_command;

      // Run plant model
      const double theta = tf2::getYaw(predicted_tf.rotation);
      predicted_tf.translation.x +=
        pred_twist.linear.x * cos(theta) * config_.mpc_simulation_sample_time;
      predicted_tf.translation.y +=
        pred_twist.linear.x * sin(theta) * config_.mpc_simulation_sample_time;
      tf2::Quaternion q;
      q.setRPY(0, 0, theta + pred_twist.angular.z * config_.mpc_simulation_sample_time);
      predicted_tf.rotation = tf2::toMsg(q);
    }
  }
  // Apply limits to the velocity
  double mpc_vel_limit =
    copysign(1.0, new_nominal_x_vel) * fmax(fabs(new_nominal_x_vel), config_.mpc_min_x_vel);

  // Revert global variables
  controller_state_ = controller_state_saved;

  return std::abs(mpc_vel_limit);
}

void Controller::printParameters() const
{
  ROS_INFO("CONTROLLER PARAMETERS");
  ROS_INFO("-----------------------------------------");
  ROS_INFO("Controller enabled: %i", enabled_);
  ROS_INFO("Controller DEBUG enabled: %i", config_.controller_debug_enabled);
  ROS_INFO("Distance L: %f", config_.l);
  ROS_INFO("Track base_link enabled?: %i", config_.track_base_link);

  ROS_INFO(
    "Target forward velocities (xv: %f, xv_end,: %f)", config_.target_x_vel,
    config_.target_end_x_vel);
  ROS_INFO(
    "Target forward (de)accelerations (xacc: %f, xdecc,: %f)", config_.target_x_acc,
    config_.target_x_decc);
  ROS_INFO("Maximum allowed forward velocity error: %f", config_.max_error_x_vel);
  ROS_INFO("Feedback (lat, ang): ( %i, %i)", config_.feedback_lat, config_.feedback_ang);
  ROS_INFO("Feedforward (lat, ang): (%i, %i)", config_.feedforward_lat, config_.feedforward_ang);
  ROS_INFO(
    "Lateral gains: (Kp: %f, Ki, %f, Kd, %f)", config_.Kp_lat, config_.Ki_lat, config_.Kd_lat);
  ROS_INFO(
    "Angular gains: (Kp: %f, Ki, %f, Kd, %f)", config_.Kp_ang, config_.Ki_ang, config_.Kd_ang);

  ROS_INFO("Robot type (holonomic): (%i)", holonomic_robot_enable_);

  ROS_INFO("Integral-windup limit: %f", windup_limit);
  ROS_INFO("Saturation limits xy: %f/%f", lat_upper_limit, lat_lower_limit);
  ROS_INFO("Saturation limits ang: %f/%f", ang_upper_limit, ang_lower_limit);
  ROS_INFO("-----------------------------------------");
}

void Controller::configure(path_tracking_pid::PidConfig & config)
{
  // Erase all queues when config changes
  controller_state_.error_lat.reset();
  controller_state_.error_deriv_lat.reset();

  controller_state_.error_ang.reset();
  controller_state_.error_deriv_ang.reset();

  config.l = copysign(config.l, config.target_x_vel);
  if (controller_state_.end_phase_enabled) {
    ROS_WARN_COND(
      abs(config.target_end_x_vel - config_.target_end_x_vel) > 1e-3,
      "Won't change end velocity in end phase");
    ROS_WARN_COND(
      abs(config.target_x_acc - config_.target_x_acc) > 1e-3,
      "Won't change accelerations in end phase");
    ROS_WARN_COND(
      abs(config.target_x_decc - config_.target_x_decc) > 1e-3,
      "Won't change accelerations in end phase");
    config.target_end_x_vel = config_.target_end_x_vel;
    config.target_x_acc = config_.target_x_acc;
    config.target_x_decc = config_.target_x_decc;
  }

  config.groups.mpc_group.state = config.use_mpc;  // Hide config options if disabled
  config.mpc_min_x_vel = fmin(config.mpc_min_x_vel, fabs(config.target_x_vel));

  config.groups.collision_group.state = config.anti_collision;  // Hide config options if disabled

  config_ = config;

  ROS_DEBUG(
    "Track base_link? Then global path poses are needed! '%d'", (int)config_.track_base_link);

  // printParameters();
}

path_tracking_pid::PidConfig Controller::getConfig() { return config_; }

void Controller::setEnabled(bool value)
{
  ROS_DEBUG("Controller::setEnabled(%d)", value);
  enabled_ = value;
}

void Controller::reset()
{
  controller_state_.current_x_vel = 0.0;
  controller_state_.current_yaw_vel = 0.0;
  controller_state_.previous_steering_angle = 0.0;
  controller_state_.previous_steering_yaw_vel = 0.0;
  controller_state_.previous_steering_x_vel = 0.0;
  controller_state_.error_integral_lat = 0.0;
  controller_state_.error_integral_ang = 0.0;
  controller_state_.error_lat.reset();
  controller_state_.error_deriv_lat.reset();
  controller_state_.error_ang.reset();
  controller_state_.error_deriv_ang.reset();
}

void Controller::setVelMaxExternal(double value)
{
  if (value < 0.0) {
    ROS_ERROR_THROTTLE(1.0, "External velocity limit (%f) has to be positive", value);
    return;
  }
  if (value < 0.1) {
    ROS_WARN_THROTTLE(
      1.0, "External velocity limit is very small (%f), this could result in standstill", value);
  }
  vel_max_external_ = value;
}

void Controller::setVelMaxObstacle(double value)
{
  ROS_WARN_COND(
    vel_max_obstacle_ != 0.0 && value == 0.0, "Collision imminent, slamming the brakes");
  vel_max_obstacle_ = value;
}

double Controller::getVelMaxObstacle() const { return vel_max_obstacle_; }

}  // namespace path_tracking_pid
