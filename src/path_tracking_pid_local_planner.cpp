//
// Created by nobleo on 12-9-18.
//

#include <mbf_msgs/ExePathResult.h>
#include <path_tracking_pid/PidDebug.h>
#include <path_tracking_pid/PidFeedback.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <path_tracking_pid/path_tracking_pid_local_planner.hpp>
#include <string>
#include <vector>

#include "common.hpp"

// register planner as move_base and move_base plugins
PLUGINLIB_EXPORT_CLASS(
  path_tracking_pid::TrackingPidLocalPlanner, mbf_costmap_core::CostmapController)

namespace path_tracking_pid
{
namespace
{
constexpr double MAP_PARALLEL_THRESH = 0.2;
constexpr double DT_MAX = 1.5;

/**
 * Convert the plan from geometry message format to tf2 format.
 *
 * @param[in] plan Plan to convert.
 * @return Converted plan.
 */
std::vector<tf2::Transform> convert_plan(const std::vector<geometry_msgs::PoseStamped> & plan)
{
  auto result = std::vector<tf2::Transform>{};

  result.reserve(plan.size());
  std::transform(
    plan.cbegin(), plan.cend(), std::back_inserter(result),
    [](const geometry_msgs::PoseStamped & msg) { return tf2_convert<tf2::Transform>(msg.pose); });

  return result;
}

}  // namespace

void TrackingPidLocalPlanner::reconfigure_pid(path_tracking_pid::PidConfig & config)
{
  pid_controller_.configure(config);
  controller_debug_enabled_ = config.controller_debug_enabled;
}

void TrackingPidLocalPlanner::initialize(
  std::string name, tf2_ros::Buffer * tf, costmap_2d::Costmap2DROS * costmap)
{
  ros::NodeHandle nh("~/" + name);
  ros::NodeHandle gn;
  ROS_DEBUG("TrackingPidLocalPlanner::initialize(%s, ..., ...)", name.c_str());
  // setup dynamic reconfigure
  pid_server_ =
    std::make_unique<dynamic_reconfigure::Server<path_tracking_pid::PidConfig>>(config_mutex_, nh);
  pid_server_->setCallback(
    [this](auto & config, auto /*unused*/) { this->reconfigure_pid(config); });
  pid_controller_.setEnabled(false);

  bool holonomic_robot;
  nh.param<bool>("holonomic_robot", holonomic_robot, false);
  pid_controller_.setHolonomic(holonomic_robot);

  bool estimate_pose_angle;
  nh.param<bool>("estimate_pose_angle", estimate_pose_angle, false);
  pid_controller_.setEstimatePoseAngle(estimate_pose_angle);

  nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");

  nh.param<bool>("use_tricycle_model", use_tricycle_model_, false);
  nh.param<std::string>("steered_wheel_frame", steered_wheel_frame_, "steer");

  visualization_ = std::make_unique<Visualization>(nh);
  debug_pub_ = nh.advertise<path_tracking_pid::PidDebug>("debug", 1);
  path_pub_ = nh.advertise<nav_msgs::Path>("visualization_path", 1, true);

  sub_odom_ = gn.subscribe("odom", 1, &TrackingPidLocalPlanner::curOdomCallback, this);
  sub_vel_max_external_ =
    nh.subscribe("vel_max", 1, &TrackingPidLocalPlanner::velMaxExternalCallback, this);
  feedback_pub_ = nh.advertise<path_tracking_pid::PidFeedback>("feedback", 1);

  map_frame_ = costmap->getGlobalFrameID();
  costmap_ = costmap;
  tf_ = tf;

  initialized_ = true;
}

bool TrackingPidLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> & global_plan)
{
  if (!initialized_) {
    ROS_ERROR(
      "path_tracking_pid has not been initialized, please call initialize() before using this "
      "planner");
    return false;
  }

  auto global_plan_map_frame = global_plan;

  std::string path_frame = global_plan_map_frame.at(0).header.frame_id;
  ROS_DEBUG("TrackingPidLocalPlanner::setPlan(%zu)", global_plan_map_frame.size());
  ROS_DEBUG("Plan is defined in frame '%s'", path_frame.c_str());

  /* If frame of received plan is not equal to mbf-map_frame, translate first */
  if (map_frame_ != path_frame) {
    ROS_DEBUG(
      "Transforming plan since my global_frame = '%s' and my plan is in frame: '%s'",
      map_frame_.c_str(), path_frame.c_str());
    geometry_msgs::TransformStamped tf_transform;
    tf_transform = tf_->lookupTransform(map_frame_, path_frame, ros::Time(0));
    // Check alignment, when path-frame is severly mis-aligned show error
    double yaw;
    double pitch;
    double roll;
    tf2::getEulerYPR(tf_transform.transform.rotation, yaw, pitch, roll);
    if (std::fabs(pitch) > MAP_PARALLEL_THRESH || std::fabs(roll) > MAP_PARALLEL_THRESH) {
      ROS_ERROR(
        "Path is given in %s frame which is severly mis-aligned with our map-frame: %s",
        path_frame.c_str(), map_frame_.c_str());
    }
    for (auto & pose_stamped : global_plan_map_frame) {
      tf2::doTransform(pose_stamped.pose, pose_stamped.pose, tf_transform);
      pose_stamped.header.frame_id = map_frame_;
      // 'Project' plan by removing z-component
      pose_stamped.pose.position.z = 0.0;
    }
  }

  if (controller_debug_enabled_) {
    nav_msgs::Path received_path;
    received_path.header = global_plan_map_frame.at(0).header;
    received_path.poses = global_plan_map_frame;
    path_pub_.publish(received_path);
  }

  try {
    ROS_DEBUG(
      "map_frame: %s, plan_frame: %s, base_link_frame: %s", map_frame_.c_str(), path_frame.c_str(),
      base_link_frame_.c_str());
    tfCurPoseStamped_ = tf_->lookupTransform(map_frame_, base_link_frame_, ros::Time(0));
  } catch (const tf2::TransformException & ex) {
    ROS_ERROR("Received an exception trying to transform: %s", ex.what());
    return false;
  }

  // Feasability check, but only when not resuming with odom-vel
  if (
    pid_controller_.getConfig().init_vel_method != Pid_Odom &&
    pid_controller_.getConfig().init_vel_max_diff >= 0.0 &&
    std::abs(latest_odom_.twist.twist.linear.x - pid_controller_.getCurrentForwardVelocity()) >
      pid_controller_.getConfig().init_vel_max_diff) {
    ROS_ERROR(
      "Significant diff between odom (%f) and controller_state (%f) detected. Aborting!",
      latest_odom_.twist.twist.linear.x, pid_controller_.getCurrentForwardVelocity());
    return false;
  }

  if (use_tricycle_model_) {
    try {
      ROS_DEBUG(
        "base_link_frame: %s, steered_wheel_frame: %s", base_link_frame_.c_str(),
        steered_wheel_frame_.c_str());
      tf_base_to_steered_wheel_stamped_ =
        tf_->lookupTransform(base_link_frame_, steered_wheel_frame_, ros::Time(0));
    } catch (const tf2::TransformException & ex) {
      ROS_ERROR("Received an exception trying to transform: %s", ex.what());
      ROS_ERROR(
        "Invalid transformation from base_link_frame to steered_wheel_frame. Tricycle model will "
        "be disabled");
      use_tricycle_model_ = false;
    }

    pid_controller_.setTricycleModel(
      use_tricycle_model_,
      tf2_convert<tf2::Transform>(tf_base_to_steered_wheel_stamped_.transform));

    // TODO(clopez): subscribe to steered wheel odom
    geometry_msgs::Twist steering_odom_twist;
    if (!pid_controller_.setPlan(
          tf2_convert<tf2::Transform>(tfCurPoseStamped_.transform), latest_odom_.twist.twist,
          tf2_convert<tf2::Transform>(tf_base_to_steered_wheel_stamped_.transform),
          steering_odom_twist, convert_plan(global_plan_map_frame))) {
      return false;
    }
  } else {
    if (!pid_controller_.setPlan(
          tf2_convert<tf2::Transform>(tfCurPoseStamped_.transform), latest_odom_.twist.twist,
          convert_plan(global_plan_map_frame))) {
      return false;
    }
  }

  pid_controller_.setEnabled(true);
  active_goal_ = true;
  prev_time_ = ros::Time(0);
  return true;
}

std::optional<geometry_msgs::Twist> TrackingPidLocalPlanner::computeVelocityCommands()
{
  ros::Time now = ros::Time::now();
  if (prev_time_.isZero()) {
    prev_time_ = now - prev_dt_;  // Initialisation round
  }
  ros::Duration dt = now - prev_time_;
  if (dt.isZero()) {
    ROS_ERROR_THROTTLE(
      5, "dt=0 detected, skipping loop(s). Possible overloaded cpu or simulating too fast");
    auto cmd_vel = geometry_msgs::Twist();
    cmd_vel.linear.x = pid_controller_.getCurrentForwardVelocity();
    cmd_vel.angular.z = pid_controller_.getCurrentYawVelocity();
    // At the first call of computeVelocityCommands() we can't calculate a cmd_vel. We can't return
    // false because of https://github.com/magazino/move_base_flex/issues/195 so the current
    // velocity is send instead.
    return cmd_vel;
  }
  if (dt < ros::Duration(0) || dt > ros::Duration(DT_MAX)) {
    ROS_ERROR("Invalid time increment: %f. Aborting", dt.toSec());
    return std::nullopt;
  }
  try {
    ROS_DEBUG("map_frame: %s, base_link_frame: %s", map_frame_.c_str(), base_link_frame_.c_str());
    tfCurPoseStamped_ = tf_->lookupTransform(map_frame_, base_link_frame_, ros::Time(0));
  } catch (const tf2::TransformException & ex) {
    ROS_ERROR("Received an exception trying to transform: %s", ex.what());
    active_goal_ = false;
    return std::nullopt;
  }

  // Handle obstacles
  if (pid_controller_.getConfig().anti_collision) {
    const std::vector<geometry_msgs::Point> footprint = costmap_->getRobotFootprint();
    auto cost = projectedCollisionCost(
      costmap_->getCostmap(), footprint, projectionSteps(), visualization_, map_frame_);

    if (cost >= costmap_2d::LETHAL_OBSTACLE) {
      pid_controller_.setVelMaxObstacle(0.0);
    } else if (pid_controller_.getConfig().obstacle_speed_reduction) {
      double max_vel = pid_controller_.getConfig().max_x_vel;
      double reduction_factor = static_cast<double>(cost) / costmap_2d::LETHAL_OBSTACLE;
      double limit = max_vel * (1 - reduction_factor);
      ROS_DEBUG("Cost: %d, factor: %f, limit: %f", cost, reduction_factor, limit);
      pid_controller_.setVelMaxObstacle(limit);
    } else {
      pid_controller_.setVelMaxObstacle(INFINITY);  // set back to inf
    }
  } else {
    pid_controller_.setVelMaxObstacle(INFINITY);  // Can be disabled live, so set back to inf
  }

  const auto update_result = pid_controller_.update_with_limits(
    tf2_convert<tf2::Transform>(tfCurPoseStamped_.transform), latest_odom_.twist.twist, dt);

  path_tracking_pid::PidFeedback feedback_msg;
  feedback_msg.eda = ros::Duration(update_result.eda);
  feedback_msg.progress = update_result.progress;
  feedback_pub_.publish(feedback_msg);

  if (cancel_requested_) {
    path_tracking_pid::PidConfig config = pid_controller_.getConfig();
    // Copysign here, such that when cancelling while driving backwards, we decelerate to -0.0 and hence
    // the sign propagates correctly
    config.target_x_vel = std::copysign(0.0, config.target_x_vel);
    config.target_end_x_vel = std::copysign(0.0, config.target_x_vel);
    boost::recursive_mutex::scoped_lock lock(config_mutex_);
    // When updating from own server no callback is called. Thus controller is updated first and then server is notified
    pid_controller_.configure(config);
    pid_server_->updateConfig(config);
    lock.unlock();
    cancel_requested_ = false;
  }

  if (controller_debug_enabled_) {
    debug_pub_.publish(update_result.pid_debug);

    // publish rviz visualization
    std_msgs::Header header;
    header.stamp = now;
    header.frame_id = map_frame_;
    const auto tfCurPose = tf2_convert<tf2::Transform>(tfCurPoseStamped_.transform);
    visualization_->publishAxlePoint(header, tfCurPose);
    visualization_->publishControlPoint(header, pid_controller_.getCurrentWithCarrot());
    visualization_->publishGoalPoint(header, pid_controller_.getCurrentGoal());
    visualization_->publishPlanPoint(header, pid_controller_.getCurrentPosOnPlan());
  }

  prev_time_ = now;
  prev_dt_ =
    dt;  // Store last known valid dt for next cycles (https://github.com/magazino/move_base_flex/issues/195)
  return update_result.velocity_command;
}

std::vector<tf2::Transform> TrackingPidLocalPlanner::projectionSteps()
{
  // Check how far we should check forward
  double x_vel = pid_controller_.getCurrentForwardVelocity();
  double collision_look_ahead_distance =
    x_vel * x_vel / (2 * pid_controller_.getConfig().target_x_decc) +
    pid_controller_.getConfig().collision_look_ahead_length_offset;
  uint n_steps = std::ceil(
    collision_look_ahead_distance / pid_controller_.getConfig().collision_look_ahead_resolution);
  double x_resolution = collision_look_ahead_distance / std::max(static_cast<int>(n_steps), 1);

  // Define a x_step transform which will be used to step forward the position.
  tf2::Transform x_step_tf;
  double target_x_vel = pid_controller_.getConfig().target_x_vel;
  double max_abs_x_vel = std::abs(x_vel) > std::abs(target_x_vel) ? x_vel : target_x_vel;
  x_step_tf.setOrigin(tf2::Vector3(copysign(x_resolution, max_abs_x_vel), 0.0, 0.0));

  // Keep track of the projected position on the path.
  auto projected_global_plan_index = pid_controller_.getCurrentGlobalPlanIndex();

  // Step until lookahead is reached, for every step project the pose back to the path
  std::vector<tf2::Vector3> step_points;
  std::vector<tf2::Vector3> poses_on_path_points;
  std::vector<tf2::Transform> projected_steps_tf;
  auto projected_step_tf = tf2_convert<tf2::Transform>(tfCurPoseStamped_.transform);
  projected_steps_tf.push_back(projected_step_tf);  // Evaluate collision at base_link
  projected_step_tf =
    pid_controller_.findPoseOnPlan(projected_step_tf, projected_global_plan_index).pose;
  projected_steps_tf.push_back(projected_step_tf);  // Add base_link projected pose
  for (uint step = 0; step < n_steps; step++) {
    tf2::Transform next_straight_step_tf = projected_step_tf * x_step_tf;
    projected_step_tf =
      pid_controller_.findPoseOnPlan(next_straight_step_tf, projected_global_plan_index).pose;
    projected_steps_tf.push_back(projected_step_tf);

    // Fill markers:
    step_points.push_back(next_straight_step_tf.getOrigin());
    poses_on_path_points.push_back(projected_step_tf.getOrigin());
  }

  // Visualize
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = map_frame_;
  visualization_->publishExtrapolatedPoses(header, step_points);
  visualization_->publishgGoalPosesOnPath(header, poses_on_path_points);

  return projected_steps_tf;
}

boost::geometry::model::ring<geometry_msgs::Point> TrackingPidLocalPlanner::projectionFootprint(
  const std::vector<geometry_msgs::Point> & footprint,
  const std::vector<tf2::Transform> & projected_steps, std::unique_ptr<Visualization> & viz,
  const std::string viz_frame)
{
  std::vector<tf2::Vector3> projected_footprint_points;
  polygon_t previous_footprint_xy;
  polygon_t projected_polygon;
  for (const auto & projection_tf : projected_steps) {
    // Project footprint forward
    double x = projection_tf.getOrigin().x();
    double y = projection_tf.getOrigin().y();
    double yaw = tf2::getYaw(projection_tf.getRotation());

    // Project footprint forward
    std::vector<geometry_msgs::Point> footprint_proj;
    costmap_2d::transformFootprint(x, y, yaw, footprint, footprint_proj);

    // Append footprint to polygon
    polygon_t two_footprints = previous_footprint_xy;
    previous_footprint_xy.clear();
    for (const auto & point : footprint_proj) {
      boost::geometry::append(two_footprints, point);
      boost::geometry::append(previous_footprint_xy, point);
    }

    boost::geometry::correct(two_footprints);
    polygon_t two_footprint_hull;
    boost::geometry::convex_hull(two_footprints, two_footprint_hull);
    projected_polygon = union_(projected_polygon, two_footprint_hull);

    // Add footprint to marker
    geometry_msgs::Point previous_point = footprint_proj.back();
    for (const auto & point : footprint_proj) {
      projected_footprint_points.push_back(tf2_convert<tf2::Vector3>(previous_point));
      projected_footprint_points.push_back(tf2_convert<tf2::Vector3>(point));
      previous_point = point;
    }
  }

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = viz_frame;
  viz->publishCollisionFootprint(header, projected_footprint_points);

  return projected_polygon;
}

uint8_t TrackingPidLocalPlanner::projectedCollisionCost(
  costmap_2d::Costmap2D * costmap2d, const std::vector<geometry_msgs::Point> & footprint,
  const std::vector<tf2::Transform> & projected_steps, std::unique_ptr<Visualization> & viz,
  const std::string viz_frame)
{
  auto collision_polygon = projectionFootprint(footprint, projected_steps, viz, viz_frame);

  // Calculate cost by checking base link location in costmap
  uint8_t max_projected_step_cost = 0;
  for (const auto & projection_tf : projected_steps) {
    int map_x, map_y;
    costmap2d->worldToMapEnforceBounds(
      projection_tf.getOrigin().x(), projection_tf.getOrigin().y(), map_x, map_y);
    uint8_t projected_step_cost = costmap2d->getCost(map_x, map_y);
    if (projected_step_cost > max_projected_step_cost) {
      max_projected_step_cost = projected_step_cost;
    }
  }

  // Create a convex hull so we can use costmap2d->convexFillCells
  polygon_t collision_polygon_hull;
  boost::geometry::convex_hull(collision_polygon, collision_polygon_hull);
  std::vector<costmap_2d::MapLocation> collision_polygon_hull_map;

  // Convert to map coordinates
  for (const auto & point : collision_polygon_hull) {
    int map_x;
    int map_y;
    costmap2d->worldToMapEnforceBounds(point.x, point.y, map_x, map_y);
    costmap_2d::MapLocation map_point{static_cast<uint>(map_x), static_cast<uint>(map_y)};
    collision_polygon_hull_map.push_back(map_point);
  }

  // Get the relevant cells
  std::vector<costmap_2d::MapLocation> cells_in_polygon;
  costmap2d->convexFillCells(collision_polygon_hull_map, cells_in_polygon);

  // Get the max cost inside the concave polygon
  tf2::Vector3 collision_point;
  uint8_t max_cost = 0.0;
  for (const auto & cell_in_polygon : cells_in_polygon) {
    // Cost checker is cheaper than polygon checker, so lets do that first
    uint8_t cell_cost = costmap2d->getCost(cell_in_polygon.x, cell_in_polygon.y);
    if (cell_cost > max_cost && cell_cost != costmap_2d::NO_INFORMATION) {
      // Check if in concave polygon
      geometry_msgs::Point point;
      costmap2d->mapToWorld(cell_in_polygon.x, cell_in_polygon.y, point.x, point.y);
      if (boost::geometry::within(point, collision_polygon)) {
        max_cost = cell_cost;
        // Set collision indicator on suspected cell with current cost
        collision_point = tf2_convert<tf2::Vector3>(point);
        if (max_cost >= costmap_2d::LETHAL_OBSTACLE) {
          max_projected_step_cost = max_cost;
          break;  // Collision detected, no need to evaluate further
        }
      }
    }
  }

  // Fiddle the polygon into a marker message
  std::vector<tf2::Vector3> collision_hull_points;
  for (const geometry_msgs::Point point : collision_polygon) {
    tf2::Vector3 point_tf2;
    tf2::fromMsg(point, point_tf2);
    collision_hull_points.push_back(point_tf2);
  }
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = viz_frame;
  viz->publishCollisionObject(header, max_cost, collision_point);
  viz->publishCollisionPolygon(header, collision_hull_points);

  return max_projected_step_cost;
}

uint32_t TrackingPidLocalPlanner::computeVelocityCommands(
  const geometry_msgs::PoseStamped & /* pose */, const geometry_msgs::TwistStamped & /* velocity */,
  geometry_msgs::TwistStamped & cmd_vel, std::string & /* message */)
{
  if (!initialized_) {
    ROS_ERROR(
      "path_tracking_pid has not been initialized, please call initialize() before using this "
      "planner");
    active_goal_ = false;
    return mbf_msgs::ExePathResult::NOT_INITIALIZED;
  }
  // TODO(Cesar): Use provided pose and odom
  const auto opt_cmd_vel = computeVelocityCommands();
  if (!opt_cmd_vel) {
    active_goal_ = false;
    return mbf_msgs::ExePathResult::FAILURE;
  }
  cmd_vel.twist = *opt_cmd_vel;
  cmd_vel.header.stamp = ros::Time::now();
  cmd_vel.header.frame_id = base_link_frame_;

  bool moving = std::abs(cmd_vel.twist.linear.x) > VELOCITY_EPS;
  if (cancel_in_progress_) {
    if (!moving) {
      ROS_INFO(
        "Cancel requested and we now (almost) reached velocity 0: %f", cmd_vel.twist.linear.x);
      cancel_in_progress_ = false;
      active_goal_ = false;
      return mbf_msgs::ExePathResult::CANCELED;
    }
    ROS_INFO_THROTTLE(1.0, "Cancel in progress... remaining x_vel: %f", cmd_vel.twist.linear.x);
    return to_underlying(ComputeVelocityCommandsResult::GRACEFULLY_CANCELLING);
  }

  if (!moving && pid_controller_.getVelMaxObstacle() < VELOCITY_EPS) {
    active_goal_ = false;
    return mbf_msgs::ExePathResult::BLOCKED_PATH;
  }

  if (isGoalReached()) {
    active_goal_ = false;
  }
  return mbf_msgs::ExePathResult::SUCCESS;
}

bool TrackingPidLocalPlanner::isGoalReached() const
{
  // Return reached boolean, but never succeed when we're preempting
  return pid_controller_.isEndReached() && !cancel_in_progress_;
}

bool TrackingPidLocalPlanner::isGoalReached(
  double /* dist_tolerance */, double /* angle_tolerance */)
{
  return isGoalReached();
}

bool TrackingPidLocalPlanner::cancel()
{
  // This function runs in a separate thread
  cancel_requested_ = true;
  cancel_in_progress_ = true;
  ros::Rate r(10);
  ROS_INFO("Cancel requested, waiting in loop for cancel to finish");
  while (active_goal_) {
    r.sleep();
  }
  ROS_INFO("Finished waiting loop, done cancelling");
  return true;
}

void TrackingPidLocalPlanner::curOdomCallback(const nav_msgs::Odometry & odom_msg)
{
  latest_odom_ = odom_msg;
}

void TrackingPidLocalPlanner::velMaxExternalCallback(const std_msgs::Float64 & msg)
{
  pid_controller_.setVelMaxExternal(msg.data);
}
}  // namespace path_tracking_pid
