//
// Created by nobleo on 12-9-18.
//

#include "path_tracking_pid/path_tracking_pid_local_planner.hpp"
#include <algorithm>
#include <geometry_msgs/TransformStamped.h>
#include <limits>
#include <memory>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

// register planner as move_base and move_base plugins
PLUGINLIB_EXPORT_CLASS(path_tracking_pid::TrackingPidLocalPlanner, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(path_tracking_pid::TrackingPidLocalPlanner, mbf_costmap_core::CostmapController)

namespace path_tracking_pid
{
TrackingPidLocalPlanner::TrackingPidLocalPlanner() = default;

TrackingPidLocalPlanner::~TrackingPidLocalPlanner() = default;

void TrackingPidLocalPlanner::reconfigure_pid(path_tracking_pid::PidConfig& config, uint32_t level)
{
  pid_controller_.configure(config);
  controller_debug_enabled_ = config.controller_debug_enabled;

  if (controller_debug_enabled_ && !global_plan_.empty())
  {
    received_path_.header = global_plan_.at(0).header;
    received_path_.poses = global_plan_;
    path_pub_.publish(received_path_);
  }
}

void TrackingPidLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap)
{
  ros::NodeHandle nh("~/" + name);
  ros::NodeHandle gn;
  ROS_DEBUG("TrackingPidLocalPlanner::initialize(%s, ..., ...)", name.c_str());
  // setup dynamic reconfigure
  pid_server_ = std::make_unique<dynamic_reconfigure::Server<path_tracking_pid::PidConfig>>(config_mutex_, nh);
  dynamic_reconfigure::Server<path_tracking_pid::PidConfig>::CallbackType cb1;
  cb1 = boost::bind(&TrackingPidLocalPlanner::reconfigure_pid, this, _1, _2);
  pid_server_->setCallback(cb1);
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


  collision_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("collision_markers", 3);
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 3);
  debug_pub_ = nh.advertise<path_tracking_pid::PidDebug>("debug", 1);
  path_pub_ = nh.advertise<nav_msgs::Path>("visualization_path", 1, true);

  sub_odom_ = gn.subscribe("odom", 1, &TrackingPidLocalPlanner::curOdomCallback, this);
  sub_vel_max_external_ = nh.subscribe("vel_max", 1, &TrackingPidLocalPlanner::velMaxExternalCallback, this);
  feedback_pub_ = nh.advertise<path_tracking_pid::PidFeedback>("feedback", 1);

  map_frame_ = costmap->getGlobalFrameID();
  costmap_ = costmap;
  tf_ = tf;

  initialized_ = true;
}

bool TrackingPidLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
{
  if (!initialized_)
  {
    ROS_ERROR("path_tracking_pid has not been initialized, please call initialize() before using this planner");
    return false;
  }

  std::string path_frame = global_plan.at(0).header.frame_id;
  ROS_DEBUG("TrackingPidLocalPlanner::setPlan(%zu)", global_plan.size());
  ROS_DEBUG("Plan is defined in frame '%s'", path_frame.c_str());

  global_plan_ = global_plan;

  /* If frame of received plan is not equal to mbf-map_frame, translate first */
  if (map_frame_.compare(path_frame))
  {
    ROS_DEBUG("Transforming plan since my global_frame = '%s' and my plan is in frame: '%s'", map_frame_.c_str(),
              path_frame.c_str());
    geometry_msgs::TransformStamped tf_transform;
    tf_transform = tf_->lookupTransform(map_frame_, path_frame, ros::Time(0));
    // Check alignment, when path-frame is severly mis-aligned show error
    double yaw, pitch, roll;
    tf2::getEulerYPR(tf_transform.transform.rotation, yaw, pitch, roll);
    if (std::fabs(pitch) > MAP_PARALLEL_THRESH || std::fabs(roll) > MAP_PARALLEL_THRESH)
    {
      ROS_ERROR("Path is given in %s frame which is severly mis-aligned with our map-frame: %s", path_frame.c_str(),
                                                                                                 map_frame_.c_str());
    }
    for (auto& pose_stamped : global_plan_)
    {
      tf2::doTransform(pose_stamped.pose, pose_stamped.pose, tf_transform);
      pose_stamped.header.frame_id = map_frame_;
      // 'Project' plan by removing z-component
      pose_stamped.pose.position.z = 0.0;
    }
  }

  if (controller_debug_enabled_)
  {
    received_path_.header = global_plan_.at(0).header;
    received_path_.poses = global_plan_;
    path_pub_.publish(received_path_);
  }

  try
  {
    ROS_DEBUG("map_frame: %s, plan_frame: %s, base_link_frame: %s", map_frame_.c_str(), path_frame.c_str(),
              base_link_frame_.c_str());
    tfCurPoseStamped_ = tf_->lookupTransform(map_frame_, base_link_frame_, ros::Time(0));
  }
  catch (tf2::TransformException ex)
  {
    ROS_ERROR("Received an exception trying to transform: %s", ex.what());
    return false;
  }

  // Feasability check, but only when not resuming with odom-vel
  if (pid_controller_.getConfig().init_vel_method != Pid_Odom &&
      pid_controller_.getConfig().init_vel_max_diff >= 0.0 &&
      std::abs(latest_odom_.twist.twist.linear.x - pid_controller_.getControllerState().current_x_vel) >
        pid_controller_.getConfig().init_vel_max_diff)
  {
    ROS_ERROR("Significant diff between odom (%f) and controller_state (%f) detected. Aborting!",
              latest_odom_.twist.twist.linear.x, pid_controller_.getControllerState().current_x_vel);
    return false;
  }

  if (use_tricycle_model_)
  {
    try
    {
      ROS_DEBUG("base_link_frame: %s, steered_wheel_frame: %s", base_link_frame_.c_str(), steered_wheel_frame_.c_str());
      tf_base_to_steered_wheel_stamped_ = tf_->lookupTransform(base_link_frame_, steered_wheel_frame_, ros::Time(0));
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("Received an exception trying to transform: %s", ex.what());
      ROS_ERROR("Invalid transformation from base_link_frame to steered_wheel_frame. Tricycle model will be disabled");
      use_tricycle_model_ = false;
    }

    pid_controller_.setTricycleModel(use_tricycle_model_, tf_base_to_steered_wheel_stamped_.transform);

    // TODO(clopez): subscribe to steered wheel odom
    geometry_msgs::Twist steering_odom_twist;
    pid_controller_.setPlan(tfCurPoseStamped_.transform, latest_odom_.twist.twist,
                            tf_base_to_steered_wheel_stamped_.transform, steering_odom_twist, global_plan_);
  }
  else
  {
    pid_controller_.setPlan(tfCurPoseStamped_.transform, latest_odom_.twist.twist, global_plan_);
  }

  pid_controller_.setEnabled(true);
  active_goal_ = true;
  prev_time_ = ros::Time(0);
  return true;
}

bool TrackingPidLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  ros::Time now = ros::Time::now();
  if (prev_time_.isZero())
  {
    prev_time_ = now - prev_dt_;  // Initialisation round
  }
  ros::Duration dt = now - prev_time_;
  if (dt.isZero())
  {
    ROS_ERROR_THROTTLE(5, "dt=0 detected, skipping loop(s). Possible overloaded cpu or simulating too fast");
    cmd_vel = geometry_msgs::Twist();
    cmd_vel.linear.x = pid_controller_.getControllerState().current_x_vel;
    cmd_vel.angular.z = pid_controller_.getControllerState().current_yaw_vel;
    return true;  // False is no use: https://github.com/magazino/move_base_flex/issues/195
  }
  else if (dt < ros::Duration(0) || dt > ros::Duration(DT_MAX))
  {
    ROS_ERROR("Invalid time increment: %f. Aborting", dt.toSec());
    return false;
  }
  try
  {
    ROS_DEBUG("map_frame: %s, base_link_frame: %s", map_frame_.c_str(), base_link_frame_.c_str());
    tfCurPoseStamped_ = tf_->lookupTransform(map_frame_, base_link_frame_, ros::Time(0));
  }
  catch (tf2::TransformException ex)
  {
    ROS_ERROR("Received an exception trying to transform: %s", ex.what());
    active_goal_ = false;
    return false;
  }

  // Handle obstacles
  if (pid_controller_.getConfig().anti_collision)
  {
    auto cost = projectedCollisionCost();

    if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      pid_controller_.setVelMaxObstacle(0.0);
    }
    else if (pid_controller_.getConfig().obstacle_speed_reduction)
    {
      double max_vel = pid_controller_.getConfig().max_x_vel;
      double reduction_factor = static_cast<double>(cost) / costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
      double limit = max_vel * (1 - reduction_factor);
      ROS_DEBUG("Cost: %d, factor: %f, limit: %f", cost, reduction_factor, limit);
      pid_controller_.setVelMaxObstacle(limit);
    }
    else
    {
      pid_controller_.setVelMaxObstacle(INFINITY);  // set back to inf
    }
  }
  else
  {
    pid_controller_.setVelMaxObstacle(INFINITY);  // Can be disabled live, so set back to inf
  }

  path_tracking_pid::PidDebug pid_debug;
  double eda = 1 / FLT_EPSILON;  // initial guess. Avoids errors in case function returns due to wrong delta_t;
  double progress = 0.0;
  cmd_vel = pid_controller_.update_with_limits(tfCurPoseStamped_.transform, latest_odom_.twist.twist,
                                               dt, &eda, &progress, &pid_debug);

  path_tracking_pid::PidFeedback feedback_msg;
  feedback_msg.eda = ros::Duration(eda);
  feedback_msg.progress = progress;
  feedback_pub_.publish(feedback_msg);

  if (cancel_requested_)
  {
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
    cancel_requested_ =  false;
  }


  if (controller_debug_enabled_)
  {
    debug_pub_.publish(pid_debug);

    visualization_msgs::Marker mkCurPose, mkControlPose, mkGoalPose, mkPosOnPlan;

    // configure rviz visualization
    mkCurPose.header.frame_id = mkControlPose.header.frame_id = map_frame_;
    mkGoalPose.header.frame_id = mkPosOnPlan.header.frame_id = map_frame_;
    mkCurPose.header.stamp = mkControlPose.header.stamp = ros::Time::now();
    mkGoalPose.header.stamp = mkPosOnPlan.header.stamp = ros::Time::now();
    mkCurPose.ns = "axle point";
    mkControlPose.ns = "control point";
    mkGoalPose.ns = "goal point";
    mkPosOnPlan.ns = "plan point";
    mkCurPose.action = mkControlPose.action = visualization_msgs::Marker::ADD;
    mkGoalPose.action = mkPosOnPlan.action = visualization_msgs::Marker::ADD;
    mkCurPose.pose.orientation.w = mkControlPose.pose.orientation.w = 1.0;
    mkGoalPose.pose.orientation.w = mkPosOnPlan.pose.orientation.w = 1.0;
    mkCurPose.id = __COUNTER__;  // id has to be unique, so using a compile-time counter :)
    mkControlPose.id = __COUNTER__;
    mkGoalPose.id = __COUNTER__;
    mkPosOnPlan.id = __COUNTER__;
    mkCurPose.type = mkControlPose.type = visualization_msgs::Marker::POINTS;
    mkGoalPose.type = mkPosOnPlan.type = visualization_msgs::Marker::POINTS;
    mkCurPose.scale.x = 0.5;
    mkCurPose.scale.y = 0.5;
    mkControlPose.scale.x = 0.5;
    mkControlPose.scale.y = 0.5;
    mkGoalPose.scale.x = 0.5;
    mkGoalPose.scale.y = 0.5;
    mkCurPose.color.b = 1.0;
    mkCurPose.color.a = 1.0;
    mkControlPose.color.g = 1.0f;
    mkControlPose.color.a = 1.0;
    mkGoalPose.color.r = 1.0;
    mkGoalPose.color.a = 1.0;
    mkPosOnPlan.scale.x = 0.5;
    mkPosOnPlan.scale.y = 0.5;
    mkPosOnPlan.color.a = 1.0;
    mkPosOnPlan.color.r = 1.0f;
    mkPosOnPlan.color.g = 0.5f;

    geometry_msgs::Point p;
    std_msgs::ColorRGBA color;
    p.x = tfCurPoseStamped_.transform.translation.x;
    p.y = tfCurPoseStamped_.transform.translation.y;
    p.z = tfCurPoseStamped_.transform.translation.z;
    mkCurPose.points.push_back(p);

    tf2::Transform tfControlPose = pid_controller_.getCurrentWithCarrot();
    p.x = tfControlPose.getOrigin().x();
    p.y = tfControlPose.getOrigin().y();
    p.z = tfControlPose.getOrigin().z();
    mkControlPose.points.push_back(p);

    tf2::Transform tfGoalPose = pid_controller_.getCurrentGoal();
    p.x = tfGoalPose.getOrigin().x();
    p.y = tfGoalPose.getOrigin().y();
    p.z = tfGoalPose.getOrigin().z();
    mkGoalPose.points.push_back(p);

    tf2::Transform tfCurPose = pid_controller_.getCurrentPosOnPlan();
    p.x = tfCurPose.getOrigin().x();
    p.y = tfCurPose.getOrigin().y();
    p.z = tfCurPose.getOrigin().z();
    mkPosOnPlan.points.push_back(p);

    marker_pub_.publish(mkCurPose);
    marker_pub_.publish(mkControlPose);
    marker_pub_.publish(mkGoalPose);
    marker_pub_.publish(mkPosOnPlan);
  }

  prev_time_ = now;
  prev_dt_ = dt;  // Store last known valid dt for next cycles (https://github.com/magazino/move_base_flex/issues/195)
  return true;
}

uint8_t TrackingPidLocalPlanner::projectedCollisionCost()
{
  // configure rviz visualization
  visualization_msgs::Marker mkSteps;
  mkSteps.header.frame_id = map_frame_;
  mkSteps.header.stamp = ros::Time::now();
  mkSteps.ns = "extrapolated poses";
  mkSteps.action = visualization_msgs::Marker::ADD;
  mkSteps.pose.orientation.w = 1.0;
  mkSteps.id = __COUNTER__;
  mkSteps.type = visualization_msgs::Marker::POINTS;
  mkSteps.scale.x = 0.5;
  mkSteps.scale.y = 0.5;
  mkSteps.color.r = 1.0;
  mkSteps.color.g = 0.5;
  mkSteps.color.a = 1.0;

  visualization_msgs::Marker mkPosesOnPath;
  mkPosesOnPath.header.frame_id = map_frame_;
  mkPosesOnPath.header.stamp = ros::Time::now();
  mkPosesOnPath.ns = "goal poses on path";
  mkPosesOnPath.action = visualization_msgs::Marker::ADD;
  mkPosesOnPath.pose.orientation.w = 1.0;
  mkPosesOnPath.id = __COUNTER__;
  mkPosesOnPath.type = visualization_msgs::Marker::POINTS;
  mkPosesOnPath.scale.x = 0.5;
  mkPosesOnPath.scale.y = 0.5;
  mkPosesOnPath.color.r = 1.0;
  mkPosesOnPath.color.g = 1.0;
  mkPosesOnPath.color.a = 1.0;

  visualization_msgs::Marker mkCollisionFootprint;
  mkCollisionFootprint.header.frame_id = map_frame_;
  mkCollisionFootprint.header.stamp = ros::Time::now();
  mkCollisionFootprint.ns = "Collision footprint";
  mkCollisionFootprint.action = visualization_msgs::Marker::ADD;
  mkCollisionFootprint.pose.orientation.w = 1.0;
  mkCollisionFootprint.id = __COUNTER__;
  mkCollisionFootprint.type = visualization_msgs::Marker::LINE_LIST;
  mkCollisionFootprint.scale.x = 0.1;
  mkCollisionFootprint.color.b = 1.0;
  mkCollisionFootprint.color.a = 0.3;

  visualization_msgs::Marker mkCollisionHull;
  mkCollisionHull.header.frame_id = map_frame_;
  mkCollisionHull.header.stamp = ros::Time::now();
  mkCollisionHull.ns = "Collision polygon";
  mkCollisionHull.action = visualization_msgs::Marker::ADD;
  mkCollisionHull.pose.orientation.w = 1.0;
  mkCollisionHull.id = __COUNTER__;
  mkCollisionHull.type = visualization_msgs::Marker::LINE_STRIP;
  mkCollisionHull.scale.x = 0.2;
  mkCollisionHull.color.r = 1.0;
  mkCollisionHull.color.a = 0.3;

  visualization_msgs::Marker mkCollisionIndicator;
  mkCollisionIndicator.header.frame_id = map_frame_;
  mkCollisionIndicator.header.stamp = ros::Time::now();
  mkCollisionIndicator.ns = "Collision object";
  mkCollisionIndicator.pose.orientation.w = 1.0;
  mkCollisionIndicator.id = __COUNTER__;
  mkCollisionIndicator.type = visualization_msgs::Marker::CYLINDER;
  mkCollisionIndicator.scale.x = 0.5;
  mkCollisionIndicator.scale.y = 0.5;
  mkCollisionIndicator.color.r = 1.0;
  mkCollisionIndicator.color.a = 0.0;
  visualization_msgs::MarkerArray mkCollision;

  // Check how far we should check forward
  double x_vel = pid_controller_.getControllerState().current_x_vel;
  double collision_look_ahead_distance = x_vel*x_vel / (2*pid_controller_.getConfig().target_x_decc)
                                         + pid_controller_.getConfig().collision_look_ahead_length_offset;
  uint n_steps = std::ceil(collision_look_ahead_distance / pid_controller_.getConfig().collision_look_ahead_resolution);
  double x_resolution = collision_look_ahead_distance / n_steps;

  // Define a x_step transform which will be used to step forward the position.
  tf2::Transform x_step_tf;
  x_step_tf.setOrigin(tf2::Vector3(copysign(x_resolution, x_vel), 0.0, 0.0));

  // Use a controller state to forward project the position on the path
  ControllerState projected_controller_state = pid_controller_.getControllerState();
  geometry_msgs::Transform current_tf = tfCurPoseStamped_.transform;

  // Step until lookahead is reached, for every step project the pose back to the path
  std::vector<tf2::Transform> projected_steps_tf;
  tf2::Transform projected_step_tf;
  tf2::fromMsg(current_tf, projected_step_tf);
  projected_steps_tf.push_back(projected_step_tf);  // Evaluate collision at base_link
  projected_step_tf = pid_controller_.findPositionOnPlan(current_tf, &projected_controller_state);
  projected_steps_tf.push_back(projected_step_tf);  // Add base_link projected pose
  for (uint step = 0; step < n_steps; step++)
  {
    tf2::Transform next_straight_step_tf = projected_step_tf * x_step_tf;
    projected_step_tf = pid_controller_.findPositionOnPlan(tf2::toMsg(next_straight_step_tf),
                                                           &projected_controller_state);
    projected_steps_tf.push_back(projected_step_tf);

    // Fill markers:
    geometry_msgs::Point mkStep;
    tf2::toMsg(next_straight_step_tf.getOrigin(), mkStep);
    mkSteps.points.push_back(mkStep);
    geometry_msgs::Point mkPointOnPath;
    tf2::toMsg(projected_step_tf.getOrigin(), mkPointOnPath);
    mkPosesOnPath.points.push_back(mkPointOnPath);
  }

  polygon_t previous_footprint_xy;
  polygon_t collision_polygon;
  for (const auto& projection_tf : projected_steps_tf)
  {
    // Project footprint forward
    double x = projection_tf.getOrigin().x();
    double y = projection_tf.getOrigin().y();
    double yaw = tf2::getYaw(projection_tf.getRotation());
    std::vector<geometry_msgs::Point> footprint;
    costmap_2d::transformFootprint(x, y, yaw, costmap_->getRobotFootprint(), footprint);

    // Append footprint to polygon
    polygon_t two_footprints = previous_footprint_xy;
    previous_footprint_xy.clear();
    for (const auto& point : footprint)
    {
      boost::geometry::append(two_footprints, point);
      boost::geometry::append(previous_footprint_xy, point);
    }

    boost::geometry::correct(two_footprints);
    polygon_t two_footprint_hull;
    boost::geometry::convex_hull(two_footprints, two_footprint_hull);
    collision_polygon = union_(collision_polygon, two_footprint_hull);

    // Add footprint to marker
    geometry_msgs::Point previous_point = footprint.back();
    for (const auto& point : footprint)
    {
      mkCollisionFootprint.points.push_back(previous_point);
      mkCollisionFootprint.points.push_back(point);
      previous_point = point;
    }
  }

  // Create a convex hull so we can use costmap2d->convexFillCells
  costmap_2d::Costmap2D* costmap2d = costmap_->getCostmap();
  polygon_t collision_polygon_hull;
  boost::geometry::convex_hull(collision_polygon, collision_polygon_hull);
  std::vector<costmap_2d::MapLocation> collision_polygon_hull_map;

  // Convert to map coordinates
  for (const auto& point : collision_polygon_hull)
  {
    int map_x, map_y;
    costmap2d->worldToMapEnforceBounds(point.x, point.y, map_x, map_y);
    costmap_2d::MapLocation map_point{static_cast<uint>(map_x), static_cast<uint>(map_y)};
    collision_polygon_hull_map.push_back(map_point);
  }

  // Get the relevant cells
  std::vector<costmap_2d::MapLocation> cells_in_polygon;
  costmap2d->convexFillCells(collision_polygon_hull_map, cells_in_polygon);

  // Get the max cost inside the concave polygon
  uint8_t max_cost = 0.0;
  for (const auto& cell_in_polygon : cells_in_polygon)
  {
    // Cost checker is cheaper than polygon checker, so lets do that first
    uint8_t cell_cost = costmap2d->getCost(cell_in_polygon.x, cell_in_polygon.y);
    if (cell_cost > max_cost && cell_cost != costmap_2d::NO_INFORMATION)
    {
      // Check if in concave polygon
      geometry_msgs::Point point;
      costmap2d->mapToWorld(cell_in_polygon.x, cell_in_polygon.y, point.x, point.y);
      if (boost::geometry::within(point, collision_polygon))
      {
        // Protip: uncomment below and 'if (cell_cost > max_cost)' to see evaluated cells
        // boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(costmap2d->getMutex()));
        // costmap2d->setCost(cell_in_polygon.x, cell_in_polygon.y, 100);

        max_cost = cell_cost;
        // Set collision indicator on suspected cell with current cost
        mkCollisionIndicator.scale.z = cell_cost / 255.0;
        mkCollisionIndicator.color.a = cell_cost / 255.0;
        point.z = mkCollisionIndicator.scale.z * 0.5;
        mkCollisionIndicator.pose.position = point;
        if (max_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          break;  // Collision detected, no need to evaluate further
        }
      }
    }
  }
  if (mkCollisionIndicator.scale.z > std::numeric_limits<float>::epsilon())
  {
    mkCollisionIndicator.action = visualization_msgs::Marker::ADD;
  }
  else
  {
    mkCollisionIndicator.action = visualization_msgs::Marker::DELETE;
  }
  mkCollision.markers.push_back(mkCollisionIndicator);

  // Fiddle the polygon into a marker message
  for (const geometry_msgs::Point point : collision_polygon)
  {
    mkCollisionHull.points.push_back(point);
  }

  mkCollision.markers.push_back(mkCollisionFootprint);
  mkCollision.markers.push_back(mkCollisionHull);
  if (n_steps > 0)
  {
    mkCollision.markers.push_back(mkSteps);
    mkCollision.markers.push_back(mkPosesOnPath);
  }
  collision_marker_pub_.publish(mkCollision);

  return max_cost;
}

uint32_t TrackingPidLocalPlanner::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                          const geometry_msgs::TwistStamped& velocity,
                                                          geometry_msgs::TwistStamped& cmd_vel, std::string& message)
{
  if (!initialized_)
  {
    ROS_ERROR("path_tracking_pid has not been initialized, please call initialize() before using this planner");
    active_goal_ = false;
    return mbf_msgs::ExePathResult::NOT_INITIALIZED;
  }
  // TODO(Cesar): Use provided pose and odom
  if (!computeVelocityCommands(cmd_vel.twist))
  {
    active_goal_ = false;
    return mbf_msgs::ExePathResult::FAILURE;
  }
  cmd_vel.header.stamp = ros::Time::now();
  cmd_vel.header.frame_id = base_link_frame_;

  bool moving = std::abs(cmd_vel.twist.linear.x) > VELOCITY_EPS;
  if (cancel_in_progress_)
  {
    if (!moving)
    {
        ROS_INFO("Cancel requested and we now (almost) reached velocity 0: %f", cmd_vel.twist.linear.x);
        cancel_in_progress_ = false;
        active_goal_ = false;
        return mbf_msgs::ExePathResult::CANCELED;
    }
    ROS_INFO_THROTTLE(1.0, "Cancel in progress... remaining x_vel: %f", cmd_vel.twist.linear.x);
    return TrackingPidLocalPlanner::GRACEFULLY_CANCELLING;
  }

  if (!moving && pid_controller_.getVelMaxObstacle() < VELOCITY_EPS)
  {
    active_goal_ = false;
    return mbf_msgs::ExePathResult::BLOCKED_PATH;
  }

  if (isGoalReached())
    active_goal_ = false;
  return mbf_msgs::ExePathResult::SUCCESS;
}

bool TrackingPidLocalPlanner::isGoalReached()
{
  // Return reached boolean, but never succeed when we're preempting
  return pid_controller_.getControllerState().end_reached && !cancel_in_progress_;
}

bool TrackingPidLocalPlanner::isGoalReached(double dist_tolerance, double angle_tolerance)
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
  while (active_goal_)
  {
      r.sleep();
  }
  ROS_INFO("Finished waiting loop, done cancelling");
  return true;
}

void TrackingPidLocalPlanner::curOdomCallback(const nav_msgs::Odometry& odom_msg)
{
  latest_odom_ = odom_msg;
}

void TrackingPidLocalPlanner::velMaxExternalCallback(const std_msgs::Float64& msg)
{
  pid_controller_.setVelMaxExternal(msg.data);
}
}  // namespace path_tracking_pid
