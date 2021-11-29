#include "path_tracking_pid/path_tracking_pid_nav2_controller.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT
// --------- new ----------- 0
using std::placeholders::_1;
// --------- new ----------- 1


namespace path_tracking_pid
{

void PathTrackingPid::cleanup()
{
}

void PathTrackingPid::activate()
{
}

void PathTrackingPid::deactivate()
{
}

void PathTrackingPid::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros; //brings the shared_ptr into the global scope
  costmap_ = costmap_ros_->getCostmap(); //returns pointer to master costmap which recieves updates from all layers
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  // --------------------------- new --------------------------- 0
  node_ = node;

  //To be able to dynamically configure params, add them to the node parameters:
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".l", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".target_x_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".target_end_x_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".target_x_acc", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".target_x_decc", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".abs_minimum_x_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_error_x_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_x_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_yaw_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_yaw_acc", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_turning_radius", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".track_base_link", rclcpp::ParameterValue(true));

  // declare_parameter_if_not_declared( //NOTE: look at this again
  //   node, plugin_name_ + ".init_vel_method", rclcpp::ParameterValue(Zero));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".init_vel_max_diff", rclcpp::ParameterValue(0.5));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".Kp_lat", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".Ki_lat", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".Kd_lat", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".Kp_ang", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".Ki_ang", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".Kd_ang", rclcpp::ParameterValue(0.5));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".feedback_lat", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".feedback_ang", rclcpp::ParameterValue(false));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".feedforward_lat", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".feedforward_ang", rclcpp::ParameterValue(false));

  // declare_parameter_if_not_declared(
  //   node, plugin_name_ + ".controller_debug_enabled", rclcpp::ParameterValue(0.5));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_mpc", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_simulation_sample_time", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_max_error_lat", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_max_fwd_iterations", rclcpp::ParameterValue(10));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_min_x_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".mpc_max_vel_optimization_iterations", rclcpp::ParameterValue(10));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_steering_angle", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_steering_x_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_steering_x_acc", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_steering_yaw_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_steering_yaw_acc", rclcpp::ParameterValue(0.5));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".anti_collision", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_speed_reduction", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".collision_look_ahead_length_offset", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".collision_look_ahead_resolution", rclcpp::ParameterValue(0.5));

  node->get_parameter(plugin_name_ + ".l", config_.l);
  node->get_parameter(plugin_name_ + ".target_x_vel", config_.target_x_vel);
  node->get_parameter(plugin_name_ + ".target_end_x_vel", config_.target_end_x_vel);
  node->get_parameter(plugin_name_ + ".target_x_acc", config_.target_x_acc);
  node->get_parameter(plugin_name_ + ".target_x_decc", config_.target_x_decc);
  node->get_parameter(plugin_name_ + ".abs_minimum_x_vel", config_.abs_minimum_x_vel);
  node->get_parameter(plugin_name_ + ".max_error_x_vel", config_.max_error_x_vel);
  node->get_parameter(plugin_name_ + ".max_x_vel", config_.max_x_vel);
  node->get_parameter(plugin_name_ + ".max_yaw_vel", config_.max_yaw_vel);
  node->get_parameter(plugin_name_ + ".max_yaw_acc", config_.max_yaw_acc);
  node->get_parameter(plugin_name_ + ".min_turning_radius", config_.min_turning_radius);
  node->get_parameter(plugin_name_ + ".track_base_link", config_.track_base_link);

  // node->get_parameter(plugin_name_ + ".init_vel_method", config_.init_vel_method);
  node->get_parameter(plugin_name_ + ".init_vel_max_diff", config_.init_vel_max_diff);

  node->get_parameter(plugin_name_ + ".Kp_lat", config_.Kp_lat);
  node->get_parameter(plugin_name_ + ".Ki_lat", config_.Ki_lat);
  node->get_parameter(plugin_name_ + ".Kd_lat", config_.Kd_lat);
  node->get_parameter(plugin_name_ + ".Kp_ang", config_.Kp_ang);
  node->get_parameter(plugin_name_ + ".Ki_ang", config_.Ki_ang);
  node->get_parameter(plugin_name_ + ".Kd_ang", config_.Kd_ang);

  node->get_parameter(plugin_name_ + ".feedback_lat", config_.feedback_lat);
  node->get_parameter(plugin_name_ + ".feedback_ang", config_.feedback_ang);

  node->get_parameter(plugin_name_ + ".feedforward_lat", config_.feedforward_lat);
  node->get_parameter(plugin_name_ + ".feedforward_ang", config_.feedforward_ang);

  // node->get_parameter(plugin_name_ + ".controller_debug_enabled", controller_debug_enabled);

  node->get_parameter(plugin_name_ + ".use_mpc", config_.use_mpc);
  node->get_parameter(plugin_name_ + ".mpc_simulation_sample_time", config_.mpc_simulation_sample_time);
  node->get_parameter(plugin_name_ + ".mpc_max_error_lat", config_.mpc_max_error_lat);
  node->get_parameter(plugin_name_ + ".mpc_max_fwd_iterations", config_.mpc_max_fwd_iterations);
  node->get_parameter(plugin_name_ + ".mpc_min_x_vel", config_.mpc_min_x_vel);
  node->get_parameter(plugin_name_ + ".mpc_max_vel_optimization_iterations", config_.mpc_max_vel_optimization_iterations);

  node->get_parameter(plugin_name_ + ".max_steering_angle", config_.max_steering_angle);
  node->get_parameter(plugin_name_ + ".max_steering_x_vel", config_.max_steering_x_vel);
  node->get_parameter(plugin_name_ + ".max_steering_x_acc", config_.max_steering_x_acc);
  node->get_parameter(plugin_name_ + ".max_steering_yaw_vel", config_.max_steering_yaw_vel);
  node->get_parameter(plugin_name_ + ".max_steering_yaw_acc", config_.max_steering_yaw_acc);

  node->get_parameter(plugin_name_ + ".anti_collision", config_.anti_collision);
  node->get_parameter(plugin_name_ + ".obstacle_speed_reduction", config_.obstacle_speed_reduction);
  node->get_parameter(plugin_name_ + ".collision_look_ahead_length_offset", config_.collision_look_ahead_length_offset);
  node->get_parameter(plugin_name_ + ".collision_look_ahead_resolution", config_.collision_look_ahead_resolution);

  //From path_tracking_pid_local_planner::initialize:
  pid_controller_.setNodePointer(node_); //NOTE: new

  pid_controller_.setEnabled(false);

  bool holonomic_robot = false;
  pid_controller_.setHolonomic(holonomic_robot);

  bool estimate_pose_angle = false;
  pid_controller_.setEstimatePoseAngle(estimate_pose_angle);

  base_link_frame_ = "base_link";

  use_tricycle_model_= false;
  std::string steered_wheel_frame_ = "steer";

  //Publishers:
  collision_marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("collision_markers", 3);
  marker_pub_           = node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 3);
  path_pub_             = node->create_publisher<nav_msgs::msg::Path>("visualization_path", 1);
  debug_pub_            = node->create_publisher<path_tracking_pid::msg::PidDebug>("debug", 1); //NOTE
  feedback_pub_         = node->create_publisher<path_tracking_pid::msg::PidFeedback>("feedback", 1); //NOTE

  //Subscribers:
  sub_odom_ = node->create_subscription<nav_msgs::msg::Odometry>("odom", 1, std::bind(&PathTrackingPid::curOdomCallback, this, _1));
  // sub_vel_max_external_ = node->create_subscription<std_msgs::Float64>("vel_max", 1, std::bind(&TrackingPidLocalPlanner::velMaxExternalCallback, this, _1));
  // sub_vel_max_external_ = nh.subscribe("vel_max", 1, &TrackingPidLocalPlanner::velMaxExternalCallback, this); //NOTE is this available for Jax?

  map_frame_ = costmap_ros->getGlobalFrameID();
  initialized_ = true;
  // --------------------------- new --------------------------- 1
}

geometry_msgs::msg::TwistStamped PathTrackingPid::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  // Explicitely made useless to avoid compilation error
  (void)speed;
  (void)goal_checker;
  (void)pose;

  geometry_msgs::msg::TwistStamped cmd_vel;

  if (!initialized_)
  {
    RCLCPP_ERROR(node_->get_logger(), "path_tracking_pid has not been initialized, please call initialize() before using this planner");
    active_goal_ = false;
    // return mbf_msgs::ExePathResult::NOT_INITIALIZED;   //NOTE: move_base_flex functionality not available
  }
  // TODO(Cesar): Use provided pose and odom
  if (!computeVelocityCommands(cmd_vel))
  {
    active_goal_ = false;
    // return mbf_msgs::ExePathResult::FAILURE;   //NOTE: move_base_flex functionality not available
  }
  cmd_vel.header.stamp = node_->now();
  cmd_vel.header.frame_id = base_link_frame_;

  bool moving = std::abs(cmd_vel.twist.linear.x) > VELOCITY_EPS;
  if (cancel_in_progress_)
  {
    if (!moving)
    {
        RCLCPP_INFO(node_->get_logger(), "Cancel requested and we now (almost) reached velocity 0: %f", cmd_vel.twist.linear.x);
        cancel_in_progress_ = false;
        active_goal_ = false;
        // return mbf_msgs::ExePathResult::CANCELED;   //NOTE: move_base_flex functionality not available
    }
    // ROS_INFO_THROTTLE(1.0, "Cancel in progress... remaining x_vel: %f", cmd_vel.twist.linear.x); //NOTE: difficulties with migration
    RCLCPP_INFO(node_->get_logger(), "Cancel in progress... remaining x_vel: %f", cmd_vel.twist.linear.x); //NOTE: difficulties with migration

    // return PathTrackingPid::GRACEFULLY_CANCELLING;   //NOTE:
  }

  if (!moving && pid_controller_.getVelMaxObstacle() < VELOCITY_EPS)
  {
    active_goal_ = false;
    // return mbf_msgs::ExePathResult::BLOCKED_PATH;   //NOTE: move_base_flex functionality not available
  }

  if (isGoalReached())
    active_goal_ = false;
  // return mbf_msgs::ExePathResult::SUCCESS;   //NOTE: move_base_flex functionality not available

  return cmd_vel;
}

bool PathTrackingPid::computeVelocityCommands(geometry_msgs::msg::TwistStamped& cmd_vel)
{
  rclcpp::Time now = node_->get_clock()->now();
  if (isZero(prev_time_)) //NOTE: isZero is an adjusted function from the ros::time api
  {
    prev_time_ = now - prev_dt_;  // Initialisation round
  }
  builtin_interfaces::msg::Duration dt = now - prev_time_;
  if (dt.sec == 0 && dt.nanosec == 0)
  {
    // RCLCPP_ERROR_THROTTLE(node_->get_logger(), node_->get_clock()->now, 5, "dt=0 detected, skipping loop(s). Possible overloaded cpu or simulating too fast"); //NOTE: not working like expected
    RCLCPP_ERROR(node_->get_logger(), "dt=0 detected, skipping loop(s). Possible overloaded cpu or simulating too fast");
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.twist.linear.x = pid_controller_.getControllerState().current_x_vel;
    cmd_vel.twist.angular.z = pid_controller_.getControllerState().current_yaw_vel;
    return true;  // False is no use: https://github.com/magazino/move_base_flex/issues/195!
  }
  else if (dt.sec < 0 || dt.sec > DT_MAX)
  {
    RCLCPP_ERROR(node_->get_logger(), "Invalid time increment: %d. Aborting", dt.sec);
    return false;
  }
  try
  {
    RCLCPP_DEBUG(node_->get_logger(), "map_frame: %s, base_link_frame: %s", map_frame_.c_str(), base_link_frame_.c_str());
    tfCurPoseStamped_ = tf_->lookupTransform(map_frame_, base_link_frame_, tf2::TimePointZero);
  }
  catch (tf2::TransformException & ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "Received an exception trying to transform: %s", ex.what());
    active_goal_ = false;
    return false;
  }

  // Handle obstacles
  if (pid_controller_.getConfig().anti_collision)
  {
    auto cost = projectedCollisionCost();

    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      pid_controller_.setVelMaxObstacle(0.0);
    }
    else if (pid_controller_.getConfig().obstacle_speed_reduction)
    {
      double max_vel = pid_controller_.getConfig().max_x_vel;
      double reduction_factor = static_cast<double>(cost) / nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
      double limit = max_vel * (1 - reduction_factor);
      RCLCPP_DEBUG(node_->get_logger(), "Cost: %d, factor: %f, limit: %f", cost, reduction_factor, limit);
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

  // PidConfig pid_debug;
  double eda = 1 / FLT_EPSILON;  // initial guess. Avoids errors in case function returns due to wrong delta_t;
  double progress = 0.0;
  cmd_vel = pid_controller_.update_with_limits(tfCurPoseStamped_.transform, latest_odom_.twist.twist,
                                               dt, &eda, &progress);
                                              //  &pid_debug);    //NOTE!

  path_tracking_pid::msg::PidFeedback feedback_msg;
  rclcpp::Duration eda_rclcpp = rclcpp::Duration(tf2::durationFromSec(eda));

  feedback_msg.eda.sec = eda_rclcpp.seconds();
  feedback_msg.eda.nanosec = eda_rclcpp.nanoseconds();
  feedback_msg.progress = progress;
  feedback_pub_->publish(feedback_msg);

  if (cancel_requested_)
  {
    // path_tracking_pid::PidConfig config = pid_controller_.getConfig();
    config_.target_x_vel = 0.0;
    config_.target_end_x_vel = 0.0;
    cancel_requested_ =  false;
  }


  if (controller_debug_enabled_)
  {
    // debug_pub_->publish(pid_debug); //NOTE!
    visualization_msgs::msg::Marker mkCurPose, mkControlPose, mkGoalPose, mkPosOnPlan;

    // configure rviz visualization
    mkCurPose.header.frame_id = mkControlPose.header.frame_id = map_frame_;
    mkGoalPose.header.frame_id = mkPosOnPlan.header.frame_id = map_frame_;
    mkCurPose.header.stamp = mkControlPose.header.stamp = node_->now();
    mkGoalPose.header.stamp = mkPosOnPlan.header.stamp = node_->now();
    mkCurPose.ns = "axle point";
    mkControlPose.ns = "control point";
    mkGoalPose.ns = "goal point";
    mkPosOnPlan.ns = "plan point";
    mkCurPose.action = mkControlPose.action = visualization_msgs::msg::Marker::ADD;
    mkGoalPose.action = mkPosOnPlan.action = visualization_msgs::msg::Marker::ADD;
    mkCurPose.pose.orientation.w = mkControlPose.pose.orientation.w = 1.0;
    mkGoalPose.pose.orientation.w = mkPosOnPlan.pose.orientation.w = 1.0;
    mkCurPose.id = __COUNTER__;  // id has to be unique, so using a compile-time counter :)
    mkControlPose.id = __COUNTER__;
    mkGoalPose.id = __COUNTER__;
    mkPosOnPlan.id = __COUNTER__;
    mkCurPose.type = mkControlPose.type = visualization_msgs::msg::Marker::POINTS;
    mkGoalPose.type = mkPosOnPlan.type = visualization_msgs::msg::Marker::POINTS;
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

    geometry_msgs::msg::Point p;
    std_msgs::msg::ColorRGBA color;
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

    marker_pub_->publish(mkCurPose);
    marker_pub_->publish(mkControlPose);
    marker_pub_->publish(mkGoalPose);
    marker_pub_->publish(mkPosOnPlan);
  }

  prev_time_ = now;
  prev_dt_ = dt;  // Store last known valid dt for next cycles (https://github.com/magazino/move_base_flex/issues/195)

  return true;
}

void PathTrackingPid::setPlan(const nav_msgs::msg::Path & path)
{
  if (!initialized_)
  {
    RCLCPP_ERROR(node_->get_logger(), "path_tracking_pid has not been initialized, please call initialize() before using this planner");
  }

  std::string path_frame = path.header.frame_id;
  RCLCPP_DEBUG(node_->get_logger(), "TrackingPidLocalPlanner::setPlan(%zu)", path.poses.size());
  RCLCPP_DEBUG(node_->get_logger(), "Plan is defined in frame '%s'", path_frame.c_str());

  global_plan_ = path.poses;

  /* If frame of received plan is not equal to mbf-map_frame, translate first */
  if (map_frame_.compare(path_frame))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Transforming plan since my global_frame = '%s' and my plan is in frame: '%s'", map_frame_.c_str(), path_frame.c_str());

    geometry_msgs::msg::TransformStamped tf_transform;
    tf_transform = tf_->lookupTransform(map_frame_, path_frame, tf2::TimePointZero);

    // Check alignment, when path-frame is severly mis-aligned show error
    double yaw, pitch, roll;
    tf2::getEulerYPR(tf_transform.transform.rotation, yaw, pitch, roll);
    if (std::fabs(pitch) > MAP_PARALLEL_THRESH || std::fabs(roll) > MAP_PARALLEL_THRESH)
    {
      RCLCPP_ERROR(node_->get_logger(), "Path is given in %s frame which is severly mis-aligned with our map-frame: %s", path_frame.c_str(), map_frame_.c_str());
    }
    for (auto& pose_stamped : global_plan_)
    // for (auto pose_stamped: global_plan)
    {
      tf2::doTransform(pose_stamped, pose_stamped, tf_transform);
      // tf2::doTransform(pose_stamped.pose, pose_stamped.pose, tf_transform);
      pose_stamped.header.frame_id = map_frame_;

      // 'Project' plan by removing z-component
      pose_stamped.pose.position.z = 0.0;
    }
  }

  if (controller_debug_enabled_)
  {
    received_path_.header = global_plan_.at(0).header;
    received_path_.poses = global_plan_;
    path_pub_->publish(received_path_);
  }

  try
  {
    RCLCPP_DEBUG(node_->get_logger(), "map_frame: %s, plan_frame: %s, base_link_frame: %s", map_frame_.c_str(), path_frame.c_str(), base_link_frame_.c_str());
    tfCurPoseStamped_ = tf_->lookupTransform(map_frame_, base_link_frame_, tf2::TimePointZero);
  }
  catch (tf2::TransformException & ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "Received an exception trying to transform: %s", ex.what());
  }

  // Feasability check, but only when not resuming with odom-vel
  if (pid_controller_.getConfig().init_vel_method != Pid_Odom && //NOTE
      pid_controller_.getConfig().init_vel_max_diff >= 0.0 &&
      std::abs(latest_odom_.twist.twist.linear.x - pid_controller_.getControllerState().current_x_vel) >
        pid_controller_.getConfig().init_vel_max_diff)
  {
    RCLCPP_ERROR(node_->get_logger(),"Significant diff between odom (%f) and controller_state (%f) detected. Aborting!",
              latest_odom_.twist.twist.linear.x, pid_controller_.getControllerState().current_x_vel);
  }

  if (use_tricycle_model_)
  {
    try
    {
      RCLCPP_DEBUG(node_->get_logger(), "base_link_frame: %s, steered_wheel_frame: %s", base_link_frame_.c_str(), steered_wheel_frame_.c_str());
      tf_base_to_steered_wheel_stamped_ = tf_->lookupTransform(base_link_frame_, steered_wheel_frame_, tf2::TimePointZero);
    }
    catch (tf2::TransformException & ex)
    {
      RCLCPP_ERROR(node_->get_logger(), "Received an exception trying to transform: %s", ex.what());
      RCLCPP_ERROR(node_->get_logger(), "Invalid transformation from base_link_frame to steered_wheel_frame. Tricycle model will be disabled");
      use_tricycle_model_ = false;
    }

    pid_controller_.setTricycleModel(use_tricycle_model_, tf_base_to_steered_wheel_stamped_.transform);

    // TODO(clopez): subscribe to steered wheel odom
    geometry_msgs::msg::Twist steering_odom_twist;
    pid_controller_.setPlan(tfCurPoseStamped_.transform, latest_odom_.twist.twist, tf_base_to_steered_wheel_stamped_.transform, steering_odom_twist, global_plan_);
  }
  else
  {
    pid_controller_.setPlan(tfCurPoseStamped_.transform, latest_odom_.twist.twist, global_plan_);

  }

  pid_controller_.setEnabled(true);
  active_goal_ = true;
  prev_time_ = node_->get_clock()->now();
}

uint8_t PathTrackingPid::projectedCollisionCost()
{
  // configure rviz visualization
  visualization_msgs::msg::Marker mkSteps;
  mkSteps.header.frame_id = map_frame_;
  mkSteps.header.stamp = node_->get_clock()->now();
  mkSteps.ns = "extrapolated poses";
  mkSteps.action = visualization_msgs::msg::Marker::ADD;
  mkSteps.pose.orientation.w = 1.0;
  mkSteps.id = __COUNTER__;
  mkSteps.type = visualization_msgs::msg::Marker::POINTS;
  mkSteps.scale.x = 0.5;
  mkSteps.scale.y = 0.5;
  mkSteps.color.r = 1.0;
  mkSteps.color.g = 0.5;
  mkSteps.color.a = 1.0;

  visualization_msgs::msg::Marker mkPosesOnPath;
  mkPosesOnPath.header.frame_id = map_frame_;
  mkPosesOnPath.header.stamp = node_->get_clock()->now();
  mkPosesOnPath.ns = "goal poses on path";
  mkPosesOnPath.action = visualization_msgs::msg::Marker::ADD;
  mkPosesOnPath.pose.orientation.w = 1.0;
  mkPosesOnPath.id = __COUNTER__;
  mkPosesOnPath.type = visualization_msgs::msg::Marker::POINTS;
  mkPosesOnPath.scale.x = 0.5;
  mkPosesOnPath.scale.y = 0.5;
  mkPosesOnPath.color.r = 1.0;
  mkPosesOnPath.color.g = 1.0;
  mkPosesOnPath.color.a = 1.0;

  visualization_msgs::msg::Marker mkCollisionFootprint;
  mkCollisionFootprint.header.frame_id = map_frame_;
  mkCollisionFootprint.header.stamp = node_->get_clock()->now();
  mkCollisionFootprint.ns = "Collision footprint";
  mkCollisionFootprint.action = visualization_msgs::msg::Marker::ADD;
  mkCollisionFootprint.pose.orientation.w = 1.0;
  mkCollisionFootprint.id = __COUNTER__;
  mkCollisionFootprint.type = visualization_msgs::msg::Marker::LINE_LIST;
  mkCollisionFootprint.scale.x = 0.1;
  mkCollisionFootprint.color.b = 1.0;
  mkCollisionFootprint.color.a = 0.3;

  visualization_msgs::msg::Marker mkCollisionHull;
  mkCollisionHull.header.frame_id = map_frame_;
  mkCollisionHull.header.stamp = node_->get_clock()->now();
  mkCollisionHull.ns = "Collision polygon";
  mkCollisionHull.action = visualization_msgs::msg::Marker::ADD;
  mkCollisionHull.pose.orientation.w = 1.0;
  mkCollisionHull.id = __COUNTER__;
  mkCollisionHull.type = visualization_msgs::msg::Marker::LINE_STRIP;
  mkCollisionHull.scale.x = 0.2;
  mkCollisionHull.color.r = 1.0;
  mkCollisionHull.color.a = 0.3;

  visualization_msgs::msg::Marker mkCollisionIndicator;
  mkCollisionIndicator.header.frame_id = map_frame_;
  mkCollisionIndicator.header.stamp = node_->get_clock()->now();
  mkCollisionIndicator.ns = "Collision object";
  mkCollisionIndicator.pose.orientation.w = 1.0;
  mkCollisionIndicator.id = __COUNTER__;
  mkCollisionIndicator.type = visualization_msgs::msg::Marker::CYLINDER;
  mkCollisionIndicator.scale.x = 0.5;
  mkCollisionIndicator.scale.y = 0.5;
  mkCollisionIndicator.color.r = 1.0;
  mkCollisionIndicator.color.a = 0.0;
  visualization_msgs::msg::MarkerArray mkCollision;

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
  geometry_msgs::msg::Transform current_tf = tfCurPoseStamped_.transform;

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
    geometry_msgs::msg::Point mkStep;
    toMsg(next_straight_step_tf.getOrigin(), mkStep);
    mkSteps.points.push_back(mkStep);
    geometry_msgs::msg::Point mkPointOnPath;
    toMsg(projected_step_tf.getOrigin(), mkPointOnPath);
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
    std::vector<geometry_msgs::msg::Point> footprint;
    nav2_costmap_2d::transformFootprint(x, y, yaw, costmap_ros_->getRobotFootprint(), footprint);

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
    geometry_msgs::msg::Point previous_point = footprint.back();
    for (const auto& point : footprint)
    {
      mkCollisionFootprint.points.push_back(previous_point);
      mkCollisionFootprint.points.push_back(point);
      previous_point = point;
    }
  }

  // Create a convex hull so we can use costmap2d->convexFillCells
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_ros_->getCostmap();
  polygon_t collision_polygon_hull;
  boost::geometry::convex_hull(collision_polygon, collision_polygon_hull);
  std::vector<nav2_costmap_2d::MapLocation> collision_polygon_hull_map;

  // Convert to map coordinates
  for (const auto& point : collision_polygon_hull)
  {
    int map_x, map_y;
    costmap2d->worldToMapEnforceBounds(point.x, point.y, map_x, map_y);
    nav2_costmap_2d::MapLocation map_point{static_cast<uint>(map_x), static_cast<uint>(map_y)};
    collision_polygon_hull_map.push_back(map_point);
  }

  // Get the relevant cells
  std::vector<nav2_costmap_2d::MapLocation> cells_in_polygon;
  costmap2d->convexFillCells(collision_polygon_hull_map, cells_in_polygon);

  // Get the max cost inside the concave polygon
  uint8_t max_cost = 0.0;
  for (const auto& cell_in_polygon : cells_in_polygon)
  {
    // Cost checker is cheaper than polygon checker, so lets do that first
    uint8_t cell_cost = costmap2d->getCost(cell_in_polygon.x, cell_in_polygon.y);
    if (cell_cost > max_cost && cell_cost != nav2_costmap_2d::NO_INFORMATION)
    {
      // Check if in concave polygon
      geometry_msgs::msg::Point point;
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
        if (max_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          break;  // Collision detected, no need to evaluate further
        }
      }
    }
  }
  if (mkCollisionIndicator.scale.z > std::numeric_limits<float>::epsilon())
  {
    mkCollisionIndicator.action = visualization_msgs::msg::Marker::ADD;
  }
  else
  {
    mkCollisionIndicator.action = visualization_msgs::msg::Marker::DELETE;
  }
  mkCollision.markers.push_back(mkCollisionIndicator);

  // Fiddle the polygon into a marker message
  for (const geometry_msgs::msg::Point point : collision_polygon)
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
  collision_marker_pub_->publish(mkCollision);

  return max_cost;
}

bool PathTrackingPid::isGoalReached()
{
  // Return reached boolean, but never succeed when we're preempting
  return pid_controller_.getControllerState().end_reached && !cancel_in_progress_;
}

bool PathTrackingPid::isGoalReached(double dist_tolerance, double angle_tolerance)
{
  //NOTE: avoid not being used error
  dist_tolerance = dist_tolerance + 0;
  angle_tolerance = angle_tolerance + 0;
  return isGoalReached();
}


void PathTrackingPid::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  // for now explicitely make an useless conversion to avoid compilation error
  (void)speed_limit;
  (void)percentage;
}

void PathTrackingPid::curOdomCallback(const nav_msgs::msg::Odometry& odom_msg)
{
  latest_odom_ = odom_msg;
}

// void PathTrackingPid::velMaxExternalCallback(const std_msgs::Float64& msg)
// {
//   pid_controller_.setVelMaxExternal(msg.data);
// }

}  // namespace path_tracking_pid

#include "pluginlib/class_list_macros.hpp"
// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  path_tracking_pid::PathTrackingPid,
  nav2_core::Controller)
