#include "path_tracking_pid/controller.hpp"

namespace path_tracking_pid
{
// ----------- new ----------- 0
void Controller::setNodePointer(nav2_util::LifecycleNode::SharedPtr nodePointer){
  node_ = nodePointer;
}
// ----------- new ----------- 1

void Controller::setHolonomic(bool holonomic)
{
  // Set configuration parameters
  if(holonomic){
  RCLCPP_WARN(node_->get_logger(), "Holonomic mode is unmaintained. Expect bugs with y-direction"); //NOTE: ros_warn_condition fix
  }
  holonomic_robot_enable_ = holonomic;
}

void Controller::setEstimatePoseAngle(bool estimate_pose_angle)
{
  // Set configuration parameters
  estimate_pose_angle_enabled_ = estimate_pose_angle;
}

void Controller::setTricycleModel(bool tricycle_model_enabled, geometry_msgs::msg::Transform tf_base_to_steered_wheel)
{
  // Set tricycle model
  use_tricycle_model_ = tricycle_model_enabled;
  tf_base_to_steered_wheel_ = tf_base_to_steered_wheel;
  double wheel_x = tf_base_to_steered_wheel_.translation.x;
  double wheel_y = tf_base_to_steered_wheel_.translation.y;

  double distance_base_to_steered_wheel = hypot(wheel_x, wheel_y);
  double wheel_theta = atan2(wheel_y, wheel_x);
  inverse_kinematics_matrix_[0][0] = 1;
  inverse_kinematics_matrix_[0][1] = - distance_base_to_steered_wheel * sin(wheel_theta);
  inverse_kinematics_matrix_[1][0] = 0;
  inverse_kinematics_matrix_[1][1] = - distance_base_to_steered_wheel * cos(wheel_theta);


  double determinant =    inverse_kinematics_matrix_[0][0] * inverse_kinematics_matrix_[1][1]
                        - inverse_kinematics_matrix_[0][1] * inverse_kinematics_matrix_[1][0];

  if (determinant == 0)
  {
    RCLCPP_ERROR(node_->get_logger(),"Steered wheel at base_link. Invalid for tricycle model, it will be disabled.");
    use_tricycle_model_ = false;
    return;
  }

  forward_kinematics_matrix_[0][0] =  inverse_kinematics_matrix_[1][1] / determinant;
  forward_kinematics_matrix_[0][1] = -inverse_kinematics_matrix_[0][1] / determinant;
  forward_kinematics_matrix_[1][0] = -inverse_kinematics_matrix_[1][0] / determinant;
  forward_kinematics_matrix_[1][1] =  inverse_kinematics_matrix_[0][0] / determinant;

  controller_state_.previous_steering_angle = tf2::getYaw(tf_base_to_steered_wheel_.rotation);
}

geometry_msgs::msg::Twist Controller::computeTricycleModelForwardKinematics(double x_vel, double steering_angle)
{
    geometry_msgs::msg::Twist estimated_base_twist;
    double x_alpha = x_vel*cos(steering_angle);
    double y_alpha = x_vel*sin(steering_angle);

    estimated_base_twist.linear.x = forward_kinematics_matrix_[0][0]*x_alpha + forward_kinematics_matrix_[0][1]*y_alpha;
    estimated_base_twist.angular.z =
        forward_kinematics_matrix_[1][0]*x_alpha + forward_kinematics_matrix_[1][1]*y_alpha;

    return estimated_base_twist;
}

TricycleSteeringCmdVel Controller::computeTricycleModelInverseKinematics(geometry_msgs::msg::Twist cmd_vel)
{
    TricycleSteeringCmdVel steering_cmd_vel;
    double x_alpha =  inverse_kinematics_matrix_[0][0]*cmd_vel.linear.x
                    + inverse_kinematics_matrix_[0][1]*cmd_vel.angular.z;
    double y_alpha =  inverse_kinematics_matrix_[1][0]*cmd_vel.linear.x
                    + inverse_kinematics_matrix_[1][1]*cmd_vel.angular.z;

    steering_cmd_vel.steering_angle = atan2(y_alpha, x_alpha);
    steering_cmd_vel.speed = hypot(x_alpha, y_alpha);

    return steering_cmd_vel;
}

void Controller::setPlan(geometry_msgs::msg::Transform current_tf, geometry_msgs::msg::Twist odom_twist, const std::vector<geometry_msgs::msg::PoseStamped>& global_plan)
{
  RCLCPP_DEBUG(node_->get_logger(), "TrackingPidLocalPlanner::setPlan(%d)", (int)global_plan.size());
  RCLCPP_DEBUG(node_->get_logger(), "Plan is defined in frame '%s'", global_plan.at(0).header.frame_id.c_str());

  tf2::Transform current_tf2;
  tf2::convert(current_tf, current_tf2);

  /* Minimal sanity check */
  global_plan_tf_.clear();
  tf2::Transform transform;
  tf2::fromMsg(global_plan[0].pose, transform);
  global_plan_tf_.push_back(transform);
  // For now do not allow repeated points or in-place rotation
  // To allow that the way the progress is checked and the interpolation is done needs to be changed
  // Also check if points suddenly go in the opposite direction, this could lead to deadlocks
  for (unsigned int pose_idx = 1; pose_idx < global_plan.size() - 1; ++pose_idx)
  {
    auto prev_pose = global_plan[pose_idx - 1].pose;
    auto pose = global_plan[pose_idx].pose;
    auto next_pose = global_plan[pose_idx + 1].pose;
    // Check if the angle of the pose is obtuse, otherwise warn and ignore this pose
    // We check using Pythagorean theorem: if c*c > (a*a + b*b) it is an obtuse angle and thus we can follow it
    double a_squared = distSquared(prev_pose, pose);
    double b_squared = distSquared(pose, next_pose);
    double c_squared = distSquared(prev_pose, next_pose);
    if (c_squared > (a_squared + b_squared))
    {
      tf2::fromMsg(pose, transform);
      global_plan_tf_.push_back(transform);
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(),"Pose %d of path is not used since it is not in the expected direction of the path!", pose_idx);
    }
  }
  // Add last pose as we didn't evaluate that one
  tf2::Transform last_transform;
  tf2::fromMsg(global_plan.back().pose, last_transform);
  global_plan_tf_.push_back(last_transform);

  if (!track_base_link_enabled_)
  {
    // Add carrot length to plan using goal pose (we assume the last pose contains correct angle)
    tf2::Transform carrotTF;
    carrotTF.setOrigin(tf2::Vector3(l_, 0.0, 0.0));
    global_plan_tf_.push_back(last_transform * carrotTF);
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
  for (int idx_path = global_plan_tf_.size() - 2; idx_path >= 0; --idx_path)
  {
    /* Get distance to segment to determine if this is the segment to start at */
    dist_to_segment = distToSegmentSquared(current_tf2, global_plan_tf_[idx_path], global_plan_tf_[idx_path + 1]);
    // Calculate 3D distance, since current_tf2 might have significant z-offset and roll/pitch values w.r.t. path-pose
    // When not doing this, we're brutely projecting in robot's frame and might snap to another segment!
    if (dist_to_segment < minimum_distance_to_path)
    {
      minimum_distance_to_path = dist_to_segment;
      controller_state_.current_global_plan_index = idx_path;
    }

    /* Create distance and turning radius vectors once for usage later */
    deltaPlan = global_plan_tf_[idx_path].inverseTimes(global_plan_tf_[idx_path + 1]);
    double dpX = deltaPlan.getOrigin().x();
    double dpY = deltaPlan.getOrigin().y();
    iterative_dist_to_goal += hypot(dpX, dpY);
    distance_to_goal_vector_[idx_path] = iterative_dist_to_goal;
    // compute turning radius based on trigonometric analysis
    // radius such that next pose is connected from current pose with a semi-circle
    double dpXY2 = dpY*dpY + dpX*dpX;
    if (dpXY2 < FLT_EPSILON)
    {
      turning_radius_inv_vector_[idx_path] = std::numeric_limits<double>::infinity();
    }
    else
    {
      //  0.5*dpY*( 1 + dpX*dpX/(dpY*dPY) );
      // turning_radius_vector[idx_path] = 0.5*(1/dpY)*( dpY*dpY + dpX*dpX );
      turning_radius_inv_vector_[idx_path] = 2*dpY/dpXY2;
    }
    RCLCPP_DEBUG(node_->get_logger(), "turning_radius_inv_vector[%d] = %f", idx_path, turning_radius_inv_vector_[idx_path]);
  }

  // Set initial velocity
  switch (local_config_.init_vel_method)
  {
    case Pid_Zero:
      reset();
      break;
    case Pid_Odom:
      reset();
      controller_state_.current_x_vel = odom_twist.linear.x;
      controller_state_.current_yaw_vel = odom_twist.angular.z;
      RCLCPP_INFO(node_->get_logger(), "Resuming on odom velocity x: %f, yaw: %f", odom_twist.linear.x, odom_twist.angular.z);
      break;
    default:
      RCLCPP_DEBUG(node_->get_logger(), "Internal controller_state stays valid");
      break;
  }

  // When velocity error is too big reset current_x_vel
  if (fabs(odom_twist.linear.x - controller_state_.current_x_vel) > max_error_x_vel_)
  {
    // TODO(clopez/mcfurry/nobleo): Give feedback to higher level software here
    RCLCPP_WARN(node_->get_logger(), "Large control error. Current_x_vel %f / odometry %f",
            controller_state_.current_x_vel, odom_twist.linear.x);
  }
  controller_state_.end_phase_enabled = false;
  controller_state_.end_reached = false;
}

void Controller::setPlan(geometry_msgs::msg::Transform current_tf, geometry_msgs::msg::Twist odom_twist,
                          geometry_msgs::msg::Transform tf_base_to_steered_wheel, geometry_msgs::msg::Twist steering_odom_twist,
                          const std::vector<geometry_msgs::msg::PoseStamped>& global_plan)
{
  steering_odom_twist = steering_odom_twist; //NOTE: to prevent unused error
  setPlan(current_tf, odom_twist, global_plan);
  controller_state_.previous_steering_angle = tf2::getYaw(tf_base_to_steered_wheel.rotation);
  // TODO(clopez) use steering_odom_twist to check if setpoint is being followed
}

// https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
// TODO(Cesar): expand to 3 dimensions
double Controller::distSquared(const tf2::Transform& pose_v, const tf2::Transform& pose_w) const
{
  // Returns square distance between 2 points
  return (pose_v.getOrigin().x() - pose_w.getOrigin().x()) * (pose_v.getOrigin().x() - pose_w.getOrigin().x()) +
         (pose_v.getOrigin().y() - pose_w.getOrigin().y()) * (pose_v.getOrigin().y() - pose_w.getOrigin().y());
}

double Controller::distSquared(const geometry_msgs::msg::Pose& pose_v, const geometry_msgs::msg::Pose& pose_w) const
{
  // Returns square distance between 2 points
  return (pose_v.position.x - pose_w.position.x) * (pose_v.position.x - pose_w.position.x) +
         (pose_v.position.y - pose_w.position.y) * (pose_v.position.y - pose_w.position.y);
}

void Controller::distToSegmentSquared(
  const tf2::Transform& pose_p, const tf2::Transform& pose_v, const tf2::Transform& pose_w,
  tf2::Transform& pose_projection, double& distance_to_p, double& distance_to_w)
{
  double l2 = distSquared(pose_v, pose_w);
  if (l2 == 0)
  {
    pose_projection = pose_w;
    distance_to_w = 0.0;
    distance_to_p = distSquared(pose_p, pose_w);
  }
  else
  {
    double t = ((pose_p.getOrigin().x() - pose_v.getOrigin().x()) * (pose_w.getOrigin().x() - pose_v.getOrigin().x()) +
                (pose_p.getOrigin().y() - pose_v.getOrigin().y()) * (pose_w.getOrigin().y() - pose_v.getOrigin().y())) /
               l2;
    t = fmax(0.0, fmin(1.0, t));
    pose_projection.setOrigin(
        tf2::Vector3(pose_v.getOrigin().x() + t * (pose_w.getOrigin().x() - pose_v.getOrigin().x()),
                    pose_v.getOrigin().y() + t * (pose_w.getOrigin().y() - pose_v.getOrigin().y()), 0.0));
    double yaw_projection = tf2::getYaw(pose_v.getRotation());  // get yaw of the first vector + t *
                                                               // (tf2::getYaw(pose_w.getRotation()) -
                                                               // tf2::getYaw(pose_v.getRotation()));
    tf2::Quaternion pose_quaternion;
    if (estimate_pose_angle_enabled_)
    {
      pose_quaternion.setRPY(0.0, 0.0, atan2(pose_w.getOrigin().y() - pose_v.getOrigin().y(),
                                             pose_w.getOrigin().x() - pose_v.getOrigin().x()));
    }
    else
    {
      pose_quaternion.setRPY(0.0, 0.0, yaw_projection);
    }

    pose_projection.setRotation(pose_quaternion);
    distance_to_w = sqrt(distSquared(pose_projection, pose_w));
    distance_to_p = distSquared(pose_p, pose_projection);
  }
}

tf2::Transform Controller::findPositionOnPlan(const geometry_msgs::msg::Transform current_tf,
                                              ControllerState* controller_state_ptr,
                                              size_t &path_pose_idx)
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
  for (unsigned int idx_path = controller_state_ptr->current_global_plan_index; idx_path < global_plan_tf_.size(); idx_path++)
  {
    error = current_tf2.inverseTimes(global_plan_tf_[idx_path]);
    // Calculate 3D distance, since current_tf2 might have significant z-offset and roll/pitch values w.r.t. path-pose
    // When not doing this, we're brutely projecting in robot's frame and might snap to another segment!
    distance_to_path = hypot(error.getOrigin().x(), error.getOrigin().y(), error.getOrigin().z());

    if (distance_to_path <= minimum_distance_to_path)
    {
      minimum_distance_to_path = distance_to_path;
      controller_state_ptr->current_global_plan_index = idx_path;
    }
    else
    {
      break;
    }
  }

  // Then look backwards
  for (int idx_path = controller_state_ptr->current_global_plan_index - 1; idx_path >= 0; idx_path--)
  {
    error = current_tf2.inverseTimes(global_plan_tf_[idx_path]);
    // Calculate 3D distance, since current_tf2 might have significant z-offset and roll/pitch values w.r.t. path-pose
    // When not doing this, we're brutely projecting in robot's frame and might snap to another segment!
    distance_to_path = hypot(error.getOrigin().x(), error.getOrigin().y(), error.getOrigin().z());

    if (distance_to_path < minimum_distance_to_path)
    {
      minimum_distance_to_path = distance_to_path;
      controller_state_ptr->current_global_plan_index = idx_path;
    }
    else
    {
      break;
    }
  }
  RCLCPP_DEBUG(node_->get_logger(), "progress: %lu of %lu", controller_state_ptr->current_global_plan_index, global_plan_tf_.size()-1);
  // To finalize, compute the indexes of the start and end points of
  // the closest line segment to the current carrot
  tf2::Transform current_goal_local;
  current_goal_local = global_plan_tf_[controller_state_ptr->current_global_plan_index];

  tf2::Transform pose_projection_ahead;
  tf2::Transform pose_projection_behind;
  double distance2_to_line_ahead;
  double distance2_to_line_behind;
  double distance_to_end_line_ahead;
  double distance_to_end_line_behind;
  if (controller_state_ptr->current_global_plan_index == 0)
  {
    distToSegmentSquared(current_tf2, global_plan_tf_[0], global_plan_tf_[1],
                         pose_projection_ahead, distance2_to_line_ahead, distance_to_end_line_ahead);
    current_goal_local = pose_projection_ahead;
    distance_to_goal_ = distance_to_goal_vector_[1] + distance_to_end_line_ahead;
    controller_state_ptr->last_visited_pose_index = 0;
    path_pose_idx = controller_state_ptr->current_global_plan_index;
  }
  else if (controller_state_ptr->current_global_plan_index == global_plan_tf_.size() - 1)
  {
    distToSegmentSquared(current_tf2, global_plan_tf_[controller_state_ptr->current_global_plan_index - 1],
                         global_plan_tf_[controller_state_ptr->current_global_plan_index],
                         pose_projection_behind, distance2_to_line_behind, distance_to_end_line_behind);
    current_goal_local = pose_projection_behind;
    distance_to_goal_ = distance_to_end_line_behind;
    controller_state_ptr->last_visited_pose_index = global_plan_tf_.size() - 2;
    path_pose_idx = controller_state_ptr->current_global_plan_index - 1;
  }
  else
  {
    distToSegmentSquared(current_tf2, global_plan_tf_[controller_state_ptr->current_global_plan_index],
                         global_plan_tf_[controller_state_ptr->current_global_plan_index + 1],
                         pose_projection_ahead, distance2_to_line_ahead, distance_to_end_line_ahead);
    distToSegmentSquared(current_tf2, global_plan_tf_[controller_state_ptr->current_global_plan_index - 1],
                         global_plan_tf_[controller_state_ptr->current_global_plan_index],
                         pose_projection_behind, distance2_to_line_behind, distance_to_end_line_behind);

    if (distance2_to_line_ahead < distance2_to_line_behind)
    {
      current_goal_local = pose_projection_ahead;
      distance_to_goal_ =
          distance_to_goal_vector_[controller_state_ptr->current_global_plan_index + 1] + distance_to_end_line_ahead;
      controller_state_ptr->last_visited_pose_index = controller_state_ptr->current_global_plan_index;
    }
    else
    {
      current_goal_local = pose_projection_behind;
      distance_to_goal_ =
          distance_to_goal_vector_[controller_state_ptr->current_global_plan_index] + distance_to_end_line_behind;
      controller_state_ptr->last_visited_pose_index = controller_state_ptr->current_global_plan_index - 1;
    }
    path_pose_idx = controller_state_ptr->current_global_plan_index;
  }
  return current_goal_local;
}

geometry_msgs::msg::TwistStamped Controller::update(const double target_x_vel,
                                        const double target_end_x_vel,
                                        const geometry_msgs::msg::Transform current_tf,
                                        const geometry_msgs::msg::Twist odom_twist,
                                        const rclcpp::Duration dt,
                                        double* eda, double* progress,
                                        path_tracking_pid::msg::PidDebug* pid_debug
                                        )
{
  double current_x_vel = controller_state_.current_x_vel;
  const double current_yaw_vel = controller_state_.current_yaw_vel;

  // Compute location of the point to be controlled
  double theda_rp = tf2::getYaw(current_tf.rotation);
  tf2::Vector3 current_with_carrot_origin;
  current_with_carrot_origin.setX(current_tf.translation.x + l_ * cos(theda_rp));
  current_with_carrot_origin.setY(current_tf.translation.y + l_ * sin(theda_rp));
  current_with_carrot_origin.setZ(0);
  RCLCPP_DEBUG(node_->get_logger(), "l_ is: %f", l_);
  RCLCPP_DEBUG(node_->get_logger(), "theda_rp is: %f", theda_rp);

  current_with_carrot_.setOrigin(current_with_carrot_origin);
  tf2::Quaternion cur_rot(current_tf.rotation.x, current_tf.rotation.y, current_tf.rotation.z, current_tf.rotation.w);
  current_with_carrot_.setRotation(cur_rot);

  RCLCPP_DEBUG(node_->get_logger(), "current_with_carrot_.x is: %f", current_with_carrot_.getOrigin().x());
  RCLCPP_DEBUG(node_->get_logger(), "current_with_carrot_.y is: %f", current_with_carrot_.getOrigin().y());


  size_t path_pose_idx;
  if (track_base_link_enabled_)
  {
    // Find closes robot position to path and then project carrot on goal
    current_pos_on_plan_ = current_goal_ = findPositionOnPlan(current_tf, &controller_state_, path_pose_idx);
    // To track the base link the goal is then transform to the control point goal
    double theda_rp = tf2::getYaw(current_goal_.getRotation());
    tf2::Vector3 newControlOrigin;
    newControlOrigin.setX(current_goal_.getOrigin().x() + l_ * cos(theda_rp));
    newControlOrigin.setY(current_goal_.getOrigin().y() + l_ * sin(theda_rp));
    newControlOrigin.setZ(0);
    current_goal_.setOrigin(newControlOrigin);
  }
  else
  {
    // find position of current position with projected carrot
    geometry_msgs::msg::Transform current_with_carrot_g;
    tf2::convert(current_with_carrot_, current_with_carrot_g);
    current_pos_on_plan_ = current_goal_ = findPositionOnPlan(current_with_carrot_g, &controller_state_, path_pose_idx);
  }

  *progress = 1.0 - distance_to_goal_ / distance_to_goal_vector_[0];

  // Compute errorPose between controlPose and currentGoalPose
  tf2::Transform error = current_with_carrot_.inverseTimes(current_goal_);

  //***** Feedback control *****//
  if (!((Kp_lat_ <= 0. && Ki_lat_ <= 0. && Kd_lat_ <= 0.) ||
        (Kp_lat_ >= 0. && Ki_lat_ >= 0. && Kd_lat_ >= 0.)))  // All 3 gains should have the same sign
    RCLCPP_WARN(node_->get_logger(), "All three gains (Kp, Ki, Kd) should have the same sign for stability.");
  if (!((Kp_ang_ <= 0. && Ki_ang_ <= 0. && Kd_ang_ <= 0.) ||
        (Kp_ang_ >= 0. && Ki_ang_ >= 0. && Kd_ang_ >= 0.)))  // All 3 gains should have the same sign
    RCLCPP_WARN(node_->get_logger(), "All three gains (Kp, Ki, Kd) should have the same sign for stability.");

  controller_state_.error_lat.at(2) = controller_state_.error_lat.at(1);
  controller_state_.error_lat.at(1) = controller_state_.error_lat.at(0);
  controller_state_.error_lat.at(0) = error.getOrigin().y();  // Current error goes to slot 0
  controller_state_.error_ang.at(2) = controller_state_.error_ang.at(1);
  controller_state_.error_ang.at(1) = controller_state_.error_ang.at(0);
  controller_state_.error_ang.at(0) =
      angles::normalize_angle(tf2::getYaw(error.getRotation()));  // Current error goes to slot 0

  // tracking error for diagnostic purposes
  // Transform current pose into local-path-frame to get tracked-frame-error
  tf2::Quaternion path_quat;
  path_quat.setEuler(0.0, 0.0,
    atan2(global_plan_tf_[path_pose_idx+1].getOrigin().y() - global_plan_tf_[path_pose_idx].getOrigin().y(),
          global_plan_tf_[path_pose_idx+1].getOrigin().x() - global_plan_tf_[path_pose_idx].getOrigin().x()));
  tf2::Transform path_segmen_tf = tf2::Transform(path_quat,
                                                 tf2::Vector3(global_plan_tf_[path_pose_idx].getOrigin().x(),
                                                              global_plan_tf_[path_pose_idx].getOrigin().y(),
                                                              global_plan_tf_[path_pose_idx].getOrigin().z()));

  tf2::Vector3 current_tracking_err = - (path_segmen_tf.inverse() * tf2::Vector3(current_tf.translation.x,
                                                                                 current_tf.translation.y,
                                                                                 current_tf.translation.z));

  // trackin_error here represents the error between tracked link and position on plan
  controller_state_.tracking_error_lat = current_tracking_err.y();
  controller_state_.tracking_error_ang = controller_state_.error_ang.at(0);

  // integrate the error
  controller_state_.error_integral_lat += controller_state_.error_lat.at(0) * dt.seconds();
  controller_state_.error_integral_ang += controller_state_.error_ang.at(0) * dt.seconds();

  // Apply windup limit to limit the size of the integral term
  if (controller_state_.error_integral_lat > fabsf(windup_limit_))
    controller_state_.error_integral_lat = fabsf(windup_limit_);
  if (controller_state_.error_integral_lat < -fabsf(windup_limit_))
    controller_state_.error_integral_lat = -fabsf(windup_limit_);
  if (controller_state_.error_integral_ang > fabsf(windup_limit_))
    controller_state_.error_integral_ang = fabsf(windup_limit_);
  if (controller_state_.error_integral_ang < -fabsf(windup_limit_))
    controller_state_.error_integral_ang = -fabsf(windup_limit_);

  // My filter reference was Julius O. Smith III, Intro. to Digital Filters With Audio Applications.
  if (cutoff_frequency_lat_ != -1)
  {
    // Check if tan(_) is really small, could cause c = NaN
    tan_filt_ = tan((cutoff_frequency_lat_ * 6.2832) * dt.seconds() / 2);

    // Avoid tan(0) ==> NaN
    if ((tan_filt_ <= 0.) && (tan_filt_ > -0.01))
      tan_filt_ = -0.01;
    if ((tan_filt_ >= 0.) && (tan_filt_ < 0.01))
      tan_filt_ = 0.01;

    c_lat_ = 1 / tan_filt_;
  }
  if (cutoff_frequency_ang_ != -1)
  {
    // Check if tan(_) is really small, could cause c = NaN
    tan_filt_ = tan((cutoff_frequency_ang_ * 6.2832) * dt.seconds() / 2);

    // Avoid tan(0) ==> NaN
    if ((tan_filt_ <= 0.) && (tan_filt_ > -0.01))
      tan_filt_ = -0.01;
    if ((tan_filt_ >= 0.) && (tan_filt_ < 0.01))
      tan_filt_ = 0.01;

    c_ang_ = 1 / tan_filt_;
  }

  controller_state_.filtered_error_lat.at(2) = controller_state_.filtered_error_lat.at(1);
  controller_state_.filtered_error_lat.at(1) = controller_state_.filtered_error_lat.at(0);
  controller_state_.filtered_error_lat.at(0) =
      (1 / (1 + c_lat_ * c_lat_ + M_SQRT2 * c_lat_)) *
      (controller_state_.error_lat.at(2) + 2 * controller_state_.error_lat.at(1) + controller_state_.error_lat.at(0) -
       (c_lat_ * c_lat_ - M_SQRT2 * c_lat_ + 1) * controller_state_.filtered_error_lat.at(2) -
       (-2 * c_lat_ * c_lat_ + 2) * controller_state_.filtered_error_lat.at(1));

  controller_state_.filtered_error_ang.at(2) = controller_state_.filtered_error_ang.at(1);
  controller_state_.filtered_error_ang.at(1) = controller_state_.filtered_error_ang.at(0);
  controller_state_.filtered_error_ang.at(0) =
      (1 / (1 + c_ang_ * c_ang_ + M_SQRT2 * c_ang_)) *
      (controller_state_.error_ang.at(2) + 2 * controller_state_.error_ang.at(1) + controller_state_.error_ang.at(0) -
       (c_ang_ * c_ang_ - M_SQRT2 * c_ang_ + 1) * controller_state_.filtered_error_ang.at(2) -
       (-2 * c_ang_ * c_ang_ + 2) * controller_state_.filtered_error_ang.at(1));

  // Take derivative of error, first the raw unfiltered data:
  controller_state_.error_deriv_lat.at(2) = controller_state_.error_deriv_lat.at(1);
  controller_state_.error_deriv_lat.at(1) = controller_state_.error_deriv_lat.at(0);
  controller_state_.error_deriv_lat.at(0) =
      (controller_state_.error_lat.at(0) - controller_state_.error_lat.at(1)) / dt.seconds();
  controller_state_.filtered_error_deriv_lat.at(2) = controller_state_.filtered_error_deriv_lat.at(1);
  controller_state_.filtered_error_deriv_lat.at(1) = controller_state_.filtered_error_deriv_lat.at(0);
  controller_state_.filtered_error_deriv_lat.at(0) =
      (1 / (1 + c_lat_ * c_lat_ + M_SQRT2 * c_lat_)) *
      (controller_state_.error_deriv_lat.at(2) + 2 * controller_state_.error_deriv_lat.at(1) +
       controller_state_.error_deriv_lat.at(0) -
       (c_lat_ * c_lat_ - M_SQRT2 * c_lat_ + 1) * controller_state_.filtered_error_deriv_lat.at(2) -
       (-2 * c_lat_ * c_lat_ + 2) * controller_state_.filtered_error_deriv_lat.at(1));

  controller_state_.error_deriv_ang.at(2) = controller_state_.error_deriv_ang.at(1);
  controller_state_.error_deriv_ang.at(1) = controller_state_.error_deriv_ang.at(0);
  controller_state_.error_deriv_ang.at(0) =
      (controller_state_.error_ang.at(0) - controller_state_.error_ang.at(1)) / dt.seconds();
  controller_state_.filtered_error_deriv_ang.at(2) = controller_state_.filtered_error_deriv_ang.at(1);
  controller_state_.filtered_error_deriv_ang.at(1) = controller_state_.filtered_error_deriv_ang.at(0);
  controller_state_.filtered_error_deriv_ang.at(0) =
      (1 / (1 + c_ang_ * c_ang_ + M_SQRT2 * c_ang_)) *
      (controller_state_.error_deriv_ang.at(2) + 2 * controller_state_.error_deriv_ang.at(1) +
       controller_state_.error_deriv_ang.at(0) -
       (c_ang_ * c_ang_ - M_SQRT2 * c_ang_ + 1) * controller_state_.filtered_error_deriv_ang.at(2) -
       (-2 * c_ang_ * c_ang_ + 2) * controller_state_.filtered_error_deriv_ang.at(1));

  // calculate the control effort
  proportional_lat_ = Kp_lat_ * controller_state_.filtered_error_lat.at(0);
  integral_lat_ = Ki_lat_ * controller_state_.error_integral_lat;
  derivative_lat_ = Kd_lat_ * controller_state_.filtered_error_deriv_lat.at(0);

  proportional_ang_ = Kp_ang_ * controller_state_.filtered_error_ang.at(0);
  integral_ang_ = Ki_ang_ * controller_state_.error_integral_ang;
  derivative_ang_ = Kd_ang_ * controller_state_.filtered_error_deriv_ang.at(0);


  /***** Compute forward velocity *****/
  // Apply acceleration limits and end velocity
  double current_target_acc;
  double acc;
  double t_end_phase_current;
  double d_end_phase;

  // Compute time to reach end velocity from current velocity
  // Compute estimate overall distance during end_phase
  // The estimates are done a bit conservative to account that robot will take longer
  // to de-accelerate and thus avoid abrupt velocity changes at the end of the trajectory
  // The sample time plays an important role on how good these estimates are.
  // Thus We add a distance to the end phase distance estimation depending on the sample time
  if (current_x_vel > target_end_x_vel)
  {
    t_end_phase_current = (target_end_x_vel - current_x_vel) / (-target_x_decc_);
    d_end_phase = current_x_vel * t_end_phase_current
                - 0.5 * (target_x_decc_) * t_end_phase_current * t_end_phase_current
                + target_x_vel * 2.0 * dt.seconds();
  }
  else
  {
    t_end_phase_current = (target_end_x_vel - current_x_vel) / (target_x_acc_);
    d_end_phase = current_x_vel * t_end_phase_current
                + 0.5 * (target_x_acc_) * t_end_phase_current * t_end_phase_current
                + target_x_vel * 2.0 * dt.seconds();
  }
  RCLCPP_DEBUG(node_->get_logger(), "t_end_phase_current: %f", t_end_phase_current);
  RCLCPP_DEBUG(node_->get_logger(), "d_end_phase: %f", d_end_phase);
  RCLCPP_DEBUG(node_->get_logger(), "distance_to_goal: %f", distance_to_goal_);

  // Get 'angle' towards current_goal
  tf2::Transform robot_pose;
  tf2::convert(current_tf, robot_pose);
  tf2::Transform base_to_goal = robot_pose.inverseTimes(current_goal_);
  double angle_to_goal = atan2(base_to_goal.getOrigin().x(), -base_to_goal.getOrigin().y());

  // If we are as close to our goal or closer then we need to reach end velocity, enable end_phase.
  // However, if robot is not facing to the same direction as the local velocity target vector, don't enable end_phase.
  // This is to avoid skipping paths that start with opposite velocity.
  if (distance_to_goal_ <= fabs(d_end_phase)
      && sign(target_x_vel) == sign(angle_to_goal))
  {
    // This state will be remebered to avoid jittering on target_x_vel
    controller_state_.end_phase_enabled = true;
  }

  if (controller_state_.end_phase_enabled && fabs(target_x_vel) > VELOCITY_EPS)
  {
    current_target_x_vel_ = target_end_x_vel;
  }
  else
  {
    controller_state_.end_phase_enabled = false;
    current_target_x_vel_ = target_x_vel;
  }

  if (current_x_vel < current_target_x_vel_)
  {
    current_target_acc = target_x_acc_;
  }
  else if (current_x_vel > current_target_x_vel_)
  {
    current_target_acc = -target_x_decc_;
  }
  else
  {
    current_target_acc = 0;
  }
  double acc_desired = (current_target_x_vel_ - current_x_vel) / dt.seconds();
  double acc_abs = fmin(fabs(acc_desired), fabs(current_target_acc));
  acc = copysign(acc_abs, current_target_acc);

  double new_x_vel = current_x_vel + acc * dt.seconds();

  // For low target_end_x_vel we have a minimum velocity to ensure the goal is reached
  double min_vel = copysign(1.0, l_) * abs_minimum_x_vel_;
  if (!controller_state_.end_reached && controller_state_.end_phase_enabled
      && fabs(target_end_x_vel) <= fabs(min_vel) + VELOCITY_EPS
      && fabs(new_x_vel) <= fabs(min_vel) + VELOCITY_EPS)
  {
    new_x_vel = min_vel;
  }

  // When velocity error is too big reset current_x_vel
  if (fabs(odom_twist.linear.x) < fabs(current_target_x_vel_) &&
      fabs(odom_twist.linear.x - new_x_vel) > max_error_x_vel_)
  {
    // TODO(clopez/mcfurry/nobleo): Give feedback to higher level software here
    RCLCPP_WARN(node_->get_logger(), "Large tracking error. Current_x_vel %f / odometry %f",
            new_x_vel, odom_twist.linear.x); //NOTE: warn_throttle fix
  }
  // RCLCPP_INFO(node_->get_logger(), "distance_to_goal_ %f", distance_to_goal_);
  // RCLCPP_INFO(node_->get_logger(), "new_x_vel %f", new_x_vel);
  // RCLCPP_INFO(node_->get_logger(), "controller_state_.end_phase_enabled %d", (int)controller_state_.end_phase_enabled);
  // RCLCPP_INFO(node_->get_logger(), "distance_to_goal_ %f", distance_to_goal_);

  // Force target_end_x_vel at the very end of the path
  // Or when the end velocity is reached.
  // Warning! If target_end_x_vel == 0 and min_vel = 0 then the robot might not reach end pose
  if ((distance_to_goal_ == 0.0 && target_end_x_vel >= VELOCITY_EPS)
      || (controller_state_.end_phase_enabled && new_x_vel >= target_end_x_vel - VELOCITY_EPS
          && new_x_vel <= target_end_x_vel + VELOCITY_EPS)
          || ( fabs(distance_to_goal_) <= POSITION_EPS && fabs(target_end_x_vel) <= VELOCITY_EPS
              && fabs(new_x_vel - min_vel) <= VELOCITY_EPS ) )
  {
    controller_state_.end_reached = true;
    controller_state_.end_phase_enabled = false;
    *progress = 1.0;
    *eda = 0.0;
    enabled_ = false;
  }
  else
  {
    controller_state_.end_reached = false;
    // eda (Estimated duration of arrival) estimation
    if (fabs(target_x_vel) > VELOCITY_EPS)
    {
      double t_const = (copysign(distance_to_goal_, target_x_vel) - d_end_phase) / target_x_vel;
      *eda = fmin(fmax(t_end_phase_current, 0.0) + fmax(t_const, 0.0), LONG_DURATION);
    }
    else
    {
      *eda = LONG_DURATION;
    }
  }
  // RCLCPP_INFO(node_->get_logger(), "controller_state_.end_reached %d", (int)controller_state_.end_reached);

  /******* end calculation of forward velocity ********/


  //***** Overall control *****//
  // Controller logic && overall control effort
  control_effort_long_ = new_x_vel;
  control_effort_lat_ = 0.0;
  control_effort_ang_ = 0.0;

  if (feedback_lat_enabled_)
    control_effort_lat_ = proportional_lat_ + integral_lat_ + derivative_lat_;
  if (feedback_ang_enabled_)
    control_effort_ang_ = proportional_ang_ + integral_ang_ + derivative_ang_;


  //***** Feedforward control *****//
  if (feedforward_lat_enabled_)
  {
    feedforward_lat_ = 0.0;  // Not implemented
    control_effort_lat_ = control_effort_lat_ + feedforward_lat_;
  }
  else
    feedforward_lat_ = 0.0;

  if (feedforward_ang_enabled_)
  {
    feedforward_ang_ = turning_radius_inv_vector_[controller_state_.last_visited_pose_index]*control_effort_long_;
    RCLCPP_DEBUG(node_->get_logger(), "turning_radius_inv_vector[%lu] = %f",
      controller_state_.last_visited_pose_index, turning_radius_inv_vector_[controller_state_.last_visited_pose_index]);

    control_effort_ang_ = control_effort_ang_ + feedforward_ang_;
  }
  else
    feedforward_ang_ = 0.0;


  // Apply saturation limits
  if (control_effort_lat_ > upper_limit_)
    control_effort_lat_ = upper_limit_;
  else if (control_effort_lat_ < lower_limit_)
    control_effort_lat_ = lower_limit_;

  if (control_effort_ang_ > ang_upper_limit_)
    control_effort_ang_ = ang_upper_limit_;
  else if (control_effort_ang_ < ang_lower_limit_)
    control_effort_ang_ = ang_lower_limit_;

  // Populate debug output
  // Error topic containing the 'control' error on which the PID acts
  pid_debug->control_error.linear.x = 0.0;
  pid_debug->control_error.linear.y = controller_state_.error_lat.at(0);
  pid_debug->control_error.angular.z = controller_state_.error_ang.at(0);
  // Error topic containing the 'tracking' error, i.e. the real error between path and tracked link
  pid_debug->tracking_error.linear.x = 0.0;
  pid_debug->tracking_error.linear.y = controller_state_.tracking_error_lat;
  pid_debug->tracking_error.angular.z = controller_state_.tracking_error_ang;

  pid_debug->proportional.linear.x = 0.0;
  pid_debug->proportional.linear.y = proportional_lat_;
  pid_debug->proportional.angular.z = proportional_ang_;

  pid_debug->integral.linear.x = 0.0;
  pid_debug->integral.linear.y = integral_lat_;
  pid_debug->integral.angular.z = integral_ang_;

  pid_debug->derivative.linear.x = 0.0;
  pid_debug->derivative.linear.y = derivative_lat_;
  pid_debug->derivative.angular.z = derivative_ang_;

  pid_debug->feedforward.linear.x = new_x_vel;
  pid_debug->feedforward.linear.y = feedforward_lat_;
  pid_debug->feedforward.angular.z = feedforward_ang_;

  geometry_msgs::msg::Twist output_combined;
  // Generate twist message
  if (holonomic_robot_enable_)
  {
    output_combined.linear.x = control_effort_long_;
    output_combined.linear.y = control_effort_lat_;
    output_combined.linear.z = 0;
    output_combined.angular.x = 0;
    output_combined.angular.y = 0;
    output_combined.angular.z = control_effort_ang_;
    output_combined.angular.z = std::clamp(output_combined.angular.z, -max_yaw_vel_, max_yaw_vel_);
  }
  else
  {
    output_combined.linear.x = control_effort_long_;
    output_combined.linear.y = 0;
    output_combined.linear.z = 0;
    output_combined.angular.x = 0;
    output_combined.angular.y = 0;
    output_combined.angular.z = copysign(1.0, l_) * control_effort_lat_ +
                                control_effort_ang_;  // Take the sign of l for the lateral control effort
    output_combined.angular.z = std::clamp(output_combined.angular.z, -max_yaw_vel_, max_yaw_vel_);
    // For non-holonomic robots apply saturation based on minimum turning radius
    double max_ang_twist_tr;
    if (minimum_turning_radius_ < RADIUS_EPS)
    {
      // Rotation in place is allowed
      // minimum_turning_radius = RADIUS_EPS; // This variable is not used anymore so it does not matter
      // do not restrict angular velocity. Thus use the biggets number possible
      max_ang_twist_tr = std::numeric_limits<double>::infinity();
    }
    else
    {
      max_ang_twist_tr = fabs(output_combined.linear.x / minimum_turning_radius_);
    }
    output_combined.angular.z = std::clamp(output_combined.angular.z, -max_ang_twist_tr, max_ang_twist_tr);
  }
  // Apply max acceleration limit to yaw
  double yaw_acc = std::clamp((output_combined.angular.z - current_yaw_vel) / dt.seconds(),
                              -max_yaw_acc_, max_yaw_acc_);
  double new_yaw_vel = current_yaw_vel + (yaw_acc * dt.seconds());
  output_combined.angular.z = new_yaw_vel;

  // Transform velocity commands at base_link to steer when using tricycle model
  if (use_tricycle_model_)
  {
    geometry_msgs::msg::Twist output_steering;
    TricycleSteeringCmdVel steering_cmd = computeTricycleModelInverseKinematics(output_combined);
    if (output_combined.linear.x < 0.0 && steering_cmd.speed > 0.0)
    {
      steering_cmd.speed = - steering_cmd.speed;
      if (steering_cmd.steering_angle > 0)
        steering_cmd.steering_angle = steering_cmd.steering_angle - M_PI;
      else
        steering_cmd.steering_angle = steering_cmd.steering_angle + M_PI;
    }
    // Apply limits to steering commands
    steering_cmd.steering_angle = std::clamp(steering_cmd.steering_angle,
                                  -max_steering_angle_, max_steering_angle_);
    double steering_yaw_vel = std::clamp((steering_cmd.steering_angle - controller_state_.previous_steering_angle)
                                          / dt.seconds(), -max_steering_yaw_vel_, max_steering_yaw_vel_);
    double steering_angle_acc = std::clamp((steering_yaw_vel - controller_state_.previous_steering_yaw_vel)/ dt.seconds(),
                                  -max_steering_yaw_acc_, max_steering_yaw_acc_);
    steering_cmd.steering_angle_velocity =   controller_state_.previous_steering_yaw_vel
                                          +  (steering_angle_acc * dt.seconds());
    steering_cmd.steering_angle =   controller_state_.previous_steering_angle
                                  + (steering_cmd.steering_angle_velocity * dt.seconds());

    steering_cmd.speed = std::clamp(steering_cmd.speed, -max_steering_x_vel_, max_steering_x_vel_);
    steering_cmd.acceleration = std::clamp((steering_cmd.speed - controller_state_.previous_steering_x_vel)/ dt.seconds(),
                                  -max_steering_x_acc_, max_steering_x_acc_);
    steering_cmd.speed =  controller_state_.previous_steering_x_vel + (steering_cmd.acceleration * dt.seconds());

    controller_state_.previous_steering_angle = steering_cmd.steering_angle;
    controller_state_.previous_steering_yaw_vel = steering_cmd.steering_angle_velocity;
    controller_state_.previous_steering_x_vel = steering_cmd.speed;

    // Compute velocities back to base_link and update controller state
    output_steering = computeTricycleModelForwardKinematics(steering_cmd.speed, steering_cmd.steering_angle);
    controller_state_.current_x_vel =  output_steering.linear.x;
    controller_state_.current_yaw_vel = output_steering.angular.z;

    //NOTE: fix pid_debug
    // pid_debug->steering_angle = steering_cmd.steering_angle;
    // pid_debug->steering_yaw_vel = steering_cmd.steering_angle_velocity;
    // pid_debug->steering_x_vel = steering_cmd.speed;

    output_combined = output_steering;
  }

  // Publish control effort if controller enabled
  if (!enabled_)  // Do nothing reset integral action and all filters
  {
    controller_state_.error_integral_lat = 0.0;
    controller_state_.error_integral_ang = 0.0;
  }

  controller_state_.current_x_vel = new_x_vel;
  controller_state_.current_yaw_vel = new_yaw_vel;

  // --------------- new --------------- 0
  geometry_msgs::msg::TwistStamped output_combined_stamped;
  output_combined_stamped.twist = output_combined;
  //NOTE: need for specifying the stamp?
  output_combined_stamped.header.frame_id = "base_link";
  // --------------- new --------------- 1

  return output_combined_stamped;
}

geometry_msgs::msg::TwistStamped Controller::update_with_limits(const geometry_msgs::msg::Transform current_tf,
                                                    const geometry_msgs::msg::Twist odom_twist,
                                                    const builtin_interfaces::msg::Duration dt,
                                                    double* eda, double* progress,
                                                    path_tracking_pid::msg::PidDebug* pid_debug
                                                    )
{
  // All limits are absolute
  double max_x_vel = std::abs(target_x_vel_);

  // Apply external limit
  max_x_vel = std::min(max_x_vel, vel_max_external_);

  // Apply obstacle limit
  max_x_vel = std::min(max_x_vel, vel_max_obstacle_);

  // Apply mpc limit (last because less iterations required if max vel is already limited)
  double vel_max_mpc = std::numeric_limits<double>::infinity();
  if (local_config_.use_mpc)
  {
    vel_max_mpc = std::abs(mpc_based_max_vel(std::copysign(max_x_vel, target_x_vel_), current_tf, odom_twist));
    max_x_vel = std::min(max_x_vel, vel_max_mpc);
  }

  // Some logging:
  RCLCPP_DEBUG(node_->get_logger(), "max_x_vel=%.3f, target_x_vel=%.3f, vel_max_external=%.3f, vel_max_obstacle=%.3f, vel_max_mpc=%.3f",
             max_x_vel, target_x_vel_, vel_max_external_, vel_max_obstacle_, vel_max_mpc);
  if (max_x_vel != target_x_vel_)
  {
    if (max_x_vel == vel_max_external_)
    {
      RCLCPP_WARN(node_->get_logger(), "External velocity limit active %.2fm/s", vel_max_external_); //NOTE: fix warn_throttle (5.0Hz)
    }
    else if (max_x_vel == vel_max_obstacle_)
    {
      RCLCPP_WARN(node_->get_logger(), "Obstacle velocity limit active %.2fm/s", vel_max_obstacle_); //NOTE: fix warn_throttle (5.0Hz)
    }
    else if (max_x_vel == vel_max_mpc)
    {
      RCLCPP_WARN(node_->get_logger(), "MPC velocity limit active %.2fm/s", vel_max_mpc); //NOTE: fix warn_throttle (5.0Hz)
    }
  }

  // The end velocity is bound by the same limits to avoid accelerating above the limit in the end phase
  double max_end_x_vel = std::min({std::abs(target_end_x_vel_), vel_max_external_, vel_max_obstacle_, vel_max_mpc});  // NOLINT
  max_end_x_vel = std::copysign(max_end_x_vel, target_end_x_vel_);

  // Update the controller with the new setting
  max_x_vel = std::copysign(max_x_vel, target_x_vel_);
  return update(max_x_vel, max_end_x_vel, current_tf, odom_twist, dt, eda, progress, pid_debug);
}

// output updated velocity command: (Current position, current measured velocity, closest point index, estimated
// duration of arrival, debug info)
double Controller::mpc_based_max_vel(const double target_x_vel, geometry_msgs::msg::Transform current_tf,
                                     geometry_msgs::msg::Twist odom_twist)
{
  // Temporary save global data
  ControllerState controller_state_saved;
  controller_state_saved = controller_state_;

  // Bisection optimisation parameters
  double target_x_vel_prev = 0.0;  // Previous iteration velocity command
  int mpc_vel_optimization_iter = 0;

  // MPC parameters
  int mpc_fwd_iter = 0;             // Reset MPC iterations

  // Create predicted position vector
  geometry_msgs::msg::Transform predicted_tf = current_tf;
  geometry_msgs::msg::Twist pred_twist = odom_twist;

  double new_nominal_x_vel = target_x_vel;  // Start off from the current velocity
  double mpc_vel_limit = new_nominal_x_vel;

  // Loop MPC
  while (mpc_fwd_iter < mpc_max_fwd_iter_ && mpc_vel_optimization_iter <= mpc_max_vel_optimization_iter_)
  {
    mpc_fwd_iter += 1;

    // Check if robot stays within bounds for all iterations, if the new_nominal_x_vel is smaller than
    // max_target_x_vel we can increase it
    if (mpc_fwd_iter == mpc_max_fwd_iter_ && fabs(controller_state_.error_lat.at(0)) <= mpc_max_error_lat_ &&
        fabs(new_nominal_x_vel) < abs(target_x_vel))
    {
      mpc_vel_optimization_iter += 1;

      // When we reach the maximum allowed mpc optimization iterations, do not change velocity anymore
      if (mpc_vel_optimization_iter > mpc_max_vel_optimization_iter_)
      {
        break;
      }

      // Increase speed
      double prev_save = new_nominal_x_vel;
      new_nominal_x_vel = copysign(1.0, new_nominal_x_vel)
                                  * abs(target_x_vel_prev - new_nominal_x_vel) / 2 + new_nominal_x_vel;
      target_x_vel_prev = prev_save;

      // Reset variables
      controller_state_ = controller_state_saved;

      predicted_tf = current_tf;
      pred_twist = odom_twist;
      mpc_fwd_iter = 0;
    }
    // If the robot gets out of bounds earlier we decrease the velocity
    else if (abs(controller_state_.error_lat.at(0)) >= mpc_max_error_lat_)
    {
      mpc_vel_optimization_iter += 1;

      // Lower speed
      double prev_save = new_nominal_x_vel;
      new_nominal_x_vel = -copysign(1.0, new_nominal_x_vel)
                                  * abs(target_x_vel_prev - new_nominal_x_vel) / 2 + new_nominal_x_vel;
      target_x_vel_prev = prev_save;

      // Reset variables
      controller_state_ = controller_state_saved;

      predicted_tf = current_tf;
      pred_twist = odom_twist;
      mpc_fwd_iter = 0;

      // Warning if new_nominal_x_vel becomes really low
      if (abs(new_nominal_x_vel) < 0.01)
      {
        RCLCPP_WARN(node_->get_logger(), "Lowering velocity did not decrease the lateral error enough."); //NOTE: ROS_WARN_THROTTLE fix (5.0Hz)
      }
    }
    else if (mpc_fwd_iter != mpc_max_fwd_iter_)
    {
      // Run controller
      // Output: pred_twist.[linear.x, linear.y, linear.z, angular.x, angular.y, angular.z]
      path_tracking_pid::msg::PidDebug pid_debug_unused;
      double eda_unused, progress_unused;
      // -------- new -------- 0
      geometry_msgs::msg::TwistStamped pred_twist_stamped;
      pred_twist_stamped.twist = odom_twist;
      pred_twist_stamped = Controller::update(new_nominal_x_vel, target_end_x_vel_, predicted_tf, pred_twist,
                                      rclcpp::Duration(tf2::durationFromSec(mpc_simulation_sample_time_)),
                                      &eda_unused, &progress_unused, &pid_debug_unused
                                      );
      // -------- new -------- 1

      // Run plant model
      double theta = tf2::getYaw(predicted_tf.rotation);
      predicted_tf.translation.x += pred_twist.linear.x * cos(theta) * mpc_simulation_sample_time_;
      predicted_tf.translation.y += pred_twist.linear.x * sin(theta) * mpc_simulation_sample_time_;
      tf2::Quaternion q;
      q.setRPY(0, 0, theta + pred_twist.angular.z * mpc_simulation_sample_time_);
      predicted_tf.rotation = tf2::toMsg(q);
    }
  }
  // Apply limits to the velocity
  mpc_vel_limit = copysign(1.0, target_x_vel) *
                      fmin(fabs(target_x_vel), fabs(new_nominal_x_vel));
  mpc_vel_limit = copysign(1.0, new_nominal_x_vel) *
                      fmax(fabs(new_nominal_x_vel), mpc_min_x_vel_);

  // Revert global variables
  controller_state_ = controller_state_saved;

  return std::abs(mpc_vel_limit);
}

void Controller::selectMode(ControllerMode mode)
{
  switch (mode)
  {
    case ControllerMode::frontAxleLateral:
      // Front axle lateral controller (default)
      l_ = 0.5;
      feedback_lat_enabled_ = true;
      feedback_ang_enabled_ = false;
      feedforward_lat_enabled_ = true;
      feedforward_ang_enabled_ = false;
      break;
    case ControllerMode::rearAxleLateral:
      // Rear axle lateral control
      l_ = 0.0;  // To prevent singular configuration
      feedback_lat_enabled_ = true;
      feedback_ang_enabled_ = false;
      feedforward_lat_enabled_ = true;
      feedforward_ang_enabled_ = false;
      break;
    case ControllerMode::rearAxleAngular:
      // Rear axle angular controller
      l_ = 0.0;
      feedback_lat_enabled_ = false;
      feedback_ang_enabled_ = true;
      feedforward_lat_enabled_ = false;
      feedforward_ang_enabled_ = false;
      break;
    case ControllerMode::fixOrientation:
      // Fix orientation controller
      l_ = 0.0;
      feedback_lat_enabled_ = false;
      feedback_ang_enabled_ = true;
      feedforward_lat_enabled_ = false;
      feedforward_ang_enabled_ = true;
      break;
  }

  printParameters();
}

void Controller::printParameters()
{
  RCLCPP_INFO(node_->get_logger(), "CONTROLLER PARAMETERS");
  RCLCPP_INFO(node_->get_logger(), "-----------------------------------------");
  RCLCPP_INFO(node_->get_logger(), "Controller enabled: %i", enabled_);
  RCLCPP_INFO(node_->get_logger(), "Controller DEBUG enabled: %i", debug_enabled_);
  RCLCPP_INFO(node_->get_logger(), "Distance L: %f", l_);
  RCLCPP_INFO(node_->get_logger(), "Track base_link enabled?: %i", track_base_link_enabled_);

  RCLCPP_INFO(node_->get_logger(), "Target forward velocities (xv: %f, xv_end,: %f)", target_x_vel_, target_end_x_vel_);
  RCLCPP_INFO(node_->get_logger(), "Target forward (de)accelerations (xacc: %f, xdecc,: %f)", target_x_acc_, target_x_decc_);
  RCLCPP_INFO(node_->get_logger(), "Maximum allowed forward velocity error: %f", max_error_x_vel_);
  RCLCPP_INFO(node_->get_logger(), "Feedback (lat, ang): ( %i, %i)", feedback_lat_enabled_, feedback_ang_enabled_);
  RCLCPP_INFO(node_->get_logger(), "Feedforward (lat, ang): (%i, %i)", feedforward_lat_enabled_, feedforward_ang_enabled_);
  RCLCPP_INFO(node_->get_logger(), "Lateral gains: (Kp: %f, Ki, %f, Kd, %f)", Kp_lat_, Ki_lat_, Kd_lat_);
  RCLCPP_INFO(node_->get_logger(), "Angular gains: (Kp: %f, Ki, %f, Kd, %f)", Kp_ang_, Ki_ang_, Kd_ang_);

  RCLCPP_INFO(node_->get_logger(), "Robot type (holonomic): (%i)", holonomic_robot_enable_);

  RCLCPP_INFO(node_->get_logger(), "Integral-windup limit: %f", windup_limit_);
  RCLCPP_INFO(node_->get_logger(), "Saturation limits xy: %f/%f", upper_limit_, lower_limit_);
  RCLCPP_INFO(node_->get_logger(), "Saturation limits ang: %f/%f", ang_upper_limit_, ang_lower_limit_);
  RCLCPP_INFO(node_->get_logger(), "-----------------------------------------");
}

void Controller::configure(PidConfig& config)
{
  // Erase all queues when config changes

  std::fill(controller_state_.error_lat.begin(), controller_state_.error_lat.end(), 0);
  std::fill(controller_state_.filtered_error_lat.begin(), controller_state_.filtered_error_lat.end(), 0);
  std::fill(controller_state_.error_deriv_lat.begin(), controller_state_.error_deriv_lat.end(), 0);
  std::fill(controller_state_.filtered_error_deriv_lat.begin(), controller_state_.filtered_error_deriv_lat.end(), 0);

  std::fill(controller_state_.error_ang.begin(), controller_state_.error_ang.end(), 0);
  std::fill(controller_state_.filtered_error_ang.begin(), controller_state_.filtered_error_ang.end(), 0);
  std::fill(controller_state_.error_deriv_ang.begin(), controller_state_.error_deriv_ang.end(), 0);
  std::fill(controller_state_.filtered_error_deriv_ang.begin(), controller_state_.filtered_error_deriv_ang.end(), 0);

  Kp_lat_ = config.Kp_lat;
  Ki_lat_ = config.Ki_lat;
  Kd_lat_ = config.Kd_lat;
  Kp_ang_ = config.Kp_ang;
  Ki_ang_ = config.Ki_ang;
  Kd_ang_ = config.Kd_ang;

  track_base_link_enabled_ = config.track_base_link;
  RCLCPP_DEBUG(node_->get_logger(), "Track base_link? Then global path poses are needed! '%d'", (int)track_base_link_enabled_);

  l_ = config.l;
  target_x_vel_ = config.target_x_vel;
  l_ = copysign(1.0, target_x_vel_) * fabs(l_);
  if (controller_state_.end_phase_enabled)
  {
    if(abs(config.target_end_x_vel - target_end_x_vel_) > 1e-3){
      RCLCPP_WARN(node_->get_logger(), "Won't change end velocity in end phase"); //NOTE: warn_cond fix
    }
    if(abs(config.target_x_acc - target_x_acc_) > 1e-3){
      RCLCPP_WARN(node_->get_logger(), "Won't change accelerations in end phase"); //NOTE: warn_cond fix
    }
    if(abs(config.target_x_decc - target_x_decc_) > 1e-3){
      RCLCPP_WARN(node_->get_logger(), "Won't change accelerations in end phase"); //NOTE: warn_cond fix
    }
    config.target_end_x_vel = target_end_x_vel_;
    config.target_x_acc = target_x_acc_;
    config.target_x_decc = target_x_decc_;
  }
  else
  {
    target_end_x_vel_ = config.target_end_x_vel;
    target_x_acc_ = config.target_x_acc;
    target_x_decc_ = config.target_x_decc;
  }
  abs_minimum_x_vel_ = config.abs_minimum_x_vel;
  max_yaw_vel_ = config.max_yaw_vel;
  max_yaw_acc_ = config.max_yaw_acc;

  max_error_x_vel_ = config.max_error_x_vel;

  minimum_turning_radius_ = config.min_turning_radius;

  debug_enabled_ = config.controller_debug_enabled;
  feedforward_lat_enabled_ = config.feedforward_lat;
  feedforward_ang_enabled_ = config.feedforward_ang;
  feedback_lat_enabled_ = config.feedback_lat;
  feedback_ang_enabled_ = config.feedback_ang;

  // MPC
  mpc_max_fwd_iter_ = config.mpc_max_fwd_iterations;
  mpc_max_vel_optimization_iter_ = config.mpc_max_vel_optimization_iterations;
  mpc_simulation_sample_time_ = config.mpc_simulation_sample_time;
  mpc_max_error_lat_ = config.mpc_max_error_lat;
  mpc_min_x_vel_ = config.mpc_min_x_vel;
  mpc_min_x_vel_ = fmin(mpc_min_x_vel_, fabs(target_x_vel_));

  // Tricycle model
  max_steering_angle_ = config.max_steering_angle;
  max_steering_x_vel_ = config.max_steering_x_vel;
  max_steering_x_acc_ = config.max_steering_x_acc;
  max_steering_yaw_vel_ = config.max_steering_yaw_vel;
  max_steering_yaw_acc_ = config.max_steering_yaw_acc;

  local_config_ = config;

  printParameters();
}

PidConfig Controller::getConfig()
{
  return local_config_;
}

void Controller::setEnabled(bool value)
{
  RCLCPP_DEBUG(node_->get_logger(), "Controller::setEnabled(%d)", value);
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
  controller_state_.error_lat = {0.0, 0.0, 0.0};
  controller_state_.filtered_error_lat = {0.0, 0.0, 0.0};
  controller_state_.error_deriv_lat = {0.0, 0.0, 0.0};
  controller_state_.filtered_error_deriv_lat = {0.0, 0.0, 0.0};
  controller_state_.error_ang = {0.0, 0.0, 0.0};
  controller_state_.filtered_error_ang = {0.0, 0.0, 0.0};
  controller_state_.error_deriv_ang = {0.0, 0.0, 0.0};
  controller_state_.filtered_error_deriv_ang = {0.0, 0.0, 0.0};
}

void Controller::setVelMaxExternal(double value)
{
  if (value < 0.0)
  {
    RCLCPP_ERROR(node_->get_logger(), "External velocity limit (%f) has to be positive", value); //NOTE: throttle fix
    return;
  }
  if (value < 0.1)
    RCLCPP_ERROR(node_->get_logger(), "External velocity limit is very small (%f), this could result in standstill", value); //NOTE: throttle fix
  vel_max_external_ = value;
}

void Controller::setVelMaxObstacle(double value)
{
  if(vel_max_obstacle_ != 0.0 && value == 0.0) {
    RCLCPP_WARN(node_->get_logger(), "Collision imminent, slamming the brakes");
  }
  vel_max_obstacle_ = value;
}

double Controller::getVelMaxObstacle()
{
  return vel_max_obstacle_;
}

}  // namespace path_tracking_pid
