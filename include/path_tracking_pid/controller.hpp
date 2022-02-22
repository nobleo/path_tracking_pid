#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <path_tracking_pid/PidConfig.h>
#include <path_tracking_pid/PidDebug.h>
#include <tf2/LinearMath/Transform.h>

#include <array>
#include <boost/noncopyable.hpp>
#include <path_tracking_pid/details/fifo_array.hpp>
#include <path_tracking_pid/details/second_order_lowpass.hpp>
#include <path_tracking_pid/units.hpp>
#include <vector>

namespace path_tracking_pid
{
struct TricycleSteeringCmdVel
{
  double steering_angle = 0.0;
  double steering_angle_velocity = 0.0;
  units::velocity_t speed = 0.0 * units::meter_per_second;
  units::acceleration_t acceleration = 0.0 * units::meter_per_second_squared;
};

struct ControllerState
{
  size_t current_global_plan_index = 0;
  size_t last_visited_pose_index = 0;
  units::velocity_t current_x_vel = 0.0 * units::meter_per_second;
  double current_yaw_vel = 0.0;
  double previous_steering_angle = 0.0;
  units::velocity_t previous_steering_x_vel = 0.0 * units::meter_per_second;
  double previous_steering_yaw_vel = 0.0;
  bool end_phase_enabled = false;
  bool end_reached = false;
  double error_integral_lat = 0.0;
  double error_integral_ang = 0.0;
  double tracking_error_lat = 0.0;
  double tracking_error_ang = 0.0;
  // Errors with little history
  details::SecondOrderLowpass error_lat;
  details::SecondOrderLowpass error_deriv_lat;
  details::SecondOrderLowpass error_ang;
  details::SecondOrderLowpass error_deriv_ang;
};

class Controller : private boost::noncopyable
{
public:
  /**
   * Set holonomic configuration of the controller
   * @param holonomic is holonomic robot?
   */
  void setHolonomic(bool holonomic);

  /**
   * Enable estimation of pose angles by looking at consecutive path points
   * @param estimate_pose_angle
   */
  void setEstimatePoseAngle(bool estimate_pose_angle);

  /**
   * Set static configuration of the controller
   * @param tricycle_model_enabled If tricycle model should be used
   * @param estimate_pose_angle The transformation from base to steered wheel
   */
  void setTricycleModel(
    bool tricycle_model_enabled, const geometry_msgs::Transform & tf_base_to_steered_wheel);

  /**
   * Set plan
   * @param current Where is the robot now?
   * @param odom_twist Robot odometry
   * @param global_plan Plan to follow
   */
  void setPlan(
    const geometry_msgs::Transform & current_tf, const geometry_msgs::Twist & odom_twist,
    const std::vector<geometry_msgs::PoseStamped> & global_plan);

  /**
   * Set plan
   * @param current Where is the robot now?
   * @param odom_twist Robot odometry
   * @param tf_base_to_steered_wheel Where is the steered wheel now?
   * @param steering_odom_twist Steered wheel odometry
   * @param global_plan Plan to follow
   */
  void setPlan(
    const geometry_msgs::Transform & current_tf, const geometry_msgs::Twist & odom_twist,
    const geometry_msgs::Transform & tf_base_to_steered_wheel,
    const geometry_msgs::Twist & steering_odom_twist,
    const std::vector<geometry_msgs::PoseStamped> & global_plan);
  /**
   * Find position on plan by looking at the surroundings of last known pose.
   * @param current Where is the robot now?
   * @param controller_state_ptr  The current state of the controller that gets updated by this function
   * @return tf of found position on plan
   * @return index of current path-pose if requested
   */
  tf2::Transform findPositionOnPlan(
    const geometry_msgs::Transform & current_tf, ControllerState * controller_state_ptr,
    size_t & path_pose_idx);
  // Overloaded function definition for users that don't require the segment index
  tf2::Transform findPositionOnPlan(
    const geometry_msgs::Transform & current_tf, ControllerState * controller_state_ptr)
  {
    size_t path_pose_idx;
    return findPositionOnPlan(current_tf, controller_state_ptr, path_pose_idx);
  }

  /**
   * Run one iteration of a PID controller
   * @param target_x_vel robot will try to reach target x velocity within (de)acceleration limits
   * @param current Where is the robot now?
   * @param odom_twist Robot odometry
   * @param dt Duration since last update
   * @return Velocity command
   * @return eda Estimated Duration of Arrival
   * @return progress Progress along the path [0,1]
   * @return pid_debug Variable with information to debug the controller
   */
  geometry_msgs::Twist update(
    units::velocity_t target_x_vel, units::velocity_t target_end_x_vel,
    const geometry_msgs::Transform & current_tf, const geometry_msgs::Twist & odom_twist,
    units::duration_t dt, units::duration_t * eda, double * progress,
    path_tracking_pid::PidDebug * pid_debug);

  /**
   * Run one iteration of a PID controller with velocity limits applied
   * @param current Where is the robot now?
   * @param odom_twist Robot odometry
   * @param dt Duration since last update
   * @return Velocity command
   * @return eda Estimated Duration of Arrival
   * @return progress Progress along the path [0,1]
   * @return pid_debug Variable with information to debug the controller
   */
  geometry_msgs::Twist update_with_limits(
    const geometry_msgs::Transform & current_tf, const geometry_msgs::Twist & odom_twist,
    units::duration_t dt, units::duration_t * eda, double * progress,
    path_tracking_pid::PidDebug * pid_debug);

  /**
   * Perform prediction steps on the lateral error and return a reduced velocity that stays within bounds
   * @param current_tf Where is the robot now?
   * @param odom_twist Robot odometry
   * @return Velocity command
   */
  units::velocity_t mpc_based_max_vel(
    units::velocity_t target_x_vel, const geometry_msgs::Transform & current_tf,
    const geometry_msgs::Twist & odom_twist);

  /**
   * Set dynamic parameters for the PID controller
   * @param config
   */
  void configure(path_tracking_pid::PidConfig & config);

  /**
   * Set whether the controller is enabled
   * @param value enable controller?
   */
  void setEnabled(bool value);

  /**
   * Reset controller state
   */
  void reset();

  /**
   * Gets current dynamic configuration of the controller
   * @return current controller configuration
   */
  path_tracking_pid::PidConfig getConfig();

  // Inline get-functions for transforms
  tf2::Transform getCurrentGoal() const { return current_goal_; }
  tf2::Transform getCurrentWithCarrot() const { return current_with_carrot_; }
  tf2::Transform getCurrentPosOnPlan() const { return current_pos_on_plan_; }

  // Inline get-function for controller-state
  ControllerState getControllerState() const { return controller_state_; }

  // Set new vel_max_external value
  void setVelMaxExternal(units::velocity_t value);

  // Set new vel_max_obstacle value
  void setVelMaxObstacle(units::velocity_t value);

  // Get vel_max_obstacle value
  units::velocity_t getVelMaxObstacle() const;

private:
  void distToSegmentSquared(
    const tf2::Transform & pose_p, const tf2::Transform & pose_v, const tf2::Transform & pose_w,
    tf2::Transform & pose_projection, units::distance_squared_t & distance_to_p,
    units::distance_t & distance_to_w) const;

  // Overloaded function for callers that don't need the additional results
  units::distance_squared_t distToSegmentSquared(
    const tf2::Transform & pose_p, const tf2::Transform & pose_v,
    const tf2::Transform & pose_w) const
  {
    tf2::Transform dummy_tf;
    units::distance_t dummy_distance;
    units::distance_squared_t result;
    distToSegmentSquared(pose_p, pose_v, pose_w, dummy_tf, result, dummy_distance);
    return result;
  }

  geometry_msgs::Twist computeTricycleModelForwardKinematics(
    units::velocity_t x_vel, double steering_angle);
  TricycleSteeringCmdVel computeTricycleModelInverseKinematics(
    const geometry_msgs::Twist & cmd_vel);
  /**
   * Output some debug information about the current parameters
   */
  void printParameters() const;

  path_tracking_pid::PidConfig config_;
  ControllerState controller_state_;

  // Global Plan variables
  std::vector<tf2::Transform> global_plan_tf_;              // Global plan vector
  std::vector<units::distance_t> distance_to_goal_vector_;  // Vector with distances to goal
  std::vector<double> turning_radius_inv_vector_;  // Vector with computed turning radius inverse
  units::distance_t distance_to_goal_ = NAN * units::meter;
  tf2::Transform current_goal_;
  tf2::Transform current_pos_on_plan_;
  tf2::Transform current_with_carrot_;

  // Auxiliary variables
  units::velocity_t current_target_x_vel_ = 0.0 * units::meter_per_second;
  double control_effort_long_ = 0.0;  // output of pid controller
  double control_effort_lat_ = 0.0;   // output of pid controller
  double control_effort_ang_ = 0.0;   // output of pid controller

  bool enabled_ = true;
  bool holonomic_robot_enable_ = false;
  bool estimate_pose_angle_enabled_ = false;

  // feedforward controller
  double feedforward_lat_ = 0.0;
  double feedforward_ang_ = 0.0;

  // tricycle model
  bool use_tricycle_model_ = false;
  geometry_msgs::Transform tf_base_to_steered_wheel_;
  std::array<std::array<double, 2>, 2> inverse_kinematics_matrix_{};
  std::array<std::array<double, 2>, 2> forward_kinematics_matrix_{};

  // Dynamic external max velocity requirement (e.g. no more power available). Can be active
  // external to the pid controller.
  units::velocity_t vel_max_external_ = INFINITY * units::meter_per_second;
  // Can be zero if lethal obstacles are detected. Can be active external to the pid controller.
  units::velocity_t vel_max_obstacle_ = INFINITY * units::meter_per_second;
};

}  // namespace path_tracking_pid
