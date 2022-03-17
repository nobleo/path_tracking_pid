#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <path_tracking_pid/PidConfig.h>
#include <path_tracking_pid/PidDebug.h>
#include <tf2/LinearMath/Transform.h>

#include <array>
#include <boost/noncopyable.hpp>
#include <path_tracking_pid/details/derivative.hpp>
#include <path_tracking_pid/details/fifo_array.hpp>
#include <path_tracking_pid/details/integral.hpp>
#include <path_tracking_pid/details/second_order_lowpass.hpp>
#include <vector>

namespace path_tracking_pid
{
struct TricycleSteeringCmdVel
{
  double steering_angle = 0.0;
  double steering_angle_velocity = 0.0;
  double speed = 0.0;
  double acceleration = 0.0;
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
    bool tricycle_model_enabled, const tf2::Transform & tf_base_to_steered_wheel);

  /**
   * Set plan
   * @param current Where is the robot now?
   * @param odom_twist Robot odometry
   * @param global_plan Plan to follow
   * @return whether the plan was successfully updated or not
   */
  bool setPlan(
    const tf2::Transform & current_tf, const geometry_msgs::Twist & odom_twist,
    const std::vector<tf2::Transform> & global_plan);

  /**
   * Set plan
   * @param current Where is the robot now?
   * @param odom_twist Robot odometry
   * @param tf_base_to_steered_wheel Where is the steered wheel now?
   * @param steering_odom_twist Steered wheel odometry
   * @param global_plan Plan to follow
   * @return whether the plan was successfully updated or not
   */
  bool setPlan(
    const tf2::Transform & current_tf, const geometry_msgs::Twist & odom_twist,
    const tf2::Transform & tf_base_to_steered_wheel,
    const geometry_msgs::Twist & steering_odom_twist,
    const std::vector<tf2::Transform> & global_plan);

  /** Result of findPoseOnPlan(). */
  struct FindPoseOnPlanResult
  {
    tf2::Transform pose;
    std::size_t path_pose_idx = 0;
    double distance_to_goal = 0;
    std::size_t last_visited_pose_index = 0;
  };

  /**
   * Find pose on plan by looking at the surroundings of last known pose.
   * @param[in]     current_tf        Where is the robot now?
   * @param[in,out] global_plan_index Global plan index where the search should start. Updated to
   *                                  current global plan index once found.
   * @return Found pose on plan and related data.
   */
  FindPoseOnPlanResult findPoseOnPlan(
    const tf2::Transform & current_tf, std::size_t & global_plan_index) const;

  // Result of update() and update_with_limits().
  struct UpdateResult
  {
    geometry_msgs::Twist velocity_command;  // Velocity command
    double eda = 0;                         // Estimated duration of arrival
    double progress = 0;                    // Progress along the path [0,1]
    PidDebug pid_debug;                     // Information to debug the controller
  };

  /**
   * Run one iteration of a PID controller
   * @param target_x_vel robot will try to reach target x velocity within (de)acceleration limits
   * @param current Where is the robot now?
   * @param odom_twist Robot odometry
   * @param dt Duration since last update
   * @return Update result
   */
  UpdateResult update(
    double target_x_vel, double target_end_x_vel, const tf2::Transform & current_tf,
    const geometry_msgs::Twist & odom_twist, ros::Duration dt);

  /**
   * Run one iteration of a PID controller with velocity limits applied
   * @param current Where is the robot now?
   * @param odom_twist Robot odometry
   * @param dt Duration since last update
   * @return Update result
   */
  UpdateResult update_with_limits(
    const tf2::Transform & current_tf, const geometry_msgs::Twist & odom_twist, ros::Duration dt);

  /**
   * Perform prediction steps on the lateral error and return a reduced velocity that stays within bounds
   * @param current_tf Where is the robot now?
   * @param odom_twist Robot odometry
   * @return Velocity command
   */
  double mpc_based_max_vel(
    double target_x_vel, const tf2::Transform & current_tf,
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

  /** Get current forward velocity. */
  double getCurrentForwardVelocity() const { return controller_state_.current_x_vel; }

  /** Get current yaw velocity. */
  double getCurrentYawVelocity() const { return controller_state_.current_yaw_vel; }

  /** Indicates if the end is reached. */
  bool isEndReached() const { return controller_state_.end_reached; }

  /** Gets the current global plan index. */
  std::size_t getCurrentGlobalPlanIndex() const
  {
    return controller_state_.current_global_plan_index;
  }

  // Set new vel_max_external value
  void setVelMaxExternal(double value);

  // Set new vel_max_obstacle value
  void setVelMaxObstacle(double value);

  // Get vel_max_obstacle value
  double getVelMaxObstacle() const;

private:
  struct ControllerState
  {
    size_t current_global_plan_index = 0;
    double current_x_vel = 0.0;
    double current_yaw_vel = 0.0;
    double previous_steering_angle = 0.0;
    double previous_steering_x_vel = 0.0;
    double previous_steering_yaw_vel = 0.0;
    bool end_phase_enabled = false;
    bool end_reached = false;
    double tracking_error_lat = 0.0;
    double tracking_error_ang = 0.0;
    // Errors with little history
    details::SecondOrderLowpass error_lat;
    details::SecondOrderLowpass error_ang;
    details::Integral error_integral_lat;
    details::Integral error_integral_ang;
    details::Derivative error_deriv_lat;
    details::Derivative error_deriv_ang;
  };

  geometry_msgs::Twist computeTricycleModelForwardKinematics(double x_vel, double steering_angle);
  TricycleSteeringCmdVel computeTricycleModelInverseKinematics(
    const geometry_msgs::Twist & cmd_vel);
  /**
   * Output some debug information about the current parameters
   */
  void printParameters() const;

  path_tracking_pid::PidConfig config_;
  ControllerState controller_state_;

  // Global Plan variables
  std::vector<tf2::Transform> global_plan_tf_;     // Global plan vector
  std::vector<double> distance_to_goal_vector_;    // Vector with distances to goal
  std::vector<double> turning_radius_inv_vector_;  // Vector with computed turning radius inverse
  tf2::Transform current_goal_;
  tf2::Transform current_pos_on_plan_;
  tf2::Transform current_with_carrot_;

  // Auxiliary variables
  double current_target_x_vel_ = 0.0;
  double control_effort_long_ = 0.0;  // output of pid controller
  double control_effort_lat_ = 0.0;   // output of pid controller
  double control_effort_ang_ = 0.0;   // output of pid controller

  bool enabled_ = true;
  bool holonomic_robot_enable_ = false;
  bool estimate_pose_angle_ = false;

  // feedforward controller
  double feedforward_lat_ = 0.0;
  double feedforward_ang_ = 0.0;

  // tricycle model
  bool use_tricycle_model_ = false;
  tf2::Transform tf_base_to_steered_wheel_;
  std::array<std::array<double, 2>, 2> inverse_kinematics_matrix_{};
  std::array<std::array<double, 2>, 2> forward_kinematics_matrix_{};

  // Velocity limits that can be active external to the pid controller:
  double vel_max_external_ =
    INFINITY;  // Dynamic external max velocity requirement (e.g. no more power available)
  double vel_max_obstacle_ = INFINITY;  // Can be zero if lethal obstacles are detected
};

}  // namespace path_tracking_pid
