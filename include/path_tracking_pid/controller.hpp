#pragma once

#include <array>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "path_tracking_pid/PidConfig.h"
#include "path_tracking_pid/PidDebug.h"
#include "tf2/LinearMath/Transform.h"

// Typesafe sign implementation with signum:
// https://stackoverflow.com/a/4609795
template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

namespace path_tracking_pid
{

inline constexpr double RADIUS_EPS = 0.001; // Smallest relevant radius [m]
inline constexpr double VELOCITY_EPS = 1e-3; // Neglegible velocity
inline constexpr double LONG_DURATION = 31556926; // A year (ros::Duration cannot be inf)

enum class ControllerMode
{
  frontAxleLateral = 0,
  rearAxleLateral = 1,
  rearAxleAngular = 2,
  fixOrientation = 3,
};

struct TricycleSteeringCmdVel
{
  double steering_angle = 0.0;
  double steering_angle_velocity = 0.0;
  double speed = 0.0;
  double acceleration = 0.0;
};

struct ControllerState
{
  size_t current_global_plan_index = 0;
  size_t last_visited_pose_index = 0;
  double current_x_vel = 0.0;
  double current_yaw_vel = 0.0;
  double previous_steering_angle = 0.0;
  double previous_steering_x_vel = 0.0;
  double previous_steering_yaw_vel = 0.0;
  bool end_phase_enabled = false;
  bool end_reached = false;
  double error_integral_lat = 0.0;
  double error_integral_ang = 0.0;
  double tracking_error_lat = 0.0;
  double tracking_error_ang = 0.0;
  // Errors with little history
  std::array<double, 3> error_lat = {0.0, 0.0, 0.0};
  std::array<double, 3> filtered_error_lat = {0.0, 0.0, 0.0};
  std::array<double, 3> error_deriv_lat = {0.0, 0.0, 0.0};
  std::array<double, 3> filtered_error_deriv_lat = {0.0, 0.0, 0.0};
  std::array<double, 3> error_ang = {0.0, 0.0, 0.0};
  std::array<double, 3> filtered_error_ang = {0.0, 0.0, 0.0};
  std::array<double, 3> error_deriv_ang = {0.0, 0.0, 0.0};
  std::array<double, 3> filtered_error_deriv_ang = {0.0, 0.0, 0.0};
};

class Controller
{
public:
  Controller() = default;

  ~Controller() = default;

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
  void setTricycleModel(bool tricycle_model_enabled, geometry_msgs::Transform tf_base_to_steered_wheel);

  /**
   * Set plan
   * @param current Where is the robot now?
   * @param odom_twist Robot odometry
   * @param global_plan Plan to follow
   */
  void setPlan(geometry_msgs::Transform current_tf, geometry_msgs::Twist odom_twist,
               const std::vector<geometry_msgs::PoseStamped>& global_plan);

  /**
   * Set plan
   * @param current Where is the robot now?
   * @param odom_twist Robot odometry
   * @param tf_base_to_steered_wheel Where is the steered wheel now?
   * @param steering_odom_twist Steered wheel odometry
   * @param global_plan Plan to follow
   */
  void setPlan(geometry_msgs::Transform current_tf, geometry_msgs::Twist odom_twist,
                           geometry_msgs::Transform tf_base_to_steered_wheel, geometry_msgs::Twist steering_odom_twist,
                           const std::vector<geometry_msgs::PoseStamped>& global_plan);
  /**
   * Find position on plan by looking at the surroundings of last known pose.
   * @param current Where is the robot now?
   * @param controller_state_ptr  The current state of the controller that gets updated by this function
   * @return tf of found position on plan
   * @return index of current path-pose if requested
   */
  tf2::Transform findPositionOnPlan(const geometry_msgs::Transform current_tf,
                                    ControllerState* controller_state_ptr,
                                    size_t &path_pose_idx);
  // Overloaded function definition for users that don't require the segment index
  tf2::Transform findPositionOnPlan(const geometry_msgs::Transform current_tf,
                                    ControllerState* controller_state_ptr)
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
  geometry_msgs::Twist update(const double target_x_vel,
                              const double target_end_x_vel,
                              const geometry_msgs::Transform current_tf,
                              const geometry_msgs::Twist odom_twist,
                              const ros::Duration dt,
                              double* eda, double* progress, path_tracking_pid::PidDebug* pid_debug);

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
  geometry_msgs::Twist update_with_limits(const geometry_msgs::Transform current_tf,
                                          const geometry_msgs::Twist odom_twist,
                                          const ros::Duration dt,
                                          double* eda, double* progress, path_tracking_pid::PidDebug* pid_debug);

  /**
   * Perform prediction steps on the lateral error and return a reduced velocity that stays within bounds
   * @param current_tf Where is the robot now?
   * @param odom_twist Robot odometry
   * @return Velocity command
   */
  double mpc_based_max_vel(const double target_x_vel, geometry_msgs::Transform current_tf,
                           geometry_msgs::Twist odom_twist);

  /**
   * Select mode for the controller
   * @param mode
   */
  void selectMode(ControllerMode mode);

  /**
   * Set dynamic parameters for the PID controller
   * @param config
   */
  void configure(path_tracking_pid::PidConfig& config);

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
  tf2::Transform getCurrentGoal() const
  {
    return current_goal_;
  }
  tf2::Transform getCurrentWithCarrot() const
  {
    return current_with_carrot_;
  }
  tf2::Transform getCurrentPosOnPlan() const
  {
    return current_pos_on_plan_;
  }

  // Inline get-function for controller-state
  ControllerState getControllerState() const
  {
    return controller_state_;
  }

  // Set new vel_max_external value
  void setVelMaxExternal(double value);

  // Set new vel_max_obstacle value
  void setVelMaxObstacle(double value);

  // Get vel_max_obstacle value
  double getVelMaxObstacle() const;

private:
  double distSquared(const tf2::Transform& pose_v, const tf2::Transform& pose_w) const;
  double distSquared(const geometry_msgs::Pose& pose_v, const geometry_msgs::Pose& pose_w) const;
  void distToSegmentSquared(const tf2::Transform& pose_p, const tf2::Transform& pose_v, const tf2::Transform& pose_w,
                            tf2::Transform& pose_projection, double& distance_to_p, double& distance_to_w);

  // Overloaded function for callers that don't need the additional results
  double distToSegmentSquared(const tf2::Transform& pose_p, const tf2::Transform& pose_v, const tf2::Transform& pose_w)
  {
    tf2::Transform dummy_tf;
    double dummy_double;
    double result;
    distToSegmentSquared(pose_p,pose_v, pose_w, dummy_tf, result, dummy_double);
    return result;
  }

  geometry_msgs::Twist computeTricycleModelForwardKinematics(double x_vel, double steering_angle);
  TricycleSteeringCmdVel computeTricycleModelInverseKinematics(geometry_msgs::Twist cmd_vel);
  /**
   * Output some debug information about the current parameters
   */
  void printParameters();

  path_tracking_pid::PidConfig local_config_;
  ControllerState controller_state_ = ControllerState();

  // Global Plan variables
  std::vector<tf2::Transform> global_plan_tf_;  // Global plan vector
  std::vector<double> distance_to_goal_vector_;  // Vector with distances to goal
  std::vector<double> turning_radius_inv_vector_;  // Vector with computed turning radius inverse
  double distance_to_goal_;
  tf2::Transform current_goal_;
  tf2::Transform current_pos_on_plan_;
  tf2::Transform current_with_carrot_;

  // Auxiliary variables
  double current_target_x_vel_ = 0.0;
  double control_effort_long_ = 0.0;  // output of pid controller
  double control_effort_lat_ = 0.0;   // output of pid controller
  double control_effort_ang_ = 0.0;   // output of pid controller

  bool enabled_ = true;
  bool feedback_lat_enabled_ = false;
  bool feedback_ang_enabled_ = false;
  bool feedforward_lat_enabled_ = false;
  bool feedforward_ang_enabled_ = false;
  bool holonomic_robot_enable_ = false;
  bool track_base_link_enabled_ = false;
  bool estimate_pose_angle_enabled_ = false;

  // feedforward controller
  double feedforward_lat_ = 0.0;
  double feedforward_ang_ = 0.0;
  double xvel_ = 0.0;
  double yvel_ = 0.0;
  double thvel_ = 0.0;

  // tricycle model
  bool use_tricycle_model_ = false;
  geometry_msgs::Transform tf_base_to_steered_wheel_;
  double max_steering_angle_;
  double max_steering_x_vel_;
  double max_steering_x_acc_;
  double max_steering_yaw_vel_;
  double max_steering_yaw_acc_;
  double inverse_kinematics_matrix_[2][2];
  double forward_kinematics_matrix_[2][2];

  bool debug_enabled_ = false;

  // Primary feedback controller parameters
  double Kp_lat_ = 0.0;
  double Ki_lat_ = 0.0;
  double Kd_lat_ = 0.0;
  double Kp_ang_ = 0.0;
  double Ki_ang_ = 0.0;
  double Kd_ang_ = 0.0;
  double l_ = 0.0;
  double target_x_vel_ = 0.0;
  double target_end_x_vel_ = 0.0;
  double target_x_acc_ = 0.0;
  double target_x_decc_ = 0.0;
  double max_error_x_vel_ = 0.0;
  double abs_minimum_x_vel_ = 0.0;
  double max_yaw_vel_ = 0.0;
  double max_yaw_acc_ = 0.0;
  double minimum_turning_radius_ = FLT_EPSILON;

  // Velocity limits that can be active external to the pid controller:
  double vel_max_external_ = INFINITY;  // Dynamic external max velocity requirement (e.g. no more power available)
  double vel_max_obstacle_ = INFINITY;  // Can be zero if lethal obstacles are detected

  // Cutoff frequency for the derivative calculation in Hz.
  // Negative -> Has not been set by the user yet, so use a default.
  double cutoff_frequency_long_ = -1.0;
  double cutoff_frequency_lat_ = -1.0;
  double cutoff_frequency_ang_ = -1.0;

  // Upper and lower saturation limits
  double upper_limit_ = 100.0;
  double lower_limit_ = -100.0;

  double ang_upper_limit_ = 100.0;
  double ang_lower_limit_ = -100.0;

  // Anti-windup term. Limits the absolute value of the integral term.
  double windup_limit_ = 1000.0;

  // Temporary variables

  double proportional_lat_ = 0;  // proportional term of output
  double integral_lat_ = 0;      // integral term of output
  double derivative_lat_ = 0;    // derivative term of output
  double proportional_ang_ = 0;  // proportional term of output
  double integral_ang_ = 0;      // integral term of output
  double derivative_ang_ = 0;    // derivative term of output

  // Used in filter calculations. Default 1.0 corresponds to a cutoff frequency at
  // 1/4 of the sample rate.
  double c_lat_ = 1.;
  double c_ang_ = 1.;

  // Used to check for tan(0)==>NaN in the filter calculation
  double tan_filt_ = 1.;

  // MPC settings
  int mpc_max_fwd_iter_;               // Define # of steps that you look into the future with MPC [-]
  int mpc_max_vel_optimization_iter_;  // Set maximum # of velocity bisection iterations
                                       // (maximum total iterations = max_opt_iter*max_iter) [-]
  double mpc_simulation_sample_time_;  // Define timestep [s]
  double mpc_max_error_lat_;           // Maximum allowed lateral error [m]
  double mpc_min_x_vel_;               // Minimum forward x velocity [m/s]
};
}  // namespace path_tracking_pid
