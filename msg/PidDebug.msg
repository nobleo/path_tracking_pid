# Error topic containing the 'control' error on which the PID acts
geometry_msgs/Twist control_error
# Error topic containing the 'tracking' error, i.e. the real error between path and tracked link
geometry_msgs/Twist tracking_error
# Control values
geometry_msgs/Twist proportional
geometry_msgs/Twist integral
geometry_msgs/Twist derivative
geometry_msgs/Twist feedforward
float32 steering_angle
float32 steering_yaw_vel
float32 steering_x_vel
