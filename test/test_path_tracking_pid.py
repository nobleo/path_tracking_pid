#!/usr/bin/env python
import unittest

import rospy
import rostest
from actionlib import GoalStatus as GS
from actionlib import SimpleActionClient
from dynamic_reconfigure.client import Client as ReconfigureClient
from geometry_msgs.msg import Point32, PoseWithCovarianceStamped, Twist
from mbf_msgs.msg import ExePathAction, ExePathGoal
from path_tracking_pid.msg import PidDebug
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler

from paths import back_and_forth, reverse_path


class SlowDownChecker(object):
    def __init__(self, target_vel):
        self.target_vel = target_vel
        self.slowed_down = False
        self.kept_slow = True  # Flag to see if we did not accidentaly speed up again
        self.vel = 0.0
        rospy.Subscriber("cmd_vel", Twist, self.cb)

    def cb(self, msg):
        decelerating = msg.linear.x < self.vel
        # if msg.linear.x < self.vel:
        self.vel = msg.linear.x  # deceleration phase
        if not self.slowed_down and decelerating and self.vel < self.target_vel + 0.01:
            rospy.loginfo("Slowed down to {}m/s".format(self.target_vel))
            self.slowed_down = True
        if self.slowed_down and self.kept_slow and self.vel > self.target_vel + 0.1:
            rospy.logwarn("Sped up again while limit is still active")
            self.kept_slow = False

class ErrorCatcher(object):
    def __init__(self):
        self.error = Twist()
        rospy.Subscriber("move_base_flex/PathTrackingPID/debug", PidDebug, self.cb)

    def cb(self, msg):
        self.error = msg.tracking_error

class TestPathTrackingPID(unittest.TestCase):
    def test_exepath_action(self):
        # Set inital pose to parameter
        if rospy.has_param("~initialpose"):
            values = rospy.get_param("~initialpose")
            rospy.loginfo("Initial pose to {}".format(values))
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = "map"
            pose.pose.pose.position.x = values[0]
            pose.pose.pose.position.y = values[1]
            quat = quaternion_from_euler(0, 0, values[2])
            pose.pose.pose.orientation.x = quat[0]
            pose.pose.pose.orientation.y = quat[1]
            pose.pose.pose.orientation.z = quat[2]
            pose.pose.pose.orientation.w = quat[3]
            initialpose_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, latch=True, queue_size=1)
            initialpose_pub.publish(pose)
            rospy.sleep(0.1)  # Fill tf buffers

        self.max_tracking_error_linear_x = rospy.get_param("~max_tracking_error_linear_x", 0.1)
        self.max_tracking_error_linear_y = rospy.get_param("~max_tracking_error_linear_y", 0.1)
        self.max_tracking_error_angular_z = rospy.get_param("~max_tracking_error_angular_z", 0.1)

        # Publisher for obstacles:
        self.obstacle_pub = rospy.Publisher("pointcloud", PointCloud, latch=True, queue_size=1)
        reconfigure = ReconfigureClient("/move_base_flex/PathTrackingPID", timeout=5)

        # Setup action client to ask question
        client = SimpleActionClient("move_base_flex/exe_path", ExePathAction)
        self.assertTrue(
            client.wait_for_server(timeout=rospy.Duration(10)),
            msg="No actionclient for recipe_control found",
        )

        # Spawn desired obstacles:
        pc = PointCloud()
        pc.header.stamp = rospy.Time.now()
        pc.header.frame_id = "map"
        for point in rospy.get_param("~obstacles", []):
            pc.points.append(Point32(x=point[0], y=point[1]))
        self.obstacle_pub.publish(pc)

        path = back_and_forth(rospy.get_param("~radius", 5.0))

        # Listen to errors:
        error_catcher = ErrorCatcher()

        # Start goal and evaluate outcome
        outcome_exp = rospy.get_param("~outcome", GS.SUCCEEDED)
        client.send_goal(ExePathGoal(path=path))

        max_vel = rospy.get_param("~max_vel", 0.0)
        if max_vel > 0.0:
            reconfigure.update_configuration({"target_end_x_vel": 5.0})
            checker = SlowDownChecker(max_vel)
            max_vel_pub = rospy.Publisher("move_base_flex/PathTrackingPID/vel_max", Float64, queue_size=1)
            rospy.sleep(3.0)  # Accelerate first
            max_vel_pub.publish(max_vel)
            rospy.sleep(10.0)
            self.assertTrue(checker.slowed_down)

        finished_in_time = client.wait_for_result(timeout=rospy.Duration(120))
        self.assertTrue(finished_in_time, msg="Action call didn't return in time")
        self.assertEqual(client.get_state(), outcome_exp, msg="Wrong action outcome")

        if max_vel > 0.0:
            self.assertTrue(checker.kept_slow)
            return

        # Check the errors
        self.assertLess(error_catcher.error.linear.x, self.max_tracking_error_linear_x)
        self.assertLess(error_catcher.error.linear.y, self.max_tracking_error_linear_y)
        self.assertLess(error_catcher.error.angular.z, self.max_tracking_error_angular_z)

        # Do the same for backward movements if last path was a success
        if client.get_state() != GS.SUCCEEDED or rospy.get_param("backward", True):
            return

        reconfigure.update_configuration({"target_x_vel": -5.0})
        client.send_goal(ExePathGoal(path=reverse_path(path)))

        finished_in_time = client.wait_for_result(timeout=rospy.Duration(60))
        self.assertTrue(finished_in_time, msg="Action call didn't return in time")
        self.assertEqual(client.get_state(), outcome_exp, msg="Wrong action outcome")


if __name__ == "__main__":
    rospy.init_node("rostest_path_tracking_pid", anonymous=False)
    rostest.rosrun("back_and_forth", "rostest_path_tracking_pid", TestPathTrackingPID)
