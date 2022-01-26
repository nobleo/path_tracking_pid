#!/usr/bin/env python
import unittest
from math import hypot

import rospy
import rostest
from actionlib import GoalStatus as GS
from actionlib import SimpleActionClient
from dynamic_reconfigure.client import Client as ReconfigureClient
from mbf_msgs.msg import ExePathAction, ExePathGoal
from nav_msgs.msg import Odometry

from paths import create_path


class TestPathTrackingPID(unittest.TestCase):
    def setUp(self):
        self.cur_odom = Odometry()

    def reconfigure(self, end_vel=0.0):
        reconfigure = ReconfigureClient("/move_base_flex/PathTrackingPID", timeout=5)
        reconfigure.update_configuration({"target_x_vel": 2.0})
        reconfigure.update_configuration({"target_end_x_vel": end_vel})
        reconfigure.update_configuration({"target_x_acc": 2.0})
        reconfigure.update_configuration({"target_x_decc": 1.0})
        reconfigure.update_configuration({"use_mpc": False})

    def odom_cb(self, msg):
        self.cur_odom = msg

    def test_exepath_action(self):

        # Setup action client to ask question
        client = SimpleActionClient("move_base_flex/exe_path", ExePathAction)
        self.assertTrue(
            client.wait_for_server(timeout=rospy.Duration(10)),
            msg="No actionclient for recipe_control found",
        )

        # Subscribe to pose messages from
        self.pose_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)

        path = create_path([0.0, 2.0], [0.0, 0.0], [0.0, 0.0])

        # Start goal and evaluate outcome
        outcome_exp = rospy.get_param("~outcome", GS.SUCCEEDED)
        self.reconfigure(end_vel = 1.0)
        rospy.logwarn("Starting path!")
        client.send_goal(ExePathGoal(path=path))

        # Preempt action after 1.5s (in deceleration phase)
        rospy.sleep(1.5)
        rospy.logwarn("Preempting path!")
        client.cancel_goal()
        rospy.logwarn("Wait for result")
        preempt_in_time = client.wait_for_result(timeout=rospy.Duration(10))
        rospy.logwarn("Got result")
        self.assertTrue(preempt_in_time, msg="Action call didn't preempt in time")
        # self.assertEqual(client.get_state(), GS.ABORTED, msg="Action didn't preempt on request")

        # # Resume action
        rospy.logwarn("Reconfiguring for resume")
        self.reconfigure(end_vel = 0.0)
        rospy.logwarn("Creating resume path")
        path = create_path([2.0, 4.0], [0.0, 0.0], [0.0, 0.0])
        rospy.logwarn("Sending new goal")
        client.send_goal(ExePathGoal(path=path))
        rospy.logwarn("Goal sent!")
        rospy.sleep(1.0)
        rospy.logwarn("Waiting for new result")
        self.assertEqual(client.get_state(), GS.ACTIVE, msg="Action didn't restart on request")
        rospy.logwarn("FINISHED!")

        finished_in_time = client.wait_for_result(timeout=rospy.Duration(5))

        # Get end-pose error
        endpose_error = hypot(path.poses[-1].pose.position.x - self.cur_odom.pose.pose.position.x,
                                path.poses[-1].pose.position.y - self.cur_odom.pose.pose.position.y)

        self.assertTrue(finished_in_time, msg="Action call didn't return in time")
        self.assertEqual(client.get_state(), outcome_exp, msg="Wrong action outcome")
        self.assertTrue(endpose_error < 0.5, msg="Did not arrive on final path's pose! \
                                                    pose: {}, {} endpoint: {}, {}".format(
                                                        self.cur_odom.pose.pose.position.x,
                                                        self.cur_odom.pose.pose.position.y,
                                                        path.poses[-1].pose.position.x,
                                                        path.poses[-1].pose.position.y))


if __name__ == "__main__":
    rospy.init_node("rostest_path_tracking_pid_preempt", anonymous=False)
    rostest.rosrun("forth", "rostest_path_tracking_pid_preempt", TestPathTrackingPID)
