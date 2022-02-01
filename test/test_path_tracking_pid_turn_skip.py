#!/usr/bin/env python
import unittest
from math import cos, hypot, pi, sin

import rospy
import rostest
from actionlib import GoalStatus as GS
from actionlib import SimpleActionClient
from dynamic_reconfigure.client import Client as ReconfigureClient
from mbf_msgs.msg import ExePathAction, ExePathGoal
from nav_msgs.msg import Odometry
from numpy import linspace

from paths import create_path


def points_on_circle(angles, radius=1, center=(0, 0)):
    x = []
    y = []
    yaw = []
    for angle in angles:
        x.append(center[0] + (cos(angle) * radius))
        y.append(center[1] + (sin(angle) * radius))
        yaw.append(angle + pi*0.5)
    return (x, y, yaw)


class TestPathTrackingPID(unittest.TestCase):
    def setUp(self):
        self.cur_odom = Odometry()

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

        path0a = create_path([0.0, 3.5], [0.0, 0.0], [0.0, 0.0])
        path0b = create_path([3.5, 4.5], [0.0, 0.0], [0.0, 0.0])
        path0c = create_path([4.5, 5.0], [0.0, 0.0], [0.0, 0.0])
        (x_circ, y_circ, yaw_circ) = points_on_circle(linspace(-0.5*pi, -0.75*pi), radius=2.0, center=(5.0, 2.0))
        path1 = create_path(x_circ, y_circ, yaw_circ)
        path2 = create_path([cos(-0.75*pi)*2.0+5.0, cos(-0.75*pi)*2.0+10.0],
                            [sin(-0.75*pi)*2.0+2.0, sin(-0.75*pi)*2.0-3.0],
                            [-0.25*pi, -0.25*pi])

        paths = [path0a, path0b, path0c, path1, path2]
        speeds = [1.2, 1.7, 0.9, -0.6, 1.2]
        endspeeds = [0.0, 0.833, 0.0, 0.0, 1.66]

        for (path, speed, endspeed) in zip(paths, speeds, endspeeds):
            # Start goal and evaluate outcome
            outcome_exp = rospy.get_param("~outcome", GS.SUCCEEDED)
            reconfigure = ReconfigureClient("/move_base_flex/PathTrackingPID", timeout=5)
            reconfigure.update_configuration({"target_x_vel": speed})
            reconfigure.update_configuration({"target_end_x_vel": endspeed})
            reconfigure.update_configuration({"use_mpc": False})
            rospy.logwarn("Starting path!")
            client.send_goal(ExePathGoal(path=path))

            finished_in_time = client.wait_for_result(timeout=rospy.Duration(60))

            # Get end-pose error
            endpose_error = hypot(path.poses[-1].pose.position.x - self.cur_odom.pose.pose.position.x,
                                  path.poses[-1].pose.position.y - self.cur_odom.pose.pose.position.y)

            self.assertTrue(finished_in_time, msg="Action call didn't return in time")
            self.assertEqual(client.get_state(), outcome_exp, msg="Wrong action outcome")
            self.assertTrue(endpose_error < 1.0, msg=
                "Did not arrive on final path's pose! pose: {}, {} endpoint: {}, {}".format(
                    self.cur_odom.pose.pose.position.x,
                    self.cur_odom.pose.pose.position.y,
                    path.poses[-1].pose.position.x,
                    path.poses[-1].pose.position.y))


if __name__ == "__main__":
    rospy.init_node("rostest_path_tracking_pid_turn_skip", anonymous=False)
    rostest.rosrun("forth", "rostest_path_tracking_pid_turn_skip", TestPathTrackingPID)
