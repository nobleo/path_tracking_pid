#!/usr/bin/env python
import unittest
from math import cos, hypot, pi, sin

import rospy
import rostest
from actionlib import GoalStatus as GS
from actionlib import SimpleActionClient
from dynamic_reconfigure.client import Client as ReconfigureClient
from geometry_msgs.msg import Point, Pose
from math import atan2
from mbf_msgs.msg import ExePathAction, ExePathGoal
from nav_msgs.msg import Odometry
from numpy import linspace
from visualization_msgs.msg import Marker

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

    def vis_cb(self, msg):
        if msg.ns == 'control point':
            self.carrot = msg.pose.position
        elif msg.ns == 'plan point':
            self.pos_on_plan = msg.pose.position

        # Only start checking when both markers are received
        if self.carrot is None or self.pos_on_plan is None:
            return

        angle = atan2(self.carrot.y - self.pos_on_plan.y,
                      self.carrot.x - self.pos_on_plan.x)
        # Check if marker direction doesn't flip, if it did, hold this boolean
        if not self.carrot_dir_flipped and self.marker_angle is not None:
            angle_diff = angle - self.marker_angle
            # 'Wrap'
            if angle_diff > pi:
                angle_diff -= 2*pi
            elif angle_diff < -pi:
                angle_diff += 2*pi
            # Check if angle flipped
            self.carrot_dir_flipped = abs(angle_diff) > 0.8*pi
        self.marker_angle = angle

    def test_exepath_action(self):

        # Setup action client to ask question
        client = SimpleActionClient("move_base_flex/exe_path", ExePathAction)
        self.assertTrue(
            client.wait_for_server(timeout=rospy.Duration(10)),
            msg="No actionclient for recipe_control found",
        )

        self.carrot = None
        self.pos_on_plan = None
        self.marker_angle = None
        self.carrot_dir_flipped = None
        # Subscribe to visualization-markers to track control direction
        self.vis_sub = rospy.Subscriber("move_base_flex/PathTrackingPID/visualization_marker", Marker, self.vis_cb)

        (x_circ, y_circ, yaw_circ) = points_on_circle(linspace(-0.5*pi, -0.75*pi), radius=3.0, center=(0.0, 3.0))
        path = create_path(x_circ, y_circ, yaw_circ)

        speed = -0.6
        endspeed = 0

        # Start goal and cancel in between
        reconfigure = ReconfigureClient("/move_base_flex/PathTrackingPID", timeout=5)
        reconfigure.update_configuration({"target_x_vel": speed})
        reconfigure.update_configuration({"target_end_x_vel": endspeed})
        reconfigure.update_configuration({"target_x_acc": 0.5})
        reconfigure.update_configuration({"target_x_decc": 0.5})
        reconfigure.update_configuration({"use_mpc": False})
        # We require debug enabled here to retrieve the markers!
        reconfigure.update_configuration({"controller_debug_enabled": True})
        rospy.logwarn("Starting path!")
        client.send_goal(ExePathGoal(path=path))

        rospy.sleep(2.0)
        rospy.logwarn("Preempting path!")
        client.cancel_goal()
        rospy.logwarn("Wait for result")
        preempt_in_time = client.wait_for_result(timeout=rospy.Duration(10))
        rospy.logwarn("Got result")
        self.assertTrue(preempt_in_time, msg="Action call didn't preempt in time")

        self.assertTrue(self.carrot_dir_flipped is False, msg="Guiding direction flipped while stopping!")


if __name__ == "__main__":
    rospy.init_node("rostest_path_tracking_pid_bw_turn_cancel", anonymous=False)
    rostest.rosrun("forth", "rostest_path_tracking_pid_bw_turn_cancel", TestPathTrackingPID)
