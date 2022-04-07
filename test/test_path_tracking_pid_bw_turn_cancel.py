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
from numpy import linspace, append
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

        (x_circ, y_circ, yaw_circ) = points_on_circle(linspace(-0.5*pi, 0.5*pi, num=300), radius=5.0, center=(0.0, 4.0))
        # path = create_path(x_circ+x_circ+x_circ, y_circ+y_circ+y_circ, yaw_circ+yaw_circ+yaw_circ)
        path = create_path(x_circ, y_circ, yaw_circ)
        path_straight = create_path([0.0, -10.0], [9.0, 9.0], [pi, pi])


        speed = 0.6
        endspeed = 0.6

        # Start goal and cancel in between
        reconfigure = ReconfigureClient("/move_base_flex/PathTrackingPID", timeout=5)
        reconfigure.update_configuration({"target_x_vel": speed})
        reconfigure.update_configuration({"l": 2.0})
        reconfigure.update_configuration({"Kp_lat": 0.5})
        reconfigure.update_configuration({"Ki_lat": 0.00})
        reconfigure.update_configuration({"Kd_lat": 0.0})
        reconfigure.update_configuration({"feedback_lat_tracking_error": True})
        reconfigure.update_configuration({"Kp_lat_track_err": 10.0})
        reconfigure.update_configuration({"max_gp_abs_rot_fb_lat_terr": 30.0})
        reconfigure.update_configuration({"feedback_ang": False})
        reconfigure.update_configuration({"Kp_ang": 0.5})
        reconfigure.update_configuration({"Kd_ang": 0.0})
        reconfigure.update_configuration({"target_end_x_vel": endspeed})
        reconfigure.update_configuration({"target_x_acc": 0.5})
        reconfigure.update_configuration({"target_x_decc": 0.5})
        reconfigure.update_configuration({"use_mpc": False})
        reconfigure.update_configuration({"feedforward_lat": False})
        reconfigure.update_configuration({"feedforward_ang": True})

        # We require debug enabled here to retrieve the markers!
        reconfigure.update_configuration({"controller_debug_enabled": True})
        rospy.logwarn("Starting path!")
        client.send_goal(ExePathGoal(path=path))
        rospy.sleep(1)

        preempt_in_time = client.wait_for_result(timeout=rospy.Duration(1000))
        rospy.logwarn(f"Got result: {client.get_state()}")
        self.assertTrue(preempt_in_time, msg="Action call didn't preempt in time")

        speed = 2.0
        endspeed = 0.0
        reconfigure.update_configuration({"target_x_vel": speed})
        reconfigure.update_configuration({"target_end_x_vel": endspeed})
        client.send_goal(ExePathGoal(path=path_straight))
        rospy.sleep(1)

        preempt_in_time = client.wait_for_result(timeout=rospy.Duration(1000))
        rospy.logwarn(f"Got result: {client.get_state()}")
        self.assertTrue(preempt_in_time, msg="Action call didn't preempt in time")

if __name__ == "__main__":
    rospy.init_node("rostest_path_tracking_pid_bw_turn_cancel", anonymous=False)
    rostest.rosrun("forth", "rostest_path_tracking_pid_bw_turn_cancel", TestPathTrackingPID)
