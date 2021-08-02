#!/usr/bin/env python
from copy import deepcopy
from math import cos, pi, sin

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from numpy import linspace
from tf.transformations import quaternion_from_euler


def points_on_circle(angles, radius=1, center=(0, 0)):
    return [
        (
            center[0] + (sin(angle) * radius),  # x
            center[1] - (cos(angle) * radius),  # y
            angle,  # yaw
        )
        for angle in angles
    ]


def back_and_forth(radius):
    # Construct path to go 10m forward, turn and go back
    path = Path()
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.orientation.w = 1.0
    pose.pose.position.x = 0.0
    path.poses.append(deepcopy(pose))

    for (x, y, yaw) in points_on_circle(linspace(0, pi), radius=radius, center=(10, radius)):
        pose.pose.position.x = x
        pose.pose.position.y = y
        quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        path.poses.append(deepcopy(pose))

    pose.pose.position.x = 0
    path.poses.append(deepcopy(pose))
    return path


def reverse_path(path):
    path.poses.reverse()
    return path


def create_path(x, y, yaw):
    # Construct a path based on three vectors
    path = Path()
    pose = PoseStamped()
    pose.header.frame_id = "map"

    for (x, y, yaw) in zip(x, y, yaw):
        pose.pose.position.x = x
        pose.pose.position.y = y
        quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        path.poses.append(deepcopy(pose))

    return path
