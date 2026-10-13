"""Launch ekf_node + fake_star_tracker with topic remaps for ~/rover_ws."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("ekf_loc"), "config", "ekf_loc.yaml")
    return LaunchDescription([
        Node(
            package="ekf_loc",
            executable="ekf_node",
            parameters=[params],
            remappings=[
                ("imu", "/imu"),
                ("wheel_odom", "/diff_drive_controller/odom"),
                ("star_tracker", "/star_tracker"),
            ],
            output="screen",
        ),
        Node(
            package="ekf_loc",
            executable="fake_star_tracker",
            parameters=[params],
            remappings=[("ground_truth", "/odometry/ground_truth")],
            output="screen",
        ),
    ])
