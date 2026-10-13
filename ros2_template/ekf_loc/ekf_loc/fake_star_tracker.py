"""Simulated star tracker: ground-truth yaw + Gaussian noise, decimated to `rate_hz`.

Subscribes:  ground_truth (nav_msgs/Odometry)
Publishes:   star_tracker (sensor_msgs/Imu, orientation only; angular_velocity/accel marked unavailable)

Stand-in for the real chain: inertial attitude -> lunar-local frame via ephemeris + position.
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu

from ekf_loc.ekf_node import yaw_from_quaternion


class FakeStarTracker(Node):
    def __init__(self) -> None:
        super().__init__("fake_star_tracker")
        self.rate_hz = self.declare_parameter("rate_hz", 1.0).value
        self.sigma = self.declare_parameter("sigma_heading", 0.005).value  # rad
        seed = self.declare_parameter("seed", 42).value
        self.rng = np.random.default_rng(seed)
        self.last_pub_t: float | None = None

        self.create_subscription(Odometry, "ground_truth", self.on_truth, 10)
        self.pub = self.create_publisher(Imu, "star_tracker", 10)

    def on_truth(self, msg: Odometry) -> None:
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.last_pub_t is not None and t - self.last_pub_t < 1.0 / self.rate_hz:
            return
        self.last_pub_t = t

        yaw_meas = yaw_from_quaternion(msg.pose.pose.orientation) + self.rng.normal(0.0, self.sigma)
        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.child_frame_id
        out.orientation.z = math.sin(yaw_meas / 2.0)
        out.orientation.w = math.cos(yaw_meas / 2.0)
        out.orientation_covariance[8] = self.sigma**2  # yaw variance; roll/pitch left at 0 (unused)
        out.angular_velocity_covariance[0] = -1.0  # ROS convention: -1 = field not provided
        out.linear_acceleration_covariance[0] = -1.0
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeStarTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
