"""ROS 2 wrapper around the standalone EKF (ekf_core.py).

Subscribes:  imu (sensor_msgs/Imu)              -> predict   (gyro z = process input)
             wheel_odom (nav_msgs/Odometry)     -> update    (twist.linear.x = wheel speed)
             star_tracker (sensor_msgs/Imu)     -> update    (orientation -> yaw = heading)
Publishes:   odometry/ekf (nav_msgs/Odometry) + TF odom_frame -> base_frame

The filter clock is message timestamps (use_sim_time), never wall time.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from ekf_loc.ekf_core import (
    EKF,
    ProcessInput,
    UnicycleBiasModel,
    heading_measurement,
    wheel_speed_measurement,
)

IDX_X, IDX_Y, IDX_THETA, IDX_V = 0, 1, 2, 3
MAX_PREDICT_DT = 0.5  # s; larger gaps mean dropped data or a sim reset -> skip, don't integrate


def stamp_to_sec(stamp) -> float:
    return Time.from_msg(stamp).nanoseconds * 1e-9


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class EkfNode(Node):
    def __init__(self) -> None:
        super().__init__("ekf_node")

        # --- parameters (see config/ekf_loc.yaml) ---
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        # child frame deliberately NOT base_footprint: ground_truth_tf.py already publishes odom->base_footprint
        self.base_frame = self.declare_parameter("base_frame", "base_footprint_ekf").value
        self.publish_tf = self.declare_parameter("publish_tf", True).value
        self.sigma_wheel = self.declare_parameter("sigma_wheel_speed", 0.05).value  # m/s
        self.sigma_heading = self.declare_parameter("sigma_heading", 0.005).value  # rad
        q_diag = self.declare_parameter("process_noise_diag", [1e-4, 1e-4, 1e-5, 1e-3, 1e-7]).value
        p0_diag = self.declare_parameter("initial_cov_diag", [0.01, 0.01, 0.01, 0.1, 1e-4]).value

        model = UnicycleBiasModel(np.diag(q_diag))
        self.ekf = EKF(model, np.zeros(5), np.diag(p0_diag))  # starts at origin, at rest, zero bias
        self.last_predict_t: float | None = None

        # --- I/O ---
        self.create_subscription(Imu, "imu", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, "wheel_odom", self.on_wheel_odom, 10)
        self.create_subscription(Imu, "star_tracker", self.on_star_tracker, 10)
        self.odom_pub = self.create_publisher(Odometry, "odometry/ekf", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(f"ekf_node up: publishing {self.odom_frame} -> {self.base_frame}")

    # --- process input ---
    def on_imu(self, msg: Imu) -> None:
        t = stamp_to_sec(msg.header.stamp)
        if self.last_predict_t is None:
            self.last_predict_t = t  # first message: nothing to integrate yet
            return
        dt = t - self.last_predict_t
        if dt <= 0.0 or dt > MAX_PREDICT_DT:
            self.get_logger().warn(f"skipping IMU sample with dt={dt:.3f} s", throttle_duration_sec=2.0)
            self.last_predict_t = t
            return
        # ASSUMPTION: imu_link axes aligned with the base frame (else rotate via static TF)
        self.ekf.predict(ProcessInput(t=t, gyro_z=msg.angular_velocity.z), dt)
        self.last_predict_t = t
        self.publish(msg.header.stamp)

    # --- measurements (applied at the current filter time; small timing error accepted) ---
    def on_wheel_odom(self, msg: Odometry) -> None:
        t = stamp_to_sec(msg.header.stamp)
        self.ekf.update(wheel_speed_measurement(t, msg.twist.twist.linear.x, self.sigma_wheel))

    def on_star_tracker(self, msg: Imu) -> None:
        t = stamp_to_sec(msg.header.stamp)
        yaw = yaw_from_quaternion(msg.orientation)
        self.ekf.update(heading_measurement(t, yaw, self.sigma_heading))

    # --- output ---
    def publish(self, stamp) -> None:
        x, P = self.ekf.x, self.ekf.P
        yaw = float(x[IDX_THETA])
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame  # pose expressed in this frame
        odom.child_frame_id = self.base_frame  # twist expressed in this frame
        odom.pose.pose.position.x = float(x[IDX_X])
        odom.pose.pose.position.y = float(x[IDX_Y])
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(x[IDX_V])

        # 6x6 row-major over (x, y, z, rotX, rotY, rotZ); unestimated z/roll/pitch get a large variance
        pose_cov = np.diag([0.0, 0.0, 1e6, 1e6, 1e6, 0.0])
        idx = [IDX_X, IDX_Y, IDX_THETA]
        ros_idx = [0, 1, 5]
        for i, ri in zip(idx, ros_idx):
            for j, rj in zip(idx, ros_idx):
                pose_cov[ri, rj] = P[i, j]
        odom.pose.covariance = pose_cov.flatten().tolist()
        twist_cov = np.diag([0.0, 1e6, 1e6, 1e6, 1e6, 1e6])
        twist_cov[0, 0] = P[IDX_V, IDX_V]
        odom.twist.covariance = twist_cov.flatten().tolist()
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = float(x[IDX_X])
            tf.transform.translation.y = float(x[IDX_Y])
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
