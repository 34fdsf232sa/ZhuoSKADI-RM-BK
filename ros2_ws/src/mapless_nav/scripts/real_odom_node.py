#!/usr/bin/env python3
"""
Real Odometry Node — Multi-backend odometry for mapless navigation.

Backends:
  - l2_imu: Basic IMU dead reckoning from Unitree L2 (/unilidar/imu)
  - external: Relay odometry from an external source (e.g. FAST_LIO, slam_toolbox)
  - fake: Static odometry for testing (same as fake_odom_node)

Publishes:
  /odom (nav_msgs/Odometry)
  odom -> base_link TF (via tf2 TransformBroadcaster)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from tf2_ros import TransformBroadcaster

import numpy as np
import math
import time


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def quaternion_to_euler(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


class RealOdomNode(Node):
    def __init__(self):
        super().__init__('real_odom_node')

        # Parameters
        self.declare_parameter('mode', 'l2_imu')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_topic', '/unilidar/imu')
        self.declare_parameter('external_odom_topic', '/odom_raw')

        # IMU filter parameters
        self.declare_parameter('imu_gyro_threshold', 0.001)    # rad/s
        self.declare_parameter('imu_accel_threshold', 0.05)     # m/s^2
        self.declare_parameter('velocity_decay', 0.95)          # per-frame velocity decay

        self.mode = self.get_parameter('mode').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # State
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        self.last_imu_time = None
        self.last_predict_time = time.time()

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers (mode-dependent)
        if self.mode == 'l2_imu':
            self.imu_sub = self.create_subscription(
                Imu, self.get_parameter('imu_topic').value,
                self.imu_callback, sensor_qos)
            self.get_logger().info('Mode: l2_imu — using L2 IMU dead reckoning')

        elif self.mode == 'external':
            self.external_sub = self.create_subscription(
                Odometry, self.get_parameter('external_odom_topic').value,
                self.external_odom_callback, sensor_qos)
            self.get_logger().info('Mode: external — relaying external odometry')

        elif self.mode == 'fake':
            self.get_logger().info('Mode: fake — static odometry for testing')

        else:
            self.get_logger().warn(f'Unknown mode: {self.mode}, falling back to fake')
            self.mode = 'fake'

        # Timer for publishing odometry
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_odom)

        self.get_logger().info(
            f'Real odom node started: {self.odom_frame} -> {self.base_frame} @ {self.publish_rate}Hz'
        )

    def imu_callback(self, msg: Imu):
        """L2 IMU-based dead reckoning."""
        now = time.time()

        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y

        if self.last_imu_time is not None:
            dt = now - self.last_imu_time
            if dt <= 0 or dt > 0.5:
                self.last_imu_time = now
                return

            # Threshold filtering
            gyro_thresh = self.get_parameter('imu_gyro_threshold').value
            accel_thresh = self.get_parameter('imu_accel_threshold').value

            if abs(gyro_z) < gyro_thresh:
                gyro_z = 0.0

            # Integrate yaw
            self.yaw += gyro_z * dt
            self.vth = gyro_z

            # World-frame acceleration (approximate)
            cos_yaw = math.cos(self.yaw)
            sin_yaw = math.sin(self.yaw)
            ax_world = accel_x * cos_yaw - accel_y * sin_yaw
            ay_world = accel_x * sin_yaw + accel_y * cos_yaw

            if abs(ax_world) < accel_thresh:
                ax_world = 0.0
            if abs(ay_world) < accel_thresh:
                ay_world = 0.0

            # Velocity integration with decay
            decay = self.get_parameter('velocity_decay').value
            self.vx = self.vx * decay + ax_world * dt
            self.vy = self.vy * decay + ay_world * dt

            # Position integration
            self.x += self.vx * dt
            self.y += self.vy * dt

        self.last_imu_time = now
        self.last_predict_time = now

    def external_odom_callback(self, msg: Odometry):
        """Relay external odometry (e.g. from FAST_LIO or slam_toolbox)."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        self.yaw = quaternion_to_euler(q)

        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vth = msg.twist.twist.angular.z

        self.last_predict_time = time.time()

    def publish_odom(self):
        """Publish odometry message and TF."""
        now = self.get_clock().now().to_msg()

        # Predict state for fake mode or between IMU updates
        if self.mode == 'fake' or self.mode == 'l2_imu':
            current = time.time()
            dt = current - self.last_predict_time
            if dt > 0 and dt < 1.0:
                decay = self.get_parameter('velocity_decay').value
                self.x += self.vx * dt
                self.y += self.vy * dt
                self.vx *= decay
                self.vy *= decay
            self.last_predict_time = current

        # Build quaternion
        q = euler_to_quaternion(0.0, 0.0, self.yaw)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth

        # Covariance (unknown for IMU mode)
        if self.mode == 'l2_imu':
            odom.pose.covariance[0] = 0.1   # x
            odom.pose.covariance[7] = 0.1   # y
            odom.pose.covariance[35] = 0.05  # yaw
        else:
            odom.pose.covariance[0] = 0.01
            odom.pose.covariance[7] = 0.01
            odom.pose.covariance[35] = 0.01

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = RealOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
