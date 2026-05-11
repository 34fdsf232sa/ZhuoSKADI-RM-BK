#!/usr/bin/env python3
"""
PointCloud2 to LaserScan Conversion Node

Converts 3D point cloud (in base_link frame) to 2D laser scan for
obstacle detection in following_controller_node.

Algorithm: projects each point to the XY plane, computes angle + range,
quantizes into angular bins, takes minimum range per bin.

Publishes:
  /scan (sensor_msgs/LaserScan)

Subscribes:
  /fused_pointcloud (sensor_msgs/PointCloud2) — must be in base_link
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
import math


class PointCloudToScanNode(Node):
    def __init__(self):
        super().__init__('pointcloud_to_scan_node')

        self.declare_parameter('input_topic', '/fused_pointcloud')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('min_height', 0.1)
        self.declare_parameter('max_height', 1.8)
        self.declare_parameter('angle_increment', math.pi / 180.0)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('publish_rate', 20.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        publish_rate = self.get_parameter('publish_rate').value

        self.num_bins = int(2.0 * math.pi / self.angle_increment)
        self.angle_min = -math.pi
        self.angle_max = math.pi

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1)

        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.cloud_callback, sensor_qos)

        self.pub = self.create_publisher(LaserScan, output_topic, 10)

        period = 1.0 / publish_rate if publish_rate > 0 else 0.05
        self.timer = self.create_timer(period, self.timer_callback)

        self.latest_ranges = np.full(self.num_bins, float('inf'), dtype=np.float32)
        self.latest_header = None
        self.cloud_received = False

        self.get_logger().info(
            f'PointCloud2→LaserScan: {input_topic} → {output_topic}, '
            f'{self.num_bins} bins, height=[{self.min_height},{self.max_height}]')

    def cloud_callback(self, msg):
        self.latest_header = msg.header

        # Extract x, y, z from point cloud using read_points iterator
        ranges = np.full(self.num_bins, float('inf'), dtype=np.float32)

        for pt in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = pt[0], pt[1], pt[2]

            # Height filter
            if z < self.min_height or z > self.max_height:
                continue

            # Compute angle and range in XY plane
            r = math.hypot(x, y)
            if r < self.range_min or r > self.range_max:
                continue

            angle = math.atan2(y, x)
            bin_idx = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= bin_idx < self.num_bins:
                if r < ranges[bin_idx]:
                    ranges[bin_idx] = r

        self.latest_ranges = ranges
        self.cloud_received = True

    def timer_callback(self):
        if not self.cloud_received:
            return

        scan = LaserScan()
        scan.header = self.latest_header
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = float(self.range_min)
        scan.range_max = float(self.range_max)

        # Replace inf with max_range for ROS compatibility
        ranges = self.latest_ranges.copy()
        ranges[np.isinf(ranges)] = float(self.range_max) + 1.0

        scan.ranges = ranges.tolist()
        self.pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
