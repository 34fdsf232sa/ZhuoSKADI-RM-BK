#!/usr/bin/env python3
"""
Waypoint Patrol Node

Navigates the robot through a sequence of waypoints using Nav2's
FollowWaypoints action. Waypoints are loaded from a YAML config file
and can be set to loop indefinitely.

Triggers:
  - /patrol/start  (std_srvs/Trigger) — begin patrol
  - /patrol/stop   (std_srvs/Trigger) — stop patrol
  - /patrol/next   (std_msgs/Empty)   — skip to next waypoint

Publishes:
  - /patrol/status          (String)         — current state
  - /patrol/current_waypoint (Int32)         — which waypoint we're heading to
  - /patrol/waypoint_markers (MarkerArray)   — RViz visualization

Dependencies:
  - Nav2 stack running (controller_server, planner_server, bt_navigator)
  - /odom for frame reference
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int32, Empty
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

from nav2_msgs.action import FollowWaypoints

import math
from enum import Enum


class PatrolState(Enum):
    IDLE = 0
    PATROLLING = 1
    PAUSED = 2
    COMPLETED = 3
    FAILED = 4
    STOPPED = 5


class WaypointPatrolNode(Node):
    def __init__(self):
        super().__init__('waypoint_patrol_node')

        # Parameters
        self.declare_parameter('waypoints_x', [1.0, 1.0, 0.0, 0.0])
        self.declare_parameter('waypoints_y', [0.0, 1.0, 1.0, 0.0])
        self.declare_parameter('waypoints_yaw', [0.0, 1.57, 3.14, 0.0])
        self.declare_parameter('loop', True)
        self.declare_parameter('nav2_action_topic', 'follow_waypoints')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('nav2_timeout', 300.0)

        self.loop = self.get_parameter('loop').value
        self.nav2_action_topic = self.get_parameter('nav2_action_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.nav2_timeout = self.get_parameter('nav2_timeout').value

        # Parse waypoints from parameter arrays
        self.waypoints = self._parse_waypoints()
        if not self.waypoints:
            self.get_logger().error('No waypoints configured!')
        else:
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints, loop={self.loop}')

        # Services
        self.start_srv = self.create_service(Trigger, '/patrol/start', self.start_callback)
        self.stop_srv = self.create_service(Trigger, '/patrol/stop', self.stop_callback)

        # Topics
        self.next_sub = self.create_subscription(Empty, '/patrol/next', self.next_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, '/patrol/status', 10)
        self.current_wp_pub = self.create_publisher(Int32, '/patrol/current_waypoint', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/patrol/waypoint_markers', 10)

        # Nav2 action client
        self.nav2_client = ActionClient(self, FollowWaypoints, self.nav2_action_topic)

        # State
        self.state = PatrolState.IDLE
        self.current_wp_idx = -1
        self.goal_handle = None
        self.patrol_start_time = None

        # Timer
        self.timer = self.create_timer(0.5, self.control_loop)

        self.get_logger().info('WaypointPatrolNode ready. Call /patrol/start to begin.')

    def _parse_waypoints(self):
        xs = self.get_parameter('waypoints_x').value
        ys = self.get_parameter('waypoints_y').value
        yaws = self.get_parameter('waypoints_yaw').value

        if len(xs) != len(ys) or len(xs) != len(yaws):
            self.get_logger().error(
                f'Waypoint array length mismatch: x={len(xs)}, y={len(ys)}, yaw={len(yaws)}')
            return []

        waypoints = []
        for x, y, yaw in zip(xs, ys, yaws):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(float(yaw) * 0.5)
            pose.pose.orientation.w = math.cos(float(yaw) * 0.5)
            waypoints.append(pose)
        return waypoints

    def start_callback(self, request, response):
        if self.state == PatrolState.PATROLLING:
            response.success = False
            response.message = 'Already patrolling'
        elif not self.waypoints:
            response.success = False
            response.message = 'No waypoints configured'
        else:
            self._start_patrol()
            response.success = True
            response.message = f'Patrol started: {len(self.waypoints)} waypoints'
        return response

    def stop_callback(self, request, response):
        if self.state == PatrolState.PATROLLING:
            self._cancel_navigation()
            self.state = PatrolState.STOPPED
            response.success = True
            response.message = 'Patrol stopped'
        else:
            response.success = False
            response.message = f'Not patrolling, state: {self.state.name}'
        return response

    def next_callback(self, msg):
        if self.state == PatrolState.PATROLLING and self.goal_handle:
            self._cancel_navigation()

    def _start_patrol(self):
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 FollowWaypoints action server not available')
            self.state = PatrolState.FAILED
            return

        self.state = PatrolState.PATROLLING
        self.current_wp_idx = 0
        self.patrol_start_time = self.get_clock().now()
        self._send_waypoints_from(0)

    def _send_waypoints_from(self, start_idx):
        goal = FollowWaypoints.Goal()
        remaining = self.waypoints[start_idx:]
        goal.poses = remaining

        self.get_logger().info(
            f'Sending {len(remaining)} poses from waypoint {start_idx}')

        send_goal_future = self.nav2_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Nav2 FollowWaypoints goal rejected')
            self.state = PatrolState.FAILED
            return

        self.get_logger().info('Nav2 accepted patrol route')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        if self.state != PatrolState.PATROLLING:
            return

        status = future.result().status
        missed = future.result().result.missed_waypoints

        # status 4 = SUCCEEDED (rclpy GoalStatus.STATUS_SUCCEEDED)
        if status == 4:
            if len(missed) > 0:
                self.get_logger().warn(
                    f'Patrol completed but {len(missed)} waypoints were missed: {missed}')
            else:
                self.get_logger().info('Completed all waypoints!')
            if self.loop:
                self.get_logger().info('Looping: restarting patrol from first waypoint')
                self.current_wp_idx = 0
                self._send_waypoints_from(0)
            else:
                self.state = PatrolState.COMPLETED
        else:
            self.get_logger().error(f'Patrol failed with status: {status}')
            self.state = PatrolState.FAILED

    def _cancel_navigation(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None

    def control_loop(self):
        self._publish_markers()
        self.status_pub.publish(String(data=self.state.name))

        if self.current_wp_idx >= 0:
            self.current_wp_pub.publish(Int32(data=self.current_wp_idx))

        # Timeout check
        if self.state == PatrolState.PATROLLING and self.patrol_start_time:
            elapsed = (self.get_clock().now() - self.patrol_start_time).nanoseconds / 1e9
            if elapsed > self.nav2_timeout:
                self.get_logger().warn(f'Patrol timed out after {elapsed:.1f}s')
                self._cancel_navigation()
                self.state = PatrolState.FAILED

    def _publish_markers(self):
        if not self.waypoints:
            return

        markers = MarkerArray()
        for i, wp in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = wp.pose
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15

            if i == self.current_wp_idx and self.state == PatrolState.PATROLLING:
                marker.color.r = 1.0  # Current target: green
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif i < self.current_wp_idx:
                marker.color.r = 0.5  # Completed: grey
                marker.color.g = 0.5
                marker.color.b = 0.5
            else:
                marker.color.r = 0.2  # Upcoming: light blue
                marker.color.g = 0.5
                marker.color.b = 1.0
            marker.color.a = 0.8

            markers.markers.append(marker)

            # Arrow showing yaw direction
            yaw = 2.0 * math.atan2(wp.pose.orientation.z, wp.pose.orientation.w)
            arrow = Marker()
            arrow.header.frame_id = self.frame_id
            arrow.header.stamp = marker.header.stamp
            arrow.ns = 'waypoint_arrows'
            arrow.id = i + 1000
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = wp.pose
            arrow.scale.x = 0.3
            arrow.scale.y = 0.05
            arrow.scale.z = 0.05
            arrow.color.r = 1.0
            arrow.color.g = 0.8
            arrow.color.b = 0.0
            arrow.color.a = 0.9
            markers.markers.append(arrow)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
