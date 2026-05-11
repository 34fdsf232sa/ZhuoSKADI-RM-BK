#!/usr/bin/env python3
"""
Return Home Navigation Node

Records the robot's starting position on startup and navigates back to it
when triggered via ROS service or topic.

Key Features:
- Auto-record home position from first /odom message (or manual via params)
- Trigger: /return_home service (std_srvs/Trigger) or /return_home/trigger topic
- Abort: /return_home/abort service
- Uses Nav2 NavigateToPose action for path planning and execution
- Publishes home pose for RViz visualization
- Configurable navigation timeout

States: IDLE -> NAVIGATING -> SUCCEEDED / FAILED / TIMEOUT / ABORTED

Dependencies:
- Nav2 stack must be running (controller_server, planner_server, behavior_server)
- /odom topic for position tracking
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger

from nav2_msgs.action import NavigateToPose

import math
from enum import Enum


class ReturnHomeState(Enum):
    IDLE = 0
    NAVIGATING = 1
    SUCCEEDED = 2
    FAILED = 3
    TIMEOUT = 4
    ABORTED = 5


class ReturnHomeNode(Node):
    def __init__(self):
        super().__init__('return_home_node')

        # Parameters
        self.declare_parameter('home_pose_x', -999.0)
        self.declare_parameter('home_pose_y', -999.0)
        self.declare_parameter('home_pose_z', 0.0)
        self.declare_parameter('home_pose_yaw', 0.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('nav2_action_topic', 'navigate_to_pose')
        self.declare_parameter('nav2_timeout', 120.0)
        self.declare_parameter('battery_topic', '')
        self.declare_parameter('battery_threshold', 0.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.odom_topic = self.get_parameter('odom_topic').value
        self.nav2_action_topic = self.get_parameter('nav2_action_topic').value
        self.nav2_timeout = self.get_parameter('nav2_timeout').value
        self.battery_topic = self.get_parameter('battery_topic').value
        self.battery_threshold = self.get_parameter('battery_threshold').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.home_pose_x = self.get_parameter('home_pose_x').value
        self.home_pose_y = self.get_parameter('home_pose_y').value
        self.home_pose_z = self.get_parameter('home_pose_z').value
        self.home_pose_yaw = self.get_parameter('home_pose_yaw').value
        self.use_manual_home = self.home_pose_x > -900.0 and self.home_pose_y > -900.0

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1)

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, sensor_qos)

        self.trigger_sub = self.create_subscription(
            Empty, '/return_home/trigger', self.trigger_callback, 10)

        if self.battery_topic:
            self.battery_sub = self.create_subscription(
                Empty, self.battery_topic, self.battery_callback, 10)

        # Publishers
        self.home_pose_pub = self.create_publisher(PoseStamped, '/home_pose', 10)
        self.status_pub = self.create_publisher(String, '/return_home/status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Services
        self.return_home_srv = self.create_service(
            Trigger, '/return_home', self.return_home_callback)
        self.abort_srv = self.create_service(
            Trigger, '/return_home/abort', self.abort_callback)

        # Nav2 action client
        self.nav2_client = ActionClient(self, NavigateToPose, self.nav2_action_topic)

        # State
        self.state = ReturnHomeState.IDLE
        self.home_pose = None
        self.current_odom = None
        self.home_recorded = False
        self.goal_handle = None
        self.nav_start_time = None

        # Timer for status publishing
        self.timer = self.create_timer(0.1, self.control_loop)

        if self.use_manual_home:
            self._set_manual_home()
            self.get_logger().info(
                f'Using manual home: x={self.home_pose_x:.2f}, y={self.home_pose_y:.2f}, '
                f'yaw={self.home_pose_yaw:.2f}')

        self.get_logger().info('ReturnHomeNode ready. '
                                'Call /return_home service to trigger return.')

    def _set_manual_home(self):
        self.home_pose = PoseStamped()
        self.home_pose.header.frame_id = 'odom'
        self.home_pose.pose.position.x = self.home_pose_x
        self.home_pose.pose.position.y = self.home_pose_y
        self.home_pose.pose.position.z = self.home_pose_z
        cy = math.cos(self.home_pose_yaw * 0.5)
        sy = math.sin(self.home_pose_yaw * 0.5)
        self.home_pose.pose.orientation.z = sy
        self.home_pose.pose.orientation.w = cy
        self.home_recorded = True

    def odom_callback(self, msg):
        self.current_odom = msg
        if not self.home_recorded and not self.use_manual_home:
            self._record_home(msg)

    def _record_home(self, odom):
        self.home_pose = PoseStamped()
        self.home_pose.header.frame_id = 'odom'
        self.home_pose.header.stamp = odom.header.stamp
        self.home_pose.pose = odom.pose.pose
        self.home_recorded = True
        self.get_logger().info(
            f'Home recorded: x={self.home_pose.pose.position.x:.3f}, '
            f'y={self.home_pose.pose.position.y:.3f}')

    def trigger_callback(self, msg):
        if self.state == ReturnHomeState.IDLE:
            self._start_navigation()

    def battery_callback(self, msg):
        if (self.state == ReturnHomeState.IDLE and self.home_recorded
                and self.current_odom is not None):
            self._start_navigation()

    def return_home_callback(self, request, response):
        if self.state == ReturnHomeState.NAVIGATING:
            response.success = False
            response.message = 'Already navigating home'
        elif not self.home_recorded:
            response.success = False
            response.message = 'Home pose not recorded yet'
        else:
            self._start_navigation()
            response.success = True
            response.message = 'Returning to home'
        return response

    def abort_callback(self, request, response):
        if self.state == ReturnHomeState.NAVIGATING:
            self._abort_navigation()
            response.success = True
            response.message = 'Navigation aborted'
        else:
            response.success = False
            response.message = 'No navigation in progress'
        return response

    def _start_navigation(self):
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            self.state = ReturnHomeState.FAILED
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.home_pose
        self.get_logger().info(
            f'Sending NavigateToPose goal: '
            f'x={self.home_pose.pose.position.x:.2f}, '
            f'y={self.home_pose.pose.position.y:.2f}')

        send_goal_future = self.nav2_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._goal_response_cb)
        self.state = ReturnHomeState.NAVIGATING
        self.nav_start_time = self.get_clock().now()

    def _goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Nav2 goal rejected')
            self.state = ReturnHomeState.FAILED
            return

        self.get_logger().info('Nav2 goal accepted, navigating home...')
        self.status_pub.publish(String(data='NAVIGATING'))
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        status = result.status

        from nav2_msgs.action import NavigateToPose
        if status == NavigateToPose.Result.SUCCESS:
            self.get_logger().info('Successfully returned to home!')
            self.state = ReturnHomeState.SUCCEEDED
            self._stop_robot()
        else:
            self.get_logger().error(f'Nav2 navigation failed with status: {status}')
            self.state = ReturnHomeState.FAILED

    def _abort_navigation(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
        self.state = ReturnHomeState.ABORTED
        self._stop_robot()
        self.get_logger().info('Navigation aborted by user')
        self.status_pub.publish(String(data='ABORTED'))

    def _stop_robot(self):
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

    def control_loop(self):
        # Publish home pose at ~1 Hz (every 10th tick at 0.1s)
        if self.home_recorded and self.home_pose is not None:
            now = self.get_clock().now()
            self.home_pose.header.stamp = now.to_msg()
            self.home_pose_pub.publish(self.home_pose)

        # Publish status string
        self.status_pub.publish(String(data=self.state.name))

        # Check timeout
        if self.state == ReturnHomeState.NAVIGATING:
            elapsed = (self.get_clock().now() - self.nav_start_time).nanoseconds / 1e9
            if elapsed > self.nav2_timeout:
                self.get_logger().warn(
                    f'Navigation timed out after {elapsed:.1f}s')
                self._abort_navigation()
                self.state = ReturnHomeState.TIMEOUT


def main(args=None):
    rclpy.init(args=args)
    node = ReturnHomeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
