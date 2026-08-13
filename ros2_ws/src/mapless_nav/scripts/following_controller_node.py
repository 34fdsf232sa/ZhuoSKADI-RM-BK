#!/usr/bin/env python3
"""
Following Controller Node

This node implements person following behavior using:
- Target position from tracker
- Nav2 local costmap for obstacle avoidance
- Smooth velocity control

Control Modes:
1. Direct PID control (for simple following)
2. Nav2 goal posting (for complex navigation)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

# Optional Nav2 import
try:
    from nav2_msgs.action import NavigateToPose
    NAV2_AVAILABLE = True
except ImportError:
    NAV2_AVAILABLE = False
    print("Nav2 msgs not installed, Nav2 integration disabled")

import numpy as np
import math
from enum import Enum
from typing import Optional
import time


class FollowingState(Enum):
    """Following state machine"""
    IDLE = 0
    SEARCHING = 1
    APPROACHING = 2
    FOLLOWING = 3
    WAITING = 4  # Target too close
    OBSTACLE_AVOIDANCE = 5
    LOST = 6


class PIDController:
    """Simple PID controller"""
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 min_output: float = -1.0, max_output: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output
        
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
    
    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
    
    def compute(self, error: float) -> float:
        current_time = time.time()
        
        if self.last_time is None:
            dt = 0.05
        else:
            dt = current_time - self.last_time
        
        self.last_time = current_time
        
        if dt <= 0:
            dt = 0.05
        
        # Proportional
        p_term = self.kp * error
        
        # Integral (with anti-windup)
        self.integral += error * dt
        self.integral = np.clip(self.integral, -10.0, 10.0)
        i_term = self.ki * self.integral
        
        # Derivative
        if dt > 0:
            d_term = self.kd * (error - self.last_error) / dt
        else:
            d_term = 0.0
        
        self.last_error = error
        
        # Total output
        output = p_term + i_term + d_term
        return np.clip(output, self.min_output, self.max_output)


class FollowingControllerNode(Node):
    """ROS2 node for following a target"""
    
    def __init__(self):
        super().__init__('following_controller_node')
        
        # Declare parameters
        self.declare_parameter('target_distance', 1.5)  # meters
        self.declare_parameter('min_distance', 0.8)  # Stop if closer
        self.declare_parameter('max_distance', 5.0)  # Lost if further
        self.declare_parameter('lost_timeout', 3.0)  # seconds
        
        self.declare_parameter('max_linear_vel', 0.5)  # m/s
        self.declare_parameter('max_angular_vel', 1.0)  # rad/s
        self.declare_parameter('linear_accel', 0.3)  # m/s^2
        self.declare_parameter('angular_accel', 1.0)  # rad/s^2
        
        self.declare_parameter('linear_kp', 0.8)
        self.declare_parameter('linear_ki', 0.0)
        self.declare_parameter('linear_kd', 0.2)
        
        self.declare_parameter('angular_kp', 1.2)
        self.declare_parameter('angular_ki', 0.0)
        self.declare_parameter('angular_kd', 0.3)
        
        self.declare_parameter('use_nav2', False)
        self.declare_parameter('obstacle_threshold', 0.4)  # meters
        
        self.declare_parameter('target_topic', '/target_tracker/primary_target')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        
        # Get parameters
        self.target_distance = self.get_parameter('target_distance').value
        self.min_distance = self.get_parameter('min_distance').value
        self.max_distance = self.get_parameter('max_distance').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.linear_accel = self.get_parameter('linear_accel').value
        self.angular_accel = self.get_parameter('angular_accel').value
        
        self.use_nav2 = self.get_parameter('use_nav2').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        
        target_topic = self.get_parameter('target_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        
        # Initialize controllers
        self.linear_pid = PIDController(
            self.get_parameter('linear_kp').value,
            self.get_parameter('linear_ki').value,
            self.get_parameter('linear_kd').value,
            -self.max_linear_vel,
            self.max_linear_vel
        )
        
        self.angular_pid = PIDController(
            self.get_parameter('angular_kp').value,
            self.get_parameter('angular_ki').value,
            self.get_parameter('angular_kd').value,
            -self.max_angular_vel,
            self.max_angular_vel
        )
        
        # State
        self.state = FollowingState.IDLE
        self.enabled = False
        
        self.target_position: Optional[np.ndarray] = None
        self.target_time = 0.0
        
        self.robot_position = np.array([0.0, 0.0, 0.0])
        self.robot_yaw = 0.0
        
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        self.obstacle_detected = False
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Subscribers
        self.target_sub = self.create_subscription(
            PoseStamped, target_topic, self.target_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, sensor_qos)
        
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, sensor_qos)
        
        self.enable_sub = self.create_subscription(
            Bool, '/following/enable', self.enable_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(Marker, '/following/state', 10)
        
        # Nav2 action client (optional)
        if self.use_nav2 and NAV2_AVAILABLE:
            self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        else:
            self.nav2_client = None
        
        # Control timer
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        
        self.get_logger().info("Following Controller Node initialized")
        self.get_logger().info(f"  Target distance: {self.target_distance}m")
        self.get_logger().info(f"  Use Nav2: {self.use_nav2 and NAV2_AVAILABLE}")
    
    def enable_callback(self, msg: Bool):
        """Enable/disable following"""
        if msg.data and not self.enabled:
            self.enabled = True
            self.state = FollowingState.SEARCHING
            self.get_logger().info("Following ENABLED")
        elif not msg.data and self.enabled:
            self.enabled = False
            self.state = FollowingState.IDLE
            self.stop_robot()
            self.get_logger().info("Following DISABLED")
    
    def target_callback(self, msg: PoseStamped):
        """Update target position"""
        # Convert from camera frame to robot frame
        # Camera: Z forward, X right, Y down
        # Robot: X forward, Y left, Z up
        
        cam_x = msg.pose.position.x
        cam_y = msg.pose.position.y
        cam_z = msg.pose.position.z
        
        # Transform: robot_x = cam_z, robot_y = -cam_x, robot_z = -cam_y
        self.target_position = np.array([cam_z, -cam_x, -cam_y])
        self.target_time = time.time()
        
        self.get_logger().debug(
            f"Target: x={self.target_position[0]:.2f}, y={self.target_position[1]:.2f}")
    
    def odom_callback(self, msg: Odometry):
        """Update robot position"""
        self.robot_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def scan_callback(self, msg: LaserScan):
        """Check for obstacles"""
        # Check front arc (±30 degrees)
        ranges = np.array(msg.ranges)
        num_ranges = len(ranges)
        
        # Front indices
        front_start = int(num_ranges * 0.4)  # ~-30 deg
        front_end = int(num_ranges * 0.6)    # ~+30 deg
        
        front_ranges = ranges[front_start:front_end]
        valid_ranges = front_ranges[np.isfinite(front_ranges) & (front_ranges > 0.01)]
        
        if len(valid_ranges) > 0:
            min_range = np.min(valid_ranges)
            self.obstacle_detected = min_range < self.obstacle_threshold
        else:
            self.obstacle_detected = False
    
    def control_loop(self):
        """Main control loop"""
        if not self.enabled:
            return
        
        # Update state machine
        self.update_state()
        
        # Compute control
        if self.state == FollowingState.FOLLOWING:
            self.follow_target()
        elif self.state == FollowingState.APPROACHING:
            self.approach_target()
        elif self.state == FollowingState.WAITING:
            self.stop_robot()
        elif self.state == FollowingState.OBSTACLE_AVOIDANCE:
            self.avoid_obstacle()
        elif self.state == FollowingState.SEARCHING:
            self.search_target()
        elif self.state == FollowingState.LOST:
            self.stop_robot()
        
        # Publish state visualization
        self.publish_state()
    
    def update_state(self):
        """Update state machine"""
        time_since_target = time.time() - self.target_time
        
        # Check if target is lost
        if time_since_target > self.lost_timeout:
            if self.state != FollowingState.LOST:
                self.get_logger().warn("Target LOST")
            self.state = FollowingState.LOST
            return
        
        # Check for obstacles
        if self.obstacle_detected and self.state not in [FollowingState.WAITING, FollowingState.SEARCHING]:
            self.state = FollowingState.OBSTACLE_AVOIDANCE
            return
        
        # Check target distance
        if self.target_position is not None:
            distance = np.linalg.norm(self.target_position[:2])
            
            if distance < self.min_distance:
                self.state = FollowingState.WAITING
            elif distance > self.max_distance:
                self.state = FollowingState.LOST
            elif distance > self.target_distance + 0.3:
                self.state = FollowingState.APPROACHING
            else:
                self.state = FollowingState.FOLLOWING
    
    def follow_target(self):
        """Follow target at desired distance"""
        if self.target_position is None:
            return
        
        # Calculate errors
        distance = np.linalg.norm(self.target_position[:2])
        angle = math.atan2(self.target_position[1], self.target_position[0])
        
        distance_error = distance - self.target_distance
        angle_error = angle  # We want to face the target
        
        # PID control
        linear_vel = self.linear_pid.compute(distance_error)
        angular_vel = self.angular_pid.compute(angle_error)
        
        # Apply smoothing (acceleration limits)
        linear_vel = self.smooth_velocity(
            self.current_linear_vel, linear_vel, self.linear_accel, 0.05)
        angular_vel = self.smooth_velocity(
            self.current_angular_vel, angular_vel, self.angular_accel, 0.05)
        
        # Reduce linear velocity when turning
        turn_factor = max(0.3, 1.0 - abs(angle_error) / math.pi)
        linear_vel *= turn_factor
        
        self.send_velocity(linear_vel, angular_vel)
    
    def approach_target(self):
        """Approach target (faster than following)"""
        if self.target_position is None:
            return
        
        distance = np.linalg.norm(self.target_position[:2])
        angle = math.atan2(self.target_position[1], self.target_position[0])
        
        # More aggressive approaching
        distance_error = distance - self.target_distance
        linear_vel = min(self.max_linear_vel, 0.5 + 0.3 * distance_error)
        angular_vel = self.angular_pid.compute(angle)
        
        # Reduce speed when angle is large
        if abs(angle) > 0.5:  # ~30 degrees
            linear_vel *= 0.3
        
        linear_vel = self.smooth_velocity(
            self.current_linear_vel, linear_vel, self.linear_accel, 0.05)
        
        self.send_velocity(linear_vel, angular_vel)
    
    def search_target(self):
        """Rotate to search for target"""
        # Slow rotation
        self.send_velocity(0.0, 0.3)
    
    def avoid_obstacle(self):
        """Simple obstacle avoidance"""
        # Back up slowly while turning
        self.send_velocity(-0.1, 0.5)
    
    def stop_robot(self):
        """Stop the robot"""
        self.send_velocity(0.0, 0.0)
    
    def smooth_velocity(self, current: float, target: float, 
                       accel: float, dt: float) -> float:
        """Smooth velocity change with acceleration limit"""
        max_change = accel * dt
        diff = target - current
        
        if abs(diff) <= max_change:
            return target
        else:
            return current + np.sign(diff) * max_change
    
    def send_velocity(self, linear: float, angular: float):
        """Send velocity command"""
        self.current_linear_vel = linear
        self.current_angular_vel = angular
        
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        
        self.cmd_vel_pub.publish(msg)
    
    def publish_state(self):
        """Publish state visualization"""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'base_link'
        marker.ns = 'following_state'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.z = 0.3
        
        # Color based on state
        if self.state == FollowingState.FOLLOWING:
            marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
        elif self.state == FollowingState.APPROACHING:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0
        elif self.state == FollowingState.LOST:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.5, 0.5, 0.5
        marker.color.a = 1.0
        
        marker.text = f"State: {self.state.name}"
        if self.target_position is not None:
            dist = np.linalg.norm(self.target_position[:2])
            marker.text += f"\nDist: {dist:.2f}m"
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 100000000
        
        self.state_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = FollowingControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot before shutdown
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
