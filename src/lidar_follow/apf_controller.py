#!/usr/bin/env python3
"""
Artificial Potential Field (APF) Obstacle Avoidance Controller

Direct Python port of jie_deamon/include/lidar_tracker.hpp APF logic.
Computes repulsive forces from nearby obstacles and integrates them with
target-following attractive forces.

For use with /scan (LaserScan) data.
"""

import math
from typing import List, Tuple, Optional
from dataclasses import dataclass


# --- Parameters (same defaults as jie_deamon common_types.hpp) ---

@dataclass
class APFParams:
    """APF parameters matching jie_deamon defaults."""
    # Following
    follow_dist:       float = 0.4    # Desired distance to target (m)
    target_radius:     float = 0.3    # Target search radius (m)

    # Speed limits
    max_linear_speed:  float = 0.5    # m/s (reduced from 1.0 for diff-drive)
    max_angular_speed: float = 1.0    # rad/s

    # Velocity scaling
    linear_scale:      float = 0.5    # vx = error * scale
    angular_scale:     float = 1.0    # wz = angle * scale
    lateral_scale:     float = 1.0    # vy = lateral_error * scale

    # APF obstacle avoidance
    apf_influence_dist: float = 0.25   # Repulsive force range (m)
    apf_repulse_gain:   float = 0.01  # Repulsive force gain
    apf_emergency_dist: float = 0.20  # Hard stop distance (m)
    apf_slowdown_dist:  float = 0.25  # Slowdown zone (m)

    # Robot frame exclusion (LiDAR may see robot's own structure)
    robot_frame_front: float = 0.15   # Exclude points in front (m)
    robot_frame_back:  float = 0.35   # Exclude points behind (m)
    robot_frame_left:  float = 0.15   # Exclude points left (m)
    robot_frame_right: float = 0.15   # Exclude points right (m)

    # Corridor width for path-following
    corridor_width:    float = 0.35   # Rectangle width for path checking (m)

    # Dead zones
    linear_dead_zone:   float = 0.05  # vx dead zone (m)
    angular_dead_zone:  float = 0.1   # wz dead zone (rad)
    lateral_dead_zone:  float = 0.03  # vy dead zone (m)


class APFController:
    """
    Artificial Potential Field obstacle avoidance.

    Combines:
    - Attractive force toward follow target
    - Repulsive forces from nearby obstacles
    - Emergency stop when too close to obstacles
    - Speed reduction in obstacle slowdown zone
    """

    def __init__(self, params: Optional[APFParams] = None):
        self.p = params or APFParams()

    def process_scan(
        self,
        ranges: List[float],
        angle_min: float,
        angle_increment: float,
        target_x: float,
        target_y: float,
    ) -> Tuple[float, float, float, dict]:
        """
        Process a LaserScan and compute velocities.

        Args:
            ranges: LaserScan ranges array
            angle_min: Start angle of scan
            angle_increment: Angular resolution of scan
            target_x, target_y: Target position in robot frame (m)

        Returns:
            (vx, vy, wz, info_dict)
        """
        info = {
            'obstacle_count': 0,
            'min_obstacle_dist': 999.0,
            'apf_repulse_x': 0.0,
            'apf_repulse_y': 0.0,
            'emergency_stop': False,
            'slowdown_active': False,
        }

        # ---- Scan analysis ----

        # Path corridor clearance
        left_y_min = -self.p.corridor_width / 2.0
        right_y_min = self.p.corridor_width / 2.0

        # APF repulsive force accumulation
        repulse_x, repulse_y = 0.0, 0.0
        min_obstacle_dist = 999.0

        # Target vector
        target_vec_x = target_x
        target_vec_y = target_y
        target_vec_len = math.hypot(target_vec_x, target_vec_y)

        # Centroid of points near target
        centroid_x, centroid_y = 0.0, 0.0
        points_near_target = 0

        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            angle = angle_min + i * angle_increment
            px = -r * math.cos(angle)
            py = -r * math.sin(angle)
            dist_to_robot = math.hypot(px, py)

            # ---- Robot frame exclusion ----
            in_robot_frame = (
                px > -self.p.robot_frame_back and
                px < self.p.robot_frame_front and
                abs(py) < self.p.robot_frame_right and
                abs(py) < self.p.robot_frame_left
            )

            if not in_robot_frame and dist_to_robot < min_obstacle_dist:
                min_obstacle_dist = dist_to_robot

            # ---- APF: Repulsive force ----
            if (not in_robot_frame and
                    dist_to_robot < self.p.apf_influence_dist and
                    px > -0.1):
                force = (self.p.apf_repulse_gain *
                         (1.0 / dist_to_robot - 1.0 / self.p.apf_influence_dist) /
                         (dist_to_robot * dist_to_robot))
                repulse_x -= force * px / dist_to_robot
                repulse_y -= force * py / dist_to_robot

            if in_robot_frame:
                continue

            info['obstacle_count'] += 1

            # ---- Check if point is near target ----
            dist_to_target = math.hypot(px - target_x, py - target_y)
            if dist_to_target < self.p.target_radius:
                centroid_x += px
                centroid_y += py
                points_near_target += 1
                continue

            # ---- Path corridor check ----
            if target_vec_len > 1e-6:
                proj_x = (px * target_vec_x + py * target_vec_y) / target_vec_len
                proj_y = (px * -target_vec_y + py * target_vec_x) / target_vec_len

                if (0.0 <= proj_x <= target_vec_len and
                        abs(proj_y) <= self.p.corridor_width / 2.0):
                    if proj_y > 0 and proj_y > left_y_min:
                        left_y_min = proj_y
                    elif proj_y <= 0 and proj_y < right_y_min:
                        right_y_min = proj_y

        # ---- Limit repulsive force ----
        repulse_mag = math.hypot(repulse_x, repulse_y)
        if repulse_mag > 1.0:
            repulse_x /= repulse_mag
            repulse_y /= repulse_mag

        info['apf_repulse_x'] = repulse_x
        info['apf_repulse_y'] = repulse_y
        info['min_obstacle_dist'] = min_obstacle_dist

        # ---- Compute target position ----
        if points_near_target > 0:
            target_x = centroid_x / points_near_target
            target_y = centroid_y / points_near_target

        # ---- Compute velocities ----
        vx, vy, wz = self._compute_velocity(
            target_x, target_y,
            left_y_min, right_y_min,
            repulse_x, repulse_y,
            min_obstacle_dist, info)

        return vx, vy, wz, info

    def _compute_velocity(
        self,
        target_x: float,
        target_y: float,
        left_y_min: float,
        right_y_min: float,
        repulse_x: float,
        repulse_y: float,
        min_obstacle_dist: float,
        info: dict,
    ) -> Tuple[float, float, float]:
        """Compute follow velocity with APF integration."""

        # ---- Emergency stop ----
        if min_obstacle_dist < self.p.apf_emergency_dist:
            info['emergency_stop'] = True
            return 0.0, 0.0, 0.0

        # ---- Forward/backward (vx) ----
        dist_error = target_x - self.p.follow_dist
        if abs(dist_error) < self.p.linear_dead_zone:
            vx = 0.0
        else:
            vx = dist_error * self.p.linear_scale
            if vx < 0:
                vx *= 0.8  # slower backward
            # Minimum speed
            min_speed = 0.06
            if 0.0 < abs(vx) < min_speed:
                vx = min_speed if vx > 0 else -min_speed

        # ---- Rotation (wz) ----
        angle_error = math.atan2(target_y, target_x)
        if abs(angle_error) < self.p.angular_dead_zone:
            wz = 0.0
        else:
            wz = angle_error * self.p.angular_scale

        # ---- Lateral (vy) — less useful for diff-drive, but include ----
        lateral_error = -(left_y_min + right_y_min)
        if abs(lateral_error) > 1.0:
            lateral_error = 0.0
        if abs(lateral_error) < self.p.lateral_dead_zone:
            vy = 0.0
        else:
            vy = lateral_error * self.p.lateral_scale

        # ---- Fuse APF repulsion ----
        vx += repulse_x
        vy += repulse_y

        # ---- Slowdown near obstacles ----
        if min_obstacle_dist < self.p.apf_slowdown_dist:
            info['slowdown_active'] = True
            factor = ((min_obstacle_dist - self.p.apf_emergency_dist) /
                      (self.p.apf_slowdown_dist - self.p.apf_emergency_dist))
            factor = max(0.1, min(1.0, factor))
            vx *= factor

        # ---- Clamp ----
        vx = max(-self.p.max_linear_speed, min(self.p.max_linear_speed, vx))
        vy = max(-self.p.max_linear_speed, min(self.p.max_linear_speed, vy))
        wz = max(-self.p.max_angular_speed, min(self.p.max_angular_speed, wz))

        return vx, vy, wz
