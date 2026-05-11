#!/usr/bin/env python3
"""
LiDAR Person Tracker — Kalman + APF Pipeline

Python port of jie_deamon/include/lidar_tracker.hpp.
Tracks a person using LiDAR /scan data with:
  - Centroid-based target following (points within TARGET_RADIUS)
  - Kalman filter position smoothing
  - APF obstacle avoidance
  - Robot frame exclusion (self-filter)

Can be used standalone (pure LiDAR) or fused with visual tracking.
"""

import math
from typing import Optional, List, Tuple, Callable
from dataclasses import dataclass, field

from kalman_filter_2d import KalmanFilter2D
from apf_controller import APFController, APFParams


@dataclass
class LidarTrackerParams:
    """Full LiDAR tracker parameters."""
    apf: APFParams = field(default_factory=APFParams)

    # Kalman
    enable_kalman: bool = True
    kalman_process_noise: float = 0.1
    kalman_measurement_noise: float = 0.05

    # OpenCV visualization (used by jie_deamon, ported optionally)
    enable_opencv: bool = False


@dataclass
class ScanPoint:
    """Single point from LaserScan, in robot frame."""
    x: float  # forward
    y: float  # left
    dist: float  # distance to robot


class LidarTracker:
    """
    LiDAR-based person follower.

    Usage:
        tracker = LidarTracker()
        tracker.set_target(0.4, 0.0)  # initial target position

        for scan in scan_messages:
            vx, wz, info = tracker.process_scan(
                scan.ranges, scan.angle_min, scan.angle_increment)
            publish_velocity(vx, wz)
    """

    def __init__(self, params: Optional[LidarTrackerParams] = None):
        self.p = params or LidarTrackerParams()
        self.apf = APFController(self.p.apf)
        self.kalman = KalmanFilter2D(
            process_noise=self.p.kalman_process_noise,
            measurement_noise=self.p.kalman_measurement_noise,
        )

        # Current target position (robot frame, meters)
        self._target_x = self.p.apf.follow_dist
        self._target_y = 0.0
        self._target_set = False

        # Callbacks
        self._velocity_callback: Optional[Callable] = None
        self._data_callback: Optional[Callable] = None

        # OpenCV (optional)
        self._opencv_enabled = self.p.enable_opencv

    # ---- Public API ----

    def set_target(self, x: float, y: float):
        """Manually set target position in robot frame."""
        self._target_x = x
        self._target_y = y
        self._target_set = True
        if self.p.enable_kalman:
            self.kalman.set_state(x, y)

    def set_velocity_callback(self, cb: Callable):
        """Callback: cb(vx, wz) for publishing velocity."""
        self._velocity_callback = cb

    def set_data_callback(self, cb: Callable):
        """Callback: cb(scan_points, target_xy) for data broadcast."""
        self._data_callback = cb

    def process_scan(
        self,
        ranges: List[float],
        angle_min: float,
        angle_increment: float,
    ) -> Tuple[float, float, dict]:
        """
        Process a LaserScan message.

        Returns:
            (vx, wz, info_dict) — velocity commands and debug info.
            WZ is already included in info_dict for diff-drive.
        """
        target_x = self._target_x
        target_y = self._target_y

        # 1. Run APF controller
        vx, vy, wz, info = self.apf.process_scan(
            ranges, angle_min, angle_increment,
            target_x, target_y)

        # 2. Extract target centroid from scan (same as jie_deamon processScan)
        centroid_x, centroid_y, new_target = self._extract_target_centroid(
            ranges, angle_min, angle_increment,
            target_x, target_y)

        if new_target:
            # 3. Kalman filter the new target position
            if self.p.enable_kalman:
                fx, fy = self.kalman.update(centroid_x, centroid_y)
            else:
                fx, fy = centroid_x, centroid_y
            self._target_x = fx
            self._target_y = fy
            info['target_x'] = fx
            info['target_y'] = fy
        else:
            info['target_x'] = self._target_x
            info['target_y'] = self._target_y

        info['target_updated'] = new_target

        # 4. Publish velocity
        if self._velocity_callback:
            self._velocity_callback(vx, wz)

        return vx, wz, info

    def _extract_target_centroid(
        self,
        ranges: List[float],
        angle_min: float,
        angle_increment: float,
        target_x: float,
        target_y: float,
    ) -> Tuple[float, float, bool]:
        """
        Find points near the expected target position and compute centroid.

        Mirror of jie_deamon LidarTracker::processScan centroid extraction.
        Returns (centroid_x, centroid_y, found).
        """
        sum_x, sum_y = 0.0, 0.0
        count = 0

        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            angle = angle_min + i * angle_increment
            px = -r * math.cos(angle)
            py = -r * math.sin(angle)

            # Robot frame exclusion
            in_frame = (
                px > -self.p.apf.robot_frame_back and
                px < self.p.apf.robot_frame_front and
                abs(py) < self.p.apf.robot_frame_right
            )
            if in_frame:
                continue

            dist_to_target = math.hypot(px - target_x, py - target_y)
            if dist_to_target < self.p.apf.target_radius:
                sum_x += px
                sum_y += py
                count += 1

        if count > 0:
            return sum_x / count, sum_y / count, True
        return 0.0, 0.0, False

    # ---- Properties ----

    @property
    def target_position(self) -> Tuple[float, float]:
        return self._target_x, self._target_y

    @property
    def kalman_filter(self) -> KalmanFilter2D:
        return self.kalman
