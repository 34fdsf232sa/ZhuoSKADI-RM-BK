"""Unit tests for APFController — Test-Driven Development."""

import sys
import os
import math
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'lidar_follow'))
from apf_controller import APFController, APFParams


def make_scan(num=360, fill=10.0):
    """Helper: generate synthetic LaserScan ranges."""
    return [fill] * num, -math.pi, 2 * math.pi / num


class TestAPFControllerClearPath:
    def test_moves_toward_target(self):
        apf = APFController()
        ranges, a_min, a_inc = make_scan()
        vx, vy, wz, info = apf.process_scan(ranges, a_min, a_inc, target_x=1.0, target_y=0.3)
        assert vx > 0, f"Expected forward vx, got {vx}"
        assert not info['emergency_stop']

    def test_stops_at_follow_distance(self):
        apf = APFController()
        ranges, a_min, a_inc = make_scan()
        vx, vy, wz, info = apf.process_scan(ranges, a_min, a_inc,
                                             target_x=apf.p.follow_dist, target_y=0.0)
        assert abs(vx) < 0.1, f"Should stop at follow distance, got vx={vx}"

    def test_rotates_toward_side_target(self):
        apf = APFController()
        ranges, a_min, a_inc = make_scan()
        vx, vy, wz, info = apf.process_scan(ranges, a_min, a_inc, target_x=0.5, target_y=1.0)
        # Should rotate left (positive wz for target on left)
        assert wz > 0, f"Expected positive wz for left target, got {wz}"


class TestAPFControllerObstacle:
    def test_emergency_stop_near_obstacle(self):
        apf = APFController()
        ranges, a_min, a_inc = make_scan()
        # Place obstacle at 90 deg (outside robot frame), 0.15m away
        idx = int((math.radians(90) - a_min) / a_inc)
        if 0 <= idx < len(ranges):
            ranges[idx] = 0.15
        vx, vy, wz, info = apf.process_scan(ranges, a_min, a_inc, target_x=1.0, target_y=0.0)
        assert info['emergency_stop'], "Should emergency stop"
        assert vx == 0.0

    def test_slowdown_near_obstacle(self):
        apf = APFController()
        ranges, a_min, a_inc = make_scan()
        idx = int((math.radians(-90) - a_min) / a_inc)
        if 0 <= idx < len(ranges):
            ranges[idx] = 0.23
        vx, vy, wz, info = apf.process_scan(ranges, a_min, a_inc, target_x=1.0, target_y=0.0)
        assert info['slowdown_active'], "Should be in slowdown zone"

    def test_robot_frame_exclusion(self):
        apf = APFController()
        ranges, a_min, a_inc = make_scan()
        # Obstacle directly ahead at 0 deg within robot frame
        idx = int((0.0 - a_min) / a_inc)
        if 0 <= idx < len(ranges):
            ranges[idx] = 0.12  # Inside robot frame (front=0.15)
        vx, vy, wz, info = apf.process_scan(ranges, a_min, a_inc, target_x=1.0, target_y=0.0)
        # Should NOT emergency stop — obstacle is within robot frame exclusion
        assert not info['emergency_stop'], "Robot frame points should be excluded"


class TestAPFControllerEdgeCases:
    def test_no_ranges(self):
        apf = APFController()
        vx, vy, wz, info = apf.process_scan([], -math.pi, 0.01, 1.0, 0.0)
        assert info['min_obstacle_dist'] == 999.0

    def test_all_inf_ranges(self):
        apf = APFController()
        ranges = [float('inf')] * 360
        vx, vy, wz, info = apf.process_scan(ranges, -math.pi, math.pi / 180, 1.0, 0.0)
        assert info['min_obstacle_dist'] == 999.0

    def test_target_directly_ahead_returns_zero_wz(self):
        apf = APFController()
        ranges, a_min, a_inc = make_scan()
        vx, vy, wz, info = apf.process_scan(ranges, a_min, a_inc, target_x=2.0, target_y=0.0)
        assert abs(wz) < 0.01, f"Target dead ahead should give ~0 wz, got {wz}"


class TestAPFParams:
    def test_custom_params(self):
        params = APFParams(follow_dist=0.3, max_linear_speed=0.8,
                           apf_emergency_dist=0.1, apf_influence_dist=0.3)
        apf = APFController(params)
        assert apf.p.follow_dist == 0.3
        assert apf.p.max_linear_speed == 0.8

    def test_default_params_unchanged(self):
        params = APFParams()
        assert params.follow_dist == 0.4
        assert params.target_radius == 0.3
        assert params.apf_emergency_dist == 0.20
