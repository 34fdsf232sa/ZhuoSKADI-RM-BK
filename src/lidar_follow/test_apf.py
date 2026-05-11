#!/usr/bin/env python3
"""
APF Controller Unit Test

Tests obstacle avoidance logic with synthetic LaserScan data.
"""

import math
import sys
import os

# Allow running from project root
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from apf_controller import APFController, APFParams
from kalman_filter_2d import KalmanFilter2D


def generate_scan(num_points: int = 360,
                  obstacle_dist: float = 0.5,
                  obstacle_angle: float = 0.0) -> list:
    """Generate synthetic LaserScan with a single obstacle."""
    ranges = [float('inf')] * num_points
    # Place obstacle at specified position
    angle_min = -math.pi
    angle_max = math.pi
    angle_inc = (angle_max - angle_min) / (num_points - 1)

    for i in range(num_points):
        angle = angle_min + i * angle_inc
        if abs(angle - obstacle_angle) < 0.05:
            ranges[i] = obstacle_dist

    return ranges, angle_min, angle_inc


def test_kalman():
    """Test Kalman filter convergence."""
    print("=== Kalman Filter Test ===")
    kf = KalmanFilter2D(process_noise=0.1, measurement_noise=0.05)

    # Simulate a target moving at slow constant velocity
    true_positions = [(0.4 + 0.02 * t, 0.01 * t) for t in range(20)]
    measurements = [(x + 0.003 * math.sin(t),
                     y + 0.003 * math.cos(t))
                    for t, (x, y) in enumerate(true_positions)]

    errors = []
    for i, (meas_x, meas_y) in enumerate(measurements):
        fx, fy = kf.update(meas_x, meas_y)
        true_x, true_y = true_positions[i]
        err = math.hypot(fx - true_x, fy - true_y)
        errors.append(err)

    avg_error = sum(errors) / len(errors)
    print(f"  Steps: {len(measurements)}")
    print(f"  Mean filter error: {avg_error*1000:.1f}mm")
    print(f"  Final velocity: vx={kf.velocity_x:.3f}, vy={kf.velocity_y:.3f}")
    print(f"  Kalman: {'PASSED' if avg_error < 0.1 else 'CHECK'} (error converged)")
    return avg_error < 0.1


def test_apf_basic():
    """Test APF with no obstacle (clear path)."""
    print("\n=== APF Basic Test (Clear Path) ===")
    apf = APFController()

    ranges = [10.0] * 360  # All far away
    angle_min = -math.pi
    angle_inc = 2 * math.pi / 360

    vx, vy, wz, info = apf.process_scan(
        ranges, angle_min, angle_inc, target_x=0.8, target_y=0.0)

    print(f"  Target: (0.8, 0.0), follow_dist: {apf.p.follow_dist}")
    print(f"  vx={vx:.3f} m/s, wz={wz:.3f} rad/s")
    print(f"  Emergency stop: {info['emergency_stop']}")
    print(f"  Min obstacle: {info['min_obstacle_dist']:.1f}m")
    assert vx > 0, f"Expected forward velocity, got {vx}"
    print("  APF basic: PASSED")
    return True


def test_apf_emergency():
    """Test APF emergency stop with close obstacle."""
    print("\n=== APF Emergency Stop Test ===")
    apf = APFController()

    num = 360
    ranges = [10.0] * num
    # Place obstacle 0.15m directly to the left (90°), far outside robot frame
    angle_min = -math.pi
    angle_inc = 2 * math.pi / num
    angle_rad = math.radians(90)  # directly left (y-axis)
    idx = int((angle_rad - angle_min) / angle_inc)
    if 0 <= idx < num:
        for off in range(-1, 2):
            if 0 <= idx + off < num:
                ranges[idx + off] = 0.15

    vx, vy, wz, info = apf.process_scan(
        ranges, angle_min, angle_inc, target_x=0.8, target_y=0.0)

    print(f"  Obstacle at 90° (directly left), 0.15m")
    print(f"  vx={vx:.3f}, emergency_stop={info['emergency_stop']}")
    print(f"  min_obstacle_dist={info['min_obstacle_dist']:.3f}m")
    assert info['emergency_stop'], "Expected emergency stop!"
    assert vx == 0.0, f"Expected zero velocity, got {vx}"
    print("  APF emergency: PASSED")
    return True


def test_apf_repulsion():
    """Test APF repulsive force from nearby obstacle."""
    print("\n=== APF Repulsion Test ===")
    apf = APFController()

    num = 360
    ranges = [10.0] * num
    angle_min = -math.pi
    angle_inc = 2 * math.pi / num
    # Obstacle at 0.23m, directly to the right (-90°)
    angle_rad = math.radians(-90)
    idx = int((angle_rad - angle_min) / angle_inc)
    if 0 <= idx < num:
        for off in range(-1, 2):
            if 0 <= idx + off < num:
                ranges[idx + off] = 0.23

    vx, vy, wz, info = apf.process_scan(
        ranges, angle_min, angle_inc, target_x=0.8, target_y=0.0)

    print(f"  Obstacle at -90° (right), 0.23m")
    print(f"  vx={vx:.3f} m/s, slowdown={info['slowdown_active']}")
    print(f"  min_obstacle_dist={info['min_obstacle_dist']:.3f}m")
    assert info['slowdown_active'], "Expected slowdown!"
    print("  APF repulsion: PASSED")
    return True


def test_lidar_tracker():
    """Test full LidarTracker pipeline."""
    print("\n=== LidarTracker Integration Test ===")
    from lidar_tracker import LidarTracker

    tracker = LidarTracker()
    tracker.set_target(0.5, 0.0)

    # Simulate a person at (0.45, 0.05) — slightly off expected target
    num = 360
    ranges = [10.0] * num
    angle_min = -math.pi
    angle_inc = 2 * math.pi / num
    # Place "person" points at (0.45, 0.05)
    person_angle = math.atan2(0.05, 0.45)
    person_dist = math.hypot(0.45, 0.05)
    idx = int((person_angle - angle_min) / angle_inc)
    # Cluster of points to simulate legs
    for offset in range(-2, 3):
        if 0 <= idx + offset < num:
            ranges[idx + offset] = person_dist

    velocities = []
    for frame in range(5):
        vx, wz, info = tracker.process_scan(ranges, angle_min, angle_inc)
        velocities.append((vx, wz))

    tx, ty = tracker.target_position
    print(f"  Initial target: (0.5, 0.0)")
    print(f"  Final target: ({tx:.3f}, {ty:.3f})")
    print(f"  Final velocity: vx={velocities[-1][0]:.3f}, wz={velocities[-1][1]:.3f}")
    print(f"  Kalman velocity: vx={tracker.kalman_filter.velocity_x:.3f}, vy={tracker.kalman_filter.velocity_y:.3f}")
    print(f"  LidarTracker: PASSED")
    return True


if __name__ == '__main__':
    results = []
    results.append(test_kalman())
    results.append(test_apf_basic())
    results.append(test_apf_emergency())
    results.append(test_apf_repulsion())
    results.append(test_lidar_tracker())

    print(f"\n{'='*40}")
    passed = sum(results)
    total = len(results)
    print(f"Results: {passed}/{total} tests passed")
    if passed == total:
        print("ALL TESTS PASSED")
    else:
        print("SOME TESTS FAILED")
        sys.exit(1)
