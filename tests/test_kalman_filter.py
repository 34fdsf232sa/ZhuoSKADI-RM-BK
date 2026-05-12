"""Unit tests for KalmanFilter2D — TDD with edge cases."""

import sys
import os
import math
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'lidar_follow'))
from kalman_filter_2d import KalmanFilter2D


class TestKalmanFilterInit:
    def test_default_construction(self):
        kf = KalmanFilter2D()
        assert not kf.is_initialized

    def test_custom_noise_params(self):
        kf = KalmanFilter2D(process_noise=0.5, measurement_noise=0.01)
        assert not kf.is_initialized

    def test_reset_clears_state(self):
        kf = KalmanFilter2D()
        kf.update(1.0, 2.0)
        assert kf.is_initialized
        kf.reset()
        assert not kf.is_initialized


class TestKalmanFilterUpdate:
    def test_first_update_sets_state(self):
        kf = KalmanFilter2D()
        x, y = kf.update(1.0, 2.0)
        assert x == 1.0
        assert y == 2.0
        assert kf.is_initialized

    def test_convergence_on_static_target(self):
        kf = KalmanFilter2D(process_noise=0.01, measurement_noise=0.1)
        for _ in range(50):
            x, y = kf.update(0.5, 0.0)
        # Should converge close to true value
        assert abs(x - 0.5) < 0.02
        assert abs(y - 0.0) < 0.02

    def test_tracks_moving_target(self):
        kf = KalmanFilter2D(process_noise=0.1, measurement_noise=0.01)
        true_path = [(0.1 * t, 0.05 * t) for t in range(30)]
        errors = []
        for tx, ty in true_path:
            fx, fy = kf.update(tx + 0.002, ty + 0.002)
            errors.append(math.hypot(fx - tx, fy - ty))
        mean_err = sum(errors) / len(errors)
        assert mean_err < 0.05, f"Tracking error {mean_err:.3f} exceeds 0.05"

    def test_velocity_estimate(self):
        kf = KalmanFilter2D(process_noise=0.01, measurement_noise=0.01)
        for t in range(50):
            kf.update(0.2 * t, 0.0)
        # Velocity should be ~0.2
        assert 0.15 < kf.velocity_x < 0.25

    def test_set_state_resets_velocity(self):
        kf = KalmanFilter2D()
        for t in range(20):
            kf.update(0.1 * t, 0.0)
        assert abs(kf.velocity_x) > 0.05  # Has velocity
        kf.set_state(3.0, 0.0)
        assert kf.is_initialized
        assert kf.velocity_x == 0.0


class TestKalmanFilterPredictOnly:
    def test_predict_without_init_returns_zero(self):
        kf = KalmanFilter2D()
        x, y = kf.predict_only(0.1)
        assert x == 0.0
        assert y == 0.0

    def test_predict_after_update(self):
        kf = KalmanFilter2D(process_noise=0.001, measurement_noise=0.001)
        kf.update(0.5, 0.0)
        for _ in range(10):
            kf.update(0.55, 0.0)
        px, py = kf.predict_only(0.1)
        # Should predict forward ~0.05 from velocity
        assert abs(px - 0.55) < 0.1  # Rough check, depends on filter state


class TestKalmanFilterEdgeCases:
    def test_identical_measurements(self):
        kf = KalmanFilter2D()
        for _ in range(100):
            x, y = kf.update(1.0, 1.0)
        assert abs(x - 1.0) < 0.01
        assert abs(y - 1.0) < 0.01

    def test_large_jump_reset(self):
        kf = KalmanFilter2D(process_noise=0.1, measurement_noise=0.1)
        kf.update(0.0, 0.0)
        for _ in range(10):
            kf.update(0.0, 0.0)
        # Large jump — filter should follow measurement
        x, y = kf.update(5.0, 0.0)
        assert x > 1.0  # Filter moves toward new value quickly

    def test_set_state_preserves_initialized(self):
        kf = KalmanFilter2D()
        assert not kf.is_initialized
        kf.set_state(1.0, 2.0)
        assert kf.is_initialized
