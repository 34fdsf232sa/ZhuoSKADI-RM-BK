#!/usr/bin/env python3
"""
2D Kalman Filter — Constant Velocity Model

Direct Python port of jie_deamon/include/kalman_filter.hpp.
Zero external dependencies (pure Python, no NumPy/Eigen required).

State: [x, y, vx, vy]
Observations: [x, y]
Model: CV (constant velocity) with dt from measurement intervals
"""

import time
from typing import Tuple


class KalmanFilter2D:
    """2D constant-velocity Kalman filter for target position smoothing."""

    def __init__(self, process_noise: float = 0.1, measurement_noise: float = 0.05):
        self._q = process_noise
        self._r = measurement_noise
        self.reset()

    def set_process_noise(self, q: float):
        self._q = q

    def set_measurement_noise(self, r: float):
        self._r = r

    def reset(self):
        self._initialized = False
        self._x = [0.0, 0.0, 0.0, 0.0]
        # 4x4 covariance matrix, flattened
        self._P = [0.0] * 16
        self._P[0] = 1.0    # var(x)
        self._P[5] = 1.0    # var(y)
        self._P[10] = 10.0  # var(vx)
        self._P[15] = 10.0  # var(vy)
        self._last_time = time.monotonic()

    def update(self, meas_x: float, meas_y: float) -> Tuple[float, float]:
        """Predict + update with measurement. Returns (filtered_x, filtered_y)."""
        now = time.monotonic()

        if not self._initialized:
            self._x[0], self._x[1] = meas_x, meas_y
            self._x[2], self._x[3] = 0.0, 0.0
            self._last_time = now
            self._initialized = True
            return meas_x, meas_y

        dt = now - self._last_time
        self._last_time = now

        # Clamp dt to sensible range
        if dt <= 0.0 or dt > 1.0:
            dt = 0.1

        self._predict(dt)
        self._correct(meas_x, meas_y)

        return self._x[0], self._x[1]

    def predict_only(self, dt: float) -> Tuple[float, float]:
        """Predict without measurement (e.g., when target is lost)."""
        if not self._initialized:
            return 0.0, 0.0
        self._predict(dt)
        return self._x[0], self._x[1]

    def set_state(self, x: float, y: float):
        """Force-set state (e.g., when user manually sets target)."""
        self._x = [x, y, 0.0, 0.0]
        self._P = [0.0] * 16
        self._P[0] = 0.5
        self._P[5] = 0.5
        self._P[10] = 10.0
        self._P[15] = 10.0
        self._initialized = True
        self._last_time = time.monotonic()

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    @property
    def velocity_x(self) -> float:
        return self._x[2]

    @property
    def velocity_y(self) -> float:
        return self._x[3]

    # ---- Internal matrix operations (expanded, no external deps) ----

    def _predict(self, dt: float):
        """State prediction: x' = F*x, P' = F*P*F^T + Q"""
        # State: x += vx*dt, y += vy*dt
        self._x[0] += self._x[2] * dt
        self._x[1] += self._x[3] * dt

        dt2 = dt * dt
        dt3 = dt2 * dt

        # F*P (temporary)
        FP = [0.0] * 16
        FP[0]  = self._P[0]  + dt * self._P[8]
        FP[1]  = self._P[1]  + dt * self._P[9]
        FP[2]  = self._P[2]  + dt * self._P[10]
        FP[3]  = self._P[3]  + dt * self._P[11]
        FP[4]  = self._P[4]  + dt * self._P[12]
        FP[5]  = self._P[5]  + dt * self._P[13]
        FP[6]  = self._P[6]  + dt * self._P[14]
        FP[7]  = self._P[7]  + dt * self._P[15]
        FP[8]  = self._P[8]
        FP[9]  = self._P[9]
        FP[10] = self._P[10]
        FP[11] = self._P[11]
        FP[12] = self._P[12]
        FP[13] = self._P[13]
        FP[14] = self._P[14]
        FP[15] = self._P[15]

        # P = FP * F^T + Q
        q = self._q
        self._P[0]  = FP[0]  + FP[2]  * dt + q * dt3 / 3.0
        self._P[1]  = FP[1]  + FP[3]  * dt
        self._P[2]  = FP[2]  + q * dt2 / 2.0
        self._P[3]  = FP[3]
        self._P[4]  = FP[4]  + FP[6]  * dt
        self._P[5]  = FP[5]  + FP[7]  * dt + q * dt3 / 3.0
        self._P[6]  = FP[6]  + q * dt2 / 2.0
        self._P[7]  = FP[7]
        self._P[8]  = FP[8]  + FP[10] * dt + q * dt2 / 2.0
        self._P[9]  = FP[9]  + FP[11] * dt
        self._P[10] = FP[10] + q * dt
        self._P[11] = FP[11]
        self._P[12] = FP[12] + FP[14] * dt
        self._P[13] = FP[13] + FP[15] * dt + q * dt2 / 2.0
        self._P[14] = FP[14] + q * dt
        self._P[15] = FP[15]

    def _correct(self, meas_x: float, meas_y: float):
        """Kalman update: K = P*H^T * inv(H*P*H^T + R), x += K*(z - H*x)"""
        # Innovation
        y0 = meas_x - self._x[0]
        y1 = meas_y - self._x[1]

        # S = H*P*H^T + R (2x2)
        s00 = self._P[0] + self._r
        s01 = self._P[1]
        s10 = self._P[4]
        s11 = self._P[5] + self._r

        # Inverse of S
        det = s00 * s11 - s01 * s10
        if abs(det) < 1e-12:
            return
        inv_det = 1.0 / det
        si00 =  s11 * inv_det
        si01 = -s01 * inv_det
        si10 = -s10 * inv_det
        si11 =  s00 * inv_det

        # K = P*H^T * S^(-1)  (4x2)
        k00 = self._P[0]  * si00 + self._P[1]  * si10
        k01 = self._P[0]  * si01 + self._P[1]  * si11
        k10 = self._P[4]  * si00 + self._P[5]  * si10
        k11 = self._P[4]  * si01 + self._P[5]  * si11
        k20 = self._P[8]  * si00 + self._P[9]  * si10
        k21 = self._P[8]  * si01 + self._P[9]  * si11
        k30 = self._P[12] * si00 + self._P[13] * si10
        k31 = self._P[12] * si01 + self._P[13] * si11

        # State update
        self._x[0] += k00 * y0 + k01 * y1
        self._x[1] += k10 * y0 + k11 * y1
        self._x[2] += k20 * y0 + k21 * y1
        self._x[3] += k30 * y0 + k31 * y1

        # Covariance update: P = (I - K*H) * P
        P_old = self._P.copy()
        self._P[0]  = (1.0 - k00) * P_old[0]  - k01 * P_old[4]
        self._P[1]  = (1.0 - k00) * P_old[1]  - k01 * P_old[5]
        self._P[2]  = (1.0 - k00) * P_old[2]  - k01 * P_old[6]
        self._P[3]  = (1.0 - k00) * P_old[3]  - k01 * P_old[7]
        self._P[4]  = -k10 * P_old[0]  + (1.0 - k11) * P_old[4]
        self._P[5]  = -k10 * P_old[1]  + (1.0 - k11) * P_old[5]
        self._P[6]  = -k10 * P_old[2]  + (1.0 - k11) * P_old[6]
        self._P[7]  = -k10 * P_old[3]  + (1.0 - k11) * P_old[7]
        self._P[8]  = -k20 * P_old[0]  - k21 * P_old[4]  + P_old[8]
        self._P[9]  = -k20 * P_old[1]  - k21 * P_old[5]  + P_old[9]
        self._P[10] = -k20 * P_old[2]  - k21 * P_old[6]  + P_old[10]
        self._P[11] = -k20 * P_old[3]  - k21 * P_old[7]  + P_old[11]
        self._P[12] = -k30 * P_old[0]  - k31 * P_old[4]  + P_old[12]
        self._P[13] = -k30 * P_old[1]  - k31 * P_old[5]  + P_old[13]
        self._P[14] = -k30 * P_old[2]  - k31 * P_old[6]  + P_old[14]
        self._P[15] = -k30 * P_old[3]  - k31 * P_old[7]  + P_old[15]
