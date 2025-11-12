from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class PIDLimits:
    output_min: float = float("-inf")
    output_max: float = float("inf")
    integral_min: float = float("-inf")
    integral_max: float = float("inf")


class PID:
    """
    Discrete PID with:
    - anti-windup (integral clamping)
    - derivative low-pass filtering (first-order)
    - optional setpoint weighting on P and D terms (beta, gamma)
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        dt: float,
        limits: Optional[PIDLimits] = None,
        deriv_filter_hz: float = 0.0,
        setpoint_weight_p: float = 1.0,
        setpoint_weight_d: float = 0.0,
        integral_on_error: bool = True,
    ) -> None:
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.dt = float(dt)
        self.limits = limits or PIDLimits()
        self.deriv_filter_hz = float(deriv_filter_hz)
        self.beta = float(setpoint_weight_p)
        self.gamma = float(setpoint_weight_d)
        self.integral_on_error = bool(integral_on_error)

        # Internal states
        self._integral = 0.0
        self._prev_measurement = 0.0
        self._prev_error = 0.0
        self._d_filtered = 0.0
        self._initialized = False

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_measurement = 0.0
        self._prev_error = 0.0
        self._d_filtered = 0.0
        self._initialized = False

    def _lowpass(self, x_prev: float, x_now: float) -> float:
        if self.deriv_filter_hz <= 0.0:
            return x_now
        # First-order low-pass: y_k = alpha * y_{k-1} + (1 - alpha) * x_k
        # alpha = exp(-2*pi*fc*dt)
        import math

        alpha = math.exp(-2.0 * math.pi * self.deriv_filter_hz * self.dt)
        return alpha * x_prev + (1.0 - alpha) * x_now

    def update(
        self,
        setpoint: float,
        measurement: float,
        feedforward: float = 0.0,
        return_terms: bool = False,
    ) -> float | Tuple[float, Tuple[float, float, float]]:
        if not self._initialized:
            self._prev_measurement = float(measurement)
            self._prev_error = float(setpoint - measurement)
            self._initialized = True

        error = float(setpoint - measurement)

        # P-term with setpoint weighting beta
        p_term = self.kp * (self.beta * setpoint - measurement)

        # I-term
        if self.integral_on_error:
            self._integral += self.ki * error * self.dt
        else:
            # Integrate on measurement instead of error (less aggressive)
            self._integral += -self.ki * (measurement) * self.dt
        # Clamp integral
        self._integral = max(self.limits.integral_min, min(self._integral, self.limits.integral_max))

        # D-term with setpoint weighting gamma and low-pass filter on derivative signal
        # Using derivative of measurement with weighting commonly reduces noise sensitivity
        d_input = (self.gamma * setpoint - measurement)
        d_raw = (d_input - (self.gamma * self._prev_measurement - self._prev_measurement)) / self.dt
        d_term_unfiltered = self.kd * d_raw
        self._d_filtered = self._lowpass(self._d_filtered, d_term_unfiltered)

        u = p_term + self._integral + self._d_filtered + float(feedforward)
        # Clamp output
        u = max(self.limits.output_min, min(u, self.limits.output_max))

        # Store previous
        self._prev_measurement = float(measurement)
        self._prev_error = float(error)

        if return_terms:
            return u, (p_term, self._integral, self._d_filtered)
        return u


