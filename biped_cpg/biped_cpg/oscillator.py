"""Two coupled phase oscillators, with stable left/right opposition.

right_dot = omega + coupling * sin(left - right - pi)
left_dot  = omega - coupling * sin(left - right - pi)

The phase difference tends to pi. This is a phase-oscillator CPG, not a neural
network or a Hopf oscillator. Sensors are not inputs to this rhythm generator.
"""
import math


class CoupledCPG:
    def __init__(self, period, coupling=2.0):
        if not math.isfinite(period) or period <= 0 or not math.isfinite(coupling) or coupling <= 0:
            raise ValueError('period and coupling must be finite and positive')
        self.omega = 2 * math.pi / period
        self.coupling = coupling
        self.right = 0.
        self.left = math.pi

    @property
    def phase_error(self):
        return math.atan2(math.sin(self.left - self.right - math.pi),
                          math.cos(self.left - self.right - math.pi))

    def step(self, dt):
        if not math.isfinite(dt) or dt < 0:
            raise ValueError('CPG time increment must be finite and nonnegative')
        # Exact integration of e_dot = -2*k*sin(e). The mean phase advances
        # uniformly, without Euler drift or an update-rate-dependent frequency.
        error = self.phase_error
        reduced = 2 * math.atan2(math.sin(error / 2) * math.exp(-2 * self.coupling * dt),
                                math.cos(error / 2))
        advance = self.omega * dt
        self.right += advance + (error - reduced) / 2
        self.left += advance - (error - reduced) / 2
        return self.right
