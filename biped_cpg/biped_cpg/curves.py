"""Small cubic Hermite tables with exact speed/acceleration bounds."""
import math
import numpy as np


class Curve:
    def __init__(self, rows, stops=()):
        self.rows = np.asarray(rows, dtype=float)
        if self.rows.ndim != 2 or self.rows.shape[1] != 12 or len(self.rows) < 3 or not np.all(np.isfinite(self.rows)):
            raise ValueError('curve requires at least three finite 12-joint poses')
        slopes = np.zeros_like(self.rows)
        slopes[1:-1] = .5 * (self.rows[2:] - self.rows[:-2])
        for index in stops:
            slopes[index] = 0.
        first, last = self.rows[:-1], self.rows[1:]
        m0, m1 = slopes[:-1], slopes[1:]
        self.a = 2*first - 2*last + m0 + m1
        self.b = -3*first + 3*last - 2*m0 - m1
        self.c, self.d = m0, first
        n = len(self.rows) - 1
        roots = np.clip(np.divide(-self.b, 3*self.a, out=np.zeros_like(self.a),
                                  where=np.abs(self.a) > 1e-15), 0, 1)
        self.speed_bound = n * max(float(np.max(np.abs(self.c))),
                                   float(np.max(np.abs(3*self.a + 2*self.b + self.c))),
                                   float(np.max(np.abs(3*self.a*roots**2 + 2*self.b*roots + self.c))))
        self.acceleration_bound = n*n * max(float(np.max(np.abs(2*self.b))),
                                            float(np.max(np.abs(6*self.a + 2*self.b))))

    def sample(self, fraction):
        if not math.isfinite(fraction):
            raise ValueError('nonfinite curve fraction')
        if fraction <= 0:
            return self.rows[0].copy()
        if fraction >= 1:
            return self.rows[-1].copy()
        coordinate = fraction * (len(self.rows) - 1)
        index = min(int(coordinate), len(self.rows) - 2)
        u = coordinate - index
        return ((self.a[index]*u + self.b[index])*u + self.c[index])*u + self.d[index]

    def duration(self, minimum, speed_limit, acceleration_limit):
        return max(minimum, self.speed_bound / speed_limit,
                   math.sqrt(self.acceleration_bound / acceleration_limit))
