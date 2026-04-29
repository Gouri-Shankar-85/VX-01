import math


class InverseKinematics:

    def __init__(self, L1: float, L2: float, L3: float):
        self._L1 = L1
        self._L2 = L2
        self._L3 = L3

    def compute(self, x: float, y: float, z: float):

        theta1 = math.atan2(y, x)
        r_plane = math.sqrt(x * x + y * y) - self._L1
        if r_plane < 0.0:
            return None

        r1 = math.sqrt(r_plane * r_plane + z * z)
        if r1 > self._L2 + self._L3 or r1 < abs(self._L2 - self._L3):
            return None

        phi2 = math.atan2(z, r_plane)
        cp1 = (self._L3 ** 2 - self._L2 ** 2 - r1 ** 2) / (-2.0 * self._L2 * r1)
        if not (-1.0 <= cp1 <= 1.0):
            return None

        phi1 = math.acos(cp1)
        theta2 = phi2 + phi1

        cp3 = (r1 ** 2 - self._L2 ** 2 - self._L3 ** 2) / (-2.0 * self._L2 * self._L3)
        cp3 = max(-1.0, min(1.0, cp3))
        
        theta3 = - (math.pi - math.acos(cp3))
        return (theta1, theta2, theta3)