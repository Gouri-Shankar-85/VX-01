import math

from .kinematics import InverseKinematics


class Leg:

    def __init__(
        self,
        leg_id: int,
        coxa_angle: float,
        body_radius: float,
        home_reach: float,
        home_z: float,
        L1: float,
        L2: float,
        L3: float,
    ):
        self._id = leg_id
        self._angle = coxa_angle
        self._ik = InverseKinematics(L1, L2, L3)

        ca = math.cos(coxa_angle)
        sa = math.sin(coxa_angle)

        self._pivot_x = body_radius * ca
        self._pivot_y = body_radius * sa
        self._home_x = (body_radius + home_reach) * ca
        self._home_y = (body_radius + home_reach) * sa
        self._home_z = home_z

    @property
    def home(self):
        return (self._home_x, self._home_y, self._home_z)

    def solve_ik(self, bx: float, by: float, bz: float):
        ca = math.cos(self._angle)
        sa = math.sin(self._angle)
        dx = bx - self._pivot_x
        dy = by - self._pivot_y
        cx = dx * ca + dy * sa
        cy = -dx * sa + dy * ca
        return self._ik.compute(cx, cy, bz)


class TripodGait:

    GROUP_A = (0, 2, 4)
    GROUP_B = (1, 3, 5)

    def __init__(self, step_period: float):
        self._period = step_period
        self._phase = 0.0

    def update(self, dt: float):
        self._phase = (self._phase + dt / self._period) % 1.0

    def swing_group(self):
        return self.GROUP_A if self._phase < 0.5 else self.GROUP_B

    def stance_group(self):
        return self.GROUP_B if self._phase < 0.5 else self.GROUP_A

    def local_t(self) -> float:
        if self._phase < 0.5:
            return self._phase * 2.0
        return (self._phase - 0.5) * 2.0

    def reset(self):
        self._phase = 0.0