import math

from ...common import AIR_DENSITY


class BladeTheory:
    """Unsteady Aerodynamic Parameter Estimation for Multirotor Helicopters [Nguyen+, 2019]"""

    a = 2 * math.pi  # 2D lift curve slope (ideal value)
    B = 0.9  # Tip loss factor
    gamma = 8.0  # Lock number (typical value, cf. Balic Helicopter Aerodynamics p.66)
    C_d0 = 0.02  # Profile drag coefficient (typical value)

    def __init__(
        self,
        num_blades: int,
        radius: float,
        blade_chord: float,
        pitch_angle: float,
        air_density: float = AIR_DENSITY,
    ) -> None:
        assert num_blades > 0
        assert radius > 0
        assert blade_chord > 0
        assert 0 < pitch_angle < math.pi
        assert air_density > 0

        self._N = num_blades
        self._R = radius
        self._c = blade_chord
        self._theta = pitch_angle
        self._rho = air_density

    def motor_const(self) -> float:
        return 4 * math.pi * self._C_T() * self._rho * self._R**4

    def moment_const(self) -> float:
        return self._R * self._lambda()

    def rotor_drag_coef(self) -> float:
        return 4 * math.pi * self._rho * self._R**3 * self._C_H()

    def _sigma(self) -> float:
        """Solidity"""
        return (self._N * self._c) / (math.pi * self._R)

    def _lambda(self) -> float:
        """Inflow ratio"""
        a_B_sigma = self.a * self.B * self._sigma()
        return a_B_sigma * self.B / 16 * (math.sqrt(1 + (64 * self._theta) / (3 * a_B_sigma)) - 1)

    def _C_T(self) -> float:
        """Thrust coefficient"""
        return 2 * self._lambda() ** 2

    def _C_H(self) -> float:
        """Horizontal force coefficient (devided by mu)"""
        sigma = self._sigma()
        lam = self._lambda()
        b0 = 0.5 * self.gamma * (self._theta / 4 - lam / 3)
        b1c = 2 * (lam - (4 / 3) * self._theta)  # devided by mu
        b1s = -(4 / 3) * b0  # devided by mu
        return (sigma / 4) * (
            self.C_d0
            + (self.a / 6) * (2 * self._theta * (3 * lam - 2 * b1c) + 9 * lam * b1c + 2 * b0 * b1s + 3 * b0**2)
        )
