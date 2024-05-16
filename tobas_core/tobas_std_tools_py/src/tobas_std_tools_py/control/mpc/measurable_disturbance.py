import numpy as np
from rich import print as rprint
from qpsolvers import solve_qp

from ...numpy_tools import make_mat_pow_arr
from ..observer import MeasurableDisturbanceObserver
from ..mpc import BasicMPC


class MeasurableDisturbanceMPC(BasicMPC):
    """状態方程式に測定可能な一定値外乱が含まれる場合のモデル予測制御(p.179)"""

    def __init__(self, observer: MeasurableDisturbanceObserver, **kwargs) -> None:
        super().__init__(observer=observer, **kwargs)
        self._Xi = self._calc_Xi()

    def step(self, y: np.ndarray, s: np.ndarray, d: np.ndarray) -> np.ndarray:
        """
        Parameters
        ----------
        y: np.ndarray
            現在のプラント出力
        s: np.ndarray
            設定値
        d: np.ndarray
            測定された外乱

        Returns
        ----------
        u: np.ndarray
            制御入力
        """

        assert y.shape == (self.y_dim,)
        assert s.shape == (self.z_dim,)
        assert d.shape == (self.d_dim,)

        # QP問題を解く
        phi = self._calc_phi(y, s, d)
        omega = self._calc_omega()
        delta_U = solve_qp(self._Phi, phi, self._Omega, omega, solver=self._qpsolver)
        if delta_U is None:
            rprint("[red]Error: failed to solve QP problem[/red]")
            return

        # 直近の制御入力を求める
        delta_u = delta_U[: self.u_dim]
        self._last_u += delta_u

        # オブザーバを進める
        self._observer.step(y, d, self._last_u)

        return self._last_u

    def _calc_Xi(self):
        """(5.9)"""

        Cz_A_pow_Bd = make_mat_pow_arr(self._observer.A, self._Hp - 1, self._observer.Cz) @ self._observer.B

        Xi = np.zeros((self.z_dim * self._Hp, self.d_dim * self._Hp))
        for i in range(0, self._Hp):
            for j in range(0, self._Hp):
                if i < j:
                    continue
                slice_r = slice(self.z_dim * i, self.z_dim * (i + 1))
                slice_c = slice(self.d_dim * j, self.d_dim * (j + 1))
                Xi[slice_r, slice_c] = Cz_A_pow_Bd[i - j, :, :]

        return Xi

    def _calc_phi(self, y, s, d):
        """(3.11)より，(3.43)のphiを計算"""

        Epsilon = self._calc_Epsilon(y, s, d)
        phi = (self._minus_2_Theta_Q @ Epsilon).ravel()
        return phi

    def _calc_Epsilon(self, y, s, d):
        """(5.10)"""

        Dm = self._calc_Dm(d)
        Epsilon = super()._calc_Epsilon(y, s) - self._Xi @ Dm
        return Epsilon

    def _calc_Dm(self, d):
        """(5.8)"""

        Dm = np.tile(d, self._Hp)
        return Dm

    @property
    def d_dim(self) -> int:
        return self._observer.d_dim
