import qpsolvers
import numpy as np
import numpy.linalg as LA
from matplotlib import pyplot as plt
from rich import print as rprint
from typing import Tuple, List

from ...numpy_tools import make_mat_pow_arr, make_mat_diag
from ..observer import BaseObserver


class BasicMPC:
    """
    基本的なモデル予測制御
    - https://www.tdupress.jp/book/b349347.html を参考

    Note
    ----------
    - 制約の緩和はしない
    - 値の正規化はしない
    """

    def __init__(
        self,
        observer: BaseObserver,
        Hw: int,
        Hp: int,
        Hu: int,
        Q_values: np.ndarray,
        R_values: np.ndarray,
        S_values: np.ndarray,
        u_range: np.ndarray = None,
        u_rate_range: np.ndarray = None,
        z_range: np.ndarray = None,
        u_const_mat: np.ndarray = None,
        u_rate_const_mat: np.ndarray = None,
        z_const_mat: np.ndarray = None,
        T_ref: float = 0.0,
        ref_traj: str = "const",
        qpsolver: str = "quadprog",
        tracking: bool = False,
        y_labels: List[str] = None,
    ) -> None:
        """
        Parameters
        ----------
        observer: BaseObserver
            状態オブザーバ
        Hw, Hp: int
            [Hw, Hp]の区間の出力を評価する
        Hu: int
            [1, Hu]の区間の入力を制御できる
        Q_values, R_values, S_values: np.ndarray
            z, u_rate, uにかかる重み
        u_range, u_rate_range, z_range: np.ndarray, default None
            入力，入力の変化率，制御変数に関する不等式制約
            制約を設けない部分には±np.infを入れる
        u_const_mat, u_rate_const_mat, z_const_mat: np.ndarray, default None
            不等式成約を行列で与える場合
            (F c) @ (u 1).T <= 0の(F c)の部分を与える
        T_ref: float, default None
            応答時定数
        ref_traj: str, default 'const'
            設定値に対する参照軌道の作成方法
        qpsolver: str, default 'quadprog'
            QP問題のソルバ(https://pypi.org/project/qpsolvers/)
        tracking: bool, default False
            Trueの場合はオフセットなし追従(p.22)
        y_labels: List[str], default None
            各出力変数の名前

        Returns
        ----------
        None

        Note
        ----------
        - u_rateの時間単位はTsと一致している必要がある
        - オフセットなし追従を行う場合は，Cy = Cz & L = 0
        - オフセットなし追従で対応できるのは安定限界まで．不安定なダイナミクスには対応不可．
        - hoge_rangeとhoge_const_matはどちらか一方を指定する
        - Q, R, Sが対称行列と仮定して計算している部分がいくつかある
        """

        assert 1 <= Hw <= Hp
        assert 1 <= Hu <= Hp
        assert Q_values.shape == (observer.z_dim,)
        assert R_values.shape == (observer.u_dim,)
        assert S_values.shape == (observer.u_dim,)
        assert u_range is None or u_const_mat is None
        assert u_rate_range is None or u_rate_const_mat is None
        assert z_range is None or z_const_mat is None
        assert T_ref >= 0.0
        assert ref_traj in {"const", "exp"}
        assert qpsolver in qpsolvers.available_solvers
        if tracking:
            assert np.allclose(observer.Cy, observer.Cz)
            assert np.all(np.isclose(observer.L, 0.0))
        if y_labels is not None:
            assert len(y_labels) == observer.y_dim

        self._observer = observer
        self._Hw = Hw
        self._Hp = Hp
        self._Hu = Hu
        self._qpsolver = qpsolver
        self._tracking = tracking
        self._y_labels = y_labels
        self._last_u = np.zeros((observer.u_dim,))
        self._decays = self._calc_decays(T_ref, ref_traj)

        # p.53
        if u_rate_range is not None:
            assert u_rate_range.shape == (self.u_dim, 2)
            delta_u_range = u_rate_range * self.Ts
            E = self._make_const_mat_from_range(delta_u_range, Hu)
        elif u_rate_const_mat is not None:
            assert u_rate_const_mat.ndim == 2
            assert u_rate_const_mat.shape[1] == self.u_dim + 1
            delta_u_const_mat = u_rate_const_mat.copy()
            delta_u_const_mat[:, -1] *= self.Ts
            E = self._make_const_mat_from_mat(delta_u_const_mat, Hu)
        else:
            delta_u_const_mat = np.empty((0, self.u_dim + 1))
            E = self._make_const_mat_from_mat(delta_u_const_mat, Hu)

        if u_range is not None:
            assert u_range.shape == (self.u_dim, 2)
            F = self._make_const_mat_from_range(u_range, Hu)
        elif u_const_mat is not None:
            assert u_const_mat.ndim == 2
            assert u_const_mat.shape[1] == self.u_dim + 1
            F = self._make_const_mat_from_mat(u_const_mat, Hu)
        else:
            u_const_mat = np.empty((0, self.u_dim + 1))
            F = self._make_const_mat_from_mat(u_const_mat, Hu)

        if z_range is not None:
            assert z_range.shape == (self.z_dim, 2)
            G = self._make_const_mat_from_range(z_range, Hp)
        elif z_const_mat is not None:
            assert z_const_mat.ndim == 2
            assert z_const_mat.shape[1] == self.z_dim + 1
            G = self._make_const_mat_from_mat(z_const_mat, Hp)
        else:
            z_const_mat = np.empty((0, self.z_dim + 1))
            G = self._make_const_mat_from_mat(z_const_mat, Hp)

        # p.91
        self._Q = self._make_weight_mat(Q_values, Hw, Hp)
        self._R = self._make_weight_mat(R_values, 1, Hu)

        # p.100
        self._W = E[:, :-1]
        self._w = -E[:, -1]

        # p.99
        self._F_gothic = self._calc_F_gothic(F)
        self._F_gothic_1 = self._F_gothic[:, 0 : self.u_dim]
        self._f = F[:, -1]

        # p.100
        self._Gamma = G[:, :-1]
        self._g = G[:, -1]

        # 練習問題3-5
        S = np.diag(S_values)
        self._Sa = self._calc_Sa(S)
        self._Sb_over_u_diff = np.array([i * S for i in reversed(range(1, Hu + 1))]).reshape(-1, self.u_dim)

        self._Psi = None
        self._Upsilon = None
        self._Theta = None
        self._Eta = None
        self._Phi = None
        self._Omega = None
        self._minus_2_Theta_Q = None
        self._minus_Gamma_Psi = None
        self._minus_Gamma_Upsilon = None
        self._R_plus_Sa = self._R + self._Sa

        self._update_internal_variables()

    def step(self, y: np.ndarray, s: np.ndarray, u_ref: np.ndarray = None) -> np.ndarray:
        """
        Parameters
        ----------
        y: np.ndarray
            現在のプラント出力
        s: np.ndarray
            設定値
        u_ref: np.ndarray, default None
            制御入力の参照値

        Returns
        ----------
        u: np.ndarray
            制御入力
        """

        assert y.shape == (self.y_dim,)
        assert s.shape == (self.z_dim,)
        if u_ref is None:
            u_ref = np.zeros((self.u_dim,))
        else:
            assert u_ref.shape == (self.u_dim,)

        # QP問題を解く
        phi = self._calc_phi(y, s, u_ref)
        omega = self._calc_omega()
        delta_U = qpsolvers.solve_qp(self._Phi, phi, self._Omega, omega, solver=self._qpsolver)
        if delta_U is None:
            rprint("[red]Error: failed to solve QP problem[/red]")
            delta_u = np.zeros((self.u_dim,))
        else:
            delta_u = delta_U[: self.u_dim]

        # 制御入力を更新する
        self._last_u += delta_u

        # オブザーバを進める
        self._observer.step(y, self._last_u)

        return self._last_u

    def plot_frequency_response(  # TODO: うまくいかない(数値誤差？)
        self, min_omega: float = 1e-2, num: int = 100, figsize: Tuple[float, float] = (12.0, 9.0)
    ) -> None:
        """ハード制約がない場合の感度関数の特異値をプロットする(p.244, 演習問題7.6)"""

        assert LA.matrix_rank(self.A) == self.x_dim

        max_omega = np.pi / self.Ts  # ナイキスト周波数
        assert 0.0 < min_omega < max_omega, f"min_omega: {min_omega}, max_omega: {max_omega}"
        omega_list = np.power(10.0, (np.linspace(np.log10(min_omega), np.log10(max_omega), num)))

        I_x = np.identity(self.x_dim, dtype=np.complex128)
        I_u = np.identity(self.u_dim, dtype=np.complex128)
        I_y = np.identity(self.y_dim, dtype=np.complex128)
        I_z_Hp = np.identity(self.z_dim * self._Hp, dtype=np.complex128)
        L_dash = (LA.inv(self.A) @ self._observer.L).astype(np.complex128)
        K_mpc = (LA.inv(self._Eta) @ self._Theta.T @ self._Q)[: self.u_dim, :].astype(np.complex128)  # (3.27)

        S_sing_list = []
        T_sing_list = []

        for omega in omega_list:
            z = np.exp(1.0j * omega * self.Ts)

            P = self.Cy @ LA.inv(z * I_x - self.A) @ self.B
            K = K_mpc @ LA.inv((1.0 - 1.0 / z) * I_z_Hp + self._Upsilon @ K_mpc)
            X0 = LA.inv(z * I_x - self.A + self._observer.L @ self.Cy)
            X1 = (I_x - L_dash @ self.Cy) @ X0
            X2 = z * X0
            X3 = P @ LA.inv(I_u + K @ self._Psi @ X1 @ self.B) @ K
            S = LA.inv(I_y + X3 @ self._Psi @ X2 @ L_dash)
            T = I_y - S

            _, S_sing, _ = LA.svd(S)
            _, T_sing, _ = LA.svd(T)
            assert S_sing.shape == T_sing.shape == (self.y_dim,)

            S_sing_list.append(S_sing.copy())
            T_sing_list.append(T_sing.copy())

        S_sing_list = np.asarray(S_sing_list).T
        T_sing_list = np.asarray(T_sing_list).T

        fig = plt.figure(figsize=figsize)
        for i in range(0, self.y_dim):
            ax = fig.add_subplot(
                self.y_dim,
                1,
                i + 1,
                title=self._y_labels[i] if self._y_labels is not None else f"No.{i}",
                ylabel="Singular Value",
                xlim=(min_omega, max_omega),
            )
            if i == self.y_dim - 1:
                ax.set_xlabel("Frequency (rad/?)")
            else:
                ax.set_xticks([])
            ax.set_xlim()
            ax.set_xscale("log")
            ax.set_yscale("log")
            ax.grid(which="both")
            ax.plot(omega_list, S_sing_list[i, :], label="S", c="k", ls="-")
            ax.plot(omega_list, T_sing_list[i, :], label="T", c="k", ls="--")
            ax.plot(omega_list, np.full((num,), np.sqrt(2.0)), c="k", ls=":")
            ax.legend(loc="best")
        plt.show()

    def update_dynamics(self, Ac: np.ndarray, Bc: np.ndarray) -> None:
        """
        状態方程式を更新する場合にMPC側で必要な処理を行う

        Parameters
        ----------
        Ac, Bc: np.ndarray
            新しい連続時間状態方程式

        Returns
        ----------
        None

        Note
        ----------
        - オブザーバは連続時間用でなければならないことに注意
        """

        self._observer.update_dynamics(Ac, Bc)
        self._update_internal_variables()

    def _update_internal_variables(self) -> None:
        # p.68
        Cz_A_pow_arr = make_mat_pow_arr(self.A, self._Hp, self.Cz)
        Cz_A_pow_cs_B = np.cumsum(Cz_A_pow_arr, axis=0) @ self.B
        self._Psi = self._calc_Psi(Cz_A_pow_arr)
        self._Upsilon = self._calc_Upsilon(Cz_A_pow_cs_B)
        self._Theta = self._calc_Theta(Cz_A_pow_cs_B)

        # (3.12), (3.43), (3.41)
        self._Eta = self._Theta.T @ self._Q @ self._Theta + self._R_plus_Sa
        self._Phi = 2.0 * self._Eta
        self._Omega = np.r_[self._F_gothic, self._Gamma @ self._Theta, self._W]

        # その他の事前に計算しておくもの
        self._minus_2_Theta_Q = -2 * self._Theta.T @ self._Q
        self._minus_Gamma_Psi = -self._Gamma @ self._Psi
        self._minus_Gamma_Upsilon = -self._Gamma @ self._Upsilon

    def _calc_decays(self, T_ref, ref_traj):
        """p.12,例題1.3を参考に減衰率を計算"""

        coin_times = self.Ts * np.arange(1, self._Hp + 1)
        if ref_traj == "const" or T_ref == 0.0:
            decays = np.zeros((self._Hp,))
        elif ref_traj == "exp":
            assert T_ref is not None
            decays = np.exp(-coin_times / T_ref)
        else:
            raise ValueError(f"unknown reference trajectory name: {ref_traj}")
        return decays.reshape(-1, 1)

    def _make_const_mat_from_range(self, range_, H):
        a = range_[:, 0]
        b = range_[:, 1]
        ab = np.r_[a, -b]
        E = np.identity(range_.shape[0])
        EE = np.r_[-E, E]

        is_valid = abs(ab) != np.inf
        ab_valid = ab[is_valid]
        EE_valid = EE[is_valid, :]

        const_mat = np.c_[EE_valid, ab_valid]
        return self._make_const_mat_from_mat(const_mat, H)

    def _make_const_mat_from_mat(self, mat, H):
        left = mat[:, :-1]
        right = mat[:, -1]
        res = np.c_[make_mat_diag(np.tile(left, (H, 1, 1))), np.tile(right, H)]
        return res

    def _make_weight_mat(self, values, Hw, Hp):
        assert values.ndim == 1
        assert 0 < Hw <= Hp

        # Hwより前のステップの重みを0にする
        dim = values.shape[0]
        mat_arr_tile = np.concatenate(
            [np.zeros((Hw - 1, dim, dim)), np.tile(np.diag(values), (Hp - Hw + 1, 1, 1))], axis=0
        )
        res = make_mat_diag(mat_arr_tile)

        return res

    def _calc_F_gothic(self, F):
        n_cond = F.shape[0]  # (3.35)の条件数
        F_tilda = [np.zeros((n_cond, self.u_dim))] + np.hsplit(F[:, :-1], self._Hu)
        F_cs = np.cumsum(F_tilda, axis=0)

        F_gothic = np.empty((n_cond, self.u_dim * self._Hu))
        for i in range(0, self._Hu):
            slice_c = slice(self.u_dim * i, self.u_dim * (i + 1))
            F_gothic[:, slice_c] = F_cs[self._Hu, :, :] - F_cs[i, :, :]

        return F_gothic

    def _calc_Sa(self, S):
        # S_sum_arr[i] := i * S
        S_sum_arr = [i * S for i in range(0, self._Hu + 1)]
        Sa = np.empty((self.u_dim * self._Hu, self.u_dim * self._Hu))
        for i in range(0, self._Hu):
            slice_r = slice(self.u_dim * i, self.u_dim * (i + 1))
            for j in range(0, self._Hu):
                slice_c = slice(self.u_dim * j, self.u_dim * (j + 1))
                Sa[slice_r, slice_c] = S_sum_arr[self._Hu - max(i, j)]
        return Sa

    def _calc_Psi(self, Cz_A_pow_arr):
        """(2.67)"""

        Psi = np.empty((self.z_dim * self._Hp, self.x_dim))
        for i in range(0, self._Hp):
            slice_r = slice(self.z_dim * i, self.z_dim * (i + 1))
            Psi[slice_r, :] = Cz_A_pow_arr[i + 1, :, :]
        return Psi

    def _calc_Upsilon(self, Cz_A_pow_cs_B):
        """(2.67)"""

        Upsilon = np.empty((self.z_dim * self._Hp, self.u_dim))
        for i in range(0, self._Hp):
            slice_r = slice(self.z_dim * i, self.z_dim * (i + 1))
            Upsilon[slice_r, :] = Cz_A_pow_cs_B[i, :, :]
        return Upsilon

    def _calc_Theta(self, Cz_A_pow_cs_B):
        """(2.67)"""

        Theta = np.zeros((self.z_dim * self._Hp, self.u_dim * self._Hu))
        for i in range(0, self._Hp):
            slice_r = slice(self.z_dim * i, self.z_dim * (i + 1))
            for j in range(0, self._Hu):
                if i < j:
                    continue
                slice_c = slice(self.u_dim * j, self.u_dim * (j + 1))
                Theta[slice_r, slice_c] = Cz_A_pow_cs_B[i - j, :, :]
        return Theta

    def _calc_phi(self, y, s, u_ref):
        """(3.11)より，(3.43)のphiを計算"""

        Sb = self._calc_Sb(u_ref)
        Epsilon = self._calc_Epsilon(y, s)
        phi = Sb + self._minus_2_Theta_Q @ Epsilon
        return phi

    def _calc_Sb(self, u_ref):
        u_diff = self._last_u - u_ref
        Sb = self._Sb_over_u_diff @ u_diff
        return Sb

    def _calc_Epsilon(self, y, s):
        """(3.6)"""

        Tau = self._calc_Tau(y, s)
        Epsilon = Tau - self._Psi @ self.x - self._Upsilon @ self._last_u
        return Epsilon

    def _calc_Tau(self, y, s):
        """p.12,例題1.3を参考にp.90のTauを計算"""

        err = s - self.z
        Tau = (s - self._decays * err).ravel()
        if self._tracking:
            Tau -= np.tile(y - self.y, self._Hp)
        return Tau

    def _calc_omega(self):
        """(3.41)の右辺を計算"""

        vec1 = -self._F_gothic_1 @ self._last_u - self._f
        vec2 = self._minus_Gamma_Psi @ self.x + self._minus_Gamma_Upsilon @ self._last_u - self._g
        omega = np.r_[vec1, vec2, self._w].ravel()
        return omega

    @property
    def Ts(self) -> float:
        return self._observer.Ts

    @property
    def x_dim(self) -> int:
        return self._observer.x_dim

    @property
    def u_dim(self) -> int:
        return self._observer.u_dim

    @property
    def y_dim(self) -> int:
        return self._observer.y_dim

    @property
    def z_dim(self) -> int:
        return self._observer.z_dim

    @property
    def x(self) -> np.ndarray:
        return self._observer.x

    @property
    def y(self) -> np.ndarray:
        return self._observer.y

    @property
    def z(self) -> np.ndarray:
        return self._observer.z

    @property
    def A(self) -> np.ndarray:
        return self._observer.A

    @property
    def B(self) -> np.ndarray:
        return self._observer.B

    @property
    def Cy(self) -> np.ndarray:
        return self._observer.Cy

    @property
    def Cz(self) -> np.ndarray:
        return self._observer.Cz
