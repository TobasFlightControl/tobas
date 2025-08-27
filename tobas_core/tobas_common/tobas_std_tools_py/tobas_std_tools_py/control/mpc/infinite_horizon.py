import numpy as np
import control
from copy import deepcopy

from ..observer import BaseObserver
from ..mpc import BasicMPC


class InfiniteHorizonMPC(BasicMPC):  # TODO: 真面目にやるなら第2章の定式化からやり直す必要あり
    """
    無限ホライズンのモデル予測制御(p.215)
    - 安定なダイナミクスを要求
    - 設定値は0のみ
    """

    def __init__(
        self,
        observer: BaseObserver,
        Hu: int,
        Q_arr: np.ndarray,
        R_arr: np.ndarray,
        u_range: np.ndarray = None,
        u_rate_range: np.ndarray = None,
        u_const_mat: np.ndarray = None,
        u_rate_const_mat: np.ndarray = None,
        T_ref: float = 0.0,
        ref_traj: str = "const",
        show_progress: bool = False,
    ) -> None:
        assert Q_arr.shape[0] == 1  # ホライズンに渡って一定の重み行列を要求
        Q = Q_arr[0, :, :]

        # 一旦z = xとしたオブザーバを作成
        observer_2 = deepcopy(observer)
        Cz = observer_2.Cz
        observer_2._Cz = np.identity(observer_2.x_dim)
        observer_2._z_dim = observer_2.x_dim
        observer_2._z = observer_2.x.copy()

        # (6.19)
        Q_arr_2 = np.empty((Hu, observer_2.x_dim, observer_2.x_dim))
        for i in range(0, Q_arr_2.shape[0] - 1):
            Q_arr_2[i, :, :] = Cz.T @ Q @ Cz
        Q_bar = control.dlyap(observer.A.T, Cz.T @ Q_arr[0, :, :] @ Cz)
        Q_arr_2[-1, :, :] = Q_bar

        super().__init__(
            observer=observer_2,  # 変更点
            Hw=1,  # 変更点
            Hp=Hu,  # 変更点
            Hu=Hu,
            Q_arr=Q_arr_2,  # 変更点
            R_arr=R_arr,
            u_range=u_range,
            u_rate_range=u_rate_range,
            z_range=None,  # 変更点
            u_const_mat=u_const_mat,
            u_rate_const_mat=u_rate_const_mat,
            z_const_mat=None,  # 変更点
            T_ref=T_ref,
            ref_traj=ref_traj,
            tracking=False,  # 変更点
            show_progress=show_progress,
        )

    def step(self, y: np.ndarray) -> np.ndarray:
        """
        Parameters
        ----------
        y: np.ndarray
            現在のプラント出力

        Returns
        ----------
        u: np.ndarray
            制御入力

        Note
        ----------
        - 設定値は0で固定
        """

        s = np.zeros((self.z_dim))
        return super().step(y, s)
