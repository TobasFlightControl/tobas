# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

import numpy as np
import control
from copy import deepcopy

from ..observer import BaseObserver
from ..mpc import BasicMPC


class InfiniteHorizonMPC(BasicMPC):  # TODO: Rebuild from the Chapter 2 formulation for a rigorous version.
    """
    Infinite-horizon model predictive control (p. 215).
    - Requires stable dynamics.
    - Supports only a zero setpoint.
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
        assert Q_arr.shape[0] == 1  # Requires a weight matrix that is constant over the horizon.
        Q = Q_arr[0, :, :]

        # Temporarily create an observer with `z = x`.
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
            observer=observer_2,  # Changed point.
            Hw=1,  # Changed point.
            Hp=Hu,  # Changed point.
            Hu=Hu,
            Q_arr=Q_arr_2,  # Changed point.
            R_arr=R_arr,
            u_range=u_range,
            u_rate_range=u_rate_range,
            z_range=None,  # Changed point.
            u_const_mat=u_const_mat,
            u_rate_const_mat=u_rate_const_mat,
            z_const_mat=None,  # Changed point.
            T_ref=T_ref,
            ref_traj=ref_traj,
            tracking=False,  # Changed point.
            show_progress=show_progress,
        )

    def step(self, y: np.ndarray) -> np.ndarray:
        """
        Parameters
        ----------
        y: np.ndarray
            Current plant output.

        Returns
        ----------
        u: np.ndarray
            Control input.

        Note
        ----------
        - The setpoint is fixed to zero.
        """

        s = np.zeros((self.z_dim))
        return super().step(y, s)
