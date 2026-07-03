# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

import numpy as np

from .basic import BasicObserverCont


class DMCObserver(BasicObserverCont):
    """
    DMC state observer.
    - Consider the model on p. 69 in continuous space.
    - Assume a constant output disturbance.
    - Assume `z = Cz @ x`.
    - Deadbeat estimation.
    """

    def __init__(
        self,
        Ac: np.ndarray,
        Bc: np.ndarray,
        Cy: np.ndarray,
        Cz: np.ndarray,
        Ts: float,
        init_x: np.ndarray = None,
    ) -> None:
        """
        Parameters
        ----------
        Ac, Bc, Cy, Cz: np.ndarray
            Coefficient matrices of the continuous-time state equation.
        Ts: float
            Sampling time.
        init_x: np.ndarray, default None
            Initial state.

        Returns
        ----------
        None

        Note
        ----------
        - Note that the number of states changes from `x_dim` to `x_dim + y_dim`.
        """

        u_dim = Bc.shape[1]
        y_dim = Cy.shape[0]
        z_dim = Cz.shape[0]
        x_dim_base = Ac.shape[0]
        x_dim = x_dim_base + y_dim

        assert isinstance(Ac, np.ndarray)
        assert isinstance(Bc, np.ndarray)
        assert isinstance(Cy, np.ndarray)
        assert isinstance(Cz, np.ndarray)
        assert Ac.shape == (x_dim_base, x_dim_base)
        assert Bc.shape == (x_dim_base, u_dim)
        assert Cy.shape == (y_dim, x_dim_base)
        assert Cz.shape == (z_dim, x_dim_base)
        assert Ts > 0.0
        assert init_x is None or init_x.shape == (x_dim_base,)

        Ac_tilda = np.r_[np.c_[Ac, np.zeros((x_dim_base, y_dim))], np.zeros((y_dim, x_dim))]
        Bc_tilda = np.r_[Bc, np.zeros((y_dim, u_dim))]
        Cy_tilda = np.c_[Cy, np.identity(y_dim)]
        Cz_tilda = np.c_[Cz, np.zeros((z_dim, y_dim))]
        L = np.r_[np.zeros((x_dim_base, y_dim)), np.identity(y_dim)]
        if init_x is None:
            init_x_tilda = np.zeros((x_dim,))
        else:
            init_x_tilda = np.r_[init_x, np.zeros((y_dim,))]

        super().__init__(
            Ac=Ac_tilda,
            Bc=Bc_tilda,
            Cy=Cy_tilda,
            Cz=Cz_tilda,
            Ts=Ts,
            L=L,
            init_x=init_x_tilda,
        )
