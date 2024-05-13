import numpy as np
import numpy.linalg as LA
from typing import Tuple

from ...numpy_tools import make_mat_pow_arr, make_block_hankel


def step2impulse(S_arr: np.ndarray) -> np.ndarray:
    """MPC, 演習問題4.1"""

    assert S_arr.ndim == 3

    y_dim, u_dim = S_arr.shape[1:]
    H_arr = S_arr[1:, :, :] - np.roll(S_arr, 1, axis=0)[1:, :, :]
    H_arr = np.concatenate((np.zeros((1, y_dim, u_dim)), H_arr), axis=0)
    return H_arr


def ss2impulse(
    m: int,
    n: int,
    A: np.ndarray,
    B: np.ndarray,
    Cy: np.ndarray,
    Dy: np.ndarray = None,
) -> np.ndarray:
    """MPC, p.135"""

    x_dim = A.shape[0]
    u_dim = B.shape[1]
    y_dim = Cy.shape[0]

    if Dy is None:
        Dy = np.zeros((y_dim, u_dim))

    assert 0 <= m <= n
    assert A.ndim == B.ndim == Cy.ndim == Dy.ndim == 2
    assert A.shape == (x_dim, x_dim)
    assert B.shape == (x_dim, u_dim)
    assert Cy.shape == (y_dim, x_dim)
    assert Dy.shape == (y_dim, u_dim)

    C_A_pow_B_arr = make_mat_pow_arr(A=A, n=n, A0=Cy) @ B
    H_arr = [Dy.copy()]
    for i in range(0, n):
        H_arr.append(C_A_pow_B_arr[i])
    H_arr = np.asarray(H_arr)
    return H_arr[m:, :, :]


def ss2step(
    m: int,
    n: int,
    A: np.ndarray,
    B: np.ndarray,
    Cy: np.ndarray,
    Dy: np.ndarray = None,
) -> np.ndarray:
    """MPC, p.135"""

    H_arr = ss2impulse(0, n, A, B, Cy, Dy)
    S_arr = np.cumsum(H_arr, axis=0)
    return S_arr[m:, :, :]


def step2Upsilon(S_arr: np.ndarray, Hw: int, Hp: int) -> np.ndarray:  # TODO: 未テスト
    """MPC, p.136"""

    assert S_arr.ndim == 3 and S_arr.shape[0] > Hp
    assert 1 <= Hw <= Hp

    y_dim, u_dim = S_arr.shape[1:]
    Upsilon = S_arr[Hw : Hp + 1, :, :].reshape(y_dim * (Hp - Hw + 1), u_dim)
    return Upsilon


def step2Theta(S_arr: np.ndarray, Hw: int, Hp: int, Hu: int) -> np.ndarray:  # TODO: 未テスト
    """MPC, p.136"""

    assert S_arr.ndim == 3 and S_arr.shape[0] > Hp
    assert 1 <= Hw <= Hp
    assert 1 <= Hu <= Hp

    y_dim, u_dim = S_arr.shape[1:]
    Theta = np.zeros((y_dim * (Hp - Hw + 1), u_dim * Hu))
    for i in range(0, Hp - Hw + 1):
        for j in range(0, Hu):
            k = Hw + i - j
            if k < 0:
                continue
            slice_r = slice(y_dim * i, y_dim * (i + 1))
            slice_c = slice(u_dim * j, u_dim * (j + 1))
            Theta[slice_r, slice_c] = S_arr[k, :, :]
    return Theta


def step2ss(S_arr: np.ndarray, n: int = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """MPC, p.141"""

    assert S_arr.ndim == 3
    # 定常状態に至るまでのステップ応答が必要
    assert np.allclose(S_arr[-2, :, :], S_arr[-1, :, :])

    N, y_dim, u_dim = S_arr.shape
    N -= 1
    max_sing_num = N * min(y_dim, u_dim)

    if n is None:
        n = max_sing_num
    else:
        assert 1 <= n <= max_sing_num, f"n: {n}, max_sing_num: {max_sing_num}"

    H_arr = step2impulse(S_arr)
    Eta_N = make_block_hankel(H_arr[1:, :, :], N)  # (4.45)
    U, sing_vals, V_T = LA.svd(Eta_N, full_matrices=False)  # (4.46)
    Sigma_N = np.diag(sing_vals)

    Sigma_n = Sigma_N.copy()  # ここはコピーでなくてもよい
    for i in range(n, max_sing_num):
        Sigma_n[i, i] = 0.0

    I_n_tilda = np.r_[np.identity(n), np.zeros((max_sing_num - n, n))]
    Sigma_n_sqrt = np.sqrt(Sigma_n)
    Omega_n = U @ Sigma_n_sqrt @ I_n_tilda  # (4.51)
    Gamma_n = I_n_tilda.T @ Sigma_n_sqrt @ V_T  # (4.52)

    A = LA.lstsq(Omega_n[:-y_dim, :], Omega_n[y_dim:, :], rcond=None)[0]
    B = Gamma_n[:, :u_dim]
    C = Omega_n[:y_dim, :]

    # 得られた状態方程式で元のステップ応答が正確に再現できることを確認
    assert np.allclose(S_arr, ss2step(0, N, A, B, C))

    return A, B, C
