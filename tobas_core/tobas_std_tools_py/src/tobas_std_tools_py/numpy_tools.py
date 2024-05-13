import numpy as np
import numpy.linalg as LA


def calc_diff(a: np.ndarray) -> np.ndarray:
    """
    aの階差を計算する \\
    第0要素はそのまま
    """

    return np.r_[a[0], a[1:] - np.roll(a, 1)[1:]]


def make_mat_pow_arr(A: np.ndarray, n: int, A0: np.ndarray = None) -> np.ndarray:
    """
    Aの0乗からn乗までの配列を返す \\
    res[i] == A0 @ A**i
    """

    assert A.ndim == 2 and A.shape[0] == A.shape[1]
    assert n >= 0
    assert A0 is None or (A0.ndim == 2 and A0.shape[1] == A.shape[0])

    dim = A.shape[0]
    A_pow_list = []
    A_pow = A0 if type(A0) is np.ndarray else np.identity(dim)
    for _ in range(n + 1):
        A_pow_list.append(A_pow)
        A_pow = A_pow @ A
    return np.array(A_pow_list)


def make_mat_diag(A_arr: np.ndarray) -> np.ndarray:
    """
    A_arrの各要素行列を対角要素としてもつ対角行列もどきを作る \\
    [A[0], 0, 0, 0] \\
    [0, A[1], 0, 0] \\
    [0, 0, A[2], 0] \\
    [0, 0, 0, A[3]]
    """

    assert A_arr.ndim == 3

    n, dim_r, dim_c = A_arr.shape
    res = np.zeros((n * dim_r, n * dim_c))
    for i in range(0, n):
        slice_r = slice(dim_r * i, dim_r * (i + 1))
        slice_c = slice(dim_c * i, dim_c * (i + 1))
        res[slice_r, slice_c] = A_arr[i]

    return res


def make_block_hankel(A_arr: np.ndarray, n: int) -> np.ndarray:
    """A_arr[i, :, :]を要素とする(n, n)のブロックハンケル行列を作成する"""

    assert A_arr.ndim == 3
    assert n > 0

    N, row_dim, col_dim = A_arr.shape

    res = np.zeros((row_dim * n, col_dim * n))
    for i in range(0, n):
        for j in range(0, n):
            k = i + j
            if k >= N:
                continue
            slice_r = slice(row_dim * i, row_dim * (i + 1))
            slice_c = slice(col_dim * j, col_dim * (j + 1))
            res[slice_r, slice_c] = A_arr[k, :, :]
    return res


def make_hankel(elements: np.ndarray, n: int) -> np.ndarray:
    """elements[i]を要素とする(n, n)のブロックハンケル行列を作成する"""

    return make_block_hankel(A_arr=elements.reshape(-1, 1, 1), n=n)


def make_cross_mat_3d(v: np.ndarray) -> np.ndarray:
    """3次元ベクトルの外積行列を作成する"""

    assert v.shape == (3,)

    x, y, z = v
    res = np.array(
        [
            [0, -z, y],
            [z, 0, -x],
            [-y, x, 0],
        ]
    )

    return res


def is_symmetric(A: np.ndarray) -> bool:
    """Aが対称行列かどうかを調べる"""

    assert A.ndim == 2 and A.shape[0] == A.shape[1]
    return np.allclose(A, A.T)


def is_positive(A: np.ndarray) -> bool:
    """Aが正定値行列かどうかを調べる"""

    assert A.ndim == 2 and A.shape[0] == A.shape[1]
    return np.all(LA.eigvals(A) > 0.0)


def is_semipositive(A: np.ndarray) -> bool:
    """Aが準正定行列かどうかを調べる"""

    assert A.ndim == 2 and A.shape[0] == A.shape[1]
    return np.all(LA.eigvals(A) >= 0.0)


def normalize(v: np.ndarray, axis: int = -1, ord: int = 2) -> np.ndarray:
    """ベクトルを正規化する"""

    l2 = LA.norm(v, ord=ord, axis=axis, keepdims=True)
    l2[l2 == 0] = 1.0
    return v / l2


def calc_euclid_full_pairs(p1: np.ndarray, p2: np.ndarray) -> np.ndarray:
    """N次元空間における全点間ユークリッド距離の計算"""

    assert p1.ndim == p2.ndim == 2
    assert p1.shape[1] == p2.shape[1]

    res = np.sqrt(((p1[:, :, np.newaxis] - p2.T[np.newaxis, :, :]) ** 2).sum(axis=1))
    return res
