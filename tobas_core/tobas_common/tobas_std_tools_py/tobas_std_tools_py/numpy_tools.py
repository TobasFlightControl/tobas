# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

import numpy as np
import numpy.linalg as LA


def calc_diff(a: np.ndarray) -> np.ndarray:
    """
    Compute the finite difference of `a`.\\
    The 0-th element is left unchanged.
    """

    return np.r_[a[0], a[1:] - np.roll(a, 1)[1:]]


def make_mat_pow_arr(A: np.ndarray, n: int, A0: np.ndarray = None) -> np.ndarray:
    """
    Return an array from the 0-th power to the n-th power of `A`.\\
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
    Create a diagonal-matrix-like matrix whose diagonal elements are the matrices in `A_arr`.\\
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
    """Create an (n, n) block Hankel matrix whose elements are `A_arr[i, :, :]`."""

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
    """Create an (n, n) block Hankel matrix whose elements are `elements[i]`."""

    return make_block_hankel(A_arr=elements.reshape(-1, 1, 1), n=n)


def make_cross_mat_3d(v: np.ndarray) -> np.ndarray:
    """Create the cross-product matrix of a 3D vector."""

    assert v.shape == (3,)

    x, y, z = v
    res = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])

    return res


def is_symmetric(A: np.ndarray) -> bool:
    """Check whether `A` is a symmetric matrix."""

    assert A.ndim == 2 and A.shape[0] == A.shape[1]
    return np.allclose(A, A.T)


def is_positive(A: np.ndarray) -> bool:
    """Check whether `A` is a positive-definite matrix."""

    assert A.ndim == 2 and A.shape[0] == A.shape[1]
    return np.all(LA.eigvals(A) > 0.0)


def is_semipositive(A: np.ndarray) -> bool:
    """Check whether `A` is a positive-semidefinite matrix."""

    assert A.ndim == 2 and A.shape[0] == A.shape[1]
    return np.all(LA.eigvals(A) >= 0.0)


def normalize(v: np.ndarray, axis: int = -1, ord: int = 2) -> np.ndarray:
    """Normalize a vector."""

    l2 = LA.norm(v, ord=ord, axis=axis, keepdims=True)
    l2[l2 == 0] = 1.0
    return v / l2


def calc_euclid_full_pairs(p1: np.ndarray, p2: np.ndarray) -> np.ndarray:
    """Compute Euclidean distances between all point pairs in N-dimensional space."""

    assert p1.ndim == p2.ndim == 2
    assert p1.shape[1] == p2.shape[1]

    res = np.sqrt(((p1[:, :, np.newaxis] - p2.T[np.newaxis, :, :]) ** 2).sum(axis=1))
    return res
