# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

import numpy as np


def logistic_kernel(x: float) -> float:
    return 1.0 / (np.exp(x) + 2.0 + np.exp(-x))


def rbf_kernel(x: np.ndarray, y: np.ndarray, gamma: float) -> float:
    return np.exp(-gamma * np.sum((x - y) ** 2))
