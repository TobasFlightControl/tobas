# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

from typing import List


def cumsum(data: List) -> List:
    """
    Compute the cumulative sum.\\
    `res[i]` is the sum from element 0 to element i of `data`.
    """
    if len(data) == 0:
        return []

    res = [0] * len(data)
    res[0] = data[0]

    for i in range(1, len(data)):
        res[i] = res[i - 1] + data[i]

    return res
