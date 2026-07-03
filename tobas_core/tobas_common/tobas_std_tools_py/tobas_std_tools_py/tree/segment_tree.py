# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.


class SegTree:
    def __init__(self, init_val, segfunc, ide_ele):
        """
        Parameters
        ----------
        init_val: List[T]
            Initial array values.
        segfunc: Callable
            Operation to apply over intervals, such as `(x, y) -> x or y`.
        ide_ele: T
            Identity element.

        Returns
        ----------
        None
        """

        n = len(init_val)
        self.segfunc = segfunc
        self.ide_ele = ide_ele
        self.num = 1 << (n - 1).bit_length()
        self.tree = [ide_ele] * 2 * self.num
        # Set array values to the leaves.
        for i in range(n):
            self.tree[self.num + i] = init_val[i]
        # Build the tree.
        for i in range(self.num - 1, 0, -1):
            self.tree[i] = self.segfunc(self.tree[2 * i], self.tree[2 * i + 1])

    def update(self, k, x):
        """
        Update the `k`-th value to `x`.

        Parameters
        ----------
        k: int
            Index.
        x: T
            Updated value.

        Returns
        ----------
        None
        """

        k += self.num
        self.tree[k] = x
        while k > 1:
            self.tree[k >> 1] = self.segfunc(self.tree[k], self.tree[k ^ 1])
            k >>= 1

    def query(self, left, right):
        """
        Return `segfunc` applied over `[left, right)`.

        Parameters
        ----------
        left: int
        right: int

        Returns
        ----------
        res: T
        """

        res = self.ide_ele

        left += self.num
        right += self.num
        while left < right:
            if left & 1:
                res = self.segfunc(res, self.tree[left])
                left += 1
            if right & 1:
                res = self.segfunc(res, self.tree[right - 1])
            left >>= 1
            right >>= 1
        return res
