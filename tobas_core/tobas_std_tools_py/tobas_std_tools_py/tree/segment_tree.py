class SegTree:
    def __init__(self, init_val, segfunc, ide_ele):
        """
        Parameters
        ----------
        init_val: List[T]
            配列の初期値
        segfunc: Callable
            区間にしたい操作((x, y) -> x or y)
        ide_ele: T
            単位元

        Returns
        ----------
        None
        """

        n = len(init_val)
        self.segfunc = segfunc
        self.ide_ele = ide_ele
        self.num = 1 << (n - 1).bit_length()
        self.tree = [ide_ele] * 2 * self.num
        # 配列の値を葉にセット
        for i in range(n):
            self.tree[self.num + i] = init_val[i]
        # 構築していく
        for i in range(self.num - 1, 0, -1):
            self.tree[i] = self.segfunc(self.tree[2 * i], self.tree[2 * i + 1])

    def update(self, k, x):
        """
        k番目の値をxに更新する

        Parameters
        ----------
        k: int
            インデックス
        x: T
            更新値

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
        [left, right)のsegfuncしたものを得る

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
