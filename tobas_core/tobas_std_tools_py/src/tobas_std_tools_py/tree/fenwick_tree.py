class FenwickTree:

    def __init__(self, n):
        self._n = n
        self.data = [0] * n

    def add(self, p, x):
        """
        pの位置にxを足す
        """
        assert 0 <= p < self._n
        p += 1
        while p <= self._n:
            self.data[p - 1] += x
            p += p & -p

    def sum(self, left, right):
        """
        [left, right)の和を求める
        """
        assert 0 <= left <= right <= self._n
        return self._sum(right) - self._sum(left)

    def _sum(self, right):
        s = 0
        while right > 0:
            s += self.data[right - 1]
            right -= right & -right
        return s


if __name__ == "__main__":
    bit = FenwickTree(10)
    bit.add(0, 3)
    bit.add(5, 5)
    print(bit.sum(0, 1), bit.sum(0, 10), bit.sum(0, 5))
