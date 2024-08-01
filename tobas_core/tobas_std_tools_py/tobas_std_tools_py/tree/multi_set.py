import heapq


class MultiSet:
    """
    マルチセット: https://tsubo.hatenablog.jp/entry/2020/06/15/124657 \\
    Heapqから中間の値を削除できるようにしたものと考えてよさそう
    """

    def __init__(self):
        self.h = []
        self.d = dict()

    def insert(self, x):
        heapq.heappush(self.h, x)
        if x not in self.d:
            self.d[x] = 1
        else:
            self.d[x] += 1

    def erase(self, x):
        if x not in self.d or self.d[x] == 0:
            raise ValueError
        else:
            self.d[x] -= 1

        while len(self.h) != 0:
            if self.d[self.h[0]] == 0:
                heapq.heappop(self.h)
            else:
                break

    def exist(self, x):
        if x in self.d and self.d[x] != 0:
            return True
        else:
            return False

    def get_min(self):
        return self.h[0]


if __name__ == "__main__":
    mst = MultiSet()
    for i in range(0, 10):
        mst.insert(i)
    print(mst.get_min())
    mst.erase(0)
    print(mst.get_min())
