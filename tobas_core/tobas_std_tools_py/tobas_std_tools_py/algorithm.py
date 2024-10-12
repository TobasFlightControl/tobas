from typing import List


def cumsum(data: List) -> List:
    """
    累積和を計算する．\\
    res[i] = dataの0番目からi番目の要素までの和
    """
    if len(data) == 0:
        return []

    res = [0] * len(data)
    res[0] = data[0]

    for i in range(1, len(data)):
        res[i] = res[i - 1] + data[i]

    return res
