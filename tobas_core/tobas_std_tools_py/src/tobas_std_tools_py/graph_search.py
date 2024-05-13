from copy import deepcopy
from heapq import heappop, heappush


def warshall_floyd(dist: list) -> list:
    """
    ワーシャルフロイド法

    Parameter
    ----------
    dist: list
        直接距離テーブル(パスが無いところはINF)

    Return
    ----------
    res: list
        最短距離を記録したテーブル

    """
    res = deepcopy(dist)
    n = len(res)
    for k in range(0, n):
        for i in range(0, n):
            for j in range(0, n):
                res[i][j] = min(res[i][j], res[i][k] + res[k][j])
    return res


def dijkstra(graph: list, s: int) -> list:
    """
    ダイクストラ法

    Parameter
    ----------
    graph: list
        頂点連結リスト
    s: int
        始点のインデックス

    Return:
    ----------
    dist: list
        sからの各点への最短距離のリスト(到達不可能なノードには-1が入る)
    """
    dist = [-1] * len(graph)
    que = [(0, s)]
    while len(que):
        pd, pn = heappop(que)
        if dist[pn] != -1:
            continue
        dist[pn] = pd
        for cn, d in graph[pn]:
            if dist[cn] != -1:
                continue
            cd = pd + d
            heappush(que, (cd, cn))
    return dist
