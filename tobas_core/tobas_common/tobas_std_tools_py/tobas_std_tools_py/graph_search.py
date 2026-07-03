# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

from copy import deepcopy
from heapq import heappop, heappush


def warshall_floyd(dist: list) -> list:
    """
    Warshall-Floyd algorithm.

    Parameter
    ----------
    dist: list
        Direct distance table.
        Entries without a path are `INF`.

    Return
    ----------
    res: list
        Table of shortest distances.

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
    Dijkstra's algorithm.

    Parameter
    ----------
    graph: list
        Vertex adjacency list.
    s: int
        Start vertex index.

    Return:
    ----------
    dist: list
        List of shortest distances from `s`.
        Unreachable nodes are set to -1.
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
