import math
from typing import List, Tuple


def euler_from_matrix(data: List[float]) -> Tuple[float, float, float]:
    """回転行列からZYXオイラー角を求める．

    Parameters
    ----------
    data : List[float]
        回転行列のデータ．

    Returns
    -------
    Tuple[float, float, float]
        roll, pitch, yaw
    """
    EPSILON = 1e-12

    pitch = math.atan2(-data[6], math.sqrt(data[0] ** 2 + data[3] ** 2))
    if abs(pitch) > math.pi / 2 - EPSILON:
        yaw = math.atan2(-data[1], data[4])
        roll = 0.0
    else:
        roll = math.atan2(data[7], data[8])
        yaw = math.atan2(data[3], data[0])

    return roll, pitch, yaw
