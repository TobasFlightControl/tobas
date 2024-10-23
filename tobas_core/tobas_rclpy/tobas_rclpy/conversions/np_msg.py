import numpy as np
from numpy.typing import NDArray
from geometry_msgs.msg import Vector3, Quaternion


def vectorNpToMsg(n: NDArray) -> Vector3:
    assert n.shape == (3,)
    return Vector3(x=n[0], y=n[1], z=n[2])


def vectorMsgToNp(m: Vector3) -> NDArray:
    return np.array([m.x, m.y, m.z])


def quaternionNpToMsg(n: NDArray) -> Quaternion:
    assert n.shape == (4,)
    return Quaternion(x=n[0], y=n[1], z=n[2], w=n[3])


def quaternionMsgToNp(m: Quaternion) -> NDArray:
    return np.array([m.x, m.y, m.z, m.w])
