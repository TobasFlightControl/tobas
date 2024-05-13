import numpy as np
from geometry_msgs.msg import Vector3, Quaternion


def vector3_msg_to_np(m: Vector3) -> np.ndarray:
    return np.array([m.x, m.y, m.z])


def vector3_np_to_msg(n: np.ndarray) -> Vector3:
    assert n.shape == (3,)
    return Vector3(x=n[0], y=n[1], z=n[2])


def quaternion_msg_to_np(m: Quaternion) -> np.ndarray:
    return np.array([m.x, m.y, m.z, m.w])


def quaternion_np_to_msg(n: np.ndarray) -> Quaternion:
    assert n.shape == (4,)
    return Quaternion(x=n[0], y=n[1], z=n[2], w=n[3])
