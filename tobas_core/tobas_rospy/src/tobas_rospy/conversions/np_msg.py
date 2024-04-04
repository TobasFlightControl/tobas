import numpy as np
from geometry_msgs.msg import Vector3


def vector3_msg_to_np(m: Vector3) -> np.ndarray:
    return np.array([m.x, m.y, m.z])


def vector3_np_to_msg(n: np.ndarray) -> Vector3:
    assert n.shape == (3,)
    return Vector3(x=n[0], y=n[1], z=n[2])
