import numpy as np
from typing import Tuple

from tobas_kdl_msgs.msg import Vector, JntArray, JntArrayVel, JntArrayAcc


def vectorNpToMsg(k: np.ndarray) -> Vector:
    return Vector(x=k[0], y=k[1], z=k[2])


def vectorMsgToNp(m: Vector) -> np.ndarray:
    return np.array([m.x, m.y, m.z])


def jntArrayNpToMsg(k: np.ndarray) -> JntArray:
    return JntArray(data=k.tolist())


def jntArrayMsgToNp(m: JntArray) -> np.ndarray:
    return np.array(m.data)


def jntArrayVelNpToMsg(q: np.ndarray, qdot: np.ndarray) -> JntArrayVel:
    return JntArrayVel(q=jntArrayNpToMsg(q), qdot=jntArrayNpToMsg(qdot))


def jntArrayVelMsgToNp(m: JntArrayVel) -> Tuple[np.ndarray, np.ndarray]:
    return jntArrayMsgToNp(m.q), jntArrayMsgToNp(m.qdot)


def jntArrayAccNpToMsg(q: np.ndarray, qdot: np.ndarray, qdotdot: np.ndarray) -> JntArrayAcc:
    return JntArrayVel(q=jntArrayNpToMsg(q), qdot=jntArrayNpToMsg(qdot), qdotdot=jntArrayNpToMsg(qdotdot))


def jntArrayAccMsgToNp(m: JntArrayAcc) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    return (jntArrayMsgToNp(m.q), jntArrayMsgToNp(m.qdot), jntArrayMsgToNp(m.qdotdot))
