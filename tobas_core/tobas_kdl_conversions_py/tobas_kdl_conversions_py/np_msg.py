import numpy as np
from numpy.typing import NDArray
from typing import Tuple

from tobas_kdl_msgs.msg import Vector, JntArray, JntArrayVel, JntArrayAcc


def vectorNpToMsg(k: NDArray) -> Vector:
    return Vector(x=k[0], y=k[1], z=k[2])


def vectorMsgToNp(m: Vector) -> NDArray:
    return np.array([m.x, m.y, m.z])


def jntArrayNpToMsg(k: NDArray) -> JntArray:
    return JntArray(data=k.tolist())


def jntArrayMsgToNp(m: JntArray) -> NDArray:
    return np.array(m.data)


def jntArrayVelNpToMsg(q: NDArray, qdot: NDArray) -> JntArrayVel:
    return JntArrayVel(q=jntArrayNpToMsg(q), qdot=jntArrayNpToMsg(qdot))


def jntArrayVelMsgToNp(m: JntArrayVel) -> Tuple[NDArray, NDArray]:
    return jntArrayMsgToNp(m.q), jntArrayMsgToNp(m.qdot)


def jntArrayAccNpToMsg(q: NDArray, qdot: NDArray, qdotdot: NDArray) -> JntArrayAcc:
    return JntArrayVel(
        q=jntArrayNpToMsg(q),
        qdot=jntArrayNpToMsg(qdot),
        qdotdot=jntArrayNpToMsg(qdotdot),
    )


def jntArrayAccMsgToNp(m: JntArrayAcc) -> Tuple[NDArray, NDArray, NDArray]:
    return (jntArrayMsgToNp(m.q), jntArrayMsgToNp(m.qdot), jntArrayMsgToNp(m.qdotdot))
