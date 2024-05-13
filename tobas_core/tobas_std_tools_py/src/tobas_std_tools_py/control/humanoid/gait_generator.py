import math
import numpy as np
from typing import Tuple, List


class CoMSubTrajectory:

    def __init__(
        self,
        Tc: float,
        fp: np.ndarray,
        p_init: np.ndarray,
        v_init: np.ndarray,
    ) -> None:
        assert Tc > 0.0
        assert fp.shape == p_init.shape == v_init.shape == (2,)

        self._Tc = Tc
        self._fp = np.array(fp)
        self._p_init = np.array(p_init)
        self._v_init = np.array(v_init)

    def __call__(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        assert t >= 0.0, t

        cosh = math.cosh(t / self._Tc)
        sinh = math.sinh(t / self._Tc)

        p = (self._p_init - self._fp) * cosh + self._Tc * self._v_init * sinh + self._fp
        v = (self._p_init - self._fp) / self._Tc * sinh + self._v_init * cosh

        return p, v


class CoMTrajectory:

    def __init__(self, sub_trajs: List[CoMSubTrajectory], T_sup: float) -> None:
        self._n = len(sub_trajs)
        self._sub_trajs = sub_trajs
        self._T_sup = T_sup

    def __call__(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        assert 0.0 <= t <= self._T_sup * self._n, f"t: {t}, T_sup: {self._T_sup}, n: {self._n}"

        idx = min(math.floor(t / self._T_sup), self._n - 1)
        t_ = t - self._T_sup * idx
        return self._sub_trajs[idx](t_)


def lip3d(
    s: List[Tuple[float, float, float]],
    T_sup: float,
    zc: float,
    fp0: Tuple[float, float],
    p0: Tuple[float, float],
    v0: Tuple[float, float],
    r: float = 10.0,
    grav: float = 9.80665,
) -> Tuple[CoMTrajectory, np.ndarray, np.ndarray]:
    """ヒューマノイドロボット, p.156, 図4.23"""

    assert len(fp0) == 2
    assert len(p0) == 2
    assert len(v0) == 2
    assert zc > 0.0
    assert r > 0.0
    assert grav > 0.0

    n = len(s)
    s_ = s + [(0.0, 0.0, 0.0)]
    Tc = math.sqrt(zc / grav)
    cosh = math.cosh(T_sup / Tc)
    sinh = math.sinh(T_sup / Tc)
    d = r * (cosh - 1.0) ** 2 + (sinh / Tc) ** 2

    fp_des = np.array(fp0)
    p_init = np.array(p0)
    v_init = np.array(v0)
    fp_des_list = [np.array(fp0)]  # 足場の目標値
    fp_list = [np.array(fp0)]  # 足場の修正値
    sub_traj_list = []

    # 最初の歩行素片(自由運動)
    sub_traj = CoMSubTrajectory(Tc, fp_des, p_init, v_init)
    p_init, v_init = sub_traj(T_sup)
    sub_traj_list.append(sub_traj)

    # ユーザが与えた歩行データに従って歩行素片を作成
    for i in range(n):
        sign = 1.0 if i % 2 == 0 else -1.0

        sx_cur, sy_cur, theta_cur = s_[i]
        trans_cur = np.array([sx_cur, sign * sy_cur])
        rot_cur = np.array(
            [
                [math.cos(theta_cur), -math.sin(theta_cur)],
                [math.sin(theta_cur), math.cos(theta_cur)],
            ]
        )
        sx_next, sy_next, theta_next = s_[i + 1]
        trans_next = np.array([sx_next, -sign * sy_next])
        rot_next = np.array(
            [
                [math.cos(theta_next), -math.sin(theta_next)],
                [math.sin(theta_next), math.cos(theta_next)],
            ]
        )

        # (4.49)
        fp_des += rot_cur @ trans_cur
        fp_des_list.append(fp_des.copy())

        # (4.50), (4.51)
        p_des_part = rot_next @ (trans_next / 2.0)
        v_des_part = rot_next @ (np.array([cosh + 1.0, cosh - 1.0]) * (trans_next / 2.0)) / (Tc * sinh)

        # (4.46)
        p_des_world = fp_des + p_des_part
        v_des_world = v_des_part

        # (4.48)
        fp1 = -r * (cosh - 1.0) / d * (p_des_world - cosh * p_init - Tc * sinh * v_init)
        fp2 = -sinh / (Tc * d) * (v_des_world - (sinh / Tc) * p_init - cosh * v_init)
        fp = fp1 + fp2
        fp_list.append(fp)

        sub_traj = CoMSubTrajectory(Tc, fp, p_init, v_init)
        p_init, v_init = sub_traj(T_sup)
        sub_traj_list.append(sub_traj)

    return CoMTrajectory(sub_traj_list, T_sup), np.array(fp_des_list), np.array(fp_list)
