from enum import Enum
from dataclasses import dataclass
from typing import Tuple, List


class TurningDirection(Enum):
    CCW = 1
    CW = -1


class RotorAxis(Enum):
    X_POSITIVE = 0
    Z_POSITIVE = 1
    UNKNOWN = 2  # TODO


class ESCMode(Enum):
    BLHELI_OPEN_LOOP = 0
    BLHELI_CLOSED_LOOP_LOW_RANGE = 1
    BLHELI_CLOSED_LOOP_MID_RANGE = 2
    BLHELI_CLOSED_LOOP_HIGH_RANGE = 3


@dataclass
class RotorConfig:
    link_name: str = ""  # プロペラのリンク名
    direction: TurningDirection = TurningDirection.CCW  # 回転方向: CCW(1) or CW(-1)
    axis: RotorAxis = RotorAxis.UNKNOWN  # 回転軸
    esc_mode: ESCMode = ESCMode.BLHELI_OPEN_LOOP  # ESCのスロットルの解釈方式
    num_poles: int = 0  # モータの極数
    max_rot_speed: float = 0.0  # 最大連続回転数 [rad/s]
    motor_constant: float = 0.0  # 推力係数 [kg*m/rad^2]
    moment_constant: float = 0.0  # 反トルク係数 [m]
    drag_constant: float = 0.0  # 空気効力定数 [kg/rad]
    rot_speed_coefs: Tuple[float, float] = (
        0.0,
        0.0,
    )  # V = c1 w + c2 w^2 (V[V], w[rad/s])
    channel: int = 0  # モータが接続されているRC出力チャンネル


RotorConfigs = List[RotorConfig]
