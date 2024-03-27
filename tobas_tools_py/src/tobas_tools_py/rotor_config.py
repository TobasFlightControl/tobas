from enum import Enum
from dataclasses import dataclass
from typing import Tuple, List


class Axis(Enum):
    X_POSITIVE = "x_positive"
    Z_POSITIVE = "z_positive"
    UNKNOWN = "unknown"  # TODO


class EscSignalMode(Enum):
    BLHELI_OPEN_LOOP = "blheli_open_loop"
    BLHELI_CLOSED_LOOP_LOW_RANGE = "blheli_closed_loop_low_range"
    BLHELI_CLOSED_LOOP_MID_RANGE = "blheli_closed_loop_mid_range"
    BLHELI_CLOSED_LOOP_HIGH_RANGE = "blheli_closed_loop_high_range"


@dataclass
class RotorConfig:
    link_name: str = ""  # プロペラのリンク名
    direction: int = 0  # 回転方向: CCW(1) or CW(-1)
    axis: Axis = Axis.UNKNOWN  # 回転軸
    esc_signal_mode: EscSignalMode = EscSignalMode.BLHELI_OPEN_LOOP  # ESCへの信号の解釈
    num_poles: int = 0  # モータの極数
    max_rot_speed: float = 0.0  # 最大連続回転数 [rad/s]
    motor_constant: float = 0.0  # 推力係数 [kg*m/rad^2]
    moment_constant: float = 0.0  # 反トルク係数 [m]
    drag_constant: float = 0.0  # 空気効力定数 [kg/rad]
    rot_speed_coefs: Tuple[float, float] = (0.0, 0.0)  # V = c1 w + c2 w^2 (V[V], w[rad/s])
    channel: int = 0  # モータが接続されているPWMチャンネル


RotorConfigs = List[RotorConfig]
