#pragma once

#include <string>
#include <vector>

namespace tobas
{
enum struct Axis
{
  X_POSITIVE,
  Z_POSITIVE,
  UNKNOWN,  // TODO
};

enum struct EscSignalMode
{
  BLHELI_OPEN_LOOP,
  BLHELI_CLOSED_LOOP_LOW_RANGE,
  BLHELI_CLOSED_LOOP_MID_RANGE,
  BLHELI_CLOSED_LOOP_HIGH_RANGE,
};

struct RotorConfig
{
  std::string link_name;                      // プロペラのリンク名
  int direction;                              // 回転方向: CCW(1) or CW(-1)
  Axis axis;                                  // 回転軸
  EscSignalMode esc_signal_mode;              // ESCへの信号の解釈方式
  size_t num_poles;                           // モータの極数
  double max_rot_speed;                       // 最大連続回転数 [rad/s]
  double motor_constant;                      // 推力係数 [kg*m/rad^2]
  double moment_constant;                     // 反トルク係数 [m]
  double drag_constant;                       // 空気効力定数 [kg/rad]
  std::pair<double, double> rot_speed_coefs;  // V = c1 w + c2 w^2 (V[V], w[rad/s])
  size_t channel;                             // モータが接続されているPWMチャンネル
};

using RotorConfigs = std::vector<RotorConfig>;
}  // namespace tobas
