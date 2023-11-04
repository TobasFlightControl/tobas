#pragma once

#include <string>
#include <vector>

#include <dh_std_tools/range.hpp>

namespace tobas
{
enum struct Axis
{
  X_POSITIVE,
  Z_POSITIVE,
  UNKNOWN,  // TODO
};

enum struct ESCType
{
  PWM,
  DSHOT,
};

struct RotorConfig
{
  std::string link_name;                      // プロペラのリンク名
  Axis axis;                                  // 回転軸
  int direction;                              // 回転方向: CCW(1) or CW(-1)
  double motor_constant;                      // 推力係数 [kg*m/rad^2]
  double moment_constant;                     // 反トルク係数 [m]
  double drag_constant;                       // 空気効力定数 [kg/rad]
  std::pair<double, double> rot_speed_coefs;  // V = c1 w + c2 w^2 (V[V], w[rad/s])
  uint32_t pin;                               // モータが接続されているピン番号
  ESCType esc_type;
};

using RotorConfigs = std::vector<RotorConfig>;
}  // namespace tobas
