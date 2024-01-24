#pragma once

#include <string>
#include <vector>

#include <tobas_std_tools/range.hpp>

namespace tobas
{
enum struct Axis
{
  X_POSITIVE,
  Z_POSITIVE,
  UNKNOWN,  // TODO
};

struct RotorConfig
{
  std::string link_name;                      // プロペラのリンク名
  Axis axis;                                  // 回転軸
  int direction;                              // 回転方向: CCW(1) or CW(-1)
  double max_rot_speed;                       // 最大連続回転数 [rad/s]
  double motor_constant;                      // 推力係数 [kg*m/rad^2]
  double moment_constant;                     // 反トルク係数 [m]
  double drag_constant;                       // 空気効力定数 [kg/rad]
  std::pair<double, double> rot_speed_coefs;  // V = c1 w + c2 w^2 (V[V], w[rad/s])
  size_t pin;                                 // モータが接続されているピン番号
};

using RotorConfigs = std::vector<RotorConfig>;
}  // namespace tobas
