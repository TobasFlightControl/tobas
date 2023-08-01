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
  double motor_constant;                      // 推力係数 [N*s^2/rad^2]
  double moment_constant;                     // 反トルク係数 [m]
  std::pair<double, double> rot_speed_coefs;  // V = c1 w + c2 w^2 (V[V], w[rad/s])
  int pin;                                    // モータが接続されているピン番号
  ESCType esc_type;
};

using RotorConfigs = std::vector<RotorConfig>;
}  // namespace tobas
