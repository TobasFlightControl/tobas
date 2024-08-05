#pragma once

#include <cinttypes>
#include <string>
#include <vector>

#include "./turning_direction.hpp"
#include "./rotor_axis.hpp"
#include "./esc.hpp"

namespace tobas
{
struct RotorConfig
{
  std::string link_name;                      // プロペラのリンク名
  TurningDirection direction;                 // 回転方向: CCW(1) or CW(-1)
  RotorAxis axis;                             // 回転軸
  EscMode esc_mode;                           // ESCのスロットルの解釈方式
  uint8_t num_poles;                          // モータの極数
  double max_rot_speed;                       // 最大連続回転数 [rad/s]
  double motor_constant;                      // 推力係数 [kg*m/rad^2]
  double moment_constant;                     // 反トルク係数 [m]
  double drag_constant;                       // 空気効力定数 [kg/rad]
  std::pair<double, double> rot_speed_coefs;  // V = c1 w + c2 w^2 (V[V], w[rad/s])
  uint8_t channel;                            // モータが接続されているチャンネル
};

using RotorConfigs = std::vector<RotorConfig>;
}  // namespace tobas
