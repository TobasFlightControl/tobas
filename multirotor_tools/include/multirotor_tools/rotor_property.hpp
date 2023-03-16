#pragma once

#include <string>
#include <vector>

#include <dh_std_tools/struct.hpp>

struct RotorProperty
{
  std::string link_name;            // プロペラのリンク名
  int direction;                    // CCW(1) or CW(-1)
  double max_velocity;              // 最大回転速度 [rad/s]
  double motor_constant;            // 推力係数 [N*s^2/rad^2]
  double moment_constant;           // 反トルク係数 [m]
  int pin;                          // モータが接続されているピン番号
  dh_std::Range<double> pwm_range;  // 回転速度の端に対応する，パルス周期の範囲 [us]
};

using RotorProperties = std::vector<RotorProperty>;

RotorProperties getRotorProperties();
