#pragma once

#include <string>
#include <vector>

struct RotorProperty
{
  std::string link_name;   // プロペラのリンク名
  int direction;           // CCW(1) or CW(-1)
  double motor_constant;   // 推力係数 [N*s^2/rad^2]
  double moment_constant;  // 反トルク係数 [m]
  double kv;               // 効率を考慮した1ボルトあたりの回転数 [rpm/V]
  int pin;                 // モータが接続されているピン番号
};

using RotorProperties = std::vector<RotorProperty>;

RotorProperties getRotorProperties();
