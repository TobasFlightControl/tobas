#pragma once

#include <string>
#include <vector>

struct RotorProperty
{
  std::string link_name;   // プロペラのリンク名
  int direction;           // CCW(1) or CW(-1)
  double motor_constant;   // 推力係数 [N*s^2/rad^2]
  double moment_constant;  // 反トルク係数 [m]
  double kv;               // 1ボルトあたりの無負荷回転数 [rpm/V]
  double efficiency;       // 無負荷回転数に対する実際の回転数の割合 [-]
  int pin;                 // モータが接続されているピン番号
};

using RotorProperties = std::vector<RotorProperty>;

RotorProperties getRotorProperties();
