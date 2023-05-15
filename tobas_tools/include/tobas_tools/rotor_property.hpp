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

struct ESCConfig_PWM
{
  double frequency;                         // データ転送の周波数 [Hz]
  dh_std::Range<double> pulse_width_range;  // パルス幅の範囲 [us]
};

struct ESCConfig_DSHOT
{
  // TODO
};

struct RotorConfig
{
  std::string link_name;   // プロペラのリンク名
  Axis axis;               // 回転軸
  int direction;           // 回転方向: CCW(1) or CW(-1)
  double motor_constant;   // 推力係数 [N*s^2/rad^2]
  double moment_constant;  // 反トルク係数 [m]
  double kv;               // 効率を考慮した1ボルトあたりの回転数 [rpm/V]
  int pin;                 // モータが接続されているピン番号
  ESCType esc_type;
  ESCConfig_PWM pwm;
  ESCConfig_DSHOT dshot;
};

using RotorConfigs = std::vector<RotorConfig>;
}  // namespace tobas
