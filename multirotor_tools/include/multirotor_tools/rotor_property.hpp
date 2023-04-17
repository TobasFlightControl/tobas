#pragma once

#include <string>
#include <vector>

enum struct ESCType
{
  PWM,
  DSHOT,
};

struct ESCConfig_PWM
{
  double frequency;        // データ転送の周波数 [Hz]
  double min_pulse_width;  // パルス幅の最小値 [us]
  double max_pulse_width;  // パルス幅の最小値 [us]
};

struct ESCConfig_DSHOT
{
  // TODO
};

struct RotorConfig
{
  std::string link_name;   // プロペラのリンク名
  int direction;           // CCW(1) or CW(-1)
  double motor_constant;   // 推力係数 [N*s^2/rad^2]
  double moment_constant;  // 反トルク係数 [m]
  double kv;               // 効率を考慮した1ボルトあたりの回転数 [rpm/V]
  int pin;                 // モータが接続されているピン番号
  ESCType esc_type;
  ESCConfig_PWM pwm;
  ESCConfig_DSHOT dshot;
};

using RotorConfigs = std::vector<RotorConfig>;

RotorConfigs getRotorConfigs();
