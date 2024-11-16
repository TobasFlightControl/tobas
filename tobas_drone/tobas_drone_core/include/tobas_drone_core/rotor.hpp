#pragma once

#include <cstdint>
#include <string>
#include <map>
#include <cassert>
#include <yaml-cpp/yaml.h>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_constants/constants.hpp>

#include "./turning_direction.hpp"
#include "./rotor_axis.hpp"

namespace tobas
{
class RotorConfig;
using RotorConfigMap = std::map<uint32_t, RotorConfig>;  // Channel -> RotorConfig

class RotorConfig
{
  static constexpr char kChannelKey[] = "channel";
  static constexpr char kLinkNameKey[] = "link_name";
  static constexpr char kDirectionKey[] = "direction";
  static constexpr char kAxisKey[] = "axis";
  static constexpr char kNumPolesKey[] = "num_poles";
  static constexpr char kMaxRotSpeedKey[] = "max_rot_speed";
  static constexpr char kKvKey[] = "kv";
  static constexpr char kInternalResistanceKey[] = "internal_resistance";
  static constexpr char kPropellerDiameterKey[] = "propeller_diameter";
  static constexpr char kMotorConstKey[] = "motor_constant";
  static constexpr char kMomentConstKey[] = "moment_constant";
  static constexpr char kDragConstKey[] = "drag_constant";
  static constexpr char kRotSpeedCoefKey[] = "rot_speed_coef";

public:
  uint32_t channel = 0;                                      // モータが接続されているチャンネル
  std::string link_name = "";                                // プロペラのリンク名
  turning_direction_t direction = turning_direction_t::CCW;  // 回転方向: CCW or CW
  rotor_axis_t axis = rotor_axis_t::UNKNOWN;                 // 回転軸
  uint32_t num_poles = 0;                                    // モータの極数
  double kv = 0;                                             // モータのKV値 [rad/s/V]
  double internal_resistance;                                // モータの内部抵抗 [Ω]
  double propeller_diameter;                                 // プロペラの直径 [m]
  double max_rot_speed = 0;                                  // 最大連続回転数 [rad/s]
  double motor_constant = 0;                                 // 推力係数 [kg*m/rad^2]
  double moment_constant = 0;                                // 反トルク係数 [m]
  double drag_constant = 0;                                  // 空気効力定数 [kg/rad]

  bool isValid() const;
  bool load(const YAML::Node& node);
  YAML::Node dump() const;

  /* CCW = 1, CW = -1 */
  inline int sign() const;

  /* 回転数 [rad/s] から印加電圧 [V] を求める． */
  inline double voltageFromRotSpeed(double tar_speed) const;

  /* 印加電圧 [V] から回転数 [rad/s] を求める． */
  inline double rotSpeedFromVoltage(double voltage) const;

  /* 回転数 [rad/s] から推力 [N] を求める． */
  inline double thrustFromRotSpeed(double tar_speed) const;

  /* 推力 [N] から回転数 [rad/s] を求める． */
  inline double rotSpeedFromThrust(double thrust) const;

  /* 印加電圧から推力 [N] を求める． */
  inline double thrustFromVoltage(double voltage) const;

  /* 回転数 [rad/s] からスロットル [0,1] を求める． */
  inline double throttleFromRotSpeed(double tar_speed, double battery_voltage) const;

  /* 推力 [N] からスロットル [0,1] を求める． */
  inline double throttleFromThrust(double thrust, double battery_voltage) const;

  /* 機械的に許容できる最大回転数から計算される推力． */
  inline double maxMechanicalThrust() const;

  /* 与えられたバッテリー電圧で出力できる最大推力．*/
  inline double maxThrust(double battery_voltage) const;

  /* 与えられたバッテリー電圧で出力できる最小推力． */
  inline double minThrust(double battery_voltage) const;
};

inline int RotorConfig::sign() const
{
  return tobas::sign(direction);
}

inline double RotorConfig::voltageFromRotSpeed(double tar_speed) const
{
  assert(tar_speed >= 0);

  const auto b = internal_resistance * kv * moment_constant * motor_constant;
  const auto c = 1. / kv;
  return tar_speed * (b * tar_speed + c);
}

inline double RotorConfig::rotSpeedFromVoltage(double voltage) const
{
  assert(voltage >= 0);

  const auto b = internal_resistance * kv * moment_constant * motor_constant;
  const auto c = 1. / kv;
  return b > 0 ? (sqrt(math::sqr(c) + 4 * b * voltage) - c) / (2 * b) : voltage * kv;
}

inline double RotorConfig::thrustFromRotSpeed(double tar_speed) const
{
  return motor_constant * math::sqr(tar_speed);
}

inline double RotorConfig::rotSpeedFromThrust(double thrust) const
{
  assert(thrust >= 0);
  return sqrt(thrust / motor_constant);
}

inline double RotorConfig::thrustFromVoltage(double voltage) const
{
  assert(voltage > 0);

  const auto tar_speed = rotSpeedFromVoltage(voltage);
  return thrustFromRotSpeed(tar_speed);
}

inline double RotorConfig::throttleFromRotSpeed(double tar_speed, double battery_voltage) const
{
  assert(tar_speed >= 0);

  const auto voltage = voltageFromRotSpeed(tar_speed);
  return voltage / battery_voltage;
}

inline double RotorConfig::throttleFromThrust(double thrust, double battery_voltage) const
{
  assert(thrust >= 0);

  const auto tar_speed = rotSpeedFromThrust(thrust);
  return throttleFromRotSpeed(tar_speed, battery_voltage);
}

inline double RotorConfig::maxMechanicalThrust() const
{
  return motor_constant * math::sqr(max_rot_speed);
}

inline double RotorConfig::maxThrust(double battery_voltage) const
{
  // 機械的な限界とエネルギー的な限界の最小値を計算
  return std::min(maxMechanicalThrust(), thrustFromVoltage(battery_voltage));
}

inline double RotorConfig::minThrust(double battery_voltage) const
{
  const auto min_voltage = battery_voltage * kArmThrot;
  return std::min(maxMechanicalThrust(), thrustFromVoltage(min_voltage));
}
}  // namespace tobas
