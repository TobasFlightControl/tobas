#pragma once

#include <cassert>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_constants/constants.hpp>

#include "../rotor.hpp"

namespace tobas
{
/* ESC + Motor + Propeller */
class ElectricRotorConfig : public RotorConfig
{
  using super = RotorConfig;

  static constexpr char kChannelKey[] = "channel";
  static constexpr char kNumPolesKey[] = "num_poles";
  static constexpr char kKvKey[] = "kv";
  static constexpr char kInternalResistanceKey[] = "internal_resistance";
  static constexpr char kPropellerDiameterKey[] = "propeller_diameter";
  static constexpr char kMotorConstKey[] = "motor_constant";

public:
  using SharedPtr = std::shared_ptr<ElectricRotorConfig>;
  using ConstSharedPtr = std::shared_ptr<const ElectricRotorConfig>;

  uint32_t channel = 0;             // モータが接続されているチャンネル
  uint32_t num_poles = 0;           // モータの極数
  double kv = 0.;                   // モータのKV値 [rad/s/V]
  double internal_resistance = 0.;  // モータの内部抵抗 [Ω]
  double propeller_diameter = 0.;   // プロペラの直径 [m]
  double motor_const = 0.;          // 推力係数 [kg*m/rad^2]

  bool isValid() const override;

  bool load(const YAML::Node& node) override;
  YAML::Node dump() const override;

  /* 回転数 [rad/s] から印加電圧 [V] を求める． */
  inline double voltageFromSpeed(double tar_speed) const;

  /* 印加電圧 [V] から回転数 [rad/s] を求める． */
  inline double speedFromVoltage(double voltage) const;

  /* 回転数 [rad/s] から推力 [N] を求める． */
  inline double thrustFromSpeed(double tar_speed) const;

  /* 推力 [N] から回転数 [rad/s] を求める． */
  inline double speedFromThrust(double thrust) const;

  /* 印加電圧から推力 [N] を求める． */
  inline double thrustFromVoltage(double voltage) const;

  /* 回転数 [rad/s] からスロットル [0,1] を求める． */
  inline double throttleFromSpeed(double tar_speed, double battery_voltage) const;

  /* 推力 [N] からスロットル [0,1] を求める． */
  inline double throttleFromThrust(double thrust, double battery_voltage) const;
};

inline double ElectricRotorConfig::voltageFromSpeed(double tar_speed) const
{
  assert(tar_speed >= 0);

  const auto b = internal_resistance * kv * moment_const * motor_const;
  const auto c = 1. / kv;
  return tar_speed * (b * tar_speed + c);
}

inline double ElectricRotorConfig::speedFromVoltage(double voltage) const
{
  assert(voltage >= 0);

  const auto b = internal_resistance * kv * moment_const * motor_const;
  const auto c = 1. / kv;
  return b > 0 ? (sqrt(math::sqr(c) + 4 * b * voltage) - c) / (2 * b) : voltage * kv;
}

inline double ElectricRotorConfig::thrustFromSpeed(double tar_speed) const
{
  return motor_const * math::sqr(tar_speed);
}

inline double ElectricRotorConfig::speedFromThrust(double thrust) const
{
  assert(thrust >= 0);
  return sqrt(thrust / motor_const);
}

inline double ElectricRotorConfig::thrustFromVoltage(double voltage) const
{
  assert(voltage > 0);

  const auto tar_speed = speedFromVoltage(voltage);
  return thrustFromSpeed(tar_speed);
}

inline double ElectricRotorConfig::throttleFromSpeed(double tar_speed, double battery_voltage) const
{
  assert(tar_speed >= 0);

  const auto voltage = voltageFromSpeed(tar_speed);
  return voltage / battery_voltage;
}

inline double ElectricRotorConfig::throttleFromThrust(double thrust, double battery_voltage) const
{
  assert(thrust >= 0);

  const auto tar_speed = speedFromThrust(thrust);
  return throttleFromSpeed(tar_speed, battery_voltage);
}
}  // namespace tobas
