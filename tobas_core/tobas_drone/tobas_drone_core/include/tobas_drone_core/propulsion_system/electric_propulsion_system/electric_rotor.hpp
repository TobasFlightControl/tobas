// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cassert>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

#include "../rotor.hpp"

namespace tobas
{
/* ESC + Motor + Propeller */
class ElectricRotorConfig : public RotorConfig
{
  using super = RotorConfig;

public:
  using SharedPtr = std::shared_ptr<ElectricRotorConfig>;
  using ConstSharedPtr = std::shared_ptr<const ElectricRotorConfig>;

  uint32_t channel = 0;              // Channel to which the motor is connected.
  uint32_t num_poles = 0;            // Number of motor poles.
  double kv = 0.0;                   // Motor KV value [rad/s/V].
  double internal_resistance = 0.0;  // Motor internal resistance [Ω].
  double min_speed = 0.0;            // Minimum motor speed [rad/s].
  double propeller_diameter = 0.0;   // Propeller diameter [m].
  double motor_const = 0.0;          // Thrust coefficient [kg*m/rad^2].
  double moment_const = 0.0;         // Reaction torque coefficient [m].

  bool isValid() const override;

  bool load(const YAML::Node& node) override;
  YAML::Node dump() const override;

  inline double momentConst() const override;
  inline double effortWeight() const override;

  /* Compute applied voltage [V] from rotational speed [rad/s]. */
  inline double voltageFromSpeed(double tar_speed) const;

  /* Compute rotational speed [rad/s] from applied voltage [V]. */
  inline double speedFromVoltage(double voltage) const;

  /* Compute thrust [N] from rotational speed [rad/s]. */
  inline double thrustFromSpeed(double tar_speed) const;

  /* Compute rotational speed [rad/s] from thrust [N]. */
  inline double speedFromThrust(double thrust) const;

  /* Compute thrust [N] from applied voltage. */
  inline double thrustFromVoltage(double voltage) const;

  /* Compute throttle [0,1] from rotational speed [rad/s]. */
  inline double throttleFromSpeed(double tar_speed, double battery_voltage) const;

  /* Compute throttle [0,1] from thrust [N]. */
  inline double throttleFromThrust(double thrust, double battery_voltage) const;
};

inline double ElectricRotorConfig::momentConst() const
{
  return moment_const;
}

inline double ElectricRotorConfig::effortWeight() const
{
  return kv * moment_const;  // Ratio of current to thrust (`I = N / kt = kv cm T`).
}

inline double ElectricRotorConfig::voltageFromSpeed(double tar_speed) const
{
  assert(tar_speed >= 0.0);

  const auto b = internal_resistance * kv * moment_const * motor_const;
  const auto c = 1.0 / kv;
  return tar_speed * (b * tar_speed + c);
}

inline double ElectricRotorConfig::speedFromVoltage(double voltage) const
{
  assert(voltage >= 0.0);

  const auto b = internal_resistance * kv * moment_const * motor_const;
  const auto c = 1.0 / kv;
  return b > 0 ? (std::sqrt(math::sqr(c) + 4 * b * voltage) - c) / (2 * b) : voltage * kv;
}

inline double ElectricRotorConfig::thrustFromSpeed(double tar_speed) const
{
  return motor_const * math::sqr(tar_speed);
}

inline double ElectricRotorConfig::speedFromThrust(double thrust) const
{
  assert(thrust >= 0.0);
  return std::sqrt(thrust / motor_const);
}

inline double ElectricRotorConfig::thrustFromVoltage(double voltage) const
{
  assert(voltage > 0);

  const auto tar_speed = speedFromVoltage(voltage);
  return thrustFromSpeed(tar_speed);
}

inline double ElectricRotorConfig::throttleFromSpeed(double tar_speed, double battery_voltage) const
{
  assert(tar_speed >= 0.0);

  const auto voltage = voltageFromSpeed(tar_speed);
  return voltage / battery_voltage;
}

inline double ElectricRotorConfig::throttleFromThrust(double thrust, double battery_voltage) const
{
  assert(thrust >= 0.0);

  const auto tar_speed = speedFromThrust(thrust);
  return throttleFromSpeed(tar_speed, battery_voltage);
}
}  // namespace tobas
