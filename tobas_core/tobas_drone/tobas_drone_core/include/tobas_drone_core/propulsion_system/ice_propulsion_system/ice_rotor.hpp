// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cassert>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/range.hpp>

#include "../../hardware_interface.hpp"
#include "../rotor.hpp"
#include "./aerodynamics.hpp"

namespace tobas
{
/* Gear + Propeller */
class IceRotorConfig : public RotorConfig
{
  using super = RotorConfig;

  static constexpr char kGearRatioKey[] = "gear_ratio";
  static constexpr char kPitchLimitKey[] = "pitch_limit";
  static constexpr char kCenterPitchKey[] = "center_pitch";
  static constexpr char kMotorConstKey[] = "motor_constant";
  static constexpr char kMomentConstKey[] = "moment_constant";
  static constexpr char kHardwareIfaceKey[] = "hw_iface";

public:
  using SharedPtr = std::shared_ptr<IceRotorConfig>;
  using ConstSharedPtr = std::shared_ptr<const IceRotorConfig>;

  double gear_ratio = 0.;                      // Reduction ratio [-].
  st::Range<double> pitch_limit = { 0., 0. };  // Propeller pitch angle range [rad].
  double center_pitch = 0.;                    // Center of the propeller pitch angle [rad].
  VppMotorConstant motor_const;
  VppMomentConstant moment_const;
  HardwareInterface hw_iface = HardwareInterface::kOther;

  bool isValid() const override;

  bool load(const YAML::Node& node) override;
  YAML::Node dump() const override;

  inline double momentConst() const override;
  inline double effortWeight() const override;

  /* Compute the thrust constant [kg*m/rad^2] from pitch angle [rad]. */
  inline double motorConst(double pitch_angle) const;

  /* Compute the reaction torque constant [m] from pitch angle [rad]. */
  inline double momentConst(double pitch_angle) const;

  /* Compute rotor speed [rad/s] from engine speed [rad/s]. */
  inline double speedEngineToRotor(double engine_speed) const;

  /* Compute engine speed [rad/s] from rotor speed [rad/s]. */
  inline double speedRotorToEngine(double rotor_speed) const;

  /* Compute thrust [N] from pitch angle [rad]. */
  inline double thrustFromPitch(double engine_speed, double pitch_angle) const;

  /* Compute pitch angle [rad] from thrust [N]. */
  inline double pitchFromThrust(double engine_speed, double thrust) const;
};

inline double IceRotorConfig::momentConst() const
{
  return moment_const.compute(center_pitch);
}

inline double IceRotorConfig::effortWeight() const
{
  return momentConst();  // Propeller efficiency is used as-is.
}

inline double IceRotorConfig::motorConst(double pitch_angle) const
{
  return motor_const.compute(pitch_angle);
}

inline double IceRotorConfig::momentConst(double pitch_angle) const
{
  return moment_const.compute(pitch_angle);
}

inline double IceRotorConfig::speedEngineToRotor(double engine_speed) const
{
  return engine_speed / gear_ratio;
}

inline double IceRotorConfig::speedRotorToEngine(double rotor_speed) const
{
  return rotor_speed * gear_ratio;
}

inline double IceRotorConfig::thrustFromPitch(double engine_speed, double pitch_angle) const
{
  assert(engine_speed >= 0.);

  const auto rot_speed = speedEngineToRotor(engine_speed);
  return motorConst(pitch_angle) * math::sqr(rot_speed);
}

inline double IceRotorConfig::pitchFromThrust(double engine_speed, double thrust) const
{
  assert(engine_speed > 0.);

  const auto rot_speed = speedEngineToRotor(engine_speed);
  return pitch_limit.clamp((thrust / math::sqr(rot_speed) - motor_const.c0) / motor_const.c1);
}
}  // namespace tobas
