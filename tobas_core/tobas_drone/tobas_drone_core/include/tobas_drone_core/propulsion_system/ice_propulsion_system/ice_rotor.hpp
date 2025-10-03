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
  static constexpr char kMotorConstKey[] = "motor_constant";
  static constexpr char kMomentConstKey[] = "moment_constant";
  static constexpr char kHardwareIfaceKey[] = "hw_iface";

public:
  using SharedPtr = std::shared_ptr<IceRotorConfig>;
  using ConstSharedPtr = std::shared_ptr<const IceRotorConfig>;

  double gear_ratio = 0.;                             // 減速比 [-]
  tobas_std::Range<double> pitch_limit = { 0., 0. };  // プロペラピッチ角の範囲 [rad]
  VppMotorConstant motor_const;
  VppMomentConstant moment_const;
  HardwareInterface hw_iface = HardwareInterface::kOther;

  bool isValid() const override;

  bool load(const YAML::Node& node) override;
  YAML::Node dump() const override;

  /* 最も効率の良いピッチ角における反トルク係数 [m] を求める． */
  inline double momentConst() const override;

  /* 最も効率の良いピッチ角 [rad] を求める． */
  inline double optimalPitch() const;

  /* ピッチ角 [rad] から推力定数 [kg*m/rad^2] を求める． */
  inline double motorConst(double pitch_angle) const;

  /* ピッチ角 [rad] から反トルク定数 [m] を求める． */
  inline double momentConst(double pitch_angle) const;

  /* エンジン回転数 [rad/s] からロータ回転数 [rad/s] を求める． */
  inline double speedEngineToRotor(double engine_speed) const;

  /* ロータ回転数 [rad/s] からエンジン回転数 [rad/s] を求める． */
  inline double speedRotorToEngine(double rotor_speed) const;

  /* ピッチ角 [rad] から推力 [N] を求める． */
  inline double thrustFromPitch(double engine_speed, double pitch_angle) const;

  /* 推力 [N] からピッチ角 [rad] を求める． */
  inline double pitchFromThrust(double engine_speed, double thrust) const;
};

inline double IceRotorConfig::momentConst() const
{
  return moment_const.compute(moment_const.optimalPitch());
}

inline double IceRotorConfig::optimalPitch() const
{
  return moment_const.optimalPitch();
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
