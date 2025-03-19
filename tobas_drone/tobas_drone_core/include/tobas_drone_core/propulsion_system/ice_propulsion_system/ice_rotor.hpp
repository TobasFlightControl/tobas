#pragma once

#include <cassert>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/range.hpp>

#include "../rotor.hpp"
#include "../../hardware_interface.hpp"

namespace tobas
{
/* Gear + Propeller */
class ICERotorConfig : public RotorConfig
{
  using super = RotorConfig;

  static constexpr char kGearRatioKey[] = "gear_ratio";
  static constexpr char kPitchReferenceKey[] = "pitch_reference";
  static constexpr char kPitchRangeKey[] = "pitch_range";
  static constexpr char kMotorConstKey[] = "motor_constant";
  static constexpr char kHardwareIfaceKey[] = "hw_iface";

public:
  using SharedPtr = std::shared_ptr<ICERotorConfig>;
  using ConstSharedPtr = std::shared_ptr<const ICERotorConfig>;

  double gear_ratio = 0.;  // 減速比 [-]
  double pitch_ref = 0.;   // プロペラピッチ角の参照値 (最も効率の良いピッチ角) [rad]
  tobas_std::Range<double> pitch_range = { 0., 0. };  // プロペラピッチ角の範囲 [rad]
  std::pair<double, double> motor_const = { 0., 0. };  // T = (aφ + b) ω^2 (φ: プロペラのピッチ角，ω: プロペラの回転数)
  hw_iface_t hw_iface = hw_iface_t::OTHER;

  bool isValid() const override;

  bool load(const YAML::Node& node) override;
  YAML::Node dump() const override;

  /* ピッチ角からプロペラ回転数の2乗と推力の比を求める． */
  inline double motorConst(double pitch_angle) const;

  /* エンジン回転数からロータ回転数を求める． */
  inline double speedEngineToRotor(double engine_speed) const;

  /* ロータ回転数からエンジン回転数を求める． */
  inline double speedRotorToEngine(double rotor_speed) const;

  /* ピッチ角から推力を求める． */
  inline double thrustFromPitch(double engine_speed, double pitch_angle) const;

  /* 推力からピッチ角を求める． */
  inline double pitchFromThrust(double engine_speed, double thrust) const;
};

inline double ICERotorConfig::motorConst(double pitch_angle) const
{
  return motor_const.first * pitch_angle + motor_const.second;
}

inline double ICERotorConfig::speedEngineToRotor(double engine_speed) const
{
  return engine_speed / gear_ratio;
}

inline double ICERotorConfig::speedRotorToEngine(double rotor_speed) const
{
  return rotor_speed * gear_ratio;
}

inline double ICERotorConfig::thrustFromPitch(double engine_speed, double pitch_angle) const
{
  assert(engine_speed >= 0.);

  const auto rot_speed = speedEngineToRotor(engine_speed);
  return motorConst(pitch_angle) * math::sqr(rot_speed);
}

inline double ICERotorConfig::pitchFromThrust(double engine_speed, double thrust) const
{
  assert(engine_speed > 0.);

  const auto rot_speed = speedEngineToRotor(engine_speed);
  return (thrust / math::sqr(rot_speed) - motor_const.second) / motor_const.first;
}
}  // namespace tobas
