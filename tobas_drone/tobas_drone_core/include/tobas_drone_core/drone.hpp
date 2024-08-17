#pragma once

#include <filesystem>
#include <yaml-cpp/yaml.h>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_constants/constants.hpp>

#include "./battery.hpp"
#include "./joint.hpp"
#include "./rotor.hpp"
#include "./fixed_wing.hpp"

namespace tobas
{
/**
 * @brief ドローンを記述するのに必要な最低限の情報のみを持つクラス．
 */
class Drone
{
  static constexpr char kNameKey[] = "name";
  static constexpr char kBatteryKey[] = "battery";
  static constexpr char kJointsKey[] = "joints";
  static constexpr char kRotorsKey[] = "rotors";
  static constexpr char kFixedWingKey[] = "fixed_wing";

public:
  static constexpr char kDroneExt[] = ".tbsdrn";

  using SharedPtr = std::shared_ptr<Drone>;
  using ConstSharedPtr = std::shared_ptr<const Drone>;

  std::string name = "";       // The name of this drone
  BatteryConfig battery;       // The battery configurations
  JointConfigMap joints;       // The joint configurations
  RotorConfigs rotors;         // The rotor configurations
  FixedWingConfig fixed_wing;  // The fixed wing configurations

  bool isValid() const;
  bool load(const YAML::Node& node);
  YAML::Node dump() const;

  bool load(const std::filesystem::path& path);
  bool save(const std::filesystem::path& path) const;

  inline size_t numJoints() const;
  inline size_t numRotors() const;
  inline size_t numControlSurfaces() const;

  inline bool isTransformable() const;

  /* 機械回転数 [rad/s] を電気回転数 [rpm] に変換する． */
  inline double erpmFromRotSpeed(size_t rotor_idx, double rot_speed) const;

  /* 与えられたバッテリー電圧で出力できる最大回転数．*/
  inline double maxRotSpeed(size_t rotor_idx, double battery_voltage) const;

  /* 与えられたバッテリー電圧で出力できる最小回転数． */
  inline double minRotSpeed(size_t rotor_idx, double battery_voltage) const;

  /* 機械的に許容できる最大回転数から計算される推力． */
  inline double maxMechanicalThrust(size_t rotor_idx) const;

  /* 与えられたバッテリー電圧で出力できる最大推力．*/
  inline double maxThrust(size_t rotor_idx, double battery_voltage) const;

  /* 与えられたバッテリー電圧で出力できる最小推力． */
  inline double minThrust(size_t rotor_idx, double battery_voltage) const;

  /* 回転数 [rad/s] から推力 [N] を求める． */
  inline double thrustFromRotSpeed(size_t rotor_idx, double rot_speed) const;

  /* 印加電圧から推力 [N] を求める． */
  inline double thrustFromVoltage(size_t rotor_idx, double voltage) const;

  /* 回転数 [rad/s] から印加電圧 [V] を求める． */
  inline double voltageFromRotSpeed(size_t rotor_idx, double rot_speed) const;

  /* 印加電圧 [V] から回転数 [rad/s] を求める． */
  inline double rotSpeedFromVoltage(size_t rotor_idx, double voltage) const;

  /* 推力 [N] から回転数 [rad/s] を求める． */
  inline double rotSpeedFromThrust(size_t rotor_idx, double thrust) const;

  /* 回転数 [rad/s] からスロットル [0,1] を求める． */
  inline double throttleFromRotSpeed(size_t rotor_idx, double rot_speed, double battery_voltage) const;

  /* 推力 [N] からスロットル [0,1] を求める． */
  inline double throttleFromThrust(size_t rotor_idx, double thrust, double battery_voltage) const;
};

inline size_t Drone::numJoints() const
{
  return joints.size();
}

inline size_t Drone::numRotors() const
{
  return rotors.size();
}

inline size_t Drone::numControlSurfaces() const
{
  return fixed_wing.control_surfaces.size();
}

inline bool Drone::isTransformable() const
{
  return numJoints() > 0;
}

inline double Drone::erpmFromRotSpeed(size_t rotor_idx, double rot_speed) const
{
  return tobas_std::rps2rpm(rot_speed) * rotors.at(rotor_idx).num_poles / 2;
}

inline double Drone::maxRotSpeed(size_t rotor_idx, double battery_voltage) const
{
  return std::min(rotors.at(rotor_idx).max_rot_speed, rotSpeedFromVoltage(rotor_idx, battery_voltage));
}

inline double Drone::minRotSpeed(size_t rotor_idx, double battery_voltage) const
{
  const auto min_voltage = battery_voltage * kArmThrot;
  return std::min(rotors.at(rotor_idx).max_rot_speed, rotSpeedFromVoltage(rotor_idx, min_voltage));
}

inline double Drone::maxMechanicalThrust(size_t rotor_idx) const
{
  const auto& rotor = rotors.at(rotor_idx);
  return rotor.motor_constant * math::sqr(rotor.max_rot_speed);
}

inline double Drone::maxThrust(size_t rotor_idx, double battery_voltage) const
{
  // 機械的な限界とエネルギー的な限界の最小値を計算
  return std::min(maxMechanicalThrust(rotor_idx), thrustFromVoltage(rotor_idx, battery_voltage));
}

inline double Drone::minThrust(size_t rotor_idx, double battery_voltage) const
{
  const auto min_voltage = battery_voltage * kArmThrot;
  return std::min(maxMechanicalThrust(rotor_idx), thrustFromVoltage(rotor_idx, min_voltage));
}

inline double Drone::thrustFromRotSpeed(size_t rotor_idx, double tar_speed) const
{
  return rotors[rotor_idx].motor_constant * math::sqr(tar_speed);
}

inline double Drone::thrustFromVoltage(size_t rotor_idx, double voltage) const
{
  assert(voltage > 0);

  const auto tar_speed = rotSpeedFromVoltage(rotor_idx, voltage);
  return thrustFromRotSpeed(rotor_idx, tar_speed);
}

inline double Drone::voltageFromRotSpeed(size_t rotor_idx, double tar_speed) const
{
  assert(tar_speed >= 0);

  const auto& a = rotors[rotor_idx].rot_speed_coefs.first;
  const auto& b = rotors[rotor_idx].rot_speed_coefs.second;
  return a * tar_speed + b * math::sqr(tar_speed);
}

inline double Drone::rotSpeedFromVoltage(size_t rotor_idx, double voltage) const
{
  assert(voltage >= 0);

  const auto& a = rotors[rotor_idx].rot_speed_coefs.first;
  const auto& b = rotors[rotor_idx].rot_speed_coefs.second;
  return b > 0 ? (sqrt(math::sqr(a) + 4 * b * voltage) - a) / (2 * b) : voltage / a;
}

inline double Drone::rotSpeedFromThrust(size_t rotor_idx, double thrust) const
{
  assert(thrust >= 0);
  return sqrt(thrust / rotors[rotor_idx].motor_constant);
}

inline double Drone::throttleFromRotSpeed(size_t rotor_idx, double tar_speed, double battery_voltage) const
{
  assert(tar_speed >= 0);

  const auto voltage = voltageFromRotSpeed(rotor_idx, tar_speed);
  return voltage / battery_voltage;
}

inline double Drone::throttleFromThrust(size_t rotor_idx, double thrust, double battery_voltage) const
{
  assert(thrust >= 0);

  const auto tar_speed = rotSpeedFromThrust(rotor_idx, thrust);
  return throttleFromRotSpeed(rotor_idx, tar_speed, battery_voltage);
}
}  // namespace tobas
