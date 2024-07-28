#pragma once

#include <ros/ros.h>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_kdl/tree.hpp>

#include "./constants.hpp"
#include "./battery_config.hpp"
#include "./joint_config.hpp"
#include "./rotor_config.hpp"
#include "./fixed_wing_tools.hpp"

namespace tobas
{
/**
 * @brief ドローンを記述するのに必要な最低限の情報のみを持つクラス．
 */
class Drone
{
public:
  explicit Drone();

  /* Load drone configurations from ROS parameter server. */
  void loadFromParam(ros::NodeHandle& nh);

  inline const kdl::Tree& tree() const;
  inline const BatteryConfig& batteryConfig() const;
  inline const JointConfigMap& jointConfigMap() const;
  inline const JointConfig& jointConfig(const std::string& jnt_name) const;
  inline const RotorConfigs& rotorConfigs() const;
  inline const RotorConfig& rotorConfig(size_t rotor_idx) const;
  inline const FixedWingConfig& fixedWing() const;
  inline const VehicleParameters& vehicle() const;
  inline const AerodynamicsCoefficients& aerodynamics() const;
  inline const ControlSurfaces& controlSurfaces() const;
  inline const ControlSurface& controlSurface(size_t cs_idx) const;

  inline const std::string& droneName() const;
  inline const bool& hasFixedWing() const;
  inline const bool& isLoaded() const;
  inline bool isTransformable() const;

  inline size_t numRotors() const;
  inline size_t numControlSurfaces() const;

  /* 機械回転数 [rad/s] を電気回転数 [rpm] に変換する． */
  inline double erpmFromRotSpeed(size_t rotor_idx, double rot_speed);

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

private:
  kdl::Tree tree_;

  BatteryConfig battery_;
  JointConfigMap joint_map_;  // プロペラ，舵面以外の可動関節
  RotorConfigs rotors_;
  FixedWingConfig fixed_wing_;

  std::string drone_name_;
  bool has_fixed_wing_;
  bool is_loaded_ = false;

  void getBatteryConfig(ros::NodeHandle& nh);
  void getJointConfigs(ros::NodeHandle& nh);
  void getJointConfig(ros::NodeHandle& nh, size_t jnt_idx);

  void getRotorConfigs(ros::NodeHandle& nh);
  void getRotorConfig(ros::NodeHandle& nh, size_t rotor_idx, RotorConfig& des);

  void getFixedWingConfig(ros::NodeHandle& nh);
  void getVehicleParameters(ros::NodeHandle& nh);
  void getAerodynamicsCoefficients(ros::NodeHandle& nh);
  void getControlSurfaces(ros::NodeHandle& nh);
  void getControlSurface(ros::NodeHandle& nh, size_t cs_idx, ControlSurface& des);
};

inline const kdl::Tree& Drone::tree() const
{
  return tree_;
}

inline const BatteryConfig& Drone::batteryConfig() const
{
  return battery_;
}

inline const JointConfigMap& Drone::jointConfigMap() const
{
  return joint_map_;
}

inline const JointConfig& Drone::jointConfig(const std::string& jnt_name) const
{
  return joint_map_.at(jnt_name);
}

inline const RotorConfigs& Drone::rotorConfigs() const
{
  return rotors_;
}

inline const RotorConfig& Drone::rotorConfig(size_t rotor_idx) const
{
  return rotors_.at(rotor_idx);
}

inline const FixedWingConfig& Drone::fixedWing() const
{
  return fixed_wing_;
}

inline const VehicleParameters& Drone::vehicle() const
{
  return fixed_wing_.vehicle;
}

inline const AerodynamicsCoefficients& Drone::aerodynamics() const
{
  return fixed_wing_.aerodynamics;
}

inline const ControlSurfaces& Drone::controlSurfaces() const
{
  return fixed_wing_.control_surfaces;
}

inline const ControlSurface& Drone::controlSurface(size_t cs_idx) const
{
  return fixed_wing_.control_surfaces.at(cs_idx);
}

inline const std::string& Drone::droneName() const
{
  return drone_name_;
}

inline const bool& Drone::hasFixedWing() const
{
  return has_fixed_wing_;
}

inline const bool& Drone::isLoaded() const
{
  return is_loaded_;
}

inline bool Drone::isTransformable() const
{
  return joint_map_.size() > 0;
}

inline size_t Drone::numRotors() const
{
  return rotors_.size();
}

inline size_t Drone::numControlSurfaces() const
{
  return fixed_wing_.control_surfaces.size();
}

inline double Drone::erpmFromRotSpeed(size_t rotor_idx, double rot_speed)
{
  return tobas_std::rps2rpm(rot_speed) * rotors_.at(rotor_idx).num_poles / 2;
}

inline double Drone::maxRotSpeed(size_t rotor_idx, double battery_voltage) const
{
  return std::min(rotors_.at(rotor_idx).max_rot_speed, rotSpeedFromVoltage(rotor_idx, battery_voltage));
}

inline double Drone::minRotSpeed(size_t rotor_idx, double battery_voltage) const
{
  const auto min_voltage = battery_voltage * kArmThrottle;
  return std::min(rotors_.at(rotor_idx).max_rot_speed, rotSpeedFromVoltage(rotor_idx, min_voltage));
}

inline double Drone::maxMechanicalThrust(size_t rotor_idx) const
{
  const auto& rotor = rotors_.at(rotor_idx);
  return rotor.motor_constant * math::sqr(rotor.max_rot_speed);
}

inline double Drone::maxThrust(size_t rotor_idx, double battery_voltage) const
{
  // 機械的な限界とエネルギー的な限界の最小値を計算
  return std::min(maxMechanicalThrust(rotor_idx), thrustFromVoltage(rotor_idx, battery_voltage));
}

inline double Drone::minThrust(size_t rotor_idx, double battery_voltage) const
{
  const auto min_voltage = battery_voltage * kArmThrottle;
  return std::min(maxMechanicalThrust(rotor_idx), thrustFromVoltage(rotor_idx, min_voltage));
}

inline double Drone::thrustFromRotSpeed(size_t rotor_idx, double tar_speed) const
{
  return rotors_[rotor_idx].motor_constant * math::sqr(tar_speed);
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

  const auto& a = rotors_[rotor_idx].rot_speed_coefs.first;
  const auto& b = rotors_[rotor_idx].rot_speed_coefs.second;
  return a * tar_speed + b * math::sqr(tar_speed);
}

inline double Drone::rotSpeedFromVoltage(size_t rotor_idx, double voltage) const
{
  assert(voltage >= 0);

  const auto& a = rotors_[rotor_idx].rot_speed_coefs.first;
  const auto& b = rotors_[rotor_idx].rot_speed_coefs.second;
  return b > 0 ? (sqrt(math::sqr(a) + 4 * b * voltage) - a) / (2 * b) : voltage / a;
}

inline double Drone::rotSpeedFromThrust(size_t rotor_idx, double thrust) const
{
  assert(thrust >= 0);
  return sqrt(thrust / rotors_[rotor_idx].motor_constant);
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
