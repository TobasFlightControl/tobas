#pragma once

#include <ros/ros.h>

#include <tobas_std_tools/math.hpp>
#include <tobas_kdl/tree.hpp>

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

  inline const KDL::Tree& tree() const;
  inline const BatteryConfig& batteryConfig() const;
  inline const JointConfigMap& jointConfigMap() const;
  inline const JointConfig& jointConfig(const std::string& jnt_name) const;
  inline const RotorConfigs& rotorConfigs() const;
  inline const RotorConfig& rotorConfig(const size_t& rotor_idx) const;
  inline const FixedWingConfig& fixedWing() const;
  inline const VehicleParameters& vehicle() const;
  inline const AerodynamicsCoefficients& aerodynamics() const;
  inline const ControlSurfaces& controlSurfaces() const;
  inline const ControlSurface& controlSurface(const size_t& cs_idx) const;

  inline const std::string& droneName() const;
  inline const double& nominalBatteryVoltage() const;
  inline const bool& hasFixedWing() const;
  inline const bool& isLoaded() const;
  inline bool isTransformable() const;

  inline size_t numRotors() const;
  inline size_t numControlSurfaces() const;

  /* 機械回転数 [rad/s] を電気回転数 [rpm] に変換する． */
  inline double erpmFromRotSpeed(const size_t& rotor_idx, const double& rot_speed);

  /* 与えられたバッテリー電圧で出力できる最大回転数．*/
  double maxRotSpeed(const size_t& rotor_idx, const double& battery_voltage) const;

  /* 与えられたバッテリー電圧で出力できる最小回転数． */
  double minRotSpeed(const size_t& rotor_idx, const double& battery_voltage) const;

  /* 機械的に許容できる最大回転数から計算される推力． */
  double maxMechanicalThrust(const size_t& rotor_idx) const;

  /* 与えられたバッテリー電圧で出力できる最大推力．*/
  double maxThrust(const size_t& rotor_idx, const double& battery_voltage) const;

  /* 与えられたバッテリー電圧で出力できる最小推力． */
  double minThrust(const size_t& rotor_idx, const double& battery_voltage) const;

  /* 回転数 [rad/s] から推力 [N] を求める． */
  double thrustFromRotSpeed(const size_t& rotor_idx, const double& rot_speed) const;

  /* 印加電圧から推力 [N] を求める． */
  double thrustFromVoltage(const size_t& rotor_idx, const double& voltage) const;

  /* 回転数 [rad/s] から印加電圧 [V] を求める． */
  double voltageFromRotSpeed(const size_t& rotor_idx, const double& rot_speed) const;

  /* 印加電圧 [V] から回転数 [rad/s] を求める． */
  double rotSpeedFromVoltage(const size_t& rotor_idx, const double& voltage) const;

  /* 推力 [N] から回転数 [rad/s] を求める． */
  double rotSpeedFromThrust(const size_t& rotor_idx, const double& thrust) const;

  /* 回転数 [rad/s] からスロットル [0,1] を求める． */
  double throttleFromRotSpeed(
    const size_t& rotor_idx,
    const double& rot_speed,
    const double& battery_voltage) const;

  /* 推力 [N] からスロットル [0,1] を求める． */
  double throttleFromThrust(
    const size_t& rotor_idx,
    const double& thrust,
    const double& battery_voltage) const;

private:
  KDL::Tree tree_;

  BatteryConfig battery_;
  JointConfigMap joint_map_;  // プロペラ，舵面以外の可動関節
  RotorConfigs rotors_;
  FixedWingConfig fixed_wing_;

  std::string drone_name_;
  bool has_fixed_wing_;
  bool is_loaded_ = false;

  void getBatteryConfig(ros::NodeHandle& nh);
  void getJointConfigs(ros::NodeHandle& nh);
  void getJointConfig(ros::NodeHandle& nh, const size_t& jnt_idx);

  void getRotorConfigs(ros::NodeHandle& nh);
  RotorConfig getRotorConfig(ros::NodeHandle& nh, const size_t& rotor_idx);

  void getFixedWingConfig(ros::NodeHandle& nh);
  void getVehicleParameters(ros::NodeHandle& nh);
  void getAerodynamicsCoefficients(ros::NodeHandle& nh);
  void getControlSurfaces(ros::NodeHandle& nh);
  ControlSurface getControlSurface(ros::NodeHandle& nh, const size_t& cs_idx);
};

inline const KDL::Tree& Drone::tree() const
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

inline const RotorConfig& Drone::rotorConfig(const size_t& rotor_idx) const
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

inline const ControlSurface& Drone::controlSurface(const size_t& cs_idx) const
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

inline double Drone::erpmFromRotSpeed(const size_t& rotor_idx, const double& rot_speed)
{
  return tobas_std::rps2rpm(rot_speed) * rotors_.at(rotor_idx).num_poles / 2;
}
}  // namespace tobas
