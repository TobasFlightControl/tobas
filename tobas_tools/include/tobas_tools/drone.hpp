#pragma once

#include <ros/ros.h>

#include <tobas_kdl/tree.hpp>

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
  inline const JointConfigs& jointConfigs() const;
  inline const JointConfig& jointConfig(const size_t& joint_idx) const;
  inline const RotorConfigs& rotorConfigs() const;
  inline const RotorConfig& rotorConfig(const size_t& rotor_idx) const;
  inline const FixedWingConfig& fixedWing() const;
  inline const VehicleParameters& vehicle() const;
  inline const AerodynamicsCoefficients& aerodynamics() const;
  inline const ControlSurfaces& controlSurfaces() const;
  inline const ControlSurface& controlSurface(const size_t& cs_idx) const;

  inline const bool& hasFixedWing() const;
  inline const bool& isLoaded() const;
  inline bool isTransformable() const;

  inline size_t numRotors() const;
  inline size_t numControlSurfaces() const;

  std::vector<std::string> postureDefiningJointNames() const;

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

  /* 推力 [N] からスロットル [0,1] を求める． */
  double throttleFromThrust(
    const size_t& rotor_idx,
    const double& thrust,
    const double& battery_voltage) const;

private:
  KDL::Tree tree_;

  JointConfigs joints_;  // プロペラ，舵面以外の可動関節
  RotorConfigs rotors_;
  FixedWingConfig fixed_wing_;

  bool has_fixed_wing_;
  bool is_loaded_ = false;

  void getJointConfigs(ros::NodeHandle& nh);
  JointConfig getJointConfig(ros::NodeHandle& nh, const size_t& joint_idx);

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

inline const JointConfigs& Drone::jointConfigs() const
{
  return joints_;
}

inline const JointConfig& Drone::jointConfig(const size_t& joint_idx) const
{
  return joints_.at(joint_idx);
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
  return joints_.size() > 0;
}

inline size_t Drone::numRotors() const
{
  return rotors_.size();
}

inline size_t Drone::numControlSurfaces() const
{
  return fixed_wing_.control_surfaces.size();
}
}  // namespace tobas
