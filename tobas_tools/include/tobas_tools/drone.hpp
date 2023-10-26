#pragma once

#include <ros/ros.h>

#include <dh_kdl/tree.hpp>

#include "./rotor_property.hpp"
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
  inline const Eigen::Vector3d& imuOffset() const;
  inline const Eigen::Vector3d& barometerOffset() const;
  inline const Eigen::Vector3d& gpsOffset() const;
  inline const std::vector<std::string>& postureDefiningJoints() const;
  inline const RotorConfigs& rotorConfigs() const;
  inline const RotorConfig& rotorConfig(const uint32_t& rotor_idx) const;
  inline const FixedWingConfig& fixedWing() const;
  inline const VehicleParameters& vehicle() const;
  inline const AerodynamicsCoefficients& aerodynamics() const;
  inline const ControlSurfaces& controlSurfaces() const;
  inline const ControlSurface& controlSurface(const uint32_t& cs_idx) const;

  inline const bool& hasFixedWing() const;
  inline const bool& isLoaded() const;
  inline bool isTransformable() const;

  inline uint32_t numRotors() const;
  inline uint32_t numControlSurfaces() const;

  /* 回転数[rad/s]から推力[N]を計算する． */
  double thrustFromRotSpeed(const uint32_t& rotor_idx, const double& rot_speed) const;

  /* 印加電圧から推力[N]を計算する． */
  double thrustFromVoltage(const uint32_t& rotor_idx, const double& voltage) const;

  /* 回転数から印加電圧を求める． */
  double voltageFromRotSpeed(const uint32_t& rotor_idx, const double& rot_speed) const;

  /* 印加電圧から回転数を求める． */
  double rotSpeedFromVoltage(const uint32_t& rotor_idx, const double& voltage) const;

  /* 推力[N]から回転数[rad/s]を求める． */
  double rotSpeedFromThrust(const uint32_t& rotor_idx, const double& thrust) const;

private:
  KDL::Tree tree_;
  Eigen::Vector3d imu_offset_;  // ルートリンクに対するIMUの位置をベースで表現したベクトル
  Eigen::Vector3d bar_offset_;  // ルートリンクに対する気圧センサの位置をベースで表現したベクトル
  Eigen::Vector3d gps_offset_;  // ルートリンクに対するGPSレシーバの位置をベースで表現したベクトル
  std::vector<std::string> posture_defining_joints_;
  RotorConfigs rotor_configs_;
  FixedWingConfig fixed_wing_config_;

  bool has_fixed_wing_;
  bool is_loaded_;

  void getRotorConfigs(ros::NodeHandle& nh);
  RotorConfig getRotorConfig(ros::NodeHandle& nh, const uint32_t& rotor_idx);

  void getFixedWingConfig(ros::NodeHandle& nh);
  void getVehicleParameters(ros::NodeHandle& nh);
  void getAerodynamicsCoefficients(ros::NodeHandle& nh);
  void getControlSurfaces(ros::NodeHandle& nh);
  ControlSurface getControlSurface(ros::NodeHandle& nh, const uint32_t& cs_idx);
};

inline const KDL::Tree& Drone::tree() const
{
  return tree_;
}

inline const Eigen::Vector3d& Drone::imuOffset() const
{
  return imu_offset_;
}

inline const Eigen::Vector3d& Drone::barometerOffset() const
{
  return bar_offset_;
}

inline const Eigen::Vector3d& Drone::gpsOffset() const
{
  return gps_offset_;
}

inline const std::vector<std::string>& Drone::postureDefiningJoints() const
{
  return posture_defining_joints_;
}

inline const RotorConfigs& Drone::rotorConfigs() const
{
  return rotor_configs_;
}

inline const RotorConfig& Drone::rotorConfig(const uint32_t& rotor_idx) const
{
  return rotor_configs_[rotor_idx];
}

inline const FixedWingConfig& Drone::fixedWing() const
{
  return fixed_wing_config_;
}

inline const VehicleParameters& Drone::vehicle() const
{
  return fixed_wing_config_.vehicle;
}

inline const AerodynamicsCoefficients& Drone::aerodynamics() const
{
  return fixed_wing_config_.aerodynamics;
}

inline const ControlSurfaces& Drone::controlSurfaces() const
{
  return fixed_wing_config_.control_surfaces;
}

inline const ControlSurface& Drone::controlSurface(const uint32_t& cs_idx) const
{
  return fixed_wing_config_.control_surfaces[cs_idx];
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
  return posture_defining_joints_.size() > 0;
}

inline uint32_t Drone::numRotors() const
{
  return rotor_configs_.size();
}

inline uint32_t Drone::numControlSurfaces() const
{
  return fixed_wing_config_.control_surfaces.size();
}
}  // namespace tobas
