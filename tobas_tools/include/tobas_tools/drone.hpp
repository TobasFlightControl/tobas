#pragma once

#include <ros/ros.h>
#include <kdl/tree.hpp>

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

  const KDL::Tree& tree() const;
  const Eigen::Vector3d& imuOffset() const;
  const Eigen::Vector3d& barometerOffset() const;
  const Eigen::Vector3d& gpsOffset() const;
  const std::vector<std::string>& postureDefiningJoints() const;
  const RotorConfigs& rotorConfigs() const;
  const RotorConfig& rotorConfig(const uint32_t& rotor_idx) const;
  const FixedWingConfig& fixedWing() const;
  const VehicleParameters& vehicle() const;
  const AerodynamicsCoefficients& aerodynamics() const;
  const ControlSurfaces& controlSurfaces() const;
  const ControlSurface& controlSurface(const uint32_t& cs_idx) const;

  const bool& hasFixedWing() const;
  const bool& isLoaded() const;

  uint32_t numRotors() const;
  uint32_t numControlSurfaces() const;

  /* 指定したロータの推力 [N]． */
  double thrustFromVoltage(const uint32_t& rotor_idx, const double& voltage) const;

  /* 指定したロータの回転数から印加電圧を求める． */
  double voltageFromRotSpeed(const uint32_t& rotor_idx, const double& rot_speed) const;

  /* 指定したロータの印加電圧から回転数を求める． */
  double rotSpeedFromVoltage(const uint32_t& rotor_idx, const double& voltage) const;

  /* 推力 [N] からロータの回転数 [rad/s] を求める． */
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
}  // namespace tobas
