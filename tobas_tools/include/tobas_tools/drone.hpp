#pragma once

#include "./rotor_property.hpp"
#include "./fixed_wing_tools.hpp"

/**
 * @brief ドローンの固有パラメータ．TBSFファイルを読み込む．
 */
class Drone
{
public:
  explicit Drone();

  /* Load drone configurations from ROS parameter server. */
  void loadFromParam(const std::string& ns);

  const bool& hasFixedWing() const;
  const double& batteryVoltage() const;
  const std::vector<std::string>& activeJointNames() const;
  const RotorConfigs& rotorConfigs() const;
  const FixedWingConfig& fixedWingConfig() const;

  uint32_t numRotors() const;

  /* RotorConfigsのうち特定の軸をもつものの添字を取得する． */
  std::vector<uint32_t> rotorConfigIdxInAxis(const Axis& axis) const;

  /* 特定の軸をもつロータの個数を返す． */
  uint32_t numRotorsInAxis(const Axis& axis) const;

  /* 指定したロータの最大回転数 [rad/s]． */
  double maxRotSpeed(uint32_t idx) const;

  /* 指定したロータの最大推力 [N]． */
  double maxThrust(uint32_t idx) const;

private:
  bool has_fixed_wing_;
  double battery_voltage_;
  std::vector<std::string> active_joint_names_;
  RotorConfigs rotor_configs_;
  FixedWingConfig fixed_wing_config_;

  void getRotorConfigs(const std::string& ns);
  void getRotorConfig(const std::string& ns, uint32_t idx);

  void getFixedWingConfig(const std::string& ns);
  void getVehicleParameters(const std::string& ns);
  void getAerodynamicsCoefficients(const std::string& ns);
  void getControlSurfaces(const std::string& ns);
};
