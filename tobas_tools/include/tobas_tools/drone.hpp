#pragma once

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
  void loadFromParam(const std::string& ns);

  const KDL::Tree& tree() const;
  const std::vector<std::string>& postureDefiningJoints() const;
  const RotorConfigs& rotorConfigs() const;
  const RotorConfig& rotorConfig(uint32_t rotor_idx) const;
  const FixedWingConfig& fixedWing() const;
  const VehicleParameters& vehicle() const;
  const AerodynamicsCoefficients& aerodynamics() const;
  const ControlSurfaces& controlSurfaces() const;
  const ControlSurface& controlSurface(uint32_t cs_idx) const;

  const bool& hasFixedWing() const;
  const bool& isLoaded() const;

  uint32_t numRotors() const;
  uint32_t numControlSurfaces() const;

  /* 指定したロータの最大回転数 [rad/s]． */
  double maxRotSpeed(uint32_t rotor_idx, double battery_voltage) const;

  /* 指定したロータの最大推力 [N]． */
  double maxThrust(uint32_t rotor_idx, double battery_voltage) const;

  /* 推力 [N] からロータの回転数 [rad/s] を求める． */
  double thrustToRotSpeed(uint32_t rotor_idx, double thrust) const;

private:
  KDL::Tree tree_;
  std::vector<std::string> posture_defining_joints_;
  RotorConfigs rotor_configs_;
  FixedWingConfig fixed_wing_config_;

  bool has_fixed_wing_;
  bool is_loaded_;

  void getTree(const std::string& ns);

  void getRotorConfigs(const std::string& ns);
  RotorConfig getRotorConfig(const std::string& ns, uint32_t rotor_idx);

  void getFixedWingConfig(const std::string& ns);
  void getVehicleParameters(const std::string& ns);
  void getAerodynamicsCoefficients(const std::string& ns);
  void getControlSurfaces(const std::string& ns);
  ControlSurface getControlSurface(const std::string& ns, uint32_t cs_idx);
};
}  // namespace tobas
