#pragma once

#include <filesystem>
#include <yaml-cpp/yaml.h>

#include "./battery.hpp"
#include "./joint/joint.hpp"
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
  JointConfigMap joints;       // The joint configurations (joint name -> config)
  RotorConfigMap rotors;       // The rotor configurations (channel -> config)
  FixedWingConfig fixed_wing;  // The fixed wing configurations

  bool isValid() const;
  bool load(const YAML::Node& node);
  YAML::Node dump() const;

  bool load(const std::filesystem::path& path);
  bool save(const std::filesystem::path& path) const;

  inline size_t numJoints() const;
  inline size_t numRotors() const;
  inline size_t numControlSurfaces() const;

  bool hasServoJoint() const;
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
}  // namespace tobas
