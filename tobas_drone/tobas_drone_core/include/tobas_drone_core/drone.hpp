#pragma once

#include <filesystem>
#include <yaml-cpp/yaml.h>

#include "./joint/joint.hpp"
#include "./pwm.hpp"
#include "./propulsion_system/propulsion_system.hpp"
#include "./propulsion_system/type.hpp"
#include "./fixed_wing/fixed_wing.hpp"

namespace tobas
{
/**
 * @brief ドローンを記述するのに必要な最低限の情報のみを持つクラス．
 */
class Drone
{
  static constexpr char kDroneExt[] = ".tbsdrn";

  static constexpr char kNameKey[] = "name";
  static constexpr char kJointsKey[] = "joints";
  static constexpr char kPwmsKey[] = "pwms";
  static constexpr char kPropulsionSystemTypeKey[] = "propulsion_system_type";
  static constexpr char kPropulsionSystemKey[] = "propulsion_system";
  static constexpr char kFixedWingKey[] = "fixed_wing";
  static constexpr char kNumSbusChannelsKey[] = "num_sbus_channels";

public:
  using SharedPtr = std::shared_ptr<Drone>;
  using ConstSharedPtr = std::shared_ptr<const Drone>;

  std::string name = "";                   // The name of this drone
  JointConfigMap joints;                   // The joint configurations (joint name -> config)
  PwmConfigMap pwms;                       // The PWM configurations (joint name -> config)
  PropulsionSystemConfig::SharedPtr prop;  // The propulsion system configurations
  FixedWingConfig::SharedPtr fixed_wing;   // The fixed wing configurations
  uint32_t num_sbus_channels = 0;          // The number of S.BUS channels

  bool isValid() const;

  bool load(const YAML::Node& node);
  YAML::Node dump() const;

  bool load(const std::filesystem::path& path);
  bool save(const std::filesystem::path& path) const;

  bool hasServoJoint() const;
};
}  // namespace tobas
