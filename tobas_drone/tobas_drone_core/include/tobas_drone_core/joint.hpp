#pragma once

#include <string>
#include <map>
#include <yaml-cpp/yaml.h>

#include "./joint_interface.hpp"

namespace tobas
{
class JointConfig;
using JointConfigMap = std::map<std::string, JointConfig>;  // Joint Name -> JointConfig

/* プロペラ，舵面以外の可動関節． */
class JointConfig
{
  static constexpr char kNameKey[] = "joint_name";
  static constexpr char kHomePosKey[] = "home_position";
  static constexpr char kMinPosKey[] = "min_position";
  static constexpr char kMaxPosKey[] = "max_position";
  static constexpr char kInterfaceKey[] = "interface";

public:
  std::string name = "";

  double home_pos = 0;  // [rad | m]
  double min_pos = 0;   // [rad | m]
  double max_pos = 0;   // [rad | m]

  joint_interface_t interface = joint_interface_t::POSITION;

  bool isValid() const;
  bool load(const YAML::Node& node);
  YAML::Node dump() const;
};
}  // namespace tobas
