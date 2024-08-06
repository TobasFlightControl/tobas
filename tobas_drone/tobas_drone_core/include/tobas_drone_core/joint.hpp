#pragma once

#include <string>
#include <map>
#include <yaml-cpp/yaml.h>

#include "./joint_conrol_type.hpp"

namespace tobas
{
/* プロペラ，舵面以外の可動関節． */
class JointConfig
{
  static constexpr char kNameKey[] = "name";
  static constexpr char kHomePosKey[] = "home_position";
  static constexpr char kMinPosKey[] = "min_position";
  static constexpr char kMaxPosKey[] = "max_position";
  static constexpr char kControlTypeKey[] = "control_type";

public:
  std::string name = "";

  double home_pos = 0;  // [rad | m]
  double min_pos = 0;   // [rad | m]
  double max_pos = 0;   // [rad | m]

  joint_control_type_t control_type = POSITION_CONTROL;

  bool isValid() const;
  bool load(const YAML::Node& node);
  YAML::Node dump() const;
};

using JointConfigMap = std::map<std::string, JointConfig>;
}  // namespace tobas
