#pragma once

#include <string>
#include <map>
#include <yaml-cpp/yaml.h>

#include "./role.hpp"
#include "./command_interface.hpp"

namespace tobas
{
class JointConfig;
using JointConfigMap = std::map<std::string, JointConfig>;  // Joint Name -> JointConfig

class JointConfig
{
  static constexpr char kNameKey[] = "joint_name";
  static constexpr char kHomePosKey[] = "home_position";
  static constexpr char kMinPosKey[] = "min_position";
  static constexpr char kMaxPosKey[] = "max_position";
  static constexpr char kRoleKey[] = "role";
  static constexpr char kCmdIfaceKey[] = "cmd_iface";

public:
  std::string name = "";

  double home_pos = 0;  // [rad | m]
  double min_pos = 0;   // [rad | m]
  double max_pos = 0;   // [rad | m]

  jnt_role_t role = jnt_role_t::MANIPULATION;

  jnt_cmd_iface_t cmd_iface = jnt_cmd_iface_t::POSITION;

  bool isValid() const;
  bool load(const YAML::Node& node);
  YAML::Node dump() const;
};
}  // namespace tobas
