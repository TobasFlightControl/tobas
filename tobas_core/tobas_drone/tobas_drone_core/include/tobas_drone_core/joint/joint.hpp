// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <map>
#include <string>

#include <yaml-cpp/yaml.h>

#include "../hardware_interface.hpp"
#include "./command_interface.hpp"
#include "./role.hpp"

namespace tobas
{
class JointConfig;
using JointConfigMap = std::map<std::string, JointConfig>;  // Joint Name -> JointConfig

class JointConfig
{
  static constexpr char kNameKey[] = "joint_name";
  static constexpr char kRoleKey[] = "role";
  static constexpr char kCommandIfaceKey[] = "cmd_iface";
  static constexpr char kHardwareIfaceKey[] = "hw_iface";
  static constexpr char kHomePosKey[] = "home_position";

public:
  std::string name = "";
  JointRole role = JointRole::kUserPassive;
  JointCommandInterface cmd_iface = JointCommandInterface::kNone;
  HardwareInterface hw_iface = HardwareInterface::kOther;
  double home_pos = 0.0;  // [rad | m]

  bool isValid() const;

  bool load(const YAML::Node& node);
  YAML::Node dump() const;

  inline bool isServoJoint() const;
};

inline bool JointConfig::isServoJoint() const
{
  return tobas::isServoJoint(role);
}
}  // namespace tobas
