// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/joint/joint.hpp"

#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas
{
namespace
{
constexpr char kNameKey[] = "joint_name";
constexpr char kRoleKey[] = "role";
constexpr char kCommandIfaceKey[] = "cmd_iface";
constexpr char kHardwareIfaceKey[] = "hw_iface";
constexpr char kHomePosKey[] = "home_position";
}  // namespace

std::expected<void, std::string> JointConfig::validate() const
{
  if (name.empty()) {
    return std::unexpected("Joint name is empty.");
  }

  return {};
}

std::expected<void, std::string> JointConfig::load(const YAML::Node& node)
{
  if (!node.IsDefined() || !node.IsMap()) {
    return std::unexpected("Configuration node must be a map.");
  }

  if (const auto result = yaml::load(kNameKey, node, name); !result) {
    return result;
  }

  if (const auto result = yaml::load(kRoleKey, node, role); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCommandIfaceKey, node, cmd_iface); !result) {
    return result;
  }

  if (const auto result = yaml::load(kHardwareIfaceKey, node, hw_iface); !result) {
    return result;
  }

  if (const auto result = yaml::load(kHomePosKey, node, home_pos); !result) {
    return result;
  }

  return {};
}

YAML::Node JointConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNameKey] = name;
  node[kRoleKey] = role;
  node[kCommandIfaceKey] = cmd_iface;
  node[kHardwareIfaceKey] = hw_iface;
  node[kHomePosKey] = yaml::format(home_pos);

  return node;
}
}  // namespace tobas
