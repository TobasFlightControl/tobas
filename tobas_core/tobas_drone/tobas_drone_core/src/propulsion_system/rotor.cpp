// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/rotor.hpp"

#include <tobas_yaml_tools/core.hpp>

namespace tobas
{
namespace
{
constexpr char kLinkNameKey[] = "link_name";
constexpr char kDirectionKey[] = "direction";
constexpr char kTiltJointName[] = "tilt_joint_name";
}  // namespace

std::expected<void, std::string> RotorConfig::validate() const
{
  if (link_name.empty()) {
    return std::unexpected("Link name is empty.");
  }

  return {};
}

std::expected<void, std::string> RotorConfig::load(const YAML::Node& node)
{
  if (!node.IsDefined() || !node.IsMap()) {
    return std::unexpected("Configuration node must be a map.");
  }

  if (const auto result = yaml::load(kLinkNameKey, node, link_name); !result) {
    return result;
  }

  if (const auto result = yaml::load(kDirectionKey, node, direction); !result) {
    return result;
  }

  if (const auto result = yaml::load(kTiltJointName, node, tilt_joint_name); !result) {
    return result;
  }

  return {};
}

YAML::Node RotorConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kLinkNameKey] = link_name;
  node[kDirectionKey] = direction;
  node[kTiltJointName] = tilt_joint_name;

  return node;
}
}  // namespace tobas
