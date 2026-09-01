// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/rotor.hpp"

#include <tobas_yaml_tools/core.hpp>

using namespace std;

namespace tobas
{
namespace
{
constexpr char kLinkNameKey[] = "link_name";
constexpr char kDirectionKey[] = "direction";
constexpr char kTiltJointName[] = "tilt_joint_name";
}  // namespace

bool RotorConfig::isValid() const
{
  if (link_name.empty()) {
    cerr << "Link name is empty." << endl;
    return false;
  }

  return true;
}

bool RotorConfig::load(const YAML::Node& node)
{
  if (!yaml::load(kLinkNameKey, node, link_name)) {
    return false;
  }

  if (!yaml::load(kDirectionKey, node, direction)) {
    return false;
  }

  if (!yaml::load(kTiltJointName, node, tilt_joint_name)) {
    return false;
  }

  return true;
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
