// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/ice_propulsion_system/drag_constant.hpp"

#include <iostream>

#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas
{
bool VppDragConstant::isValid() const
{
  if (c1 <= 0.0) {
    std::cerr << "The second term of the drag constant must be positive." << std::endl;
    return false;
  }

  return true;
}

bool VppDragConstant::load(const YAML::Node& node)
{
  if (!yaml::load(kC0Key, node, c0)) {
    return false;
  }

  if (!yaml::load(kC1Key, node, c1)) {
    return false;
  }

  return true;
}

YAML::Node VppDragConstant::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kC0Key] = yaml::format(c0);
  node[kC1Key] = yaml::format(c1);

  return node;
}
}  // namespace tobas
