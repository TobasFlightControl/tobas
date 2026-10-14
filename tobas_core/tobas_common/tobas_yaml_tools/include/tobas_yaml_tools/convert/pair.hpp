// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

#include "../format.hpp"

namespace YAML
{
template <>
struct convert<std::pair<double, double>>
{
  static Node encode(const std::pair<double, double>& _rhs)
  {
    Node node(NodeType::Sequence);

    node.push_back(tobas::yaml::format(_rhs.first));
    node.push_back(tobas::yaml::format(_rhs.second));

    return node;
  }

  static bool decode(const Node& _node, std::pair<double, double>& _rhs)
  {
    if (!_node.IsSequence()) {
      return false;
    }
    if (_node.size() != 2) {
      return false;
    }

    _rhs.first = _node[0].as<double>();
    _rhs.second = _node[1].as<double>();

    return true;
  }
};
}  // namespace YAML
