// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_std_tools/range.hpp>

#include "../format.hpp"

namespace YAML
{
template <typename T>
struct convert<tobas::st::Range<T>>
{
  static Node encode(const tobas::st::Range<T>& _rhs)
  {
    Node node(NodeType::Sequence);

    if constexpr (std::is_floating_point_v<T>) {
      node.push_back(tobas::yaml::format(_rhs.lower));
      node.push_back(tobas::yaml::format(_rhs.upper));
    }
    else {
      node.push_back(_rhs.lower);
      node.push_back(_rhs.upper);
    }

    return node;
  }

  static bool decode(const Node& _node, tobas::st::Range<T>& _rhs)
  {
    if (!_node.IsSequence()) {
      return false;
    }
    if (_node.size() != 2) {
      return false;
    }

    _rhs.lower = _node[0].as<T>();
    _rhs.upper = _node[1].as<T>();

    return true;
  }
};
}  // namespace YAML
