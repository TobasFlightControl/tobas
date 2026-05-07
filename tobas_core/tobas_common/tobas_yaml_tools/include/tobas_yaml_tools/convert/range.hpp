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
  static Node encode(const tobas::st::Range<T>& rhs)
  {
    Node node(NodeType::Sequence);

    if constexpr (std::is_floating_point_v<T>) {
      node.push_back(tobas::yaml::format(rhs.lower));
      node.push_back(tobas::yaml::format(rhs.upper));
    }
    else {
      node.push_back(rhs.lower);
      node.push_back(rhs.upper);
    }

    return node;
  }

  static bool decode(const Node& node, tobas::st::Range<T>& rhs)
  {
    if (!node.IsSequence()) {
      return false;
    }
    if (node.size() != 2) {
      return false;
    }

    rhs.lower = node[0].as<T>();
    rhs.upper = node[1].as<T>();

    return true;
  }
};
}  // namespace YAML
