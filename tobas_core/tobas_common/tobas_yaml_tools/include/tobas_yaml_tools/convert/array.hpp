// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>

#include <yaml-cpp/yaml.h>

#include "../format.hpp"

namespace YAML
{
template <typename T, size_t N>
struct convert<std::array<T, N>>
{
  static Node encode(const std::array<T, N>& _rhs)
  {
    Node node(NodeType::Sequence);

    for (auto& value : _rhs) {
      if constexpr (std::is_floating_point_v<T>) {
        node.push_back(tobas::yaml::format(value));
      }
      else {
        node.push_back(value);
      }

      node.push_back(value);
    }

    return node;
  }

  static bool decode(const Node& _node, std::array<T, N>& _rhs)
  {
    if (!_node.IsSequence()) {
      return false;
    }
    if (_node.size() != N) {
      return false;
    }

    for (size_t i = 0; i < N; ++i) {
      _rhs[i] = _node[i].as<T>();
    }

    return true;
  }
};
}  // namespace YAML
