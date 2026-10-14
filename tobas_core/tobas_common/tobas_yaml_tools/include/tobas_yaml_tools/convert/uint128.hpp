// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

namespace YAML
{
template <>
struct convert<__uint128_t>
{
  static Node encode(__uint128_t _rhs)
  {
    Node node(NodeType::Sequence);

    for (int i = 0; i < 4; ++i) {
      const auto shift = 32 * i;
      node.push_back(static_cast<uint32_t>((_rhs >> shift) & 0xFFFFFFFF));
    }

    return node;
  }

  static bool decode(const Node& _node, __uint128_t& _rhs)
  {
    if (!_node.IsSequence()) {
      return false;
    }
    if (_node.size() != 4) {
      return false;
    }

    _rhs = 0;
    for (int i = 0; i < 4; ++i) {
      const auto shift = 32 * i;
      _rhs |= (static_cast<__uint128_t>(_node[i].as<uint32_t>()) << shift);
    }

    return true;
  }
};
}  // namespace YAML
