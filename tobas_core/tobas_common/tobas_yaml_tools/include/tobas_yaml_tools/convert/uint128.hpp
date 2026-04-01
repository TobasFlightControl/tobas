// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

namespace YAML
{
template <>
struct convert<__uint128_t>
{
  static Node encode(const __uint128_t& rhs)
  {
    Node node(NodeType::Sequence);

    for (int i = 0; i < 4; ++i) {
      const auto shift = 32 * i;
      node.push_back(static_cast<uint32_t>((rhs >> shift) & 0xFFFFFFFF));
    }

    return node;
  }

  static bool decode(const Node& node, __uint128_t& rhs)
  {
    if (!node.IsSequence()) {
      return false;
    }
    if (node.size() != 4) {
      return false;
    }

    rhs = 0;
    for (int i = 0; i < 4; ++i) {
      const auto shift = 32 * i;
      rhs |= (static_cast<__uint128_t>(node[i].as<uint32_t>()) << shift);
    }

    return true;
  }
};
}  // namespace YAML
