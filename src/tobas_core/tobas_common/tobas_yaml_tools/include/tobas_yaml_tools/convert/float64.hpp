// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <stdfloat>

#include <yaml-cpp/yaml.h>

namespace YAML
{
template <typename T>
  requires(std::same_as<T, std::float64_t> && !std::same_as<T, double>)
struct convert<T>
{
  static Node encode(const T& _rhs) noexcept
  {
    return Node(static_cast<double>(_rhs));
  }

  static bool decode(const Node& _node, T& _rhs) noexcept
  {
    if (!_node.IsScalar()) {
      return false;
    }

    try {
      _rhs = static_cast<T>(_node.as<double>());
      return true;
    }
    catch (const YAML::Exception&) {
      return false;
    }
  }
};
}  // namespace YAML
