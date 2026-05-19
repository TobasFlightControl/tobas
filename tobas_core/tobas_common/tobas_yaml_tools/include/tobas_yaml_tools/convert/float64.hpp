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
  static Node encode(const T& rhs) noexcept
  {
    return Node(static_cast<double>(rhs));
  }

  static bool decode(const Node& node, T& rhs) noexcept
  {
    if (!node.IsScalar()) {
      return false;
    }

    try {
      rhs = static_cast<T>(node.as<double>());
      return true;
    }
    catch (const YAML::Exception&) {
      return false;
    }
  }
};
}  // namespace YAML
