// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/ice_propulsion_system/moment_constant.hpp"

#include <cassert>

#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas
{
namespace
{
constexpr char kAKey[] = "a";
constexpr char kBKey[] = "b";
constexpr char kCKey[] = "c";
constexpr char kPhi0Key[] = "phi0";
}  // namespace

std::expected<void, std::string> VppMomentConstant::validate() const
{
  if (a <= 0.0 || c <= 0.0) {
    return std::unexpected("The first and third term of the moment constant must be positive.");
  }

  return {};
}

std::expected<void, std::string> VppMomentConstant::load(const YAML::Node& _node)
{
  if (!_node.IsDefined() || !_node.IsMap()) {
    return std::unexpected("Configuration node must be a map.");
  }

  if (const auto result = yaml::load(kAKey, _node, a); !result) {
    return result;
  }

  if (const auto result = yaml::load(kBKey, _node, b); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCKey, _node, c); !result) {
    return result;
  }

  if (const auto result = yaml::load(kPhi0Key, _node, phi0); !result) {
    return result;
  }

  return {};
}

YAML::Node VppMomentConstant::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kAKey] = yaml::format(a);
  node[kBKey] = yaml::format(b);
  node[kCKey] = yaml::format(c);
  node[kPhi0Key] = yaml::format(phi0);

  return node;
}

double VppMomentConstant::compute(double _phi) const
{
  assert(_phi > phi0);

  const auto x = _phi - phi0;
  return a * x + b + c / x;
}
}  // namespace tobas
