// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/ice_propulsion_system/moment_constant.hpp"

#include <iostream>
#include <limits>

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

bool VppMomentConstant::isValid() const
{
  if (a <= 0.0 || c <= 0.0) {
    std::cerr << "The first and third term of the moment constant must be positive." << std::endl;
    return false;
  }

  return true;
}

bool VppMomentConstant::load(const YAML::Node& node)
{
  if (!yaml::load(kAKey, node, a)) {
    return false;
  }

  if (!yaml::load(kBKey, node, b)) {
    return false;
  }

  if (!yaml::load(kCKey, node, c)) {
    return false;
  }

  if (!yaml::load(kPhi0Key, node, phi0)) {
    return false;
  }

  return true;
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

double VppMomentConstant::compute(double phi) const
{
  if (phi <= phi0) {
    std::cerr << "The moment constant cannot be computed because the pitch angle of " << phi
              << "[rad] is smaller than the negative stall angle of " << phi0 << "[rad]." << std::endl;
    return std::numeric_limits<double>::max();
  }

  const auto x = phi - phi0;
  return a * x + b + c / x;
}
}  // namespace tobas
