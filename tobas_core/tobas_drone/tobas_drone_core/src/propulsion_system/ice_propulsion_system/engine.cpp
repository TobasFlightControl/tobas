// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/ice_propulsion_system/engine.hpp"

#include <cassert>

#include <tobas_math/core.hpp>
#include <tobas_yaml_tools/convert/pair.hpp>
#include <tobas_yaml_tools/core.hpp>

namespace tobas
{
bool EngineConfig::isValid() const
{
  if (engine_const.first <= 0. || engine_const.second <= 0.) {
    std::cerr << "Engine constants must be positive." << std::endl;
    return false;
  }

  return true;
}

bool EngineConfig::load(const YAML::Node& node)
{
  if (!yaml::load(kEngineConstantKey, node, engine_const)) {
    return false;
  }

  if (!yaml::load(kHardwareIfaceKey, node, hw_iface)) {
    return false;
  }

  return true;
}

YAML::Node EngineConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kEngineConstantKey] = engine_const;
  node[kHardwareIfaceKey] = hw_iface;

  return node;
}

double EngineConfig::computeTorque(double speed, double throttle)
{
  assert(speed >= 0.);
  assert(0. <= throttle && throttle <= 1.);

  if (throttle < std::numeric_limits<double>::epsilon()) {
    return 0.;
  }

  const auto& [A, B] = engine_const;

  // FIXME: 実際はゼロスロットル (アイドリング) でも出力トルクはゼロではなく，エンジンモデルの改善が必要．
  const auto phi = M_PI_2 * throttle;
  const auto f = math::sqr(A / (1 - std::cos(phi)));
  return 2 * B * speed / (sqrt(1 + 4 * B * math::sqr(speed) * f) + 1);
}

double EngineConfig::computeThrottle(double torque, double speed)
{
  assert(torque >= 0.);
  assert(speed >= 0.);

  if (speed < std::numeric_limits<double>::epsilon()) {
    return 0.;
  }

  const auto& [A, B] = engine_const;

  const auto cos_phi = 1. - A * torque / sqrt(std::max(B - torque / speed, 0.));
  const auto phi = std::acos(std::clamp(cos_phi, 0., 1.));
  return phi / M_PI_2;
}

std::ostream& operator<<(std::ostream& os, const EngineConfig& arg)
{
  os << "Engine Constant: " << arg.engine_const.first << ", " << arg.engine_const.second << std::endl;
  return os;
}
}  // namespace tobas
