// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/ice_propulsion_system/engine.hpp"

#include <cassert>

#include <tobas_math/core.hpp>
#include <tobas_yaml_tools/convert/pair.hpp>
#include <tobas_yaml_tools/core.hpp>

namespace tobas
{
namespace
{
constexpr char kEngineConstantKey[] = "engine_constant";
constexpr char kHardwareIfaceKey[] = "hw_iface";
}  // namespace

std::expected<void, std::string> EngineConfig::validate() const
{
  if (engine_const.first <= 0.0 || engine_const.second <= 0.0) {
    return std::unexpected("Engine constants must be positive.");
  }

  return {};
}

std::expected<void, std::string> EngineConfig::load(const YAML::Node& _node)
{
  if (!_node.IsDefined() || !_node.IsMap()) {
    return std::unexpected("Configuration node must be a map.");
  }

  if (const auto result = yaml::load(kEngineConstantKey, _node, engine_const); !result) {
    return result;
  }

  if (const auto result = yaml::load(kHardwareIfaceKey, _node, hw_iface); !result) {
    return result;
  }

  return {};
}

YAML::Node EngineConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kEngineConstantKey] = engine_const;
  node[kHardwareIfaceKey] = hw_iface;

  return node;
}

double EngineConfig::computeTorque(double _speed, double _throttle)
{
  assert(_speed >= 0.0);
  assert(0.0 <= _throttle && _throttle <= 1.0);

  if (_throttle < std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  const auto& [A, B] = engine_const;

  // FIXME: Output torque is not actually zero even at zero throttle due to idling, so the engine model needs to be improved.
  const auto phi = M_PI_2 * _throttle;
  const auto f = math::sqr(A / (1 - std::cos(phi)));
  return 2 * B * _speed / (sqrt(1 + 4 * B * math::sqr(_speed) * f) + 1);
}

double EngineConfig::computeThrottle(double _torque, double _speed)
{
  assert(_torque >= 0.0);
  assert(_speed >= 0.0);

  if (_speed < std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  const auto& [A, B] = engine_const;

  const auto cos_phi = 1.0 - A * _torque / sqrt(std::max(B - _torque / _speed, 0.0));
  const auto phi = std::acos(std::clamp(cos_phi, 0.0, 1.0));
  return phi / M_PI_2;
}

std::ostream& operator<<(std::ostream& _os, const EngineConfig& _arg)
{
  _os << "Engine Constant: " << _arg.engine_const.first << ", " << _arg.engine_const.second << '\n';
  return _os;
}
}  // namespace tobas
