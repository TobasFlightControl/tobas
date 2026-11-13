// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/type.hpp"

namespace tobas
{
namespace
{
constexpr char kElectricText[] = "electric";
constexpr char kIceText[] = "ice";
}  // namespace

std::string textFromEnum(PropulsionSystem _cmd_iface)
{
  switch (_cmd_iface) {
    case PropulsionSystem::kElectric:
      return kElectricText;
    case PropulsionSystem::kIce:
      return kIceText;
    default:
      throw;
  }
}

bool enumFromText(const std::string& _text, PropulsionSystem& _dst)
{
  if (_text == kElectricText) {
    _dst = PropulsionSystem::kElectric;
    return true;
  }
  else if (_text == kIceText) {
    _dst = PropulsionSystem::kIce;
    return true;
  }
  else {
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::PropulsionSystem>::encode(tobas::PropulsionSystem _rhs)
{
  Node node;
  node = tobas::textFromEnum(_rhs);
  return Node(tobas::textFromEnum(_rhs));
}

bool convert<tobas::PropulsionSystem>::decode(const Node& _node, tobas::PropulsionSystem& _rhs)
{
  if (!_node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(_node.as<std::string>(), _rhs);
}
}  // namespace YAML
