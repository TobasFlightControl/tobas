// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/turning_direction.hpp"

namespace tobas
{
namespace
{
constexpr char kCcwText[] = "ccw";
constexpr char kCwText[] = "cw";
}  // namespace

std::string textFromEnum(TurningDirection _cmd_iface)
{
  switch (_cmd_iface) {
    case TurningDirection::CCW:
      return kCcwText;
    case TurningDirection::CW:
      return kCwText;
    default:
      throw;
  }
}

bool enumFromText(const std::string& _text, TurningDirection& _dst)
{
  if (_text == kCcwText) {
    _dst = TurningDirection::CCW;
    return true;
  }
  else if (_text == kCwText) {
    _dst = TurningDirection::CW;
    return true;
  }
  else {
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::TurningDirection>::encode(tobas::TurningDirection _rhs)
{
  Node node;
  node = tobas::textFromEnum(_rhs);
  return Node(tobas::textFromEnum(_rhs));
}

bool convert<tobas::TurningDirection>::decode(const Node& _node, tobas::TurningDirection& _rhs)
{
  if (!_node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(_node.as<std::string>(), _rhs);
}
}  // namespace YAML
