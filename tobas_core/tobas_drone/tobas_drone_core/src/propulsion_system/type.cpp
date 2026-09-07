// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/type.hpp"

#include <iostream>

using namespace std;

namespace tobas
{
namespace
{
constexpr char kElectricText[] = "electric";
constexpr char kIceText[] = "ice";
}  // namespace

string textFromEnum(PropulsionSystem cmd_iface)
{
  switch (cmd_iface) {
    case PropulsionSystem::kElectric:
      return kElectricText;
    case PropulsionSystem::kIce:
      return kIceText;
    default:
      throw;
  }
}

bool enumFromText(const string& text, PropulsionSystem& dst)
{
  if (text == kElectricText) {
    dst = PropulsionSystem::kElectric;
    return true;
  }
  else if (text == kIceText) {
    dst = PropulsionSystem::kIce;
    return true;
  }
  else {
    cerr << "Invalid propulsion system type: " << text << endl;
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::PropulsionSystem>::encode(const tobas::PropulsionSystem& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::PropulsionSystem>::decode(const Node& node, tobas::PropulsionSystem& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
