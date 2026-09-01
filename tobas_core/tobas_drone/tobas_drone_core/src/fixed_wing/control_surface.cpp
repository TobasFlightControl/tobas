// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/fixed_wing/control_surface.hpp"

#include <tobas_yaml_tools/convert/range.hpp>
#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/format.hpp>

using namespace std;

namespace tobas
{
namespace
{
constexpr char kChannelKey[] = "channel";
constexpr char kLinkNameKey[] = "link_name";
constexpr char kCLiftDeltaKey[] = "c_lift_delta";
constexpr char kCDragAbsDeltaKey[] = "c_drag_abs_delta";
constexpr char kCSideDeltaKey[] = "c_side_delta";
constexpr char kCRollDeltaKey[] = "c_roll_delta";
constexpr char kCPitchDeltaKey[] = "c_pitch_delta";
constexpr char kCYawDeltaKey[] = "c_yaw_delta";
}  // namespace

bool ControlSurface::isValid() const
{
  if (link_name.empty()) {
    cerr << "Link name is empty." << endl;
    return false;
  }

  // TODO: Check the joint range.
  // TODO: Check the signs of stability derivatives.

  return true;
}

bool ControlSurface::load(const YAML::Node& node)
{
  if (!yaml::load(kLinkNameKey, node, link_name)) {
    return false;
  }

  if (!yaml::load(kCLiftDeltaKey, node, c_lift_delta)) {
    return false;
  }

  if (!yaml::load(kCDragAbsDeltaKey, node, c_drag_abs_delta)) {
    return false;
  }

  if (!yaml::load(kCSideDeltaKey, node, c_side_delta)) {
    return false;
  }

  if (!yaml::load(kCRollDeltaKey, node, c_roll_delta)) {
    return false;
  }

  if (!yaml::load(kCPitchDeltaKey, node, c_pitch_delta)) {
    return false;
  }

  if (!yaml::load(kCYawDeltaKey, node, c_yaw_delta)) {
    return false;
  }

  return true;
}

YAML::Node ControlSurface::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kLinkNameKey] = link_name;
  node[kCLiftDeltaKey] = yaml::format(c_lift_delta);
  node[kCDragAbsDeltaKey] = yaml::format(c_drag_abs_delta);
  node[kCSideDeltaKey] = yaml::format(c_side_delta);
  node[kCRollDeltaKey] = yaml::format(c_roll_delta);
  node[kCPitchDeltaKey] = yaml::format(c_pitch_delta);
  node[kCYawDeltaKey] = yaml::format(c_yaw_delta);

  return node;
}
}  // namespace tobas
