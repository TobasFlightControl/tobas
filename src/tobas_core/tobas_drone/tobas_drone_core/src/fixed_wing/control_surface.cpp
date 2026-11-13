// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/fixed_wing/control_surface.hpp"

#include <tobas_yaml_tools/convert/range.hpp>
#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas
{
namespace
{
constexpr char kLinkNameKey[] = "link_name";
constexpr char kCLiftDeltaKey[] = "c_lift_delta";
constexpr char kCDragAbsDeltaKey[] = "c_drag_abs_delta";
constexpr char kCSideDeltaKey[] = "c_side_delta";
constexpr char kCRollDeltaKey[] = "c_roll_delta";
constexpr char kCPitchDeltaKey[] = "c_pitch_delta";
constexpr char kCYawDeltaKey[] = "c_yaw_delta";
}  // namespace

std::expected<void, std::string> ControlSurface::validate() const
{
  if (link_name.empty()) {
    return std::unexpected("Link name is empty.");
  }

  // TODO: Check the joint range.
  // TODO: Check the signs of stability derivatives.

  return {};
}

std::expected<void, std::string> ControlSurface::load(const YAML::Node& _node)
{
  if (!_node.IsDefined() || !_node.IsMap()) {
    return std::unexpected("Configuration node must be a map.");
  }

  if (const auto result = yaml::load(kLinkNameKey, _node, link_name); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCLiftDeltaKey, _node, c_lift_delta); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCDragAbsDeltaKey, _node, c_drag_abs_delta); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCSideDeltaKey, _node, c_side_delta); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCRollDeltaKey, _node, c_roll_delta); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCPitchDeltaKey, _node, c_pitch_delta); !result) {
    return result;
  }

  if (const auto result = yaml::load(kCYawDeltaKey, _node, c_yaw_delta); !result) {
    return result;
  }

  return {};
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
