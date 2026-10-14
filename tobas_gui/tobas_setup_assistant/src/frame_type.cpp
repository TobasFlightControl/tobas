// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/frame_type.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace
{
constexpr char kUndefined[] = "Undefined";
constexpr char kPlanarMulticopter[] = "Planar Multicopter";
constexpr char kNonPlanarMulticopter[] = "Non-Planar Multicopter";
constexpr char kYAxisTiltMulticopter[] = "Y Axis Tilt Multicopter";
constexpr char kRandomAxisTiltMulticopter[] = "Random Axis Tilt Multicopter";
constexpr char kFixedWing[] = "Fixed Wing";
}  // namespace

std::string textFromEnum(FrameType _arg)
{
  switch (_arg) {
    case FrameType::kUndefined:
      return kUndefined;
    case FrameType::kPlanarMulticopter:
      return kPlanarMulticopter;
    case FrameType::kNonPlanarMulticopter:
      return kNonPlanarMulticopter;
    case FrameType::kYAxisTiltMulticopter:
      return kYAxisTiltMulticopter;
    case FrameType::kRandomAxisTiltMulticopter:
      return kRandomAxisTiltMulticopter;
    case FrameType::kFixedWing:
      return kFixedWing;
    default:
      throw;
  }
}

bool enumFromText(const std::string& _text, FrameType& _dst)
{
  if (_text == kUndefined) {
    _dst = FrameType::kUndefined;
    return true;
  }
  else if (_text == kPlanarMulticopter) {
    _dst = FrameType::kPlanarMulticopter;
    return true;
  }
  else if (_text == kNonPlanarMulticopter) {
    _dst = FrameType::kNonPlanarMulticopter;
    return true;
  }
  else if (_text == kYAxisTiltMulticopter) {
    _dst = FrameType::kYAxisTiltMulticopter;
    return true;
  }
  else if (_text == kRandomAxisTiltMulticopter) {
    _dst = FrameType::kRandomAxisTiltMulticopter;
    return true;
  }
  else if (_text == kFixedWing) {
    _dst = FrameType::kFixedWing;
    return true;
  }
  else {
    return false;
  }
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas

namespace YAML
{
Node convert<tobas::gui::sa::FrameType>::encode(tobas::gui::sa::FrameType _rhs)
{
  Node node;
  node = tobas::gui::sa::textFromEnum(_rhs);
  return Node(tobas::gui::sa::textFromEnum(_rhs));
}

bool convert<tobas::gui::sa::FrameType>::decode(const Node& _node, tobas::gui::sa::FrameType& _rhs)
{
  if (!_node.IsScalar()) {
    return false;
  }

  return tobas::gui::sa::enumFromText(_node.as<std::string>(), _rhs);
}
}  // namespace YAML
