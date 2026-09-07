// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/frame_type.hpp"

#include <iostream>

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

std::string textFromEnum(FrameType arg)
{
  switch (arg) {
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

bool enumFromText(const std::string& text, FrameType& dst)
{
  if (text == kUndefined) {
    dst = FrameType::kUndefined;
    return true;
  }
  else if (text == kPlanarMulticopter) {
    dst = FrameType::kPlanarMulticopter;
    return true;
  }
  else if (text == kNonPlanarMulticopter) {
    dst = FrameType::kNonPlanarMulticopter;
    return true;
  }
  else if (text == kYAxisTiltMulticopter) {
    dst = FrameType::kYAxisTiltMulticopter;
    return true;
  }
  else if (text == kRandomAxisTiltMulticopter) {
    dst = FrameType::kRandomAxisTiltMulticopter;
    return true;
  }
  else if (text == kFixedWing) {
    dst = FrameType::kFixedWing;
    return true;
  }
  else {
    std::cerr << "Invalid frame type: " << text << std::endl;
    return false;
  }
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas

namespace YAML
{
Node convert<tobas::gui::sa::FrameType>::encode(const tobas::gui::sa::FrameType& rhs)
{
  Node node;
  node = tobas::gui::sa::textFromEnum(rhs);
  return Node(tobas::gui::sa::textFromEnum(rhs));
}

bool convert<tobas::gui::sa::FrameType>::decode(const Node& node, tobas::gui::sa::FrameType& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::gui::sa::enumFromText(node.as<std::string>(), rhs);
}
}  // namespace YAML
