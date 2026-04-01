// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/frame_type.hpp"

#include <iostream>

#define UNDEFINED "Undefined"
#define PLANAR_MULTICOPTER "Planar Multicopter"
#define NON_PLANAR_MULTICOPTER "Non-Planar Multicopter"
#define Y_AXIS_TILT_MULTICOPTER "Y Axis Tilt Multicopter"
#define RANDOM_AXIS_TILT_MULTICOPTER "Random Axis Tilt Multicopter"
#define FIXED_WING "Fixed Wing"

namespace tobas
{
namespace gui
{
namespace sa
{
std::string textFromEnum(FrameType arg)
{
  switch (arg) {
    case FrameType::kUndefined:
      return UNDEFINED;
    case FrameType::kPlanarMulticopter:
      return PLANAR_MULTICOPTER;
    case FrameType::kNonPlanarMulticopter:
      return NON_PLANAR_MULTICOPTER;
    case FrameType::kYAxisTiltMulticopter:
      return Y_AXIS_TILT_MULTICOPTER;
    case FrameType::kRandomAxisTiltMulticopter:
      return RANDOM_AXIS_TILT_MULTICOPTER;
    case FrameType::kFixedWing:
      return FIXED_WING;
    default:
      throw;
  }
}

bool enumFromText(const std::string& text, FrameType& dst)
{
  if (text == UNDEFINED) {
    dst = FrameType::kUndefined;
    return true;
  }
  else if (text == PLANAR_MULTICOPTER) {
    dst = FrameType::kPlanarMulticopter;
    return true;
  }
  else if (text == NON_PLANAR_MULTICOPTER) {
    dst = FrameType::kNonPlanarMulticopter;
    return true;
  }
  else if (text == Y_AXIS_TILT_MULTICOPTER) {
    dst = FrameType::kYAxisTiltMulticopter;
    return true;
  }
  else if (text == RANDOM_AXIS_TILT_MULTICOPTER) {
    dst = FrameType::kRandomAxisTiltMulticopter;
    return true;
  }
  else if (text == FIXED_WING) {
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
