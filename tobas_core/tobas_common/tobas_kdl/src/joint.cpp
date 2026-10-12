// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/joint.hpp"

namespace tobas
{
namespace kdl
{
std::expected<void, std::string> Joint::validate() const
{
  if (name.empty()) {
    return std::unexpected("Joint name is empty.");
  }

  return {};
}

const char* Joint::typeToText(JointType type)
{
  switch (type) {
    case kRotation:
      return "Rotation";
    case kTranslation:
      return "Translation";
    case kFixed:
      return "Fixed";
    default:
      throw;
  }
}

std::ostream& operator<<(std::ostream& os, const Joint& arg)
{
  os << "Name: " << arg.name << std::endl;
  os << "Type: " << Joint::typeToText(arg.type) << std::endl;
  os << "Origin: " << arg.origin << std::endl;
  os << "Axis: " << arg.axis() << std::endl;
  os << "Damping: " << arg.damping << std::endl;
  os << "Friction: " << arg.friction << std::endl;
  os << "Lower Limit: " << arg.lower_limit << std::endl;
  os << "Upper Limit: " << arg.upper_limit << std::endl;
  os << "Max Effort: " << arg.max_effort << std::endl;
  os << "Max Velocity: " << arg.max_velocity;
  return os;
}
}  // namespace kdl
}  // namespace tobas
