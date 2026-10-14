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

const char* Joint::typeToText(JointType _type)
{
  switch (_type) {
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

std::ostream& operator<<(std::ostream& _os, const Joint& _arg)
{
  _os << "Name: " << _arg.name << '\n';
  _os << "Type: " << Joint::typeToText(_arg.type) << '\n';
  _os << "Origin: " << _arg.origin << '\n';
  _os << "Axis: " << _arg.axis() << '\n';
  _os << "Damping: " << _arg.damping << '\n';
  _os << "Friction: " << _arg.friction << '\n';
  _os << "Lower Limit: " << _arg.lower_limit << '\n';
  _os << "Upper Limit: " << _arg.upper_limit << '\n';
  _os << "Max Effort: " << _arg.max_effort << '\n';
  _os << "Max Velocity: " << _arg.max_velocity;
  return _os;
}
}  // namespace kdl
}  // namespace tobas
