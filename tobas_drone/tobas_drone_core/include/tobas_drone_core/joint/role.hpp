#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum struct JointRole
{
  kTiltJoint,
  kControlSurface,
  kLandingGear,
  kPassiveWheel,
  kManipulation,
  kOther,
};

std::string textFromEnum(JointRole role);
bool enumFromText(const std::string& text, JointRole& dst);

bool isServoJoint(JointRole role);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::JointRole>
{
  static Node encode(const tobas::JointRole& rhs);
  static bool decode(const Node& node, tobas::JointRole& rhs);
};
}  // namespace YAML
