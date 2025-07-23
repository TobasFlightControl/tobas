#pragma once

#include <yaml-cpp/yaml.h>

namespace gui
{
namespace sa
{
enum struct FrameType
{
  kUndefined,
  kPlanarMulticopter,
  kNonPlanarMulticopter,
  kActiveTiltMulticopter,
  kFixedWing,
};

std::string textFromEnum(FrameType arg);
bool enumFromText(const std::string& text, FrameType& dst);
}  // namespace sa
}  // namespace gui

namespace YAML
{
template <>
struct convert<gui::sa::FrameType>
{
  static Node encode(const gui::sa::FrameType& rhs);
  static bool decode(const Node& node, gui::sa::FrameType& rhs);
};
}  // namespace YAML
