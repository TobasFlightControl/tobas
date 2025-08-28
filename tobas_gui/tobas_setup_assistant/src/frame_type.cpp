#include "tobas_setup_assistant/frame_type.hpp"

#include <iostream>

#define UNDEFINED "Undefined"
#define PLANAR_MULTICOPTER "Planar Multicopter"
#define NON_PLANAR_MULTICOPTER "Non-Planar Multicopter"
#define ACTIVE_TILT_MULTICOPTER "Active Tilt Multicopter"
#define FIXED_WING "Fixed Wing"

using namespace std;

namespace gui
{
namespace sa
{
string textFromEnum(FrameType arg)
{
  switch (arg) {
    case FrameType::kUndefined:
      return UNDEFINED;
    case FrameType::kPlanarMulticopter:
      return PLANAR_MULTICOPTER;
    case FrameType::kNonPlanarMulticopter:
      return NON_PLANAR_MULTICOPTER;
    case FrameType::kRandomAxisTiltMulticopter:
      return ACTIVE_TILT_MULTICOPTER;
    case FrameType::kFixedWing:
      return FIXED_WING;
    default:
      throw;
  }
}

bool enumFromText(const string& text, FrameType& dst)
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
  else if (text == ACTIVE_TILT_MULTICOPTER) {
    dst = FrameType::kRandomAxisTiltMulticopter;
    return true;
  }
  else if (text == FIXED_WING) {
    dst = FrameType::kFixedWing;
    return true;
  }
  else {
    cerr << "Invalid frame type: " << text << endl;
    return false;
  }
}
}  // namespace sa
}  // namespace gui

namespace YAML
{
Node convert<gui::sa::FrameType>::encode(const gui::sa::FrameType& rhs)
{
  Node node;
  node = gui::sa::textFromEnum(rhs);
  return Node(gui::sa::textFromEnum(rhs));
}

bool convert<gui::sa::FrameType>::decode(const Node& node, gui::sa::FrameType& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return gui::sa::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
