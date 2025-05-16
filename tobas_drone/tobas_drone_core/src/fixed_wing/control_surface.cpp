#include "tobas_drone_core/fixed_wing/control_surface.hpp"

#include <tobas_yaml_tools/convert/range.hpp>
#include <tobas_yaml_tools/core.hpp>

using namespace std;

namespace tobas
{
bool ControlSurface::isValid() const
{
  if (link_name.empty()) {
    cerr << "Link name is empty." << endl;
    return false;
  }

  // TODO: ジョイントの範囲をチェック
  // TODO: 安定微係数の符号をチェック

  return true;
}

bool ControlSurface::load(const YAML::Node& node)
{
  if (!yaml::load(kChannelKey, node, channel)) {
    return false;
  }

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

  node[kChannelKey] = channel;
  node[kLinkNameKey] = link_name;
  node[kCLiftDeltaKey] = c_lift_delta;
  node[kCDragAbsDeltaKey] = c_drag_abs_delta;
  node[kCSideDeltaKey] = c_side_delta;
  node[kCRollDeltaKey] = c_roll_delta;
  node[kCPitchDeltaKey] = c_pitch_delta;
  node[kCYawDeltaKey] = c_yaw_delta;

  return node;
}
}  // namespace tobas
