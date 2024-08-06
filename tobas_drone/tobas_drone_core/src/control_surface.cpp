#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/convert/range.hpp>

#include "../include/tobas_drone_core/control_surface.hpp"

using namespace std;

namespace tobas
{
bool ControlSurface::isValid() const
{
  if (!angle_limit.isValid())
  {
    cerr << "The angle limit is invalid." << endl;
    return false;
  }

  if (!angle_limit.inRange(0))
  {
    cerr << "The angle limit must include 0." << endl;
    return false;
  }

  if (max_angle_rate <= 0)
  {
    cerr << "The maximum angle rate must be positive." << endl;
    return false;
  }

  return true;
}

bool ControlSurface::load(const YAML::Node& node)
{
  if (!yaml::load(kChannelKey, node, channel))
    return false;

  if (!yaml::load(kJointNameKey, node, joint_name))
    return false;

  if (!yaml::load(kAngleLimitKey, node, angle_limit))
    return false;

  if (!yaml::load(kMaxAngleRateMKey, node, max_angle_rate))
    return false;

  if (!yaml::load(kCLiftDeltaKey, node, c_lift_delta))
    return false;

  if (!yaml::load(kCDragAbsDeltaKey, node, c_drag_abs_delta))
    return false;

  if (!yaml::load(kCSideDeltaKey, node, c_side_delta))
    return false;

  if (!yaml::load(kCRollDeltaKey, node, c_roll_delta))
    return false;

  if (!yaml::load(kCPitchDeltaKey, node, c_pitch_delta))
    return false;

  if (!yaml::load(kCYawDeltaKey, node, c_yaw_delta))
    return false;

  return true;
}

YAML::Node ControlSurface::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kChannelKey] = channel;
  node[kJointNameKey] = joint_name;
  node[kAngleLimitKey] = angle_limit;
  node[kMaxAngleRateMKey] = max_angle_rate;
  node[kCLiftDeltaKey] = c_lift_delta;
  node[kCDragAbsDeltaKey] = c_drag_abs_delta;
  node[kCSideDeltaKey] = c_side_delta;
  node[kCRollDeltaKey] = c_roll_delta;
  node[kCPitchDeltaKey] = c_pitch_delta;
  node[kCYawDeltaKey] = c_yaw_delta;

  return node;
}
}  // namespace tobas
