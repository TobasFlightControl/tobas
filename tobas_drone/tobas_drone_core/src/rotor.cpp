#include <tobas_yaml_tools/core.hpp>

#include "../include/tobas_drone_core/rotor.hpp"

using namespace std;

namespace tobas
{
bool RotorConfig::isValid() const
{
  if (num_poles <= 0)
  {
    cerr << "The number of poles must be positive." << endl;
    return false;
  }

  if (num_poles % 2 != 0)
  {
    cerr << "The number of poles must be even." << endl;
    return false;
  }

  if (max_rot_speed <= 0)
  {
    cerr << "Maximum rotating speed must be positive." << endl;
    return false;
  }

  if (motor_constant <= 0)
  {
    cerr << "Motor constant must be positive." << endl;
    return false;
  }

  if (moment_constant <= 0)
  {
    cerr << "Moment constant must be positive." << endl;
    return false;
  }

  if (drag_constant <= 0)
  {
    cerr << "Drag constant must be positive." << endl;
    return false;
  }

  if (rot_speed_coefs.first <= 0)
  {
    cerr << "The first term of the rotating speed coefficients must be positive." << endl;
    return false;
  }

  if (rot_speed_coefs.second < 0)
  {
    cerr << "The second term of the rotating speed coefficients must be non-negative." << endl;
    return false;
  }

  return true;
}

bool RotorConfig::load(const YAML::Node& node)
{
  if (!yaml::load(kChannelKey, node, channel))
    return false;

  if (!yaml::load(kLinkNameKey, node, link_name))
    return false;

  if (!yaml::load(kDirectionKey, node, direction))
    return false;

  if (!yaml::load(kAxisKey, node, axis))
    return false;

  if (!yaml::load(kEscModeKey, node, esc_mode))
    return false;

  if (!yaml::load(kNumPolesKey, node, num_poles))
    return false;

  if (!yaml::load(kMaxRotSpeedKey, node, max_rot_speed))
    return false;

  if (!yaml::load(kMotorConstKey, node, motor_constant))
    return false;

  if (!yaml::load(kMomentConstKey, node, moment_constant))
    return false;

  if (!yaml::load(kRotSpeedCoefKey, node, rot_speed_coefs))
    return false;

  return true;
}

YAML::Node RotorConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kChannelKey] = channel;
  node[kLinkNameKey] = link_name;
  node[kDirectionKey] = direction;
  node[kAxisKey] = axis;
  node[kEscModeKey] = esc_mode;
  node[kNumPolesKey] = num_poles;
  node[kMaxRotSpeedKey] = max_rot_speed;
  node[kMotorConstKey] = motor_constant;
  node[kMomentConstKey] = moment_constant;
  node[kRotSpeedCoefKey] = rot_speed_coefs;

  return node;
}
}  // namespace tobas
