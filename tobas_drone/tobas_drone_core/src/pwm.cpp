#include <tobas_yaml_tools/core.hpp>

#include "tobas_drone_core/pwm.hpp"

using namespace std;

namespace tobas
{
bool PwmConfig::isValid() const
{
  if (joint_name.empty())
  {
    cerr << "Joint name is empty." << endl;
    return false;
  }

  if (min_period > max_period)
  {
    cerr << "Invalid PWM period range." << endl;
    return false;
  }

  if (min_angle > max_angle)
  {
    cerr << "Invalid PWM angle range." << endl;
    return false;
  }

  return true;
}

bool PwmConfig::load(const YAML::Node& node)
{
  if (!yaml::load(kChannelKey, node, channel))
    return false;

  if (!yaml::load(kJointNameKey, node, joint_name))
    return false;

  if (!yaml::load(kMinPeriodKey, node, min_period))
    return false;

  if (!yaml::load(kMaxPeriodKey, node, max_period))
    return false;

  if (!yaml::load(kMinAngleKey, node, min_angle))
    return false;

  if (!yaml::load(kMaxAngleKey, node, max_angle))
    return false;

  if (!yaml::load(kReverseKey, node, reverse))
    return false;

  return true;
}

YAML::Node PwmConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kChannelKey] = channel;
  node[kJointNameKey] = joint_name;
  node[kMinPeriodKey] = min_period;
  node[kMaxPeriodKey] = max_period;
  node[kMinAngleKey] = min_angle;
  node[kMaxAngleKey] = max_angle;
  node[kReverseKey] = reverse;

  return node;
}
}  // namespace tobas
