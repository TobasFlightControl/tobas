#include <tobas_yaml_tools/core.hpp>

#include "tobas_drone_core/pwm.hpp"

using namespace std;

namespace tobas
{
bool PwmConfig::isValid() const
{
  if (name.empty())
  {
    cerr << "PWM name is empty." << endl;
    return false;
  }

  if (min_period > max_period)
  {
    cerr << "Invalid PWM period range." << endl;
    return false;
  }

  if (min_value > max_value)
  {
    cerr << "Invalid PWM value range." << endl;
    return false;
  }

  return true;
}

bool PwmConfig::load(const YAML::Node& node)
{
  if (!yaml::load(kChannelKey, node, channel))
    return false;

  if (!yaml::load(kNameKey, node, name))
    return false;

  if (!yaml::load(kMinPeriodKey, node, min_period))
    return false;

  if (!yaml::load(kMaxPeriodKey, node, max_period))
    return false;

  if (!yaml::load(kMinValueKey, node, min_value))
    return false;

  if (!yaml::load(kMaxValueKey, node, max_value))
    return false;

  if (!yaml::load(kReverseKey, node, reverse))
    return false;

  return true;
}

YAML::Node PwmConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kChannelKey] = channel;
  node[kNameKey] = name;
  node[kMinPeriodKey] = min_period;
  node[kMaxPeriodKey] = max_period;
  node[kMinValueKey] = min_value;
  node[kMaxValueKey] = max_value;
  node[kReverseKey] = reverse;

  return node;
}
}  // namespace tobas
