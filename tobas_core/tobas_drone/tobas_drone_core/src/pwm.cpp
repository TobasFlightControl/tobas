#include "tobas_drone_core/pwm.hpp"

#include <tobas_yaml_tools/core.hpp>

using namespace std;

namespace tobas
{
bool PwmConfig::isValid() const
{
  if (name.empty()) {
    cerr << "PWM name is empty." << endl;
    return false;
  }

  if (period_range.first <= 0. || period_range.second <= 0.) {
    cerr << "PWM period range of \"" << name << "\" must be positive." << endl;
    return false;
  }

  return true;
}

bool PwmConfig::load(const YAML::Node& node)
{
  if (!yaml::load(kChannelKey, node, channel)) {
    return false;
  }

  if (!yaml::load(kNameKey, node, name)) {
    return false;
  }

  if (!yaml::load(kPeriodRangeKey, node, period_range)) {
    return false;
  }

  if (!yaml::load(kValueRangeKey, node, value_range)) {
    return false;
  }

  return true;
}

YAML::Node PwmConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kChannelKey] = channel;
  node[kNameKey] = name;
  node[kPeriodRangeKey] = period_range;
  node[kValueRangeKey] = value_range;

  return node;
}
}  // namespace tobas
