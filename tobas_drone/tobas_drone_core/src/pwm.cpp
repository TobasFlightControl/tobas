#include "tobas_drone_core/pwm.hpp"

#include <tobas_math/core.hpp>
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

uint16_t PwmConfig::periodFromValue(double value) const
{
  const auto period =
    math::remap<double>(value, value_range.first, value_range.second, period_range.first, period_range.second);
  return clampPeriod(period);
}

double PwmConfig::valueFromPeriod(uint16_t period) const
{
  period = clampPeriod(period);
  return math::remap<double>(period, period_range.first, period_range.second, value_range.first, value_range.second);
}

uint16_t PwmConfig::clampPeriod(uint16_t period) const
{
  if (period_range.first < period_range.second) {
    return clamp(period, period_range.first, period_range.second);
  }
  else {
    return clamp(period, period_range.second, period_range.first);
  }
}
}  // namespace tobas
