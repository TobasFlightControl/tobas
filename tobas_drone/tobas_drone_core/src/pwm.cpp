#include <tobas_math/core.hpp>
#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/convert/range.hpp>

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

  if (!period_range.isValid())
  {
    cerr << "Invalid PWM period range." << endl;
    return false;
  }

  if (!value_range.isValid())
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

  if (!yaml::load(kPeriodRangeKey, node, period_range))
    return false;

  if (!yaml::load(kValueRangeKey, node, value_range))
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
  node[kPeriodRangeKey] = period_range;
  node[kValueRangeKey] = value_range;
  node[kReverseKey] = reverse;

  return node;
}

uint16_t PwmConfig::periodFromValue(double value) const
{
  uint16_t period;
  if (reverse)
    period = math::remap<double>(value, value_range.lower, value_range.upper, period_range.upper, period_range.lower);
  else
    period = math::remap<double>(value, value_range.lower, value_range.upper, period_range.lower, period_range.upper);

  return period_range.clamp(period);
}

double PwmConfig::valueFromPeriod(uint16_t period) const
{
  period = period_range.clamp(period);

  if (reverse)
    return math::remap<double>(period, period_range.lower, period_range.upper, value_range.upper, value_range.lower);
  else
    return math::remap<double>(period, period_range.lower, period_range.upper, value_range.lower, value_range.upper);
}
}  // namespace tobas
