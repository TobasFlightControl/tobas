#include <tobas_yaml_tools/core.hpp>

#include "../include/tobas_drone_core/battery.hpp"

using namespace std;

namespace tobas
{
bool BatteryConfig::isValid() const
{
  if (sag_voltage <= 0)
  {
    cerr << "Sag voltage must be positive." << endl;
    return false;
  }

  if (nominal_voltage <= sag_voltage)
  {
    cerr << "Nominal voltage must be greater than sag voltage." << endl;
    return false;
  }

  if (max_voltage <= nominal_voltage)
  {
    cerr << "Max voltage must be greater than nominal voltage." << endl;
    return false;
  }

  if (max_current <= 0)
  {
    cerr << "Max current must be positive." << endl;
    return false;
  }

  return true;
}

bool BatteryConfig::load(const YAML::Node& node)
{
  if (!yaml::load(kNominalVoltageKey, node, nominal_voltage))
    return false;

  if (!yaml::load(kMaxVoltageKey, node, max_voltage))
    return false;

  if (!yaml::load(kSagVoltageKey, node, sag_voltage))
    return false;

  if (!yaml::load(kMaxCurrentKey, node, max_current))
    return false;

  return true;
}

YAML::Node BatteryConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNominalVoltageKey] = nominal_voltage;
  node[kMaxVoltageKey] = max_voltage;
  node[kSagVoltageKey] = sag_voltage;
  node[kMaxCurrentKey] = max_current;

  return node;
}
}  // namespace tobas
