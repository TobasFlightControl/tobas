#include "tobas_drone_core/propulsion_system/electric_propulsion_system/electric_rotor.hpp"

#include <tobas_yaml_tools/core.hpp>

using namespace std;

namespace tobas
{
bool ElectricRotorConfig::isValid() const
{
  if (!super::isValid()) {
    return false;
  }

  if (num_poles <= 0) {
    cerr << "The number of poles must be positive." << endl;
    return false;
  }

  if (num_poles % 2 != 0) {
    cerr << "The number of poles must be even." << endl;
    return false;
  }

  if (kv <= 0) {
    cerr << "Kv value must be positive." << endl;
    return false;
  }

  if (internal_resistance <= 0) {
    cerr << "Internal resistance must be positive." << endl;
    return false;
  }

  if (propeller_diameter <= 0) {
    cerr << "Propeller diameter must be positive." << endl;
    return false;
  }

  if (motor_const <= 0) {
    cerr << "Motor constant must be positive." << endl;
    return false;
  }

  return true;
}

bool ElectricRotorConfig::load(const YAML::Node& node)
{
  if (!super::load(node)) {
    return false;
  }

  if (!yaml::load(kChannelKey, node, channel)) {
    return false;
  }

  if (!yaml::load(kNumPolesKey, node, num_poles)) {
    return false;
  }

  if (!yaml::load(kKvKey, node, kv)) {
    return false;
  }

  if (!yaml::load(kInternalResistanceKey, node, internal_resistance)) {
    return false;
  }

  if (!yaml::load(kPropellerDiameterKey, node, propeller_diameter)) {
    return false;
  }

  if (!yaml::load(kMotorConstKey, node, motor_const)) {
    return false;
  }

  return true;
}

YAML::Node ElectricRotorConfig::dump() const
{
  auto node = super::dump();

  node[kChannelKey] = channel;
  node[kNumPolesKey] = num_poles;
  node[kKvKey] = kv;
  node[kInternalResistanceKey] = internal_resistance;
  node[kPropellerDiameterKey] = propeller_diameter;
  node[kMotorConstKey] = motor_const;

  return node;
}
}  // namespace tobas
