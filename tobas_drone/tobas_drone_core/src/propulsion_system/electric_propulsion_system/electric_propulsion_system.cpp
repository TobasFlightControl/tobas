#include "tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp"

#include <iostream>

using namespace std;

namespace tobas
{
bool ElectricPropulsionSystemConfig::isValid() const
{
  // Rotors
  for (const auto& [_, rotor] : rotors) {
    if (!rotor->isValid()) {
      cerr << "The configurations of rotor \"" << rotor->link_name << "\" are invalid." << endl;
      return false;
    }
  }

  // Battery
  if (!battery.isValid()) {
    return false;
  }

  return true;
}

bool ElectricPropulsionSystemConfig::load(const YAML::Node& node)
{
  // Rotors
  rotors.clear();
  if (!node[kRotorsKey].IsSequence()) {
    cerr << "Rotors field is not defined." << endl;
    return false;
  }
  for (const auto& rotor_node : node[kRotorsKey]) {
    const auto rotor = make_shared<ElectricRotorConfig>();
    if (!rotor->load(rotor_node)) {
      cerr << "Failed to load the configurations of rotors." << endl;
      return false;
    }
    rotors[rotor->link_name] = rotor;
  }

  // Battery
  if (!node[kBatteryKey].IsDefined()) {
    cerr << "Battery field is not defined." << endl;
    return false;
  }
  if (!battery.load(node[kBatteryKey])) {
    cerr << "Failed to load the configurations of battery." << endl;
    return false;
  }

  return true;
}

YAML::Node ElectricPropulsionSystemConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  // Rotors
  node[kRotorsKey] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto& [_, rotor] : rotors) {
    node[kRotorsKey].push_back(rotor->dump());
  }

  // Battery
  node[kBatteryKey] = battery.dump();

  return node;
}

propulsion_system_t ElectricPropulsionSystemConfig::type() const
{
  return propulsion_system_t::ELECTRIC;
}

double ElectricPropulsionSystemConfig::minSpeed(const std::string& link_name) const
{
  // TODO: モータやプロペラのパラメータから最小回転数を決定
  (void)link_name;
  return kMinSpeed;
}

double ElectricPropulsionSystemConfig::maxSpeed(const std::string& link_name) const
{
  // FIXME: 電動またはICEの誤差を吸収しつつバッテリーまたはエンジンの状態を考慮した最大推力を反映したい．
  // PropulsionLimitCalculatorから回転数や推力の最大値をトピックで発行するのが良さそう．
  const auto rotor = getRotor(link_name);
  return rotor->speedFromVoltage(battery.nominal_voltage);
}

double ElectricPropulsionSystemConfig::minThrust(const std::string& link_name) const
{
  const auto rotor = getRotor(link_name);
  return rotor->thrustFromSpeed(kMinSpeed);
}

double ElectricPropulsionSystemConfig::maxThrust(const std::string& link_name) const
{
  const auto rotor = getRotor(link_name);
  return rotor->thrustFromSpeed(maxSpeed(link_name));
}
}  // namespace tobas
