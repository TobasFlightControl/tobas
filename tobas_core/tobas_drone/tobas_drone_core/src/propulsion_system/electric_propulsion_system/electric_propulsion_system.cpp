// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp"

#include <iostream>

#include <boost/polymorphic_pointer_cast.hpp>

using namespace std;

namespace tobas
{
namespace
{
constexpr char kBatteryKey[] = "battery";
constexpr char kRotorsKey[] = "rotors";
}  // namespace

bool ElectricPropulsionSystemConfig::isValid() const
{
  // Rotors
  for (const auto& [_, rotor] : rotors) {
    if (!rotor->isValid()) {
      cerr << "The configuration of rotor \"" << rotor->link_name << "\" is invalid." << endl;
      return false;
    }
  }

  // Battery
  if (!battery.isValid()) {
    return false;
  }

  return true;
}

bool ElectricPropulsionSystemConfig::load(const YAML::Node& root_node)
{
  clear();

  // Rotors
  const auto rotors_node = root_node[kRotorsKey];
  if (!rotors_node.IsDefined()) {
    cerr << "\"" << kRotorsKey << "\" is not defined." << endl;
    return false;
  }
  if (!rotors_node.IsSequence()) {
    cerr << "\"" << kRotorsKey << "\" must be a sequence." << endl;
    return false;
  }
  for (const auto& rotor_node : rotors_node) {
    const auto rotor = make_shared<ElectricRotorConfig>();
    if (!rotor->load(rotor_node)) {
      cerr << "Failed to load the configuration of rotors." << endl;
      return false;
    }
    rotors[rotor->link_name] = rotor;
  }

  // Battery
  const auto battery_node = root_node[kBatteryKey];
  if (!battery_node.IsDefined()) {
    cerr << "\"" << kBatteryKey << "\" is not defined." << endl;
    return false;
  }
  if (!battery.load(battery_node)) {
    cerr << "Failed to load the configuration of battery." << endl;
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

PropulsionSystem ElectricPropulsionSystemConfig::type() const
{
  return PropulsionSystem::kElectric;
}

double ElectricPropulsionSystemConfig::minSpeed(const string& link_name)
{
  const auto rotor = getRotor(link_name);
  return rotor->min_speed;
}

double ElectricPropulsionSystemConfig::maxSpeed(const string& link_name)
{
  // FIXME: Reflect the maximum thrust while absorbing errors between electric and ICE models
  // and considering the battery or engine state.
  // It may be better for `PropulsionLimitCalculator` to publish maximum speed and thrust values as topics.
  const auto rotor = getRotor(link_name);
  return rotor->speedFromVoltage(battery.nominal_voltage);
}

double ElectricPropulsionSystemConfig::minThrust(const string& link_name)
{
  const auto rotor = getRotor(link_name);
  return rotor->thrustFromSpeed(minSpeed(link_name));
}

double ElectricPropulsionSystemConfig::maxThrust(const string& link_name)
{
  const auto rotor = getRotor(link_name);
  return rotor->thrustFromSpeed(maxSpeed(link_name));
}

double ElectricPropulsionSystemConfig::thrustFromThrottle(const std::string& link_name, double throttle)
{
  const auto rotor = getRotor(link_name);
  const auto input_voltage = battery.nominal_voltage * throttle;
  return rotor->thrustFromVoltage(input_voltage);
}

ElectricRotorConfig::SharedPtr ElectricPropulsionSystemConfig::getRotor(const std::string& link_name)
{
  const auto it = rotors.find(link_name);
  if (it == rotors.end()) {
    cerr << "Electric rotor link \"" << link_name << "\" is not found." << endl;
    return nullptr;
  }
  return boost::polymorphic_pointer_downcast<ElectricRotorConfig>(it->second);
}

ElectricRotorConfig::ConstSharedPtr ElectricPropulsionSystemConfig::getRotor(const std::string& link_name) const
{
  const auto it = rotors.find(link_name);
  if (it == rotors.end()) {
    cerr << "Electric rotor link \"" << link_name << "\" is not found." << endl;
    return nullptr;
  }
  return boost::polymorphic_pointer_downcast<ElectricRotorConfig>(it->second);
}
}  // namespace tobas
