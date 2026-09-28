// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp"

#include <iostream>
#include <memory>

namespace tobas
{
namespace
{
constexpr char kBatteryKey[] = "battery";
constexpr char kRotorsKey[] = "rotors";
}  // namespace

std::expected<void, std::string> ElectricPropulsionSystemConfig::validate() const
{
  for (const auto& [link_name, rotor] : rotors) {
    if (!rotor) {
      return std::unexpected("Rotor '" + link_name + "': configuration is missing.");
    }
    if (const auto result = rotor->validate(); !result) {
      return std::unexpected("Rotor '" + link_name + "': " + result.error());
    }
  }

  return battery.validate();
}

bool ElectricPropulsionSystemConfig::load(const YAML::Node& root_node)
{
  clear();

  // Rotors
  const auto rotors_node = root_node[kRotorsKey];
  if (!rotors_node.IsDefined()) {
    std::cerr << "'" << kRotorsKey << "' is not defined." << std::endl;
    return false;
  }
  if (!rotors_node.IsSequence()) {
    std::cerr << "'" << kRotorsKey << "' must be a sequence." << std::endl;
    return false;
  }
  for (const auto& rotor_node : rotors_node) {
    const auto rotor = std::make_shared<ElectricRotorConfig>();
    if (!rotor->load(rotor_node)) {
      std::cerr << "Failed to load the configuration of rotors." << std::endl;
      return false;
    }
    rotors[rotor->link_name] = rotor;
  }

  // Battery
  const auto battery_node = root_node[kBatteryKey];
  if (!battery_node.IsDefined()) {
    std::cerr << "'" << kBatteryKey << "' is not defined." << std::endl;
    return false;
  }
  if (!battery.load(battery_node)) {
    std::cerr << "Failed to load the configuration of battery." << std::endl;
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

double ElectricPropulsionSystemConfig::minSpeed(const std::string& link_name)
{
  const auto rotor = getRotor(link_name);
  return rotor->min_speed;
}

double ElectricPropulsionSystemConfig::maxSpeed(const std::string& link_name)
{
  // FIXME: Reflect the maximum thrust while absorbing errors between electric and ICE models
  // and considering the battery or engine state.
  // It may be better for `PropulsionLimitCalculator` to publish maximum speed and thrust values as topics.
  const auto rotor = getRotor(link_name);
  return rotor->speedFromVoltage(battery.nominal_voltage);
}

double ElectricPropulsionSystemConfig::minThrust(const std::string& link_name)
{
  const auto rotor = getRotor(link_name);
  return rotor->thrustFromSpeed(minSpeed(link_name));
}

double ElectricPropulsionSystemConfig::maxThrust(const std::string& link_name)
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
    std::cerr << "Electric rotor link '" << link_name << "' is not found." << std::endl;
    return nullptr;
  }
  return std::static_pointer_cast<ElectricRotorConfig>(it->second);
}

ElectricRotorConfig::ConstSharedPtr ElectricPropulsionSystemConfig::getRotor(const std::string& link_name) const
{
  const auto it = rotors.find(link_name);
  if (it == rotors.end()) {
    std::cerr << "Electric rotor link '" << link_name << "' is not found." << std::endl;
    return nullptr;
  }
  return std::static_pointer_cast<ElectricRotorConfig>(it->second);
}
}  // namespace tobas
