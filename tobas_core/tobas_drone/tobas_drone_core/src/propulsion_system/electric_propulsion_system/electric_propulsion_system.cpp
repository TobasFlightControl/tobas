// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp"

#include <iostream>
#include <memory>
#include <ranges>

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

std::expected<void, std::string> ElectricPropulsionSystemConfig::load(const YAML::Node& root_node)
{
  if (!root_node.IsDefined() || !root_node.IsMap()) {
    return std::unexpected("Electric propulsion system node must be a map.");
  }

  clear();

  // Rotors
  const auto rotors_node = root_node[kRotorsKey];
  if (!rotors_node.IsDefined()) {
    return std::unexpected(std::string("'") + kRotorsKey + "' is not defined.");
  }
  if (!rotors_node.IsSequence()) {
    return std::unexpected(std::string("'") + kRotorsKey + "' must be a sequence.");
  }
  for (const auto& [idx, rotor_node] : std::views::enumerate(rotors_node)) {
    const auto erotor = std::make_shared<ElectricRotorConfig>();
    if (const auto result = erotor->load(rotor_node); !result) {
      return std::unexpected("Rotors[" + std::to_string(idx) + "]: " + result.error());
    }
    rotors[erotor->link_name] = erotor;
  }

  // Battery
  const auto battery_node = root_node[kBatteryKey];
  if (!battery_node.IsDefined()) {
    return std::unexpected(std::string("'") + kBatteryKey + "' is not defined.");
  }
  if (const auto result = battery.load(battery_node); !result) {
    return std::unexpected("Battery: " + result.error());
  }

  return {};
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
