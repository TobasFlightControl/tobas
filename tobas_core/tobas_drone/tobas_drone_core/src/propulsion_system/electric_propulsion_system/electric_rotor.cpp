// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/electric_propulsion_system/electric_rotor.hpp"

#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas
{
namespace
{
constexpr char kChannelKey[] = "channel";
constexpr char kNumPolesKey[] = "num_poles";
constexpr char kKvKey[] = "kv";
constexpr char kInternalResistanceKey[] = "internal_resistance";
constexpr char kMinSpeed[] = "minimum_speed";
constexpr char kPropellerDiameterKey[] = "propeller_diameter";
constexpr char kMotorConstKey[] = "motor_constant";
constexpr char kMomentConstKey[] = "moment_constant";
}  // namespace

std::expected<void, std::string> ElectricRotorConfig::validate() const
{
  if (const auto result = super::validate(); !result) {
    return result;
  }

  if (num_poles <= 0) {
    return std::unexpected("The number of poles must be positive.");
  }

  if (num_poles % 2 != 0) {
    return std::unexpected("The number of poles must be even.");
  }

  if (kv <= 0.0) {
    return std::unexpected("Kv value must be positive.");
  }

  if (internal_resistance <= 0.0) {
    return std::unexpected("Internal resistance must be positive.");
  }

  if (min_speed < 0.0) {
    return std::unexpected("Minimum rotation speed must be non-negative.");
  }

  if (propeller_diameter <= 0.0) {
    return std::unexpected("Propeller diameter must be positive.");
  }

  if (motor_const <= 0.0) {
    return std::unexpected("Motor constant must be positive.");
  }

  if (moment_const <= 0.0) {
    return std::unexpected("Moment constant must be positive.");
  }

  return {};
}

std::expected<void, std::string> ElectricRotorConfig::load(const YAML::Node& node)
{
  if (!node.IsDefined() || !node.IsMap()) {
    return std::unexpected("Configuration node must be a map.");
  }

  if (const auto result = super::load(node); !result) {
    return result;
  }

  if (const auto result = yaml::load(kChannelKey, node, channel); !result) {
    return result;
  }

  if (const auto result = yaml::load(kNumPolesKey, node, num_poles); !result) {
    return result;
  }

  if (const auto result = yaml::load(kKvKey, node, kv); !result) {
    return result;
  }

  if (const auto result = yaml::load(kInternalResistanceKey, node, internal_resistance); !result) {
    return result;
  }

  if (const auto result = yaml::load(kMinSpeed, node, min_speed); !result) {
    return result;
  }

  if (const auto result = yaml::load(kPropellerDiameterKey, node, propeller_diameter); !result) {
    return result;
  }

  if (const auto result = yaml::load(kMotorConstKey, node, motor_const); !result) {
    return result;
  }

  if (const auto result = yaml::load(kMomentConstKey, node, moment_const); !result) {
    return result;
  }

  return {};
}

YAML::Node ElectricRotorConfig::dump() const
{
  auto node = super::dump();

  node[kChannelKey] = channel;
  node[kNumPolesKey] = num_poles;
  node[kKvKey] = yaml::format(kv);
  node[kInternalResistanceKey] = yaml::format(internal_resistance);
  node[kMinSpeed] = yaml::format(min_speed);
  node[kPropellerDiameterKey] = yaml::format(propeller_diameter);
  node[kMotorConstKey] = yaml::format(motor_const);
  node[kMomentConstKey] = yaml::format(moment_const);

  return node;
}
}  // namespace tobas
