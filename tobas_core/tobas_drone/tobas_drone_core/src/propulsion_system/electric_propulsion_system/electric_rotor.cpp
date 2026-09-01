// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/electric_propulsion_system/electric_rotor.hpp"

#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/format.hpp>

using namespace std;

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

  if (kv <= 0.0) {
    cerr << "Kv value must be positive." << endl;
    return false;
  }

  if (internal_resistance <= 0.0) {
    cerr << "Internal resistance must be positive." << endl;
    return false;
  }

  if (min_speed < 0.0) {
    cerr << "Minimum rotation speed must be non-negative." << endl;
    return false;
  }

  if (propeller_diameter <= 0.0) {
    cerr << "Propeller diameter must be positive." << endl;
    return false;
  }

  if (motor_const <= 0.0) {
    cerr << "Motor constant must be positive." << endl;
    return false;
  }

  if (moment_const <= 0.0) {
    cerr << "Moment constant must be positive." << endl;
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

  if (!yaml::load(kMinSpeed, node, min_speed)) {
    return false;
  }

  if (!yaml::load(kPropellerDiameterKey, node, propeller_diameter)) {
    return false;
  }

  if (!yaml::load(kMotorConstKey, node, motor_const)) {
    return false;
  }

  if (!yaml::load(kMomentConstKey, node, moment_const)) {
    return false;
  }

  return true;
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
