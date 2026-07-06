// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/electric_propulsion_system/battery.hpp"

#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/format.hpp>

using namespace std;

namespace tobas
{
bool BatteryConfig::isValid() const
{
  if (sag_voltage <= 0.0) {
    cerr << "Battery sag voltage must be positive." << endl;
    return false;
  }

  if (nominal_voltage <= sag_voltage) {
    cerr << "Battery nominal voltage must be greater than sag voltage." << endl;
    return false;
  }

  if (max_voltage <= nominal_voltage) {
    cerr << "Battery max voltage must be greater than nominal voltage." << endl;
    return false;
  }

  if (max_current <= 0.0) {
    cerr << "Battery max current must be positive." << endl;
    return false;
  }

  if (internal_resistance < 0.0) {
    cerr << "Battery internal resistance must be non-negative." << endl;
    return false;
  }

  return true;
}

bool BatteryConfig::load(const YAML::Node& node)
{
  if (!yaml::load(kNominalVoltageKey, node, nominal_voltage)) {
    return false;
  }

  if (!yaml::load(kMaxVoltageKey, node, max_voltage)) {
    return false;
  }

  if (!yaml::load(kSagVoltageKey, node, sag_voltage)) {
    return false;
  }

  if (!yaml::load(kMaxCurrentKey, node, max_current)) {
    return false;
  }

  if (!yaml::load(kInternalResistanceKey, node, internal_resistance)) {
    return false;
  }

  return true;
}

YAML::Node BatteryConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNominalVoltageKey] = yaml::format(nominal_voltage);
  node[kMaxVoltageKey] = yaml::format(max_voltage);
  node[kSagVoltageKey] = yaml::format(sag_voltage);
  node[kMaxCurrentKey] = yaml::format(max_current);
  node[kInternalResistanceKey] = yaml::format(internal_resistance);

  return node;
}
}  // namespace tobas
