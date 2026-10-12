// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/electric_propulsion_system/battery.hpp"

#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas
{
namespace
{
constexpr char kNominalVoltageKey[] = "nominal_voltage";
constexpr char kMaxVoltageKey[] = "max_voltage";
constexpr char kSagVoltageKey[] = "sag_voltage";
constexpr char kMaxCurrentKey[] = "max_current";
constexpr char kInternalResistanceKey[] = "internal_resistance";
}  // namespace

std::expected<void, std::string> BatteryConfig::validate() const
{
  if (sag_voltage <= 0.0) {
    return std::unexpected("Battery sag voltage must be positive.");
  }

  if (nominal_voltage <= sag_voltage) {
    return std::unexpected("Battery nominal voltage must be greater than sag voltage.");
  }

  if (max_voltage <= nominal_voltage) {
    return std::unexpected("Battery max voltage must be greater than nominal voltage.");
  }

  if (max_current <= 0.0) {
    return std::unexpected("Battery max current must be positive.");
  }

  if (internal_resistance < 0.0) {
    return std::unexpected("Battery internal resistance must be non-negative.");
  }

  return {};
}

std::expected<void, std::string> BatteryConfig::load(const YAML::Node& node)
{
  if (!node.IsDefined() || !node.IsMap()) {
    return std::unexpected("Configuration node must be a map.");
  }

  if (const auto result = yaml::load(kNominalVoltageKey, node, nominal_voltage); !result) {
    return result;
  }

  if (const auto result = yaml::load(kMaxVoltageKey, node, max_voltage); !result) {
    return result;
  }

  if (const auto result = yaml::load(kSagVoltageKey, node, sag_voltage); !result) {
    return result;
  }

  if (const auto result = yaml::load(kMaxCurrentKey, node, max_current); !result) {
    return result;
  }

  if (const auto result = yaml::load(kInternalResistanceKey, node, internal_resistance); !result) {
    return result;
  }

  return {};
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
