// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/fixed_wing/fixed_wing.hpp"

#include <ranges>

#include <tobas_yaml_tools/core.hpp>

namespace tobas
{
namespace
{
constexpr char kVehicleKey[] = "vehicle";
constexpr char kAerodynamicsKey[] = "aerodynamics";
constexpr char kControlSurfacesKey[] = "control_surfaces";
}  // namespace

void FixedWingConfig::clear()
{
  control_surfaces.clear();
}

std::expected<void, std::string> FixedWingConfig::validate() const
{
  if (const auto result = vehicle.validate(); !result) {
    return std::unexpected("Vehicle parameters: " + result.error());
  }

  if (const auto result = aerodynamics.validate(); !result) {
    return std::unexpected("Aerodynamic parameters: " + result.error());
  }

  for (const auto& [_, cs] : control_surfaces) {
    if (const auto result = cs.validate(); !result) {
      return std::unexpected("Control surface '" + cs.link_name + "': " + result.error());
    }
  }

  return {};
}

std::expected<void, std::string> FixedWingConfig::load(const YAML::Node& root_node)
{
  if (!root_node.IsDefined() || !root_node.IsMap()) {
    return std::unexpected("Fixed wing node must be a map.");
  }

  clear();

  // Vehicle
  const auto vehicle_node = root_node[kVehicleKey];
  if (!vehicle_node.IsDefined()) {
    return std::unexpected(std::string("'") + kVehicleKey + "' is not defined.");
  }
  if (const auto result = vehicle.load(vehicle_node); !result) {
    return std::unexpected("Vehicle parameters: " + result.error());
  }

  // Aerodynamics
  const auto aero_node = root_node[kAerodynamicsKey];
  if (!aero_node.IsDefined()) {
    return std::unexpected(std::string("'") + kAerodynamicsKey + "' is not defined.");
  }
  if (const auto result = aerodynamics.load(aero_node); !result) {
    return std::unexpected("Aerodynamic parameters: " + result.error());
  }

  // Control surfaces
  const auto css_node = root_node[kControlSurfacesKey];
  if (!css_node.IsDefined() || !css_node.IsSequence()) {
    return std::unexpected(std::string("'") + kControlSurfacesKey + "' must be a sequence.");
  }
  for (const auto& [idx, cs_node] : std::views::enumerate(css_node)) {
    ControlSurface cs;
    if (const auto result = cs.load(cs_node); !result) {
      return std::unexpected("Control surfaces[" + std::to_string(idx) + "]: " + result.error());
    }
    control_surfaces[cs.link_name] = cs;
  }

  return {};
}

YAML::Node FixedWingConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  // Vehicle
  node[kVehicleKey] = vehicle.dump();

  // Aerodynamics
  node[kAerodynamicsKey] = aerodynamics.dump();

  // Control surfaces
  node[kControlSurfacesKey] = YAML::Node(YAML::NodeType::Sequence);
  for (auto& [_, cs] : control_surfaces) {
    node[kControlSurfacesKey].push_back(cs.dump());
  }

  return node;
}
}  // namespace tobas
