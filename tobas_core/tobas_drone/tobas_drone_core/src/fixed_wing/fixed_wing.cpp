// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/fixed_wing/fixed_wing.hpp"

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

bool FixedWingConfig::load(const YAML::Node& root_node)
{
  clear();

  // Vehicle
  const auto vehicle_node = root_node[kVehicleKey];
  if (!vehicle_node.IsDefined()) {
    std::cerr << "'" << kVehicleKey << "' is not defined." << std::endl;
    return false;
  }
  if (!vehicle.load(vehicle_node)) {
    std::cerr << "Failed to load vehicle parameters." << std::endl;
    return false;
  }

  // Aerodynamics
  const auto aero_node = root_node[kAerodynamicsKey];
  if (!aero_node.IsDefined()) {
    std::cerr << "'" << kAerodynamicsKey << "' is not defined." << std::endl;
    return false;
  }
  if (!aerodynamics.load(aero_node)) {
    std::cerr << "Failed to load aerodynamic parameters." << std::endl;
    return false;
  }

  // Control surfaces
  const auto css_node = root_node[kControlSurfacesKey];
  if (!css_node.IsSequence()) {
    std::cerr << "'" << kControlSurfacesKey << "' is not defined." << std::endl;
    return false;
  }
  for (const auto& cs_node : css_node) {
    ControlSurface cs;
    if (!cs.load(cs_node)) {
      std::cerr << "Failed to load the configuration of control surfaces." << std::endl;
      return false;
    }
    control_surfaces[cs.link_name] = cs;
  }

  return true;
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
