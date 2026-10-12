// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/fixed_wing/vehicle_params.hpp"

#include <tobas_yaml_tools/convert/eigen.hpp>
#include <tobas_yaml_tools/convert/range.hpp>
#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/format.hpp>

namespace tobas
{
namespace
{
constexpr char kWingSurfaceKey[] = "wing_surface";
constexpr char kWingSpanKey[] = "wing_span";
constexpr char kMACKey[] = "mean_aerodynamic_chord";
constexpr char kAeroCenterKey[] = "aerodynamic_center";
constexpr char kAlphaLimitLKey[] = "alpha_limit";
}  // namespace

std::expected<void, std::string> VehicleParameters::validate() const
{
  if (wing_surface <= 0) {
    return std::unexpected("Wing surface must be positive.");
  }

  if (wing_span <= 0) {
    return std::unexpected("Wing span must be positive.");
  }

  if (mac <= 0) {
    return std::unexpected("Mean aerodynamic chord must be positive.");
  }

  if (!alpha_limit.isValid()) {
    return std::unexpected("Invalid stall angles.");
  }

  return {};
}

std::expected<void, std::string> VehicleParameters::load(const YAML::Node& node)
{
  if (!node.IsDefined() || !node.IsMap()) {
    return std::unexpected("Configuration node must be a map.");
  }

  if (const auto result = yaml::load(kWingSurfaceKey, node, wing_surface); !result) {
    return result;
  }

  if (const auto result = yaml::load(kWingSpanKey, node, wing_span); !result) {
    return result;
  }

  if (const auto result = yaml::load(kMACKey, node, mac); !result) {
    return result;
  }

  if (const auto result = yaml::load(kAeroCenterKey, node, ac.data); !result) {
    return result;
  }

  if (const auto result = yaml::load(kAlphaLimitLKey, node, alpha_limit); !result) {
    return result;
  }

  return {};
}

YAML::Node VehicleParameters::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kWingSurfaceKey] = yaml::format(wing_surface);
  node[kWingSpanKey] = yaml::format(wing_span);
  node[kMACKey] = yaml::format(mac);
  node[kAeroCenterKey] = ac.data;
  node[kAlphaLimitLKey] = alpha_limit;

  return node;
}
}  // namespace tobas
