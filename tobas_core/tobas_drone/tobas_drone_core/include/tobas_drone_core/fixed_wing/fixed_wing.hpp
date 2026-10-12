// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include <yaml-cpp/yaml.h>

#include "./aerodynamic_coefs.hpp"
#include "./control_surface.hpp"
#include "./vehicle_params.hpp"

namespace tobas
{
class FixedWingConfig
{
public:
  using SharedPtr = std::shared_ptr<FixedWingConfig>;
  using ConstSharedPtr = std::shared_ptr<const FixedWingConfig>;

  VehicleParameters vehicle;
  AerodynamicCoefficients aerodynamics;
  ControlSurfaceMap control_surfaces;

  void clear();

  std::expected<void, std::string> validate() const;

  std::expected<void, std::string> load(const YAML::Node& node);
  YAML::Node dump() const;

  inline size_t numControlSurfaces() const;
};

inline size_t FixedWingConfig::numControlSurfaces() const
{
  return control_surfaces.size();
}
}  // namespace tobas
