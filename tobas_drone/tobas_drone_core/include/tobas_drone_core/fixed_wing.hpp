#pragma once

#include <yaml-cpp/yaml.h>

#include "./vehicle_params.hpp"
#include "./aerodynamic_coefs.hpp"
#include "./control_surface.hpp"

namespace tobas
{
class FixedWingConfig
{
  static constexpr char kEquippedKey[] = "equipped";
  static constexpr char kVehicleKey[] = "vehicle";
  static constexpr char kAerodynamicsKey[] = "aerodynamics";
  static constexpr char kControlSurfacesKey[] = "control_surfaces";

public:
  bool equipped = false;
  VehicleParameters vehicle;
  AerodynamicCoefficients aerodynamics;
  ControlSurfaceMap control_surfaces;

  bool isValid() const;
  bool load(const YAML::Node& node);
  YAML::Node dump() const;
};
}  // namespace tobas
