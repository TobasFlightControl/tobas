#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_kdl/vector.hpp>
#include <tobas_std_tools/range.hpp>

namespace tobas
{
class VehicleParameters
{
  static constexpr char kWingSurfaceKey[] = "wing_surface";
  static constexpr char kWingSpanKey[] = "wing_span";
  static constexpr char kMACKey[] = "mean_aerodynamic_chord";
  static constexpr char kAeroCenterKey[] = "aerodynamic_center";
  static constexpr char kAlphaLimitLKey[] = "alpha_limit";

public:
  double wing_surface = 0;                          // Wing surface [m^2]
  double wing_span = 0;                             // Wing span [m]
  double mac = 0;                                   // Mean Aerodynamic Chord [m]
  kdl::Vector ac = { 0, 0, 0 };                     // Aerodynamic Center wrt the frame origin (NWU) [m]
  tobas_std::Range<double> alpha_limit = { 0, 0 };  // Stall angles [rad]

  bool isValid() const;

  bool load(const YAML::Node& node);
  YAML::Node dump() const;
};
}  // namespace tobas
