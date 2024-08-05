#pragma once

#include <vector>
#include <eigen3/Eigen/Core>

#include <tobas_std_tools/range.hpp>
#include <tobas_kdl/frames.hpp>

namespace tobas
{
struct VehicleParameters
{
  double wing_surface = -1;              // Wing surface [m^2]
  double wing_span = -1;                 // Wing span [m]
  double mac = -1;                       // Mean Aerodynamic Chord [m]
  kdl::Vector ac;                        // Aerodynamic Center wrt the frame origin (NWU) [m]
  tobas_std::Range<double> alpha_limit;  // Stall angles [rad]
};

/**
 * @brief Aerodynamics stability derivatives independent of control surfaces.
 * The moment reference point for the wing is at the wing quarter-chord.
 */
struct AerodynamicsCoefficients
{
  // Lift force
  double c_lift_0;      // [-]
  double c_lift_alpha;  // [/rad]

  // Drag force
  double c_drag_0;      // [-]
  double c_drag_alpha;  // [/rad]

  // Side force
  double c_side_beta;  // [/rad]

  // Roll moment
  double c_roll_beta;  // [/rad]
  double c_roll_p;     // [s/rad]
  double c_roll_r;     // [s/rad]

  // Pitch moment
  double c_pitch_0;           // [-]
  double c_pitch_alpha;       // [/rad]
  double c_pitch_abs_beta;    // [/rad]
  double c_pitch_alpha_rate;  // [s/rad]
  double c_pitch_q;           // [s/rad]

  // Yaw moment
  double c_yaw_beta;  // [/rad]
  double c_yaw_p;     // [s/rad]
  double c_yaw_r;     // [s/rad]
};

/**
 * @brief Control sufrace.
 * The moment reference point for the wing is at the wing quarter-chord.
 * A rotation axis parallel to the Y or Z axis is assumed.
 */
struct ControlSurface
{
  int index;  // 舵角配列における添字
  std::string joint_name;
  tobas_std::Range<double> angle_limit;
  double max_angle_rate;

  double c_lift_delta;      // [/rad]
  double c_drag_abs_delta;  // [/rad], 舵角の正負にかかわらず抗力が発生するモデル
  double c_side_delta;      // [/rad]
  double c_roll_delta;      // [/rad]
  double c_pitch_delta;     // [/rad]
  double c_yaw_delta;       // [/rad]
};

using ControlSurfaces = std::vector<ControlSurface>;

struct FixedWingConfig
{
  VehicleParameters vehicle;
  AerodynamicsCoefficients aerodynamics;
  ControlSurfaces control_surfaces;
};
}  // namespace tobas
