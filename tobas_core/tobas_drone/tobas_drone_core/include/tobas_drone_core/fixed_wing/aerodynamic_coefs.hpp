// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
/**
 * @brief Aerodynamics stability derivatives independent of control surfaces.
 * The moment reference point for the wing is at the wing quarter-chord.
 */
class AerodynamicCoefficients
{
public:
  // Lift force
  double c_lift_0 = 0;      // [-]
  double c_lift_alpha = 0;  // [/rad]

  // Drag force
  double c_drag_0 = 0;      // [-]
  double c_drag_alpha = 0;  // [/rad]

  // Side force
  double c_side_beta = 0;  // [/rad]

  // Roll moment
  double c_roll_beta = 0;  // [/rad]
  double c_roll_p = 0;     // [s/rad]
  double c_roll_r = 0;     // [s/rad]

  // Pitch moment
  double c_pitch_0 = 0;           // [-]
  double c_pitch_alpha = 0;       // [/rad]
  double c_pitch_abs_beta = 0;    // [/rad]
  double c_pitch_alpha_rate = 0;  // [s/rad]
  double c_pitch_q = 0;           // [s/rad]

  // Yaw moment
  double c_yaw_beta = 0;  // [/rad]
  double c_yaw_p = 0;     // [s/rad]
  double c_yaw_r = 0;     // [s/rad]

  bool isValid() const;

  bool load(const YAML::Node& node);
  YAML::Node dump() const;
};
}  // namespace tobas
