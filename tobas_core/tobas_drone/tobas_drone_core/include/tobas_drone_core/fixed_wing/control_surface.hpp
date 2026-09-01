// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstdint>
#include <map>
#include <string>

#include <yaml-cpp/yaml.h>

#include <tobas_std_tools/range.hpp>

namespace tobas
{
class ControlSurface;
using ControlSurfaceMap = std::map<std::string, ControlSurface>;  // Joint Name -> ControlSurface

/**
 * @brief Control sufrace.
 * The moment reference point for the wing is at the wing quarter-chord.
 * A rotation axis parallel to the Y or Z axis is assumed.
 */
class ControlSurface
{
public:
  std::string link_name = "";

  double c_lift_delta = 0.0;      // [/rad]
  double c_drag_abs_delta = 0.0;  // [/rad]
  double c_side_delta = 0.0;      // [/rad]
  double c_roll_delta = 0.0;      // [/rad]
  double c_pitch_delta = 0.0;     // [/rad]
  double c_yaw_delta = 0.0;       // [/rad]

  bool isValid() const;

  bool load(const YAML::Node& node);
  YAML::Node dump() const;
};
}  // namespace tobas
