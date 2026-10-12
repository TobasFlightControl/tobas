// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <map>
#include <string>

#include <urdf/model.h>

#include "./control_surface.hpp"
#include "./thrust.hpp"
#include "./tilt_rotor.hpp"

namespace tobas
{
namespace uadf
{
class Model
{
public:
  ::urdf::ModelInterfaceSharedPtr urdf;

  std::map<std::string, Thrust> thrusts;
  std::map<std::string, ControlSurface> control_surfaces;
  std::map<std::string, TiltJoint> tilts;

  explicit Model();

  void clear();

  std::expected<void, std::string> validate() const;
};
}  // namespace uadf
}  // namespace tobas
