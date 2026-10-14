// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_system_plugins/sdf.hpp"

#include <tobas_drone_core/propulsion_system/turning_direction.hpp>

namespace tobas::gazebo
{
bool getTurningDirection(const sdf::ElementConstPtr& _sdf, int& _dst)
{
  constexpr char kDirectionKey[] = "turningDirection";

  if (!_sdf->HasElement(kDirectionKey)) {
    return false;
  }
  const auto direction_text = _sdf->Get<std::string>(kDirectionKey);

  TurningDirection direction_enum;
  if (!enumFromText(direction_text, direction_enum)) {
    return false;
  }

  _dst = sign(direction_enum);
  return true;
}
}  // namespace tobas::gazebo
