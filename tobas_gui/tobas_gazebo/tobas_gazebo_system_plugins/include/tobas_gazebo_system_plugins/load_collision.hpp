// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <optional>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

namespace tobas
{
namespace gazebo
{
/* Check a proposed box load against every collision in the aircraft model. */
std::optional<std::string> checkLoadCollision(
  gz::sim::Entity model,
  const gz::math::Pose3d& load_pose,
  const gz::math::Vector3d& load_size,
  const gz::sim::EntityComponentManager& ecm);
}  // namespace gazebo
}  // namespace tobas
