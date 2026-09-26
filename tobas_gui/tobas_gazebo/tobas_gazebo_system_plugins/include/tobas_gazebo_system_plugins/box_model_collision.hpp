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
/* Check a box against every collision in a model. */
std::optional<std::string> checkBoxModelCollision(
  gz::sim::Entity model,
  const gz::math::Pose3d& box_pose,
  const gz::math::Vector3d& box_size,
  const gz::sim::EntityComponentManager& ecm);
}  // namespace gazebo
}  // namespace tobas
