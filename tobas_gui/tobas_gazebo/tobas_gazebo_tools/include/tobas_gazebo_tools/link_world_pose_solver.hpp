// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <gz/math/Pose3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

namespace tobas
{
namespace gazebo
{
class LinkWorldPoseSolver
{
public:
  explicit LinkWorldPoseSolver();

  bool initialize(const gz::sim::Entity& model, const gz::sim::EntityComponentManager& ecm);

  bool solve(const gz::sim::Entity& link, const gz::sim::EntityComponentManager& ecm);

  const gz::math::Pose3d& getWorldPose() const
  {
    return pose_out_;
  }

private:
  gz::sim::Entity model_;
  gz::math::Pose3d pose_out_;
};
}  // namespace gazebo
}  // namespace tobas
