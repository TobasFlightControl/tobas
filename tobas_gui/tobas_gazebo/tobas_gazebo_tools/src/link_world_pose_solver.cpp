// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_tools/link_world_pose_solver.hpp"

#include <ostream>

#include <gz/common/Console.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>

namespace tobas
{
namespace gazebo
{
LinkWorldPoseSolver::LinkWorldPoseSolver()
{
}

bool LinkWorldPoseSolver::initialize(const gz::sim::Entity& _model, const gz::sim::EntityComponentManager& _ecm)
{
  if (!_ecm.Component<gz::sim::components::Model>(_model)) {
    ignerr << "Model does not exist.\n";
    return false;
  }

  model_ = _model;
  return true;
}

bool LinkWorldPoseSolver::solve(const gz::sim::Entity& _link, const gz::sim::EntityComponentManager& _ecm)
{
  const auto world = _ecm.ParentEntity(model_);
  if (!_ecm.Component<gz::sim::components::World>(world)) {
    ignerr << "Model must be a direct child of the world.\n";
    return false;
  }

  if (_ecm.ParentEntity(_link) != model_) {
    ignerr << "Link must be a direct child of the model.\n";
    return false;
  }

  const auto T1 = _ecm.Component<gz::sim::components::Pose>(model_);
  if (!T1) {
    ignerr << "Model does not have a pose component.\n";
    return false;
  }

  const auto T2 = _ecm.Component<gz::sim::components::Pose>(_link);
  if (!T2) {
    ignerr << "Link does not have a pose component.\n";
    return false;
  }

  pose_out_ = T1->Data() * T2->Data();
  return true;
}
}  // namespace gazebo
}  // namespace tobas
