// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/components.hh>

namespace tobas
{
namespace gazebo
{
/* Get the component directly under an entity. Create a new one if it does not exist. */
template <typename CompType>
CompType* getComponent(const gz::sim::Entity& entity, gz::sim::EntityComponentManager& ecm)
{
  if (ecm.EntityHasComponentType(entity, CompType().TypeId())) {
    return ecm.Component<CompType>(entity);
  }
  else {
    return ecm.CreateComponent(entity, CompType());
  }
}

bool belongsTo(const gz::sim::Entity& entity, const gz::sim::Entity& target, const gz::sim::EntityComponentManager& ecm);

std::optional<gz::sim::Entity>
findJointWithChildLink(const gz::sim::EntityComponentManager& ecm, const std::string& link_name);
}  // namespace gazebo
}  // namespace tobas
